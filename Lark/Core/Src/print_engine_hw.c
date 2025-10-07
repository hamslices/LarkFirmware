/*
 * print_engine_hw.c
 *
 *  Created on: Sep 4, 2025
 *      Author: HamSlices
 */

#include "main.h"
#include "print_engine_hw.h"
#include "hardware_definitions.h"
#include "s-curve.h"
#include "timer_calculations.h"

// --- Private Module Variables ---

// Pointers to shared atomic variables managed by the main print_engine.c module.
// These are used for safe, interrupt-proof communication between this hardware
// layer and the main state machine.
static volatile atomic_uint_least32_t* g_status_ptr       = NULL;
static volatile atomic_print_engine_state_t* g_state_ptr  = NULL;
static volatile atomic_uint_least16_t* g_step_counter_ptr = NULL;
static volatile atomic_int_least32_t*  g_move_counter_ptr = NULL;

// ISR-managed variables for hardware operation.
static volatile uint32_t g_active_pulse_ticks       = 0; // Pre-calculated strobe timer ticks for the current job.
static volatile uint16_t g_manual_feed_step_counter = 1; // Tracks current motor phase for manual feed/move.
static volatile uint16_t g_motor_phase_counter      = 1; // Tracks current motor phase during a print job.

static uint16_t g_strobe_group_size_steps      = 0; // Pre-calculated number of steps in one strobe group.
static volatile uint16_t g_strobe_step_counter = 0; // A counter for steps within the current strobe group.

// Atomic flag for graceful stop requests from the main loop
static volatile atomic_bool g_stop_request_flag = ATOMIC_VAR_INIT(false);

// Pre-calculated motor period for constant speed
static uint32_t g_const_motor_period_ticks = 0;

// Local copy of job settings and pointer to the current microstepping Look-Up Table (LUT).
static print_job_settings_t g_hw_job_settings;
static IO_State const * g_current_io_state_lut = NULL;

// --- S-Curve Integration Variables ---
// This LUT is populated by generate_scurve_lut() and contains timer tick values for the acceleration profile.
extern uint32_t g_accel_profile_ticks[MAX_ACCEL_STEPS];
static uint16_t g_accel_ramp_steps = 0; // The actual number of steps in the generated ramp.

// Timers + SPI
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern SPI_HandleTypeDef hspi1;

#define SPI_DATA_SIZE_PER_LINE 216

// --- Private Function Prototypes ---
static void handle_printing_step_isr(void);
static void handle_manual_feed_step_isr(void);
static void handle_manual_move_step_isr(void);
static void handle_ramp_down_step_isr(void);

static inline void hw_set_motor_period(uint32_t period_ticks);
static uint32_t calculate_and_apply_timer_settings(const print_job_settings_t* settings);

static void update_lut_pointer(uint8_t resolution);
static IO_State get_io_state_for_step(int step_index, uint16_t seq_len);

static inline void isr_advance_motor_phase(volatile uint16_t* phase_counter, uint16_t seq_len);
static inline void isr_handle_ramp_up_phase(int remaining_steps, uint16_t seq_len);
static inline void isr_handle_printing_phase(int remaining_steps, uint16_t seq_len);

// --- Public Function Implementations ---

void print_engine_hw_init(volatile atomic_uint_least32_t* status_ptr,
                          volatile atomic_print_engine_state_t* state_ptr,
                          volatile atomic_uint_least16_t* step_counter_ptr,
                          volatile atomic_int_least32_t* move_counter_ptr)
{
    // Store the pointers to the shared atomic variables from the main engine.
    g_status_ptr = status_ptr;
    g_state_ptr = state_ptr;
    g_step_counter_ptr = step_counter_ptr;
    g_move_counter_ptr = move_counter_ptr;
}

void print_engine_hw_apply_settings(const print_job_settings_t* settings)
{
    // Store a local copy of the settings for quick access within ISRs.
    g_hw_job_settings = *settings;

    // Select the correct microstepping Look-Up Table (LUT) based on the desired resolution.
    update_lut_pointer(settings->step_resolution);

    // Configure the physical motor driver IC (direction, current, microstep mode).
    hardware_step_set_resolution(settings->step_resolution);
    hardware_motor_set_direction(settings->direction == DIRECTION_FORWARD);
    hardware_motor_set_current(settings->motor_current_ma);
}

void print_engine_hw_request_stop(void)
{
    atomic_store(&g_stop_request_flag, true);
}

void print_engine_hw_start_printing(void)
{
    // Reset the stop request flag at the beginning of a new job.
    atomic_store(&g_stop_request_flag, false);

    // Calculate the S-curve acceleration profile for the current job's speed.
    g_accel_ramp_steps = generate_scurve_lut(&g_hw_job_settings);

    // --- Pre-calculate all constant-speed timer parameters ---
    // This is done once per job, outside of any ISR.
    timer_params_t const_params = calculate_all_timer_params(&g_hw_job_settings);
    g_const_motor_period_ticks  = const_params.motor_period_ticks;
    g_active_pulse_ticks        = const_params.strobe_pulse_ticks;

    // Pre-calculate the strobe group size once per job.
    g_strobe_group_size_steps = g_hw_job_settings.sequence_length / STROBE_GROUPS_PER_CYCLE;
    g_strobe_step_counter = 0; // Ensure counter starts at 0 for the first line.

    if (g_accel_ramp_steps > 0) {
        // RAMP REQUIRED: Pre-load the motor timer with the period for the very first step.
        hw_set_motor_period(g_accel_profile_ticks[0]);
        atomic_store(g_step_counter_ptr, g_accel_ramp_steps);
    } else {
        // NO RAMP: The target speed is low. Set the timer to the constant target speed immediately.
        calculate_and_apply_timer_settings(&g_hw_job_settings);
        // The "run-up" phase will be just one step long to handle the state transition.
        atomic_store(g_step_counter_ptr, 1);
    }

    // Reset the motor phase for the start of the job.
    g_motor_phase_counter = 1;

    // Energize the motor and start the master step timer to begin movement.
    hardware_motor_active();

    // Start watchdog for all print activity
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim6);

    __HAL_TIM_ENABLE(&htim3);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

void print_engine_hw_start_manual_feed(const print_job_settings_t* settings)
{
    // Reset the stop request flag at the beginning of a new job.
    atomic_store(&g_stop_request_flag, false);

    print_engine_hw_apply_settings(settings);
    // For simple moves, calculate and apply settings directly.
    calculate_and_apply_timer_settings(settings);
    g_manual_feed_step_counter = 1;

    hardware_motor_active();

    // start the watch dog
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim6);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
}

void print_engine_hw_start_move(const print_job_settings_t* settings)
{
    // Reset the stop request flag at the beginning of a new job.
    atomic_store(&g_stop_request_flag, false);

    print_engine_hw_apply_settings(settings);
    // For simple moves, calculate and apply settings directly.
    calculate_and_apply_timer_settings(settings);
    g_manual_feed_step_counter = 1;

    hardware_motor_active();

    // Start watchdog for manual move
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim6);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
}

void print_engine_hw_stop_all(void)
{
    // A critical safety function. Disable all timers and their associated interrupts.
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE); // Motor Step Timer
    __HAL_TIM_DISABLE(&htim3);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC1);    // Strobe Pulse Timer
    __HAL_TIM_DISABLE(&htim5);
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE); // Watchdog Timer
    __HAL_TIM_DISABLE(&htim6);

    // Ensure all physical outputs are in a safe, de-energized state.
    hardware_all_strobes_off();
    hardware_strobes_disable();
    hardware_motor_idle();
}

HAL_StatusTypeDef print_engine_hw_transmit_line(uint8_t* buffer)
{
    return HAL_SPI_Transmit_DMA(&hspi1, buffer, SPI_DATA_SIZE_PER_LINE);
}


// --- ISRs and State-Specific Handlers ---

void print_engine_hw_step_isr(void)
{
    // This is the main timer ISR. It acts as a dispatcher based on the system's current state.
    print_engine_state_t current_state = atomic_load(g_state_ptr);

    switch (current_state)
    {
        case PE_STATE_MANUAL_MOVE:
            handle_manual_move_step_isr();
            break;
        case PE_STATE_MANUAL_FEED:
            handle_manual_feed_step_isr();
            break;
        // All printing-related states before ramp-down are handled by the same function.
        case PE_STATE_AWAITING_RUN_UP_START:
        case PE_STATE_RAMP_UP:
        case PE_STATE_PRINTING:
        case PE_STATE_FINISHING:
            handle_printing_step_isr();
            break;
        // The post-print deceleration has its own dedicated handler.
        case PE_STATE_RAMP_DOWN:
            handle_ramp_down_step_isr();
            break;
        // In other states (IDLE, PAUSED), an interrupt might fire once before being disabled. Do nothing.
        default:
            break;
    }
}

void print_engine_hw_watchdog_isr(void)
{
    // This ISR should ideally never be called. If it is, it means the main step ISR
    // is blocked or stalled. This is a catastrophic failure.
    atomic_fetch_or(g_status_ptr, STATUS_FATAL_ERROR);
    atomic_store(g_state_ptr, PE_STATE_FAULT);
}

void print_engine_hw_strobe_off_isr(void)
{
    // This ISR has a single, simple job: turn off the printhead strobes at the end of a pulse.
    hardware_all_strobes_off();
}

void print_engine_hw_spi_tx_complete_callback(void)
{
    // This callback, triggered by the SPI DMA ISR, signals that the data for the
    // *next* line is now physically present in the printhead's shift registers.
    atomic_fetch_or(g_status_ptr, STATUS_NEXT_LINE_DATA_READY);
}

/**
 * @brief ISR handler for a single step during a continuous manual feed.
 * @details Now checks for the graceful stop request flag.
 */
static void handle_manual_feed_step_isr(void)
{
    // "Pet" the watchdog at the start of every ISR tick.
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    // 1. Check for a stop request from the main loop. This ensures the previous pulse
    //    has completed before we shut down the hardware.
    if (atomic_load(&g_stop_request_flag)) {
        print_engine_hw_stop_all();
        atomic_store(g_state_ptr, PE_STATE_JOB_DONE);
        return; // Halt further processing
    }

    // 2. If no stop is requested, just advance the motor phase for the next step.
    isr_advance_motor_phase(&g_manual_feed_step_counter, g_hw_job_settings.sequence_length);
}

/**
 * @brief ISR handler for a single step during a finite manual move.
 * @details Now checks for a graceful stop request flag at the
 *          beginning of its execution, ensuring the previous pulse is complete.
 */
static void handle_manual_move_step_isr(void)
{
    // "Pet" the watchdog at the start of every ISR tick.
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    // 1. Check for a stop request from the main loop. This is the highest priority.
    //    Because we check this *first*, we know the previous step's pulse has finished.
    if (atomic_load(&g_stop_request_flag)) {
        print_engine_hw_stop_all();
        atomic_store(g_state_ptr, PE_STATE_JOB_DONE);
        return; // Halt further processing
    }

    // 2. Always advance the motor phase.
    isr_advance_motor_phase(&g_manual_feed_step_counter, g_hw_job_settings.sequence_length);

    // 3. Decrement the counter and check if the move has naturally completed.
    if (atomic_fetch_sub(g_move_counter_ptr, 1) == 1) {
        // The step we just took was the last one. Stop everything.
        print_engine_hw_stop_all();
        atomic_store(g_state_ptr, PE_STATE_JOB_DONE);
    }
}

/**
 * @brief Top-level ISR handler for all active printing phases.
 */
static void handle_printing_step_isr(void)
{
    // "Pet" the watchdog at the start of every ISR tick.
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    const uint16_t seq_len = g_hw_job_settings.sequence_length;
    int current_state = atomic_load(g_state_ptr);

    // No matter the state, every step interrupt must advance the motor commutation.
    isr_advance_motor_phase(&g_motor_phase_counter, seq_len);

    // On the very first step interrupt of a job, transition from AWAITING to RAMP_UP.
    if (current_state == PE_STATE_AWAITING_RUN_UP_START) {
        atomic_store(g_state_ptr, PE_STATE_RAMP_UP);
        current_state = PE_STATE_RAMP_UP; // Update local copy for this ISR instance.
    }

    // Decrement the shared step counter for the current phase (ramp-up or print line).
    int remaining_steps = atomic_fetch_sub(g_step_counter_ptr, 1) - 1;

    // Delegate to the appropriate sub-handler based on the current phase.
    if (current_state == PE_STATE_RAMP_UP) {
        isr_handle_ramp_up_phase(remaining_steps, seq_len);
    } else if (current_state == PE_STATE_PRINTING || current_state == PE_STATE_FINISHING) {
        isr_handle_printing_phase(remaining_steps, seq_len);
    }
}

/**
 * @brief ISR handler for the deceleration ramp after the final print line.
 * @details This logic ensures the final pulse completes by stopping
 *          the hardware on the ISR tick *after* the final step is finished.
 */
static void handle_ramp_down_step_isr(void)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    // Read the counter value at the start of the ISR.
    int steps_remaining = atomic_load(g_step_counter_ptr);

    if (steps_remaining > 0) {
        // --- Execute a step ---
        // 1. Advance the motor phase for the current step.
        isr_advance_motor_phase(&g_motor_phase_counter, g_hw_job_settings.sequence_length);

        // 2. Decrement the counter for the next cycle.
        atomic_fetch_sub(g_step_counter_ptr, 1);

        // 3. If this was NOT the final step, set the timer period for the *next* step.
        if (steps_remaining > 1) {
            // The deceleration profile reads the S-curve LUT in reverse.
            // If N steps are remaining, we need the value at index N-2 for the next step.
            hw_set_motor_period(g_accel_profile_ticks[steps_remaining - 2]);
        }
        // If it was the final step (steps_remaining == 1), we do nothing here.
        // We let the ISR exit so the timer can run for its full final duration.
    } else {
        // --- Stop the motor ---
        // If steps_remaining was 0 when this ISR fired, it means the previous
        // ISR was the final one. The final pulse has now completed.
        // It is now safe to stop all hardware.
        print_engine_hw_stop_all();
        atomic_store(g_state_ptr, PE_STATE_JOB_DONE);
    }
}

// --- Inline ISR Helper Functions ---

/**
 * @brief Advances the motor commutation by one microstep.
 * @details This helper looks up the correct GPIO state from the active LUT and applies it.
 * @param phase_counter Pointer to the counter tracking the current motor phase.
 * @param seq_len The total number of steps in the current sequence (e.g., 64 for 1/8 step).
 */
static inline void isr_advance_motor_phase(volatile uint16_t* phase_counter, uint16_t seq_len)
{
    // --- INCREMENT STEP COUNTER HERE ---
    // Calculate microsteps per full step and pass it to the accumulator.
    // This ISR is called twice per microstep. We only increment the counter on the
    // first call of the pair (when the phase counter is odd) to ensure the
    // software counter exactly matches the number of physical steps taken.
    if ((*phase_counter % 2) == 1) {
        // The formula for microsteps per full step is seq_len / 8.
        // This is correct because seq_len is the number of ISR calls for 8 full steps.
        uint32_t microsteps_per_full_step = seq_len / 8;
        config_manager_increment_motor_steps(microsteps_per_full_step);
    }

    IO_State motor_io_state = get_io_state_for_step(*phase_counter, seq_len);
    hardware_apply_motor_step(motor_io_state);

    // Advance and wrap the phase counter for the next step.
    (*phase_counter)++;
    if (*phase_counter > seq_len) { *phase_counter = 1; }
}

/**
 * @brief Handles the logic for the "ramp-up" phase at the start of a print job.
 * @details This phase uses the pre-calculated S-curve LUT to smoothly accelerate the motor.
 * @param remaining_steps The number of steps left in the acceleration ramp.
 * @param seq_len The total sequence length for one line of printing.
 */
static inline void isr_handle_ramp_up_phase(int remaining_steps, uint16_t seq_len)
{
    if (g_accel_ramp_steps > 0) {
        // Calculate the next step's index in the S-curve LUT.
        uint16_t ramp_step_index = g_accel_ramp_steps - remaining_steps;
        if (ramp_step_index < g_accel_ramp_steps) {
             // Update the timer's period for the *next* step.
             hw_set_motor_period(g_accel_profile_ticks[ramp_step_index]);
        }
    }

    // Check if the ramp (or single setup step) is complete.
    if (remaining_steps == 0) {
        // ACCELERATION COMPLETE.
        // 1. Set the motor timer to the PRE-CALCULATED constant printing speed.
        hw_set_motor_period(g_const_motor_period_ticks);

        // 2. Enable strobe drivers and start the safety watchdog timer.
        hardware_strobes_enable();

        // 3. Reload the step counter for a full print line.
        atomic_store(g_step_counter_ptr, seq_len);

        // 4. Transition the state machine to the main printing state.
        atomic_store(g_state_ptr, PE_STATE_PRINTING);
    }
}

/**
 * @brief Handles the logic for the main printing and finishing phases.
 * @details This function is the heart of the printing data pipeline. It has been
 *          optimized to remove expensive division and modulo operations from the
 *          strobe-firing logic, replacing them with a simple, pre-calculated counter
 *          for maximum performance inside this time-critical ISR.
 *
 * --- DATA PIPELINE LIFECYCLE ---
 * The sequence of events for printing one line (Line N) is as follows:
 *
 * 1.  [MAIN LOOP]: The main loop sees `STATUS_PIPELINE_SLOT_FREE` is set.
 * 2.  [MAIN LOOP]: It starts a DMA transfer of data for the *next* line (N+1).
 * 3.  [DMA ISR -> `hw_spi_tx_complete_callback`]: Sets `STATUS_NEXT_LINE_DATA_READY`.
 *     The data for N+1 is now in the printhead's input shift registers.
 * 4.  [THIS ISR]: On the very first microstep of printing Line N, this function
 *     fires a `hardware_latch_pulse()`.
 * 5.  [THIS ISR]: The latch pulse transfers the data for N+1 to the output drivers.
 * 6.  [THIS ISR]: It clears `STATUS_NEXT_LINE_DATA_READY` and sets
 *     `STATUS_PIPELINE_SLOT_FREE`, signaling the main loop to send Line N+2.
 *
 * This overlapping process maximizes throughput. A data underrun occurs if
 * step 3 has not completed by the time step 4 is executed.
 *
 * @param remaining_steps The number of steps left in the current print line.
 * @param seq_len The total sequence length.
 */
static inline void isr_handle_printing_phase(int remaining_steps, uint16_t seq_len)
{
    // Calculate a forward index (1 to seq_len) for easier logic.
    int forward_index = seq_len - remaining_steps;

    // --- Latch Pulse Logic ---
    // At the very beginning of the line (first step), we latch the data for this line.
    if (forward_index == 1) {
        // At the start of a new line, reset the strobe step counter.
        g_strobe_step_counter = 0;

        // Check if the SPI transfer for this line's data has completed.
        if (atomic_load(g_status_ptr) & STATUS_NEXT_LINE_DATA_READY) {

            // --- INCREMENT LINE COUNTER HERE ---
            // A line is considered "printed" the moment we successfully latch its data
            // and prepare to fire the strobes. This is the most accurate place to count it.
            config_manager_increment_lines_printed(1);

            // Yes. Fire the latch pulse to move data from shift registers to output drivers.
            hardware_latch_pulse();
            // Clear the "data ready" flag and set the "pipeline free" flag so the main loop can send the next line.
            atomic_fetch_and(g_status_ptr, ~STATUS_NEXT_LINE_DATA_READY);
            atomic_fetch_or(g_status_ptr, STATUS_PIPELINE_SLOT_FREE);
        } else {
           // DATA UNDERRUN: The data was not ready in time. This is a job-ending fault.

            // 1. DO NOT stop the hardware here. We want a graceful ramp-down.
            // 2. Set the status flag so the host and LED know WHY the job ended.
            atomic_fetch_or(g_status_ptr, STATUS_DATA_UNDERRUN);

            // 3. CRITICAL: Transition the state machine to the graceful shutdown state.
            //    This will cause the print engine to finish the current line, ramp down the
            //    motor, and clean up, just as if it had finished the job normally.
            atomic_store(g_state_ptr, PE_STATE_FINISHING);
            return;
        }
    }

    // --- Strobe Firing Logic ---
    if (g_strobe_step_counter == 0) {
        IO_State motor_io_state = get_io_state_for_step(g_motor_phase_counter - 1, seq_len);

        // Start the one-shot strobe timer (TIM5) for the pre-calculated pulse duration.
        __HAL_TIM_DISABLE(&htim5);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, g_active_pulse_ticks);
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
        __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);
        __HAL_TIM_ENABLE(&htim5);

        // Only physically turn on the strobes if darkness is non-zero.
        if (g_active_pulse_ticks > 0) {
            hardware_apply_strobe_state(motor_io_state);
        } else {
            // If darkness is zero, we must manually call the strobe-off ISR to ensure
            // any logic inside it is still processed correctly.
            print_engine_hw_strobe_off_isr();
        }
    }

    // Increment the counter for the next microstep.
    g_strobe_step_counter++;
    if (g_strobe_step_counter >= g_strobe_group_size_steps) {
        // When the counter reaches the group size, reset it for the next group.
        g_strobe_step_counter = 0;
    }

    // --- End of Line / End of Job Logic ---
    if (remaining_steps == 0) {
        // Check if this was the final line of the entire job.
        if (atomic_load(g_state_ptr) == PE_STATE_FINISHING) {
            // LAST LINE COMPLETE. Now decide whether to ramp down or stop abruptly.
            if (g_accel_ramp_steps > 0) {
                // Transition to the dedicated RAMP_DOWN state.
                atomic_store(g_state_ptr, PE_STATE_RAMP_DOWN);
                // Load the counter with the number of steps required for the deceleration ramp.
                atomic_store(g_step_counter_ptr, g_accel_ramp_steps);
            } else {
                // No ramp profile exists, so stop everything immediately.
                print_engine_hw_stop_all();
                atomic_store(g_state_ptr, PE_STATE_JOB_DONE);
            }
        } else {
            // Normal end-of-line. Reload the step counter for the next line.
            atomic_store(g_step_counter_ptr, seq_len);
        }
    }
}

// --- Low-Level Static Helpers ---

/**
 * @brief Sets the motor timer's auto-reload register (period).
 * @param period_ticks The value to set in the timer's ARR.
 */
static inline void hw_set_motor_period(uint32_t period_ticks)
{
    // This inline function provides a single, clear point of control for the timer period.
    __HAL_TIM_SET_AUTORELOAD(&htim3, period_ticks);
}

/**
 * @brief Selects the correct microstepping Look-Up Table (LUT) based on resolution.
 * @param resolution The desired step resolution enum.
 */
static void update_lut_pointer(uint8_t resolution)
{
    switch (resolution)
    {
        case RESOLUTION_FULL_STEP:    g_current_io_state_lut = io_state_lut_full;     break;
        case RESOLUTION_HALF_STEP:    g_current_io_state_lut = io_state_lut_half;     break;
        case RESOLUTION_QUARTER_STEP: g_current_io_state_lut = io_state_lut_quarter;  break;
        case RESOLUTION_EIGHTH_STEP:  g_current_io_state_lut = io_state_lut_eighth;   break;
        default:                      g_current_io_state_lut = io_state_lut_quarter;  break; // Safe default
    }
}

/**
 * @brief Retrieves the motor GPIO state for a given step index from the active LUT.
 * @param step_index The 1-based index of the step in the sequence.
 * @param seq_len The total length of the sequence.
 * @return An IO_State struct with the required pin states.
 */
static IO_State get_io_state_for_step(int step_index, uint16_t seq_len)
{
    // Bounds checking for safety. Return a "motor off" state on error.
    if (step_index < 1 || step_index > seq_len || !g_current_io_state_lut) {
        return (IO_State){0, 0};
    }
    // The LUT is 0-indexed, so we subtract 1 from the 1-based step_index.
    return g_current_io_state_lut[step_index - 1];
}

/**
 * @brief Calculates all timer parameters and applies the motor period.
 * @param settings The job settings to use for the calculation.
 * @return The calculated strobe pulse duration in timer ticks.
 */
static uint32_t calculate_and_apply_timer_settings(const print_job_settings_t* settings)
{
    timer_params_t params = calculate_all_timer_params(settings);
    hw_set_motor_period(params.motor_period_ticks);
    return params.strobe_pulse_ticks;
}
