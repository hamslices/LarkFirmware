/*
 * config_manager.c
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#include <string.h>
#include <stddef.h> // Required for offsetof
#include "crc.h"
#include "config_manager.h"
#include "hardware.h"
#include "hardware_definitions.h"

/* --- Private Definitions --- */

#define CONFIG_MAGIC_NUMBER   0xB01DFACE
#define COUNTERS_MAGIC_NUMBER 0xC0FFEEBE

// Hardcoded factory default values.
#define DEFAULT_LINES_PER_SECOND    (4.0f * DPI)
#define DEFAULT_DARKNESS            80.0f
#define DEFAULT_MOTOR_CURRENT_MA    800
#define DEFAULT_DIRECTION           DIRECTION_FORWARD
#define DEFAULT_RESOLUTION          RESOLUTION_QUARTER_STEP

// --- Safety Limits ---
#define MIN_LINES_PER_SECOND    (1.0f * DPI)
#define MAX_LINES_PER_SECOND    (8.0f * DPI)
#define MIN_DARKNESS            0.0f
#define MAX_DARKNESS            100.0f
#define MIN_MOTOR_CURRENT_MA    300
#define MAX_MOTOR_CURRENT_MA    1000

// --- Flash Storage Logic Constants ---
#define SEQUENCE_NUMBER_ROLLOVER_THRESHOLD (0x80000000)

/**
 * @brief Defines the function signature for a "clamping" function.
 */
typedef void (*param_clamp_func_t)(uint32_t* value);


/* --- Linker Script Symbols --- */
extern const uint8_t _config_start_address[];
extern const uint8_t _config_end_address[];
extern const uint8_t _counters_start_address[];
extern const uint8_t _counters_end_address[];


/* --- Module-Private (Static) Variables --- */

static uint32_t config_table[CONFIG_PARAM_COUNT];
static SystemCounters_t g_system_counters;
static uint32_t g_motor_step_accumulator   = 0;
static bool g_config_initialized           = false;
static uint32_t g_flash_error_code         = 0;

__attribute__((aligned(32), section(".cacheless_buffers")))
static persistent_settings_t g_settings_write_buffer;

__attribute__((aligned(32), section(".cacheless_buffers")))
static persistent_counters_t g_counters_write_buffer;

/* --- Forward Declarations for Private Functions --- */

// Core NVM logic - User Settings
static void load_defaults(void);
static bool erase_config_sector(void);
static int32_t find_latest_config_slot(void);
static uint32_t get_next_config_sequence_num(void);
static bool is_config_slot_valid(const persistent_settings_t* slot_ptr);
static uint32_t calculate_config_struct_crc(const persistent_settings_t* settings_ptr);

// Core NVM logic - System Counters
static bool config_manager_load_counters_from_flash(void);
static bool erase_counters_sector(void);
static int32_t find_latest_counters_slot(void);
static uint32_t get_next_counters_sequence_num(void);
static bool is_counters_slot_valid(const persistent_counters_t* slot_ptr);
static uint32_t calculate_counters_struct_crc(const persistent_counters_t* counters_ptr);

// Common helper
static inline void safe_unaligned_memcpy(void* dest, const void* src, size_t n);
static bool program_flash(uint32_t target_address, const uint32_t* source_ptr, size_t num_flash_words);

// Parameter clamping functions
static void clamp_lines_per_second(uint32_t* value);
static void clamp_darkness(uint32_t* value);
static void clamp_motor_current(uint32_t* value);
static void clamp_step_resolution(uint32_t* value);
static void clamp_direction(uint32_t* value);
static void clamp_none(uint32_t* value);

/* --- Parameter Behavior Table --- */

/**
 * @brief Maps each parameter enum to its specific clamping function.
 */
static const param_clamp_func_t param_clampers[CONFIG_PARAM_COUNT] = {
    [CONFIG_LINES_PER_SECOND] = clamp_lines_per_second,
    [CONFIG_DARKNESS]         = clamp_darkness,
    [CONFIG_MOTOR_CURRENT]    = clamp_motor_current,
    [CONFIG_STEP_RESOLUTION]  = clamp_step_resolution,
    [CONFIG_DIRECTION]        = clamp_direction,
    [CONFIG_DEBUGING]         = clamp_none,
};

/*****************************************************************************
 * Public Function Implementations
 ****************************************************************************/

void config_manager_init(void)
{
    uint32_t config_sector_size = (uint32_t)_config_end_address - (uint32_t)_config_start_address;
    uint32_t counters_sector_size = (uint32_t)_counters_end_address - (uint32_t)_counters_start_address;
    SCB_InvalidateDCache_by_Addr((uint32_t*)_config_start_address, config_sector_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)_counters_start_address, counters_sector_size);

    if (!config_manager_load_from_flash()) {
        load_defaults();
    }

    if (!config_manager_load_counters_from_flash()) {
        memset(&g_system_counters, 0, sizeof(SystemCounters_t));
    }

    g_system_counters.power_on_cycles++;
    g_config_initialized = true;
}

bool config_manager_load_from_flash(void)
{
    int32_t latest_slot = find_latest_config_slot();
    if (latest_slot == -1) {
        return false; // No valid configuration found.
    }

    const persistent_settings_t* settings_in_flash = (const persistent_settings_t*)(_config_start_address + (latest_slot * sizeof(persistent_settings_t)));

    // Use safe, byte-by-byte copy to prevent HardFaults from unaligned reads from flash.
    safe_unaligned_memcpy(config_table, &settings_in_flash->config_table, sizeof(config_table));

    return true;
}

bool config_manager_save_to_flash(void)
{
    if (!g_config_initialized || print_engine_is_busy()) {
        return false;
    }

    g_system_counters.user_settings_saves++;

    int32_t latest_slot = find_latest_config_slot();
    int32_t next_slot = (latest_slot == -1) ? 0 : (latest_slot + 1);
    uint32_t num_slots = ((uint32_t)_config_end_address - (uint32_t)_config_start_address) / sizeof(persistent_settings_t);

    if (next_slot >= num_slots) {
        if (!erase_config_sector()) {
            return false;
        }
        next_slot = 0;
    }

    g_settings_write_buffer.magic_number = CONFIG_MAGIC_NUMBER;
    g_settings_write_buffer.sequence_number = get_next_config_sequence_num();
    memcpy(g_settings_write_buffer.config_table, config_table, sizeof(config_table));
    g_settings_write_buffer.crc32 = calculate_config_struct_crc(&g_settings_write_buffer);

    uint32_t target_address = (uint32_t)_config_start_address + (next_slot * sizeof(persistent_settings_t));
    size_t num_flash_words = sizeof(persistent_settings_t) / 32;
    bool success;

    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    success = program_flash(target_address, (const uint32_t*)&g_settings_write_buffer, num_flash_words);

    HAL_FLASH_Lock();
    __enable_irq();  // Re-enable interrupts only after everything is locked and finished

    if (success) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)target_address, sizeof(persistent_settings_t));
    } else {
    	g_system_counters.user_settings_saves--; //something broke, reset counter.
    }

    return success;
}

bool config_manager_save_counters_to_flash(void)
{
    if (!g_config_initialized || print_engine_is_busy()) {
        return false;
    }
    g_system_counters.counter_snapshots++;

    int32_t latest_slot = find_latest_counters_slot();
    int32_t next_slot = (latest_slot == -1) ? 0 : (latest_slot + 1);
    uint32_t num_slots = ((uint32_t)_counters_end_address - (uint32_t)_counters_start_address) / sizeof(persistent_counters_t);

    if (next_slot >= num_slots) {
        if (!erase_counters_sector()) {
            return false;
        }
        next_slot = 0;
    }

    g_counters_write_buffer.magic_number = COUNTERS_MAGIC_NUMBER;
    g_counters_write_buffer.sequence_number = get_next_counters_sequence_num();
    memcpy(&g_counters_write_buffer.system_counters, &g_system_counters, sizeof(SystemCounters_t));
    g_counters_write_buffer.crc32 = calculate_counters_struct_crc(&g_counters_write_buffer);

    uint32_t target_address = (uint32_t)_counters_start_address + (next_slot * sizeof(persistent_counters_t));
    size_t num_flash_words = sizeof(persistent_counters_t) / 32;
    bool success;

    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    success = program_flash(target_address, (const uint32_t*)&g_counters_write_buffer, num_flash_words);

    HAL_FLASH_Lock();
    __enable_irq();  // Re-enable interrupts only after everything is locked and finished

    if (success) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)target_address, sizeof(persistent_counters_t));
    } else {
    	g_system_counters.counter_snapshots--;
    }

    return success;
}

bool config_manager_erase_from_flash(void)
{
    bool config_erased = erase_config_sector();
    bool counters_erased = erase_counters_sector();

    if (config_erased && counters_erased) {
        load_defaults();
        memset(&g_system_counters, 0, sizeof(SystemCounters_t));
        return true;
    }
    return false;
}

void config_manager_set(config_param_t index, uint32_t value)
{
    if (index >= CONFIG_PARAM_COUNT) { return; }
    config_table[index] = value;
    param_clampers[index](&config_table[index]);
}

uint32_t config_manager_get(config_param_t index)
{
    if (index >= CONFIG_PARAM_COUNT) { return 0; }
    return config_table[index];
}

void config_manager_get_current_settings(print_job_settings_t* settings)
{
    if (!settings) return;
    uint32_t lps_bits = config_manager_get(CONFIG_LINES_PER_SECOND);
    uint32_t darkness_bits = config_manager_get(CONFIG_DARKNESS);
    memcpy(&settings->lines_per_second, &lps_bits, sizeof(float));
    memcpy(&settings->darkness, &darkness_bits, sizeof(float));
    settings->step_resolution = config_manager_get(CONFIG_STEP_RESOLUTION);
    settings->direction = config_manager_get(CONFIG_DIRECTION);
    settings->motor_current_ma = config_manager_get(CONFIG_MOTOR_CURRENT);
    settings->sequence_length = 0;
    settings->lut_pointer = NULL;
}

/* --- System Counter Management --- */

void config_manager_increment_lines_printed(uint32_t amount) {
    g_system_counters.lines_printed += amount;
}

void config_manager_increment_motor_steps(uint32_t microsteps_per_full_step)
{
    g_motor_step_accumulator++;
    if (g_motor_step_accumulator >= microsteps_per_full_step) {
        g_system_counters.motor_steps_moved++;
        g_motor_step_accumulator = 0;
    }
}

void config_manager_increment_print_jobs_started(void) {
    g_system_counters.print_jobs_started++;
}

const SystemCounters_t* config_manager_get_counters(void) {
    return &g_system_counters;
}

/*****************************************************************************
 * Private Function Implementations
 ****************************************************************************/

/* --- Common NVM Logic --- */

/**
 * @brief  Safely copies memory from a potentially unaligned source.
 * @details This function prevents HardFaults on ARM Cortex-M cores by reading
 *          the source data one byte at a time, avoiding multi-byte access
 *          instructions that require address alignment.
 * @param  dest Pointer to the destination buffer (in RAM).
 * @param  src Pointer to the source data (e.g., in Flash, can be unaligned).
 * @param  n Number of bytes to copy.
 */
static inline void safe_unaligned_memcpy(void* dest, const void* src, size_t n)
{
    char* d = dest;
    const char* s = src;
    for (size_t i = 0; i < n; ++i) {
        d[i] = s[i];
    }
}

/**
 * @brief  Programs a block of flash memory from a RAM buffer.
 * @details Executes from Flash (Bank 1) to write to a different bank (Bank 2),
 *          avoiding any read-while-write conflicts. The calling function is
 *          expected to handle the overall critical section (interrupt disable).
 * @param  target_address The starting address in Flash to write to. Must be 32-byte aligned.
 * @param  source_ptr Pointer to the source data buffer in RAM.
 * @param  num_flash_words The number of 32-byte flash words to program.
 * @return `true` on success, `false` on failure.
 */
static bool program_flash(uint32_t target_address, const uint32_t* source_ptr, size_t num_flash_words)
{
    for (size_t i = 0; i < num_flash_words; ++i) {
        // A flash word is 32 bytes, which is 8 words of uint32_t.
        // We calculate the source address for the start of the current 32-byte chunk.
        uint32_t current_source_address = (uint32_t)&source_ptr[i * 8];

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, target_address, current_source_address) != HAL_OK) {
            g_flash_error_code = HAL_FLASH_GetError();
            return false; // Exit immediately on failure
        }

        target_address += 32;
    }

    return true;
}

/* --- User Settings NVM Logic --- */

/**
 * @brief  Calculates the CRC32 checksum for a settings data structure.
 * @details The calculation covers all data from the beginning of the struct
 *          up to (but not including) the crc32 member itself. This function
 *          assumes the source pointer is to an aligned buffer in RAM.
 * @param  settings_ptr Pointer to the settings struct to be hashed.
 * @return The calculated 32-bit CRC value.
 */
static uint32_t calculate_config_struct_crc(const persistent_settings_t* settings_ptr)
{
    const size_t data_to_hash_size_bytes = offsetof(persistent_settings_t, crc32);
    // The HAL_CRC_Calculate function's size parameter must be in 32-bit words.
    const uint32_t data_to_hash_size_words = data_to_hash_size_bytes / 4;

    __HAL_CRC_DR_RESET(&hcrc);

    // No intermediate copy is needed as the source is always an aligned RAM buffer.
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)settings_ptr, data_to_hash_size_words);
}

/**
 * @brief  Checks if a single settings slot in flash is valid.
 * @details Performs a full validation by:
 *          1. Invalidating the D-Cache for the flash address.
 *          2. Safely copying the data to an aligned RAM buffer.
 *          3. Checking the magic number.
 *          4. Calculating the CRC and comparing it to the stored value.
 * @param  slot_ptr A direct pointer to the start of the slot in flash memory.
 * @return `true` if the slot is valid, `false` otherwise.
 */
static bool is_config_slot_valid(const persistent_settings_t* slot_ptr)
{
    persistent_settings_t local_settings;

	SCB_InvalidateDCache_by_Addr((uint32_t*)slot_ptr, sizeof(persistent_settings_t));
    safe_unaligned_memcpy(&local_settings, slot_ptr, sizeof(persistent_settings_t));

    if (local_settings.magic_number != CONFIG_MAGIC_NUMBER) {
        return false;
    }

    return (calculate_config_struct_crc(&local_settings) == local_settings.crc32);
}

/**
 * @brief  Finds the most recent valid settings slot in the flash sector.
 * @details Iterates through all available slots, validates each one, and tracks
 *          the one with the highest sequence number. It correctly handles the
 *          rollover of the 32-bit sequence number.
 * @return The index (0-based) of the latest valid slot, or -1 if no valid slots are found.
 */
static int32_t find_latest_config_slot(void)
{
    uint32_t latest_seq_num = 0;
    int32_t latest_slot = -1;
    bool is_first_valid_slot = true;
    uint32_t num_slots = ((uint32_t)_config_end_address - (uint32_t)_config_start_address) / sizeof(persistent_settings_t);

    for (uint32_t i = 0; i < num_slots; ++i) {
        const persistent_settings_t* slot_ptr = (const persistent_settings_t*)(_config_start_address + (i * sizeof(persistent_settings_t)));

        if (is_config_slot_valid(slot_ptr)) {
            uint32_t current_seq_num;
            // Must perform a safe read from the raw flash pointer.
            safe_unaligned_memcpy(&current_seq_num, &slot_ptr->sequence_number, sizeof(uint32_t));

            if (is_first_valid_slot) {
                latest_seq_num = current_seq_num;
                latest_slot = i;
                is_first_valid_slot = false;
            } else {
                // Check for normal increment or a rollover event.
                if ((current_seq_num > latest_seq_num && (current_seq_num - latest_seq_num) < SEQUENCE_NUMBER_ROLLOVER_THRESHOLD ) ||
                    (current_seq_num < latest_seq_num && (latest_seq_num - current_seq_num) > SEQUENCE_NUMBER_ROLLOVER_THRESHOLD ))
                {
                    latest_seq_num = current_seq_num;
                    latest_slot = i;
                }
            }
        }
    }
    return latest_slot;
}

/**
 * @brief  Gets the next sequence number to use for a settings save operation.
 * @details Finds the latest valid slot and returns its sequence number plus one.
 *          If no valid slots exist, it returns 1.
 * @return The next sequence number.
 */
static uint32_t get_next_config_sequence_num(void)
{
    int32_t latest_slot = find_latest_config_slot();
    if (latest_slot == -1) {
        return 1;
    }
    const persistent_settings_t* latest_settings = (const persistent_settings_t*)(_config_start_address + (latest_slot * sizeof(persistent_settings_t)));

    // Perform a safe read from the raw flash pointer to get the sequence number.
    uint32_t sequence_number;
    safe_unaligned_memcpy(&sequence_number, &latest_settings->sequence_number, sizeof(uint32_t));
    return sequence_number + 1;
}

/**
 * @brief  Erases the entire flash sector reserved for user settings.
 * @details This is a blocking operation. It will not proceed if the print
 *          engine is active. After erasing, the cache for the sector is invalidated.
 * @return `true` on success, `false` on failure.
 */
static bool erase_config_sector(void)
{
    if (print_engine_is_busy()) {
        return false;
    }

    __disable_irq();
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_SECTORS,
        .Banks       = FLASH_BANK_2,
        .Sector      = FLASH_SECTOR_7,
        .NbSectors   = 1,
        .VoltageRange= FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_error = 0;
    bool success = (HAL_FLASHEx_Erase(&erase_init, &sector_error) == HAL_OK);
    HAL_FLASH_Lock();
    __enable_irq();

    if (success) {
        uint32_t config_sector_size = (uint32_t)_config_end_address - (uint32_t)_config_start_address;
        SCB_InvalidateDCache_by_Addr((uint32_t*)_config_start_address, config_sector_size);
    }
    return success;
}

/* --- System Counters NVM Logic --- */

/**
 * @brief  Loads the latest system counters from flash into the global RAM variable.
 * @details Finds the latest valid counters slot in the counters sector and copies
 *          the data into the `g_system_counters` global variable.
 * @return `true` if valid counters were found and loaded, `false` otherwise.
 */
static bool config_manager_load_counters_from_flash(void)
{
    int32_t latest_slot = find_latest_counters_slot();
    if (latest_slot == -1) {
        return false; // No valid counters found.
    }

    const persistent_counters_t* counters_in_flash = (const persistent_counters_t*)(_counters_start_address + (latest_slot * sizeof(persistent_counters_t)));

    // Safely copy just the system_counters portion from the flash structure.
    safe_unaligned_memcpy(&g_system_counters, &counters_in_flash->system_counters, sizeof(SystemCounters_t));

    return true;
}

/**
 * @brief  Calculates the CRC32 checksum for a counters data structure.
 * @details The calculation covers all data from the beginning of the struct
 *          up to (but not including) the crc32 member itself. This function
 *          assumes the source pointer is to an aligned buffer in RAM.
 * @param  counters_ptr Pointer to the counters struct to be hashed.
 * @return The calculated 32-bit CRC value.
 */
static uint32_t calculate_counters_struct_crc(const persistent_counters_t* counters_ptr)
{
    const size_t data_to_hash_size_bytes = offsetof(persistent_counters_t, crc32);
    // The HAL_CRC_Calculate function's size parameter must be in 32-bit words.
    const uint32_t data_to_hash_size_words = data_to_hash_size_bytes / 4;

    __HAL_CRC_DR_RESET(&hcrc);

    // No intermediate copy is needed as the source is always an aligned RAM buffer.
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)counters_ptr, data_to_hash_size_words);
}

/**
 * @brief  Checks if a single counters slot in flash is valid.
 * @details Performs a full validation by checking the magic number and CRC.
 *          (See `is_config_slot_valid` for more details).
 * @param  slot_ptr A direct pointer to the start of the slot in flash memory.
 * @return `true` if the slot is valid, `false` otherwise.
 */
static bool is_counters_slot_valid(const persistent_counters_t* slot_ptr)
{
    persistent_counters_t local_counters;

	SCB_InvalidateDCache_by_Addr((uint32_t*)slot_ptr, sizeof(persistent_counters_t));
    safe_unaligned_memcpy(&local_counters, slot_ptr, sizeof(persistent_counters_t));

    if (local_counters.magic_number != COUNTERS_MAGIC_NUMBER) {
        return false;
    }

    return (calculate_counters_struct_crc(&local_counters) == local_counters.crc32);
}

/**
 * @brief  Finds the most recent valid counters slot in the flash sector.
 * @details Iterates through all available slots to find the one with the
 *          highest valid sequence number. Handles rollover.
 * @return The index (0-based) of the latest valid slot, or -1 if none are found.
 */
static int32_t find_latest_counters_slot(void)
{
    uint32_t latest_seq_num = 0;
    int32_t latest_slot = -1;
    bool is_first_valid_slot = true;
    uint32_t num_slots = ((uint32_t)_counters_end_address - (uint32_t)_counters_start_address) / sizeof(persistent_counters_t);

    for (uint32_t i = 0; i < num_slots; ++i) {
        const persistent_counters_t* slot_ptr = (const persistent_counters_t*)(_counters_start_address + (i * sizeof(persistent_counters_t)));

        if (is_counters_slot_valid(slot_ptr)) {
            uint32_t current_seq_num;
            safe_unaligned_memcpy(&current_seq_num, &slot_ptr->sequence_number, sizeof(uint32_t));

            if (is_first_valid_slot) {
                latest_seq_num = current_seq_num;
                latest_slot = i;
                is_first_valid_slot = false;
            } else {
                if ((current_seq_num > latest_seq_num && (current_seq_num - latest_seq_num) < SEQUENCE_NUMBER_ROLLOVER_THRESHOLD ) ||
                    (current_seq_num < latest_seq_num && (latest_seq_num - current_seq_num) > SEQUENCE_NUMBER_ROLLOVER_THRESHOLD ))
                {
                    latest_seq_num = current_seq_num;
                    latest_slot = i;
                }
            }
        }
    }
    return latest_slot;
}

/**
 * @brief  Gets the next sequence number to use for a counters save operation.
 * @details Finds the latest valid slot and returns its sequence number plus one.
 *          If no valid slots exist, it returns 1.
 * @return The next sequence number.
 */
static uint32_t get_next_counters_sequence_num(void)
{
    int32_t latest_slot = find_latest_counters_slot();
    if (latest_slot == -1) {
        return 1;
    }
    const persistent_counters_t* latest_counters = (const persistent_counters_t*)(_counters_start_address + (latest_slot * sizeof(persistent_counters_t)));

    // Perform a safe read from the raw flash pointer.
    uint32_t sequence_number;
    safe_unaligned_memcpy(&sequence_number, &latest_counters->sequence_number, sizeof(uint32_t));
    return sequence_number + 1;
}

/**
 * @brief  Erases the entire flash sector reserved for system counters.
 * @details This is a blocking operation that will not proceed if the print
 *          engine is active. The cache is invalidated for the sector on success.
 * @return `true` on success, `false` on failure.
 */
static bool erase_counters_sector(void)
{
    if (print_engine_is_busy()) {
        return false;
    }

    __disable_irq();
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_SECTORS,
        .Banks       = FLASH_BANK_2,
        .Sector      = FLASH_SECTOR_6,
        .NbSectors   = 1,
        .VoltageRange= FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_error = 0;
    bool success = (HAL_FLASHEx_Erase(&erase_init, &sector_error) == HAL_OK);
    HAL_FLASH_Lock();
    __enable_irq();

    if (success) {
        uint32_t counters_sector_size = (uint32_t)_counters_end_address - (uint32_t)_counters_start_address;
        SCB_InvalidateDCache_by_Addr((uint32_t*)_counters_start_address, counters_sector_size);
    }
    return success;
}

/**
 * @brief  Loads the factory default settings into the live configuration table.
 * @details This function populates the `config_table` in RAM with the hardcoded
 *          default values defined at the top of the file.
 */
static void load_defaults(void)
{
    float initial_lps = DEFAULT_LINES_PER_SECOND;
    float initial_darkness = DEFAULT_DARKNESS;
    uint32_t lps_bits, darkness_bits;
    memcpy(&lps_bits, &initial_lps, sizeof(initial_lps));
    memcpy(&darkness_bits, &initial_darkness, sizeof(initial_darkness));
    config_manager_set(CONFIG_LINES_PER_SECOND, lps_bits);
    config_manager_set(CONFIG_DARKNESS, darkness_bits);
    config_manager_set(CONFIG_STEP_RESOLUTION, DEFAULT_RESOLUTION);
    config_manager_set(CONFIG_MOTOR_CURRENT, DEFAULT_MOTOR_CURRENT_MA);
    config_manager_set(CONFIG_DIRECTION, DEFAULT_DIRECTION);
    config_manager_set(CONFIG_DEBUGING, 0);
}

/* --- Parameter Clamping Functions --- */

static void clamp_lines_per_second(uint32_t* value)
{
    float lps_val;
    memcpy(&lps_val, value, sizeof(lps_val));
    if (lps_val < MIN_LINES_PER_SECOND) lps_val = MIN_LINES_PER_SECOND;
    if (lps_val > MAX_LINES_PER_SECOND) lps_val = MAX_LINES_PER_SECOND;
    memcpy(value, &lps_val, sizeof(lps_val));
}

static void clamp_darkness(uint32_t* value)
{
    float darkness_val;
    memcpy(&darkness_val, value, sizeof(darkness_val));
    if (darkness_val < MIN_DARKNESS) darkness_val = MIN_DARKNESS;
    if (darkness_val > MAX_DARKNESS) darkness_val = MAX_DARKNESS;
    memcpy(value, &darkness_val, sizeof(darkness_val));
}

static void clamp_motor_current(uint32_t* value)
{
    if (*value < MIN_MOTOR_CURRENT_MA) *value = MIN_MOTOR_CURRENT_MA;
    if (*value > MAX_MOTOR_CURRENT_MA) *value = MAX_MOTOR_CURRENT_MA;
}

static void clamp_step_resolution(uint32_t* value)
{
    if (*value > RESOLUTION_EIGHTH_STEP) *value = RESOLUTION_EIGHTH_STEP;
    if (*value < RESOLUTION_FULL_STEP)   *value = RESOLUTION_FULL_STEP;
}

static void clamp_direction(uint32_t* value)
{
    if (*value > DIRECTION_REVERSE) *value = DIRECTION_REVERSE;
    if (*value < DIRECTION_FORWARD) *value = DIRECTION_FORWARD;
}

static void clamp_none(uint32_t* value)
{
    (void)value;
}
