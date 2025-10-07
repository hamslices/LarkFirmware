# Lark Print Engine Firmware

**Version:** 01.04  
**Author:** HamSlices  
**License:** GNU General Public License v3.0

This repository contains the complete firmware for the Lark Print Engine, a sophisticated, high-performance thermal printing system built on the STM32H7 microcontroller platform.

## Key Features

*   **High-Performance Printing:** A non-blocking, interrupt-driven architecture ensures maximum print speed and data throughput without dropping lines.
*   **Advanced Motor Control:** Implements S-Curve acceleration and deceleration for smooth, precise, and quiet paper movement, reducing mechanical stress.
*   **Robust Communication:** A custom, packet-based protocol over USB CDC (Virtual COM Port) for reliable command and data transmission, featuring a state machine parser with timeout and error handling.
*   **Persistent Storage with Wear Leveling:** User settings and system counters (e.g., lines printed, power-on cycles) are saved to dedicated flash sectors using a wear-leveling algorithm to maximize the lifespan of the non-volatile memory.
*   **Comprehensive Fault Handling:**
    *   **Fatal Error Indicator:** In the event of a hard fault or critical HAL error, the system enters a locked state and blinks "S.O.S." in Morse code on the status LED for clear visual diagnostics.
    *   **Job Fault Latching:** Operational faults like paper-out or thermal warnings are latched, ensuring they are not missed by the host application.
    *   **Fault Logging:** A ring buffer logs the last 25 fault events with timestamps and system status, which can be retrieved by the host for debugging.
*   **Hardware Abstraction:** A clean hardware abstraction layer allows the firmware to be compiled for different hardware targets (`TARGET_DEV_BOARD` vs. `TARGET_PRODUCTION`) by changing a single preprocessor definition.
*   **Dynamic Status Indicator:** A multi-color RGB LED provides clear, at-a-glance feedback on the printer's status, including idle, busy, printing, fault, and warning conditions.
*   **On-the-Fly Configuration:** All major print parameters (speed, darkness, motor current, step resolution) can be configured and saved to flash via the communication protocol.
*   **Built-in Self-Test:** A complete, high-resolution test image is embedded in the firmware and can be printed at any time via a button press or a software command.

## System Architecture

The firmware is built on a bare-metal, non-blocking, cooperative multitasking model. The main loop is divided into two parts:

1.  **High-Priority Fast Loop:** Executes on every iteration to manage the critical data pipeline between the USB host and the print engine. This ensures the data buffers are always serviced with minimal latency.
2.  **Scheduled Housekeeping Loop:** A simple scheduler runs lower-priority tasks (polling buttons, reading hardware status pins, updating the LED) at fixed intervals (e.g., every 10-50ms).

Timing-critical operations, such as generating motor steps and printhead strobe pulses, are handled entirely by hardware timers and their associated Interrupt Service Routines (ISRs) to guarantee precise timing, independent of the main loop's workload.

### Core Modules

*   **`main.c`**: The application entry point. Initializes all peripherals and modules, and contains the main processing loop.
*   **`print_engine.c`**: The central state machine for the printer. Manages high-level states like `IDLE`, `PRINTING`, `BUFFERING`, `MANUAL_FEED`, and `FAULT`. It is the "brain" of the printing operation.
*   **`print_engine_hw.c`**: The low-level hardware control layer, driven entirely by ISRs. It is responsible for generating motor step pulses, firing printhead strobes, and managing the SPI DMA for line data.
*   **`protocol_handler.c`**: Implements the command and data protocol. It uses a parser (`cdc_parser.c`) to decode incoming USB data packets and a dispatch table to execute the corresponding command handlers.
*   **`config_manager.c`**: Manages all user settings and system counters. Handles the complex logic for reading from and writing to the dedicated flash sectors, including CRC validation and a wear-leveling algorithm that cycles through multiple storage "slots".
*   **`led_manager.c`**: Controls the RGB status LED. It interprets the global system status flags and translates them into specific colors and blinking patterns (e.g., Solid Green for Idle, Blinking Red for Fatal Error).
*   **`button_handler.c`**: A simple state machine to debounce and handle short presses vs. long holds for the dual-action "Self-Test / Clear Faults" button.
*   **`fault_logger.c`**: A simple ring buffer implementation to store a history of fault events for later diagnostics.
*   **`error_reporter.c`**: A robust, self-contained module for handling fatal system errors. It takes over the CPU in case of a hard fault, initializes its own GPIOs, and uses a blocking delay loop to blink an S.O.S. pattern, ensuring it can function even if SysTick and other peripherals have failed.
*   **`hardware.c`**: The Hardware Abstraction Layer (HAL) that provides a consistent API for interacting with the hardware, regardless of the target board. Contains implementations for both the development and production boards.

## Communication Protocol

The printer communicates over a USB CDC (Virtual COM Port) interface using a custom binary packet format.

**Packet Structure:**
| Field        | Size (bytes) | Description                                  |
|--------------|--------------|----------------------------------------------|
| SOP1         | 1            | Start of Packet 1: `0xDE`                    |
| SOP2         | 1            | Start of Packet 2: `0xAD`                    |
| Command      | 1            | The command ID byte.                         |
| Length (LSB) | 1            | The length of the payload (Little Endian).   |
| Length (MSB) | 1            | The length of the payload (Little Endian).   |
| Payload      | 0-N          | The command-specific payload data.           |

### Host-to-Device Commands

| Command Name                | ID   | Payload Length | Payload Description / Action                                     |
|-----------------------------|------|----------------|------------------------------------------------------------------|
| `CMD_IMAGE_DATA`            | 0x01 | 216            | A single line of print data.                                     |
| `CMD_RESET`                 | 0x02 | 0              | Performs a software reset of the microcontroller.                |
| `CMD_BOOTLOADER`            | 0x03 | 0              | Jumps to the built-in ST DFU bootloader.                         |
| `CMD_PURGE`                 | 0x04 | 0              | Aborts the current job and clears all data buffers.              |
| `CMD_CLEAR_FAULTS`          | 0x05 | 4              | `(uint32_t) flags_to_clear`. Clears specific latched fault flags. |
| `CMD_SET_CONFIG`            | 0x06 | 5              | `(uint8_t) index, (uint32_t) value`. Sets a configuration parameter.|
| `CMD_GET_CONFIG`            | 0x07 | 1              | `(uint8_t) index`. Requests the value of a configuration parameter.|
| `CMD_GET_SYSTEM`            | 0x08 | 1              | `(uint8_t) sub_command`. Requests a system value (e.g., status, version).|
| `CMD_SELF_TEST`             | 0x09 | 0              | Initiates a full self-test print using the embedded image.       |
| `CMD_LOAD_FLASH`            | 0x0A | 0              | Loads the last saved settings from flash into the active configuration. |
| `CMD_SAVE_FLASH`            | 0x0B | 0              | Saves the current active configuration to flash.                 |
| `CMD_ERASE_FLASH`           | 0x0C | 0              | Erases all settings and counters, restoring factory defaults.    |
| `CMD_MOVE_MOTOR`            | 0x0D | 4              | `(int32_t) lines`. Moves the motor a specified number of lines.  |
| `CMD_GET_FAULT_LOG_INFO`    | 0x0E | 0              | Requests the number of valid entries in the fault log.           |
| `CMD_GET_FAULT_LOG_ENTRY`   | 0x0F | 1              | `(uint8_t) index`. Requests a specific entry from the fault log. |
| `CMD_GET_SYSTEM_COUNTERS`   | 0x10 | 0              | Requests the entire block of system diagnostic counters.         |
| `CMD_GET_UID`               | 0x11 | 0              | Requests the unique 96-bit device ID of the microcontroller.     |

## Flash Memory Layout

The firmware utilizes both Bank 1 and Bank 2 of the STM32H7's internal flash memory. Bank 1 is reserved for the application code, while Bank 2 is partitioned for data storage.

| Section                | Location       | Size       | Description                                              |
|------------------------|----------------|------------|----------------------------------------------------------|
| Application Firmware   | Bank 1         | ~73 KB     | The main executable code.                                |
| Self-Test Image        | Bank 2, S 4-5  | 163.5 KB   | The embedded binary image for the self-test print.       |
| Statistics Counters    | Bank 2, S 6    | 128 KB     | Wear-leveled storage for system counters (e.g., power-ons). |
| User Configuration     | Bank 2, S 7    | 128 KB     | Wear-leveled storage for user settings (e.g., speed, darkness). |

## Building the Firmware

This project is configured to be built with the STM32CubeIDE environment.

**Dependencies:**
*   ARM GCC Toolchain
*   STM32CubeIDE or compatible Eclipse-based IDE
*   ST-LINK/V2 or V3 for programming and debugging

**Build Targets:**
*   **Debug:** Compiles with debug symbols and no optimization (`-O0`).
*   **Release:** Compiles with full optimization (`-O3`) for production use.
*   **Flash:** Identical to the Release build, but runs a post-build script to generate a combined binary for deployment.

---

*Lark Print Engine © HamSlices 2025*