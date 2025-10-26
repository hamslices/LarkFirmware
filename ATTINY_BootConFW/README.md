# Lark Power Management and Boot Controller

**Author:** HamSlices  
**License:** GNU General Public License v3.0

This repository contains the firmware for the Lark Power Management and Boot Controller, which runs on an ATTINY microcontroller. It serves as a simple and robust power and boot mode controller for the main Lark Print Engine (STM32).

![alt text](https://github.com/hamslices/LarkFirmware/blob/main/ATTINY_BootConFW/img/AD.png?raw=true)

## Key Features

*   **Dual Boot-Mode Control:** Allows the main STM32 processor to be started in either normal application mode or the DFU bootloader mode.
*   **Single-Button Interface:** A single pushbutton provides control for power on, power off, and bootloader mode selection.
*   **Safe Power Sequencing:** Ensures the main processor is powered on and its reset line is managed correctly for stable startups.
*   **Visual Feedback:** A debug LED provides clear status on boot mode selection and button presses.
*   **Hardware Debounce:** The firmware is designed to work with a hardware-debounced pushbutton for reliable input, as it does not perform debouncing in software.
*   **Minimal Footprint:** Optimized for the low-resource ATTINY platform, using only 11% of program space and 2% of dynamic memory.

## Functional Description

The controller's primary role is to manage the power-on and reset sequence for the main Lark board. It uses a single pushbutton to control all operations.

### Normal Operation (Power On / Power Off)

*   **To Power On:** A short press of the button when the system is off will initiate the power-on sequence. The firmware enables power to the main board, waits 300ms for the power to stabilize, and then releases the main processor from reset, allowing it to boot normally from its application firmware.
*   **To Power Off:** A short press of the button when the system is on will place the main processor back into reset and then remove power.

### Bootloader Mode

To force the main STM32 processor into its built-in DFU (Device Firmware Update) bootloader, a special procedure is required:

1.  With the system's main power off, **press and hold** the control pushbutton.
2.  While holding the button, **apply power** to the ATTINY controller (e.g., by plugging in the device).
3.  The onboard debug LED will light up, indicating that the controller has registered the request to enter bootloader mode.
4.  Once the button is released, the controller **automatically powers on** the main STM32 processor with the `Boot0` signal asserted. This forces the STM32 to start up in its internal bootloader, allowing for firmware updates. This sequence does not require a second button press.
5.  This bootloader mode is only active for this single, specific startup event. Once completed, any subsequent power cycles initiated by the button will be normal application boots.

## Pin Definitions

The firmware is configured for the following pin connections on the ATTINY microcontroller:

| Pin Name      | I/O       | MCU Pin | Description                                                 |
|---------------|-----------|---------|-------------------------------------------------------------|
| `PowerEnable` | `OUTPUT`  | 0       | Controls the main power supply for the STM32 board.         |
| `nReset`      | `OUTPUT`  | 1       | Active-low reset pin for the STM32 processor.               |
| `Boot0`       | `OUTPUT`  | 2       | Controls the STM32 Boot0 pin to select the boot source.     |
| `DebugLED`    | `OUTPUT`  | 3       | Provides visual status feedback.                            |
| `PushBTN`     | `INPUT`   | 4       | The user-facing control button.                             |

## Building the Firmware

This firmware is written as a standard Arduino sketch. It can be compiled and flashed to a compatible ATTINY microcontroller using the Arduino IDE with the appropriate ATTINY core installed.

---

*Lark Power Management and Boot Controller © HamSlices 2025*