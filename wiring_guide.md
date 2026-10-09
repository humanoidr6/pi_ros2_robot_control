Hardware Wiring Specification

This document details the GPIO pin mappings and power distribution requirements between the Raspberry Pi 4, the IBT_2 motor drivers, and the Hall-effect encoders. 

All logic connections use standard BCM numbering. Physical board pin numbers are provided for reference.

1. Left Motor Driver (IBT_2)

| IBT_2 Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V | Pin 1 | Logic Power |
| GND | GND | Pin 9 | Logic Ground |
| R_EN | GPIO 27 | Pin 13 | Right Enable (Active High) |
| L_EN | GPIO 17 | Pin 11 | Left Enable (Active High) |
| RPWM | GPIO 10 | Pin 19 | Forward PWM Signal |
| LPWM | GPIO 22 | Pin 15 | Reverse PWM Signal |
| R_S | N/C | N/A | Current Sense (Unused) |
| L_S | N/C | N/A | Current Sense (Unused) |

2. Right Motor Driver (IBT_2)

| IBT_2 Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V | Pin 17 | Logic Power |
| GND | GND | Pin 25 | Logic Ground |
| R_EN | GPIO 11 | Pin 23 | Right Enable (Active High) |
| L_EN | GPIO 9 | Pin 21 | Left Enable (Active High) |
| RPWM | GPIO 6 | Pin 31 | Forward PWM Signal |
| LPWM | GPIO 5 | Pin 29 | Reverse PWM Signal |
| R_S | N/C | N/A | Current Sense (Unused) |
| L_S | N/C | N/A | Current Sense (Unused) |

3. Quadrature Encoders (Hall Effect)

| Encoder Pin | Pi BCM Pin | Pi Physical Pin | Function |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V / 5V | Variable | Sensor Power (Check motor spec) |
| GND | GND | Variable | Sensor Ground |
| Left Phase A | GPIO 23 | Pin 16 | Left Interrupt Trigger |
| Left Phase B | GPIO 24 | Pin 18 | Left Direction Check |
| Right Phase A| GPIO 25 | Pin 22 | Right Interrupt Trigger |
| Right Phase B| GPIO 8 | Pin 24 | Right Direction Check |

4. Power Distribution & Safety Notes

- Common Ground Requirement: The logic ground (GND) on both IBT_2 drivers must be tied directly to a ground pin on the Raspberry Pi. The main battery ground must also share this reference. Failure to establish a common ground will result in floating PWM signals and erratic motor behavior.

- High Current Isolation: Route main battery power (+12V/24V) directly to the B+ and B- terminals on the IBT_2 blocks. Route the M+ and M- terminals directly to the motors. Do not route high-current battery power through the Raspberry Pi or its breadboard rails.

- Logic Level Constraints: The Raspberry Pi GPIO pins operate at 3.3V. While the IBT_2 logic inputs (VCC) are generally 5V tolerant, supplying them with 3.3V from the Pi ensures the PWM and Enable signals are interpreted correctly without requiring a logic level shifter.
