# Project Overview

Custom open-source firmware for the inexpensive BL4818 "massage gun" / "fascia gun" brushless motor driver boards, turning them into low-cost servo drives. The firmware runs on a Nuvoton MS51FB9AE microcontroller (8051 core) and provides a UART binary ring protocol for daisy-chaining and controlling multiple motors. It implements six-step (trapezoidal) commutation, closed-loop position/velocity control (via PID), and soft/hard current limiting.

# Architecture & Main Components

- **MCU:** Nuvoton MS51FB9AE (Enhanced 1T 8051, 24 MHz, 16 KB flash, 256B IRAM + 1KB XRAM, TSSOP-20).
- **Control Loop:** A 1 kHz polled timer tick runs in `src/main.c`. Between ticks, the main loop handles UART protocol processing.
- **Key Modules (`src/`):**
  - `motor.c`: Core motor state machine (IDLE, RUN, BRAKE, REVERSING, FAULT).
  - `pwm.c`: Hardware complementary PWM (20 kHz, 3 pairs) with 1 µs dead-time via the PWM0 module.
  - `commutation.c`: Hall state-to-PWM masking lookup tables for six-step commutation.
  - `hall.c`: Hall sensor reading and speed measurement (period to RPM).
  - `adc.c`: 20 mΩ shunt current sensing (direct to ADC, no amp) and battery voltage monitoring.
  - `pid.c`: Q8 fixed-point PID loop with anti-windup for velocity and position control.
  - `uart.c` & `protocol.c`: Interrupt-driven UART1 with a binary ring protocol allowing enumerated, broadcast, and addressed commands.
  - `flash.c`: In-Application Programming (IAP) for storing non-volatile parameters.

# Building and Flashing

**Prerequisites:** [SDCC 4.2+](https://sdcc.sourceforge.net/) (Small Device C Compiler), GNU Make, and Python. A Raspberry Pi Pico/Pico 2 running the in-repo `nu-link` bridge firmware is used for ISP/ICP flashing.

**Key Commands:**
- `make` — Build the production firmware (outputs to `build/bl4818-servo.ihx`).
- `make clean` — Remove build artifacts.
- `make size` — Show flash and RAM memory usage.
- `make flash` — Flash the firmware using the Nu-Link programmer.
- `make -f Makefile.bench` — Build the minimal bench image (`build/bl4818-bench.bin`) for direct hardware validation (bypassing the motor control stack).

**Configuration and Recovery:**
The project uses checked-in configuration profiles in `ms51_flash_config.py`.
- `python flash.py --write-config` — Writes the default configuration profile.
- `python flash.py --recover` — Recovers the board from a bad configuration state (requires a switched target power setup via the Pico bridge).

# Development Conventions & Hardware Constraints

- **8051 Architecture:** The MS51 has no native 32-bit types. Strictly prefer `uint8_t` and `uint16_t` for performance and code density.
- **Strict Memory Limits:** The 16 KB flash size is very restrictive. Always build with `--opt-code-size` and verify the output with `make size` after making changes.
- **SDCC Specifics:** Use compiler-specific keywords for hardware access and memory placement: `__sfr`, `__sbit`, `__xdata` (for variables exceeding the 256-byte IRAM), `__idata`, and `__code` (for constants). ISRs require the `__interrupt(N)` syntax.
- **Linker Order:** `main.c` must be linked first to ensure correct 8051 startup code placement (this is enforced by the `Makefile`).
- **Pin Constraints:** UART0 pins (P0.6/P0.7) are used for ADC, so the firmware uses **UART1** (P1.6/P0.2) on the programming header for serial communication.
- **Configuration:** All hardware parameters, pin assignments, and feature toggles (e.g., `FEATURE_UART`, `FEATURE_PID`) are centralized in `include/ms51_config.h`. Register definitions are found in `include/ms51_reg.h`.
- **Gate Drives:** All 6 gate drives are driven active-high from the MCU. The commutation tables in `commutation.c` explicitly handle phase differences and dead-time logic; do not invert signals directly in the PWM output.