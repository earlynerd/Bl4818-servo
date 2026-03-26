# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Custom firmware for BL4818 brushless motor driver boards (cheap "massage gun" drivers), turning them into servo drives. Runs on a **Nuvoton MS51FB9AE** (enhanced 1T 8051, 24 MHz, 16 KB flash, 256 B IRAM + 1 KB XRAM, TSSOP-20). Compiled with **SDCC** targeting MCS-51.

## Build Commands

```bash
make              # Build firmware → build/bl4818-servo.ihx
make clean        # Remove build artifacts
make flash        # Flash via Nu-Link programmer (nulink_8051_fwupdate)
make size         # Show flash/RAM usage
```

Prerequisites: SDCC 4.2+, GNU Make. Flashing requires a Nuvoton Nu-Link programmer.

## Architecture

**Control loop**: 1 kHz polled timer tick in `main.c`. Between ticks, the main loop processes UART commands.

**Motor state machine** (`motor.c`): States are IDLE → RUN / BRAKE / REVERSING / FAULT. Supports four control modes: open-loop PWM, velocity PID, position PID, and torque (current) limiting.

**Signal flow**: Hall sensors → commutation table lookup → PWM mask update → 3-phase bridge

Key modules and their roles:
- `pwm.c` — Hardware complementary PWM (20 kHz, 6 channels in 3 pairs) with 1 µs dead-time via PWM0 module. Uses PMEN/PMD mask registers for six-step switching.
- `commutation.c` — Forward/reverse lookup tables mapping hall state (1-6) + direction → PMEN/PMD values. One phase PWM-active, one low-side on, one floating.
- `hall.c` — Reads 3 hall sensors on different ports (P1.5, P1.7, P3.0), tracks transitions for speed measurement (period → RPM).
- `adc.c` — Current sensing via 20 mΩ shunt direct to ADC (no amplifier). Soft limit 3 A, hard fault at 5 A. Also reads battery voltage.
- `pid.c` — Q8 fixed-point PID with anti-windup. Used for velocity and position loops.
- `uart.c` — UART1 (not UART0, which conflicts with ADC pins) at 115200 baud on the programming header. Interrupt-driven with 64-byte ring buffers.
- `protocol.c` — ASCII command parser: P/V/T/D for targets, S/R for stop/release, ? for status query, G/W for parameter get/set.
- `flash.c` — IAP for non-volatile parameter storage (mode, PID gains, torque limit, encoder CPR).
- `encoder.c` — Quadrature decoder (currently unpopulated on the board, code ready).

## Hardware Constraints

- **8051 architecture**: No native 32-bit types. Prefer `uint8_t`/`uint16_t`. Use `__xdata` for variables that don't fit in 256-byte IRAM.
- **16 KB flash**: Code size is tight. Build with `--opt-code-size`. Check `make size` after changes.
- **SDCC quirks**: Use `__sfr`, `__sbit`, `__xdata`, `__idata`, `__code` keywords. ISRs use `__interrupt(N)` syntax. SFR page switching needed for PWM4/5 duty registers.
- **main.c must link first** (Makefile enforces this) for correct 8051 startup code placement.
- All 6 gate drives are **active-high** from the MCU (high-side P-FETs use an inverting N-FET driver stage on the board).
- UART0 pins (P0.6/P0.7) are used for ADC, so serial uses UART1 on the programming header.

## Configuration

All hardware parameters, pin assignments, and feature toggles are in `include/ms51_config.h`. Register definitions are in `include/ms51_reg.h`.

Feature toggles (`FEATURE_ENCODER`, `FEATURE_UART`, `FEATURE_PID`, `FEATURE_FLASH_SAVE`, `FEATURE_LED`) enable/disable modules at compile time.
