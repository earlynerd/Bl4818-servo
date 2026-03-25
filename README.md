# BL4818-Servo: Custom Firmware for BL4818 Brushless Motor Drivers

Custom open-source firmware for the inexpensive BL4818 "massage gun" / "fascia gun"
brushless motor driver boards, turning them into low-cost servo drives.

## Motivation

BL4818 integrated brushless motor driver boards are available from China for a few
dollars. They contain everything needed for a basic brushless servo drive:

- Nuvoton MS51FB9AE microcontroller (8051 core, 16KB flash)
- Three hall effect sensors for rotor position
- Complementary three-phase MOSFET bridge with gate drivers
- Low-side current shunt resistor for overcurrent protection
- PWM speed input and direction control pin

The stock firmware has several limitations that prevent use as a servo:

1. **Poor stall behavior** — the motor loses commutation and draws excessive current
   when stalled or heavily loaded at low speed
2. **Direction change requires stop** — the direction input is ignored unless the
   motor is completely stopped, making closed-loop position control impossible
3. **No encoder interface** — no support for external position feedback
4. **No serial interface** — no way to command position/velocity/torque targets

This project rewrites the firmware from scratch to fix these issues.

## Hardware

### Target Board

BL4818 brushless motor driver board (commonly sold for massage gun / fascia gun motors).

| Parameter          | Value                     |
|--------------------|---------------------------|
| MCU                | Nuvoton MS51FB9AE         |
| Architecture       | Enhanced 1T 8051, 24 MHz  |
| Flash              | 16 KB (APROM)             |
| RAM                | 256B IRAM + 1KB XRAM      |
| Package            | TSSOP-20                  |
| Supply Voltage     | 12–24V DC (3S–6S LiPo)   |
| Bridge             | 3-phase complementary (P+N pair per phase) |
| Position Sensing   | 3× Hall effect sensors    |
| Current Sensing    | Low-side shunt resistor   |

### Pin Mapping (MS51FB9AE TSSOP-20)

From the official Nuvoton datasheet:

```
Pin  Port   Key Functions                      Board Connection
───  ─────  ─────────────────────────────────  ─────────────────────
 1   P0.5   T0 / PWM0_CH2 / ADC_CH4           TBD — VERIFY
 2   P0.6   UART0_TXD / ADC_CH3               TBD — VERIFY
 3   P0.7   UART0_RXD / ADC_CH2               TBD — VERIFY
 4   P2.0   nRESET                             Prog header "R"
 5   P3.0   INT0 / ADC_CH1                     TBD — VERIFY
 6   P1.7   INT1 / ADC_CH0                     TBD — VERIFY
 7   VSS    Ground                             GND (confirmed)
 8   P1.6   ICE_DAT / UART1_TX / [I2C0_SDA]   Prog header "P"
 9   VDD    Power (2.4–5.5V)                   VDD (confirmed)
10   P1.5   PWM0_CH5 / SPI0_SS                 TBD — VERIFY
11   P1.4   PWM0_BRAKE / PWM0_CH1              TBD — VERIFY
12   P1.3   I2C0_SCL                           TBD — VERIFY
13   P1.2   PWM0_CH0                           TBD — VERIFY
14   P1.1   ADC_CH7 / PWM0_CH1 / CLKO         TBD — VERIFY
15   P1.0   PWM0_CH2 / SPI0_CLK                TBD — VERIFY
16   P0.0   PWM0_CH3 / SPI0_MOSI / T1         TBD — VERIFY
17   P0.1   PWM0_CH4 / SPI0_MISO               TBD — VERIFY
18   P0.2   ICE_CLK / UART1_RX / [I2C0_SCL]   Prog header "S"
19   P0.3   ADC_CH6 / PWM0_CH5                 TBD — VERIFY
20   P0.4   ADC_CH5 / PWM0_CH3 / STADC         TBD — VERIFY
```

**Programming header** (silkscreen: P, R, S, +, −):
P = ICE_DAT (pin 8), R = nRESET (pin 4), S = ICE_CLK (pin 18),
\+ = VDD (pin 9), − = GND (pin 7)

> **Note:** Board-specific pin assignments (which pin drives which gate, etc.)
> are being mapped with a multimeter. See [docs/pinout.md](docs/pinout.md) for
> the detailed worksheet and current progress.

### UART

UART0 TX/RX are on P0.6 (pin 2) and P0.7 (pin 3) per the datasheet.
These pins also have ADC capability (CH3/CH2). Availability depends on
what the stock board uses them for.

## Features

### Implemented

- [x] Six-step (trapezoidal) commutation from hall sensors
- [x] Complementary PWM with configurable dead time
- [x] Active braking and regeneration on direction change (no stop required)
- [x] Current limiting via shunt ADC with fast overcurrent fault brake
- [x] Smooth low-speed operation with stall detection and recovery
- [x] Incremental encoder interface (quadrature decoding)
- [x] PID position and velocity control loops
- [x] UART command protocol for target position/velocity/torque
- [x] Configurable parameters stored in flash (IAP)
- [x] Watchdog safety timeout

### Planned

- [ ] Field-oriented control (FOC) with single-shunt reconstruction
- [ ] CAN bus interface (via SPI-to-CAN bridge)
- [ ] Step/direction input mode (like a stepper driver)
- [ ] Sensorless startup (back-EMF zero crossing)

## Building

### Prerequisites

- [SDCC](https://sdcc.sourceforge.net/) 4.2+ (Small Device C Compiler)
- GNU Make
- [Nuvoton Nu-Link](https://www.nuvoton.com/tool-and-software/debugger-and-programmer/1-to-1-debugger-and-programmer/nu-link/) programmer (for flashing via ICP)

### Compile

```bash
make            # Build firmware (output: build/bl4818-servo.ihx)
make clean      # Remove build artifacts
make flash      # Flash via Nu-Link (requires nulink-cli or openocd)
make size       # Show flash/RAM usage
```

### Toolchain Notes

This project uses **SDCC** (open-source) rather than Keil C51. SDCC is freely
available and produces comparable code for the 8051 target. The MS51 BSP headers
have been adapted from the [MS51BSP_SDCC](https://github.com/danchouzhou/MS51BSP_SDCC)
project.

## Project Structure

```
├── README.md              This file
├── Makefile               Build system
├── include/
│   ├── ms51_reg.h         MS51FB9AE register definitions
│   ├── ms51_config.h      Clock, pin, and feature configuration
│   ├── pwm.h              PWM and dead-time control
│   ├── adc.h              ADC (current sense, voltage)
│   ├── commutation.h      Six-step commutation tables and logic
│   ├── hall.h             Hall sensor reading and state machine
│   ├── encoder.h          Quadrature encoder interface
│   ├── motor.h            Motor control state machine
│   ├── pid.h              PID controller
│   ├── uart.h             UART driver
│   ├── protocol.h         Serial command protocol
│   └── flash.h            IAP flash parameter storage
├── src/
│   ├── main.c             Entry point, init, main loop
│   ├── pwm.c              PWM setup and duty cycle control
│   ├── adc.c              ADC sampling and current measurement
│   ├── commutation.c      Commutation table and phase switching
│   ├── hall.c             Hall sensor ISR and state tracking
│   ├── encoder.c          Encoder counting ISR
│   ├── motor.c            Motor state machine (run/brake/fault)
│   ├── pid.c              PID loop implementation
│   ├── uart.c             UART TX/RX with ring buffer
│   ├── protocol.c         Command parser and response
│   └── flash.c            IAP read/write for parameters
└── docs/
    └── pinout.md          Detailed pinout and board photos
```

## Serial Protocol

Default: 115200 baud, 8N1

Commands are ASCII text terminated by newline (`\n`):

| Command          | Description                          | Response         |
|------------------|--------------------------------------|------------------|
| `P<counts>`      | Set target position (encoder counts) | `OK` or `ERR`    |
| `V<rpm>`         | Set target velocity (RPM)            | `OK` or `ERR`    |
| `T<mA>`          | Set torque limit (milliamps)         | `OK` or `ERR`    |
| `D<duty>`        | Direct PWM duty (0–1000, open-loop)  | `OK` or `ERR`    |
| `S`              | Stop (brake)                         | `OK`             |
| `R`              | Release (coast)                      | `OK`             |
| `?`              | Query status                         | See below        |
| `G<param>`       | Get parameter                        | `<value>`        |
| `W<param>=<val>` | Set parameter (saved to flash)       | `OK` or `ERR`    |

Status response format: `pos:<counts>,vel:<rpm>,cur:<mA>,state:<state>,fault:<code>`

## License

MIT License. See [LICENSE](LICENSE) for details.

## Contributing

Contributions welcome. Please open an issue to discuss changes before submitting
a pull request. Hardware photos and board revision documentation are especially
appreciated.

## References

- [Nuvoton MS51FB9AE Datasheet](https://www.nuvoton.com/products/microcontrollers/8bit-8051-mcus/industrial-8051-series/ms51fb9ae/)
- [MS51 Technical Reference Manual](https://www.nuvoton.com/export/resource-files/TRM_MS51FB9AE_MS51XB9AE_MS51XB9BE_EN_Rev1.03.pdf)
- [MS51BSP_SDCC (GitHub)](https://github.com/danchouzhou/MS51BSP_SDCC)
- [SDCC Compiler](https://sdcc.sourceforge.net/)
