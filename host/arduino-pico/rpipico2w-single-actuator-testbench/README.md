# RP Pico 2 W Single-Actuator Testbench

This project is a bring-up scaffold for one BL4818 actuator plus one AS5047
encoder. It runs on `rpipico2w` with the Arduino-Pico core under PlatformIO.

What it does:

- reads one or more AS5047 encoders through a daisy-chained SPI link
- closes a simple external PD loop on actuator 0 using encoder position
- drives the BL4818 node through the `BL4818RingMaster` library
- exposes a USB serial console for tuning, zeroing, and telemetry

Default wiring:

- Ring UART TX: `GP4`
- Ring UART RX: `GP5`
- SPI MISO: `GP16`
- SPI CS: `GP17`
- SPI SCK: `GP18`
- SPI MOSI: `GP19`

Notes:

- The current production BL4818 firmware default is `250000` baud, so this
  testbench uses `250000` on the ring by default.
- The AS5047 chain is configured for SPI mode `1`.
- Only actuator address `0` is actively controlled. If more devices enumerate,
  the others receive zero duty.

USB serial commands:

- `help`
- `status`
- `enum`
- `arm`
- `disarm`
- `target <deg>`
- `home <deg>`
- `strike <deg>`
- `tap [hold_ms]`
- `zero`
- `dir <1|-1>`
- `gains <kp> <kd>`
- `maxduty <0..1200>`
- `torque <mA>`
- `stream <0|1>`
