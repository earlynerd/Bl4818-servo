# BL4818 Driver Board — Hardware Pinout & Notes

> **Board:** XT48-12 V02 (silkscreen on stator side)
> **Status:** Pin assignments CONFIRMED by multimeter probing.

## MCU: Nuvoton MS51FB9AE (TSSOP-20)

From the official Nuvoton datasheet, with confirmed board connections:

```
                          ┌──────────┐
 IC6/T0/PWM0_CH2/ADC4/P0.5  1 ───┤          ├─── 20  P0.4/ADC5/PWM0_CH3/IC3/STADC
       UART0_TXD/ADC3/P0.6  2 ───┤          ├─── 19  P0.3/ADC6/PWM0_CH5/IC5
       UART0_RXD/ADC2/P0.7  3 ───┤          ├─── 18  P0.2/ICE_CLK/UART1_RXD/[I2C0_SCL]
              nRESET / P2.0  4 ───┤MS51FB9AE ├─── 17  P0.1/PWM0_CH4/IC4/SPI0_MISO
      INT0/OSCIN/ADC1/P3.0  5 ───┤          ├─── 16  P0.0/PWM0_CH3/IC3/SPI0_MOSI/T1
          INT1/ADC0  /P1.7  6 ───┤          ├─── 15  P1.0/PWM0_CH2/IC2/SPI0_CLK
                       VSS  7 ───┤          ├─── 14  P1.1/ADC7/PWM0_CH1/IC1/CLKO
 [I2C0_SDA]/UART1_TXD/     8 ───┤          ├─── 13  P1.2/PWM0_CH0/IC0
         ICE_DAT/P1.6       │    │          │
                       VDD  9 ───┤          ├─── 12  P1.3/I2C0_SCL/[STADC]
  IC7/SPI0_SS/PWM0_CH5/   10 ───┤          ├─── 11  P1.4/I2C0_SDA/PWM0_BRAKE/PWM0_CH1
                    P1.5     │    └──────────┘
```

## Confirmed Pin Assignments

| Pin | Port | Datasheet Functions                  | Board Connection              | Confirmed |
|-----|------|--------------------------------------|-------------------------------|-----------|
|  1  | P0.5 | T0 / PWM0_CH2 / ADC_CH4             | Tach output (via N-FET, inverted) | YES |
|  2  | P0.6 | UART0_TXD / ADC_CH3                 | Current shunt (direct, no amp) | YES |
|  3  | P0.7 | UART0_RXD / ADC_CH2                 | Battery voltage divider (10k/10k) | YES |
|  4  | P2.0 | nRESET                               | Prog header "R" (reset)       | YES |
|  5  | P3.0 | INT0 / OSCIN / ADC_CH1              | **Hall sensor 3**             | YES |
|  6  | P1.7 | INT1 / ADC_CH0                       | **Hall sensor 2**             | YES |
|  7  | VSS  | Ground                               | GND                          | YES |
|  8  | P1.6 | ICE_DAT / UART1_TXD / [I2C0_SDA]   | Prog header "P" (ICE_DAT)    | YES |
|  9  | VDD  | Power (2.4–5.5V)                     | **5V** from onboard regulator | YES |
| 10  | P1.5 | SPI0_SS / PWM0_CH5                   | **Hall sensor 1**             | YES |
| 11  | P1.4 | PWM0_BRAKE / PWM0_CH1 / I2C0_SDA   | **Direction input**           | YES |
| 12  | P1.3 | I2C0_SCL / [STADC]                   | Possibly NC                  | YES |
| 13  | P1.2 | **PWM0_CH0** / IC0                   | **Phase U low-side gate**     | YES |
| 14  | P1.1 | ADC_CH7 / **PWM0_CH1** / IC1 / CLKO | **Phase U high-side gate**    | YES |
| 15  | P1.0 | **PWM0_CH2** / IC2 / SPI0_CLK       | **Phase V high-side gate**    | YES |
| 16  | P0.0 | **PWM0_CH3** / IC3 / SPI0_MOSI / T1 | **Phase V low-side gate**     | YES |
| 17  | P0.1 | **PWM0_CH4** / IC4 / SPI0_MISO      | **Phase W low-side gate**     | YES |
| 18  | P0.2 | ICE_CLK / UART1_RXD / [I2C0_SCL]    | Prog header "S" (ICE_CLK)    | YES |
| 19  | P0.3 | ADC_CH6 / **PWM0_CH5** / IC5        | **Phase W high-side gate**    | YES |
| 20  | P0.4 | ADC_CH5 / PWM0_CH3 / IC3 / STADC    | PWM speed input (no RC filter, unpop) | YES |

## Three-Phase Bridge — CONFIRMED

All 6 gate pins map to hardware PWM channels:

| Phase | High-side Pin    | High PWM Ch | Low-side Pin     | Low PWM Ch  |
|-------|------------------|-------------|------------------|-------------|
| **U** | P1.1 (pin 14)   | PWM0_CH1    | P1.2 (pin 13)   | PWM0_CH0    |
| **V** | P1.0 (pin 15)   | PWM0_CH2    | P0.0 (pin 16)   | PWM0_CH3    |
| **W** | P0.3 (pin 19)   | PWM0_CH5    | P0.1 (pin 17)   | PWM0_CH4    |

All gate drives are **active-high** from the MCU.

## Gate Drive Topology — CONFIRMED

Complementary half-bridge per phase: **P-channel high-side + N-channel low-side**
in a single 4-pin + tab package. Tab = phase output (U/V/W).

```
                    VCC_MOTOR (12V)
                       │
                  ┌────┴────┐
                  │ High-side│
                  │  P-ch    │──── Gate ◄── pulled up to VCC via R
                  └────┬────┘                    │
                       │                    ┌────┴────┐
                       │                    │ Small   │
                       │                    │ N-FET   │──── Gate ◄── MCU pin ──[R]──
                       │                    └────┬────┘
                       │                         │
                       │                        GND
                       │
                       ├─────────── Phase output (U / V / W) = package tab
                       │
                  ┌────┴────┐
                  │ Low-side │
                  │  N-ch    │──── Gate ◄── MCU pin ──[R_gate]──
                  └────┬────┘                │
                       │              [R_pulldown]
                       │                     │
                    GND (via shunt)          GND
```

**All active-high from MCU:** MCU HIGH = FET ON (for both high and low sides).

## Hall Sensors — CONFIRMED

| Sensor   | MCU Pin         | MCU Features        |
|----------|-----------------|---------------------|
| Hall 1   | P1.5 (pin 10)  | SPI0_SS / PWM0_CH5  |
| Hall 2   | P1.7 (pin 6)   | INT1 / ADC_CH0      |
| Hall 3   | P3.0 (pin 5)   | INT0 / ADC_CH1      |

Note: Hall 2 has INT1, Hall 3 has INT0 — hardware interrupt support for
fast commutation updates on those two channels.

## Current Sensing — CONFIRMED

- **Shunt:** Single low-side, direct connection to MCU (NO op-amp amplifier)
- **ADC pin:** P0.6 (pin 2) = **ADC_CH3**
- **ADC reference:** VDD = **5.0V**
- **Shunt resistance:** ___ mΩ (read marking or measure)

With no amplifier, the ADC reads the raw shunt voltage:
```
V_adc = I_motor × R_shunt
ADC_val = V_adc / 5.0 × 4096

I_mA = ADC_val × 5000 / (4096 × R_shunt_ohm)
```

For a typical 50mΩ shunt:
- 1A → 50mV → ADC reading 41
- 5A → 250mV → ADC reading 205
- Resolution is coarse without amplification (~12mA per LSB at 50mΩ)

> TODO: Measure or read shunt resistance value

## Battery Voltage — CONFIRMED

- **Voltage divider:** 10k / 10k (2:1 ratio)
- **ADC pin:** P0.7 (pin 3) = **ADC_CH2**
- **ADC reference:** VDD = **5.0V**

```
V_battery = ADC_val / 4096 × 5.0 × 2
```

At 12.6V (fully charged 3S): divider output = 6.3V → clamps at 5V (ADC reads 4095).
At 12.0V: divider output = 6.0V → still clamps.
At 10.0V: divider output = 5.0V → ADC reads 4095.
At 9.0V: divider output = 4.5V → ADC reads 3686.

**Note:** With a 10k/10k divider and 5V VDD, the divider only gives useful
readings below ~10V. Above that it's clipped. This is marginal for 12V operation.
May only be useful for low-battery detection.

## Tach Output — CONFIRMED

- **Pin:** P0.5 (pin 1)
- **Type:** Open-drain via external N-FET (inverted logic)
- MCU HIGH → N-FET on → tach output pulled low
- MCU LOW → N-FET off → tach output pulled high (by external pull-up)

## Control Inputs — CONFIRMED

| Signal     | MCU Pin         | Type         | Notes                           |
|------------|-----------------|--------------|----------------------------------|
| Direction  | P1.4 (pin 11)  | Digital input | Also has PWM0_BRAKE function    |
| PWM input  | P0.4 (pin 20)  | Analog/PWM   | Unpopulated RC filter footprint |
| Tach out   | P0.5 (pin 1)   | Output (inv) | Via external N-FET              |

## Programming Header — CONFIRMED

Silkscreen labels: **P, R, S, +, −**

| Label | Signal    | MCU Pin            |
|-------|-----------|--------------------|
| +     | VDD       | Pin 9 (5V)         |
| −     | GND       | Pin 7 (VSS)        |
| R     | nRESET    | Pin 4 (P2.0)       |
| P     | ICE_DAT   | Pin 8 (P1.6)       |
| S     | ICE_CLK   | Pin 18 (P0.2)      |

### Nu-Link Connection

| Nu-Link Pin | Board Header |
|-------------|-------------|
| VCC         | +           |
| GND         | −           |
| nRESET      | R           |
| ICE_DAT     | P           |
| ICE_CLK     | S           |

## Power Supply — CONFIRMED

- Motor voltage: **12V** (XT48-12 designation)
- Logic voltage: **5V** (onboard regulator → MCU VDD)
- Regulator type/part: ___ (TBD — read marking)

## Available Pins for Expansion

| Pin | Port | Current Use        | Available For           |
|-----|------|--------------------|-------------------------|
| 12  | P1.3 | Possibly NC        | Encoder A, GPIO, I2C_SCL |
| 11  | P1.4 | Direction input    | Could repurpose for encoder or PWM_BRAKE |
| 20  | P0.4 | PWM input          | Could repurpose for encoder B |
| 1   | P0.5 | Tach output        | Could repurpose                |
| 2   | P0.6 | Current shunt ADC  | Shared: UART0_TX (if shunt moved) |
| 3   | P0.7 | Voltage divider ADC| Shared: UART0_RX (if divider disconnected) |

**For servo mode (encoder + UART):**
- Encoder A → P1.3 (pin 12, currently NC)
- Encoder B → P0.4 (pin 20, repurpose PWM input)
- UART TX → P0.6 (pin 2, sacrifice current sense — or use UART1 on P1.6/P0.2 via programming header)
- UART RX → P0.7 (pin 3, sacrifice voltage sense)

Or use UART1 on the programming header pads (P=TX, S=RX) to keep ADC.
