# BL4818 Driver Board — Hardware Pinout & Notes

> **Board:** XT48-12 V02 (silkscreen on stator side)
> **Status:** Work in progress — fill in as you probe the board with a multimeter.

## MCU: Nuvoton MS51FB9AE (TSSOP-20)

From the official Nuvoton datasheet (Figure 4.1-1):

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

## Pin-by-Pin Worksheet

Fill in the "Board Connection" column as you trace each pin.

| Pin | Port | Datasheet Functions                              | Board Connection | Notes |
|-----|------|--------------------------------------------------|------------------|-------|
|  1  | P0.5 | IC6 / T0 / **PWM0_CH2** / ADC_CH4               |                  |       |
|  2  | P0.6 | **UART0_TXD** / ADC_CH3                          |                  |       |
|  3  | P0.7 | **UART0_RXD** / ADC_CH2                          |                  |       |
|  4  | P2.0 | **nRESET**                                       |                  | Prog header "R" |
|  5  | P3.0 | INT0 / OSCIN / ADC_CH1                           |                  |       |
|  6  | P1.7 | INT1 / **ADC_CH0**                               |                  |       |
|  7  | VSS  | **Ground**                                       | GND              | Confirmed |
|  8  | P1.6 | [I2C0_SDA] / UART1_TXD / **ICE_DAT**            |                  | Prog header "P" |
|  9  | VDD  | **Power (2.4–5.5V)**                             | VDD              | Confirmed. 3.3V or 5V? |
| 10  | P1.5 | IC7 / SPI0_SS / **PWM0_CH5**                     |                  |       |
| 11  | P1.4 | I2C0_SDA / **PWM0_BRAKE** / PWM0_CH1             |                  | HW fault brake! |
| 12  | P1.3 | I2C0_SCL / [STADC]                               |                  |       |
| 13  | P1.2 | **PWM0_CH0** / IC0                               |                  |       |
| 14  | P1.1 | ADC_CH7 / **PWM0_CH1** / IC1 / CLKO             |                  |       |
| 15  | P1.0 | **PWM0_CH2** / IC2 / SPI0_CLK                    |                  |       |
| 16  | P0.0 | **PWM0_CH3** / IC3 / SPI0_MOSI / T1             |                  |       |
| 17  | P0.1 | **PWM0_CH4** / IC4 / SPI0_MISO                   |                  |       |
| 18  | P0.2 | **ICE_CLK** / UART1_RXD / [I2C0_SCL]             |                  | Prog header "S" |
| 19  | P0.3 | ADC_CH6 / **PWM0_CH5** / IC5                     |                  |       |
| 20  | P0.4 | ADC_CH5 / **PWM0_CH3** / IC3 / STADC             |                  |       |

### PWM Channel Summary

The MS51FB9AE has 6 PWM channels. Each can be mapped to multiple pins:

| PWM Channel | Available Pins              | Likely Use           |
|-------------|----------------------------|----------------------|
| PWM0_CH0    | P1.2 (pin 13)             | Bridge gate          |
| PWM0_CH1    | P1.1 (pin 14), P1.4 (pin 11) | Bridge gate      |
| PWM0_CH2    | P0.5 (pin 1), P1.0 (pin 15)  | Bridge gate      |
| PWM0_CH3    | P0.0 (pin 16), P0.4 (pin 20) | Bridge gate      |
| PWM0_CH4    | P0.1 (pin 17)             | Bridge gate          |
| PWM0_CH5    | P1.5 (pin 10), P0.3 (pin 19) | Bridge gate      |
| PWM0_BRAKE  | P1.4 (pin 11)             | Overcurrent shutdown |

### ADC Channel Summary

| ADC Channel | Pin              | Likely Use           |
|-------------|------------------|----------------------|
| ADC_CH0     | P1.7 (pin 6)    |                      |
| ADC_CH1     | P3.0 (pin 5)    |                      |
| ADC_CH2     | P0.7 (pin 3)    |                      |
| ADC_CH3     | P0.6 (pin 2)    |                      |
| ADC_CH4     | P0.5 (pin 1)    |                      |
| ADC_CH5     | P0.4 (pin 20)   |                      |
| ADC_CH6     | P0.3 (pin 19)   |                      |
| ADC_CH7     | P1.1 (pin 14)   |                      |

## Gate Drive Topology

Complementary half-bridge per phase: **P-channel high-side + N-channel low-side**
in a single 4-pin + tab package. All outputs are **active-high** from the MCU.

```
                    VCC_MOTOR
                       │
                  ┌────┴────┐
                  │ High-side│
                  │  P-ch    │──── Gate ◄── pulled up to VCC_MOTOR via resistor
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

**How it works (all active-high from MCU):**
- **Low-side ON:** MCU pin HIGH → gate resistor → N-ch gate HIGH → N-ch conducts
- **High-side ON:** MCU pin HIGH → small N-FET gate HIGH → N-FET conducts →
  pulls P-ch gate to GND → P-ch conducts (P-FET turns on with low gate)
- **Both OFF:** MCU pin LOW → low-side N-ch off; small N-FET off →
  P-ch gate pulled up to VCC by resistor → P-ch off

No inversion from the MCU's perspective.

### Confirm by probing:

- [ ] What package are the 3 complementary MOSFET pairs? (4-pin + tab)
- [ ] Can you read the part marking? _______________
- [ ] Confirm P-ch high-side / N-ch low-side by checking tab to VCC vs GND
- [ ] Confirm small N-FETs (likely SOT-23) — one per phase near the main FET

### Package identification help

Common complementary MOSFET packages (N+P pair, 4-pin + tab):
- **SO-8:** e.g., AO4606 (N+P, 30V), AO4616 (N+P, 30V)
- **SOP-8 / TSSOP-8:** various Chinese dual complementary parts
- **DFN / PDFN:** compact dual complementary

> Note the part marking on the MOSFET package if readable: _______________

## Three-Phase Bridge

6 gate drive signals needed (high + low for each of U, V, W).
Pins with PWM capability on the right side (pins 10-17) are prime candidates.

| Phase | MOSFET Pkg | High-side MCU Pin | Low-side MCU Pin | Confirmed? |
|-------|-----------|-------------------|------------------|------------|
| U     |           |                   |                  | [ ]        |
| V     |           |                   |                  | [ ]        |
| W     |           |                   |                  | [ ]        |

## High-Side Gate Drive N-FETs

| Phase | Small N-FET Package | MCU Pin (gate input) | Confirmed? |
|-------|--------------------|-----------------------|------------|
| U     |                    |                       | [ ]        |
| V     |                    |                       | [ ]        |
| W     |                    |                       | [ ]        |

## Hall Sensors

| Sensor | Wire Color | MCU Pin | Confirmed? |
|--------|-----------|---------|------------|
| Hall A |           |         | [ ]        |
| Hall B |           |         | [ ]        |
| Hall C |           |         | [ ]        |

## Current Shunt

Single low-side shunt (confirmed by inspection).

- [ ] Shunt resistor location: between bridge ground and battery negative?
- [ ] Shunt resistance value: ___ mΩ (read marking or measure)
- [ ] Amplifier circuit: op-amp? direct to ADC pin?
- [ ] ADC pin for shunt voltage: ___

## Control Inputs / Outputs

| Signal      | Connector Pin | MCU Pin | Type          | Notes |
|-------------|--------------|---------|---------------|-------|
| PWM input   |              |         | Input         | Speed control |
| Direction   |              |         | Input (logic) |       |
| Enable      |              |         | Input (logic) |       |
| LED         |              |         | Output        |       |
| VCC (logic) |              |         | Power         | 3.3V or 5V? |
| GND         |              |         | Power         |       |

## Programming Header

Silkscreen labels: **P, R, S, +, −**

| Pad | Label | Signal    | MCU Pin            | Confirmed? |
|-----|-------|-----------|--------------------|------------|
|     | +     | VDD       | Pin 9 (VDD)        | [ ]        |
|     | −     | GND       | Pin 7 (VSS)        | [ ]        |
|     | R     | nRESET    | Pin 4 (P2.0)       | [ ]        |
|     | P     | ICE_DAT   | Pin 8 (P1.6)       | [ ]        |
|     | S     | ICE_CLK   | Pin 18 (P0.2)      | [ ]        |

### Nu-Link Connection

| Nu-Link Pin | Board Header |
|-------------|-------------|
| VCC         | +           |
| GND         | −           |
| nRESET      | R           |
| ICE_DAT     | P           |
| ICE_CLK     | S           |

## Power Supply

- [x] Motor voltage: 12V (XT48-12 designation)
- [ ] Logic voltage: ___ V (MCU VDD — likely 3.3V or 5V from onboard regulator)
- [ ] Regulator type/part: ___ (LDO? SOT-23-5?)

## Probing Tips

1. **Power first:** Confirm VDD (pin 9) and VSS (pin 7) — already confirmed connected to power. Measure VDD voltage with board powered to determine 3.3V vs 5V logic.
2. **Programming header:** Confirm continuity: "P" pad → pin 8 (ICE_DAT), "S" pad → pin 18 (ICE_CLK), "R" pad → pin 4 (nRESET). This lets you connect the Nu-Link immediately.
3. **Gate outputs:** With power off, check continuity from pins on the right side of the MCU (pins 10-17: P1.5, P1.4, P1.3, P1.2, P1.1, P1.0, P0.0, P0.1) to the gate resistors on the MOSFETs. Also check pin 1 (P0.5) and pins 19-20 (P0.3, P0.4). These PWM-capable pins drive the bridge.
4. **PWM0_BRAKE:** Pin 11 (P1.4) has hardware fault brake — may be wired to the current sense comparator for fast overcurrent shutdown.
5. **Hall sensors:** Trace the 3 wires from the motor's hall sensor cable. Look for pins NOT used by PWM — candidates: pin 5 (P3.0), pin 6 (P1.7), pin 12 (P1.3), or pins on the left side (P0.5-P0.7).
6. **Current shunt:** Find the sense resistor and trace to an ADC-capable pin. ADC channels available on: P1.7 (pin 6), P3.0 (pin 5), P0.7 (pin 3), P0.6 (pin 2), P0.5 (pin 1), P0.4 (pin 20), P0.3 (pin 19), P1.1 (pin 14).
7. **UART:** If you want serial debug, P0.6 (pin 2) = UART0_TX, P0.7 (pin 3) = UART0_RX. Check if these are used for anything else on the board.

## Board Photos

> Add photos here or link to them — top side, bottom side, close-up of MCU area.
