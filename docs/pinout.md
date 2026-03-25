# BL4818 Driver Board — Hardware Pinout & Notes

> **Status:** Work in progress — fill in as you probe the board with a multimeter.

## MCU: Nuvoton MS51FB9AE (TSSOP-20)

```
                    ┌────────────┐
        P0.5  1 ───┤            ├─── 20  P3.0
        P0.6  2 ───┤            ├─── 19  P0.4
        P0.7  3 ───┤            ├─── 18  P0.3
  ICE_DAT/P2.0 4 ───┤ MS51FB9AE ├─── 17  P0.2/ICE_CLK
        VDD   5 ───┤            ├─── 16  P0.1
        VSS   6 ───┤            ├─── 15  P0.0
        P1.7  7 ───┤            ├─── 14  P1.0
        P1.6  8 ───┤            ├─── 13  P1.1
        P1.5  9 ───┤            ├─── 12  P1.2
        P1.4 10 ───┤            ├─── 11  P1.3
                    └────────────┘
```

## Pin-by-Pin Worksheet

Fill in the "Board Connection" column as you trace each pin.

| Pin | Port | Datasheet Functions           | Board Connection | Notes |
|-----|------|-------------------------------|------------------|-------|
|  1  | P0.5 | PWM0_CH5 / GPIO               |                  |       |
|  2  | P0.6 | ADC6 / UART0_TX / GPIO        |                  |       |
|  3  | P0.7 | ADC7 / UART0_RX / GPIO        |                  |       |
|  4  | P2.0 | ICE_DAT / GPIO                |                  | Programming header DAT? |
|  5  | VDD  | Power (2.4–5.5V)              |                  | 3.3V or 5V rail? |
|  6  | VSS  | Ground                        |                  | GND |
|  7  | P1.7 | PWM0_CH0 / GPIO               |                  |       |
|  8  | P1.6 | PWM0_CH1 / GPIO               |                  |       |
|  9  | P1.5 | PWM0_CH2 / GPIO               |                  |       |
| 10  | P1.4 | PWM0_CH3 / GPIO               |                  |       |
| 11  | P1.3 | PWM0_CH4 / GPIO               |                  |       |
| 12  | P1.2 | GPIO / INT                    |                  |       |
| 13  | P1.1 | GPIO / INT                    |                  |       |
| 14  | P1.0 | GPIO / INT / T2               |                  |       |
| 15  | P0.0 | GPIO / INT0                   |                  |       |
| 16  | P0.1 | GPIO / CLK_OUT / SPCLK        |                  |       |
| 17  | P0.2 | ICE_CLK / GPIO                |                  | Programming header CLK? |
| 18  | P0.3 | GPIO                          |                  |       |
| 19  | P0.4 | GPIO                          |                  |       |
| 20  | P3.0 | GPIO                          |                  |       |

## Gate Drive Topology

Based on visual inspection:

```
                    VCC_MOTOR
                       │
                  ┌────┴────┐
                  │ High-side│
                  │  P-ch?   │◄── Tab = phase output (U/V/W)
                  │  N-ch?   │
                  └────┬────┘
                       │
     MCU pin ──[R]──┤  Q_HS (small N-FET level shifter)
                    │  (inverts MCU signal to drive high-side gate)
                    ├──[R_pulldown]── GND
                    │
                       ├─────────── Phase output (U / V / W)
                       │
                  ┌────┴────┐
                  │ Low-side │
                  │  N-ch    │
                  └────┬────┘
                       │
     MCU pin ──[R_gate]──┤ Gate
                         ├──[R_pulldown]── GND
                         │
                      GND (via shunt)
```

### Confirm by probing:

- [ ] What package are the 3 main MOSFETs? (e.g., SO-8 dual, SOT-23-6?)
- [ ] Are they complementary pairs (N+P in one package) or dual N-channel?
- [ ] If dual N-ch: the small external N-FET inverts the MCU signal for the high-side (bootstrap-less charge-pump-less gate drive — high-side gate is pulled up by a resistor to VCC, and the small N-FET pulls it low to turn it ON or OFF?)
- [ ] If complementary N+P: MCU drives both gates together through resistor

### Package identification help

Common 4-pin + tab SMT MOSFET packages for this application:
- **SO-8 (half-bridge):** e.g., AO4606 (N+P complementary pair, 30V)
- **SOT-23-6 (dual):** e.g., Si1902DL
- **PMPAK / PDFN-8:** various dual MOSFETs

> Note the part marking on the MOSFET package if readable: _______________

## Three-Phase Bridge

| Phase | MOSFET Package | High-side MCU Pin | Low-side MCU Pin | Confirmed? |
|-------|---------------|-------------------|------------------|------------|
| U     |               |                   |                  | [ ]        |
| V     |               |                   |                  | [ ]        |
| W     |               |                   |                  | [ ]        |

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

- [ ] Shunt resistor location: between bridge ground and battery negative?
- [ ] Shunt resistance value: ___ mΩ (read marking or measure)
- [ ] Amplifier circuit: op-amp? direct to ADC pin?
- [ ] ADC pin for shunt voltage: ___

## Control Inputs / Outputs

| Signal      | Connector Pin | MCU Pin | Type          | Notes |
|-------------|--------------|---------|---------------|-------|
| PWM input   |              |         | Input (5V?)   | Speed control from ESC/main board |
| Direction   |              |         | Input (logic) | May be active-low |
| Enable      |              |         | Input (logic) | |
| LED         |              |         | Output        | |
| VCC (logic) |              |         | Power         | 3.3V or 5V? |
| GND         |              |         | Power         | |

## Programming Header

| Pad  | Signal  | MCU Pin | Confirmed? |
|------|---------|---------|------------|
|      | VDD     | Pin 5   | [ ]        |
|      | GND     | Pin 6   | [ ]        |
|      | ICE_DAT | Pin 4 (P2.0) | [ ]   |
|      | ICE_CLK | Pin 17 (P0.2) | [ ]  |
|      | RESET   |         | [ ]        |

## Power Supply

- [ ] Motor voltage: ___ V (battery pack voltage)
- [ ] Logic voltage: ___ V (MCU VDD — likely 3.3V or 5V from onboard regulator)
- [ ] Regulator type/part: ___ (LDO? SOT-23-5?)

## Probing Tips

1. **Start with power:** Identify VDD (pin 5) and VSS (pin 6). Measure the voltage on VDD with the board powered — this tells you 3.3V vs 5V logic.
2. **Programming header:** Look for 4-5 pads near the MCU. Confirm ICE_DAT (pin 4) and ICE_CLK (pin 17) with continuity.
3. **Gate outputs:** With power off, check continuity from pins 7-11 (right side of MCU, P1.3-P1.7) and pin 1 (P0.5) to the gate resistors on the MOSFETs. These are almost certainly the 6 bridge drive signals.
4. **Hall sensors:** Trace the 3 wires from the motor's hall sensor cable to MCU pins. Likely P1.0-P1.2 (pins 12-14) or P0.x pins.
5. **Current shunt:** Find the sense resistor (very low value, near bridge ground). Trace from there to an ADC-capable pin (P0.6 or P0.7).
6. **Control inputs:** The remaining pins (P0.0, P0.1, P0.3, P0.4, P3.0) handle PWM input, direction, enable, LED, etc.

## Board Photos

> Add photos here or link to them — top side, bottom side, close-up of MCU area.
