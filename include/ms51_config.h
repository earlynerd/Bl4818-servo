/*
 * BL4818-Servo Hardware Configuration
 *
 * Pin assignments, clock settings, and feature toggles for the
 * BL4818 motor driver board (XT48-12 V02).
 *
 * MCU Pin Mapping (from Nuvoton datasheet Figure 4.1-1):
 *
 *   Pin  Port  Key Functions
 *   ───  ────  ──────────────────────────────────────────
 *    1   P0.5  T0 / PWM0_CH2 / ADC_CH4
 *    2   P0.6  UART0_TXD / ADC_CH3
 *    3   P0.7  UART0_RXD / ADC_CH2
 *    4   P2.0  nRESET
 *    5   P3.0  INT0 / OSCIN / ADC_CH1
 *    6   P1.7  INT1 / ADC_CH0
 *    7   VSS   Ground
 *    8   P1.6  UART1_TXD / ICE_DAT / [I2C0_SDA]
 *    9   VDD   Power (2.4–5.5V)
 *   10   P1.5  SPI0_SS / PWM0_CH5
 *   11   P1.4  PWM0_BRAKE / PWM0_CH1 / I2C0_SDA
 *   12   P1.3  I2C0_SCL / [STADC]
 *   13   P1.2  PWM0_CH0 / IC0
 *   14   P1.1  ADC_CH7 / PWM0_CH1 / IC1 / CLKO
 *   15   P1.0  PWM0_CH2 / IC2 / SPI0_CLK
 *   16   P0.0  PWM0_CH3 / IC3 / SPI0_MOSI / T1
 *   17   P0.1  PWM0_CH4 / IC4 / SPI0_MISO
 *   18   P0.2  ICE_CLK / UART1_RXD / [I2C0_SCL]
 *   19   P0.3  ADC_CH6 / PWM0_CH5 / IC5
 *   20   P0.4  ADC_CH5 / PWM0_CH3 / IC3 / STADC
 */
#ifndef MS51_CONFIG_H
#define MS51_CONFIG_H

/* ── System Clock ────────────────────────────────────────────────────────── */
#define FSYS            24000000UL  /* 24 MHz HIRC */

/* ── PWM Configuration ───────────────────────────────────────────────────── */
#define PWM_FREQUENCY   20000       /* 20 kHz switching frequency */
#define PWM_PERIOD      (FSYS / PWM_FREQUENCY)  /* 1200 counts at 24MHz */
#define PWM_DEAD_TIME   24          /* 1us dead time at 24MHz */
#define PWM_MAX_DUTY    (PWM_PERIOD - 2 * PWM_DEAD_TIME)

/* ── Pin Assignments ─────────────────────────────────────────────────────── */

/*
 * Three-phase bridge gate drive outputs — ALL ACTIVE HIGH.
 *
 * Gate drive topology (confirmed by inspection):
 *   Each phase uses a complementary MOSFET pair (P-ch high / N-ch low)
 *   in a single 4-pin + tab package. Tab = phase output (U/V/W).
 *
 *   Low-side (N-ch):  MCU HIGH → gate resistor → FET on.
 *   High-side (P-ch): MCU HIGH → small N-FET on → pulls P-ch gate low → FET on.
 *                     MCU LOW  → small N-FET off → gate pulled up to VCC → FET off.
 *
 *   Both high and low sides are active-high from the MCU's perspective.
 *
 * Pin assignments are PROVISIONAL — update after multimeter probing.
 * PWM-capable pins on right side of IC (pins 10-17) are prime candidates.
 * 6 gate signals needed: 3× high-side + 3× low-side.
 *
 * Best guess based on PWM channel availability:
 *   Phase U: CH0 (P1.2, pin 13) + CH1 (P1.1, pin 14)
 *   Phase V: CH2 (P1.0, pin 15) + CH3 (P0.0, pin 16)
 *   Phase W: CH4 (P0.1, pin 17) + CH5 (P1.5, pin 10)
 *
 * But could be any permutation — VERIFY WITH MULTIMETER.
 */
#define PHASE_U_HI_PIN  P12     /* P1.2 pin 13 - PWM0_CH0 — VERIFY */
#define PHASE_U_LO_PIN  P11     /* P1.1 pin 14 - PWM0_CH1 — VERIFY */
#define PHASE_V_HI_PIN  P10     /* P1.0 pin 15 - PWM0_CH2 — VERIFY */
#define PHASE_V_LO_PIN  P00     /* P0.0 pin 16 - PWM0_CH3 — VERIFY */
#define PHASE_W_HI_PIN  P01     /* P0.1 pin 17 - PWM0_CH4 — VERIFY */
#define PHASE_W_LO_PIN  P15     /* P1.5 pin 10 - PWM0_CH5 — VERIFY */

/* Aliases used by pwm.c (maps A/B/C to U/V/W) */
#define PHASE_A_HI_PIN  PHASE_U_HI_PIN
#define PHASE_A_LO_PIN  PHASE_U_LO_PIN
#define PHASE_B_HI_PIN  PHASE_V_HI_PIN
#define PHASE_B_LO_PIN  PHASE_V_LO_PIN
#define PHASE_C_HI_PIN  PHASE_W_HI_PIN
#define PHASE_C_LO_PIN  PHASE_W_LO_PIN

/* Hardware fault brake pin (directly supported by PWM0 module) */
#define BRAKE_PIN       P14     /* P1.4 pin 11 - PWM0_BRAKE */

/*
 * Hall sensor inputs — VERIFY pin assignments.
 * These need to be on GPIO pins with interrupt capability.
 * Candidates for non-PWM pins: P1.7 (pin 6), P3.0 (pin 5), P1.3 (pin 12),
 * P0.5 (pin 1), P0.3 (pin 19), P0.4 (pin 20).
 *
 * P1.7 has INT1, P3.0 has INT0 — good for edge-triggered hall ISR.
 */
#define HALL_A_PIN      P05     /* P0.5 pin 1  — VERIFY */
#define HALL_B_PIN      P17     /* P1.7 pin 6  — VERIFY */
#define HALL_C_PIN      P30     /* P3.0 pin 5  — VERIFY */

/* Hall sensor reading — must update if pins aren't on same port */
/* If halls are on different ports, we read them individually */
#define HALL_READ_INDIVIDUAL  1  /* Set to 1 if halls are on different ports */

/*
 * ADC channels (from datasheet):
 *   CH0 = P1.7 (pin 6)    CH4 = P0.5 (pin 1)
 *   CH1 = P3.0 (pin 5)    CH5 = P0.4 (pin 20)
 *   CH2 = P0.7 (pin 3)    CH6 = P0.3 (pin 19)
 *   CH3 = P0.6 (pin 2)    CH7 = P1.1 (pin 14)
 *
 * Current shunt ADC pin — VERIFY which channel is used.
 * UART0_TX (P0.6) = ADC_CH3, UART0_RX (P0.7) = ADC_CH2.
 * If UART is on P0.6/P0.7, current sense must be elsewhere.
 */
#define ADC_CH_CURRENT  6       /* ADC_CH6 = P0.3 (pin 19) — VERIFY */
#define ADC_CH_VOLTAGE  5       /* ADC_CH5 = P0.4 (pin 20) — VERIFY */

/* Control inputs — VERIFY */
#define LED_PIN         P04     /* P0.4 pin 20 — VERIFY (or may be ADC) */

/* UART pins (directly from datasheet) */
#define UART_TX_PIN     P06     /* P0.6 pin 2  - UART0_TXD */
#define UART_RX_PIN     P07     /* P0.7 pin 3  - UART0_RXD */

/* Programming header (from board silkscreen: P,R,S,+,-) */
/* P = ICE_DAT = P1.6 (pin 8)  */
/* S = ICE_CLK = P0.2 (pin 18) */
/* R = nRESET  = P2.0 (pin 4)  */
/* + = VDD     = pin 9          */
/* - = VSS     = pin 7          */

/* ── Motor Parameters ────────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    7       /* BL4818 = 14 poles / 7 pole pairs */
#define HALL_STATES_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 42 states/mech rev */

/* ── Current Sensing ─────────────────────────────────────────────────────── */
#define SHUNT_RESISTANCE_MOHM   50      /* 50 milliohm shunt — VERIFY */
#define CURRENT_AMP_GAIN        10      /* Op-amp gain — VERIFY (may be direct) */
#define ADC_VREF_MV             3300    /* ADC reference = VDD — VERIFY 3.3V or 5V */
#define ADC_RESOLUTION          4096    /* 12-bit ADC */
#define CURRENT_LIMIT_MA        5000    /* 5A overcurrent threshold */
#define CURRENT_WARN_MA         3000    /* Soft current limit for regulation */

/* Convert ADC reading to milliamps:
 * V_shunt = I * R_shunt
 * V_adc = V_shunt * GAIN
 * ADC_val = V_adc / V_ref * 4096
 * I_mA = ADC_val * V_ref / (4096 * GAIN * R_shunt_ohm)
 *       = ADC_val * 3300 / (4096 * 10 * 0.050)
 *       = ADC_val * 3300 / 2048
 *       = ADC_val * 825 / 512
 *
 * NOTE: Recalculate after confirming GAIN, R_shunt, and VDD voltage.
 */
#define ADC_TO_MA_NUM       825
#define ADC_TO_MA_DEN       512

/* ── PID Defaults ────────────────────────────────────────────────────────── */
#define PID_POS_KP      100
#define PID_POS_KI      5
#define PID_POS_KD      50
#define PID_VEL_KP      50
#define PID_VEL_KI      10
#define PID_VEL_KD      5
#define PID_SCALE       256     /* Fixed-point scale factor (Q8) */

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ     1000    /* 1 kHz control loop */
#define STALL_TIMEOUT_MS    500     /* Stall detection timeout */
#define WATCHDOG_TIMEOUT_MS 1000    /* Watchdog period */
#define COMM_TIMEOUT_MS     2000    /* Serial command timeout */

/* ── Feature Toggles ─────────────────────────────────────────────────────── */
#define FEATURE_ENCODER     1       /* Enable encoder input */
#define FEATURE_UART        1       /* Enable UART command interface */
#define FEATURE_PID         1       /* Enable PID loops */
#define FEATURE_FLASH_SAVE  1       /* Enable parameter save to flash */
#define FEATURE_LED         1       /* Enable LED status indication */

/* ── Encoder ─────────────────────────────────────────────────────────────── */
#define ENCODER_CPR         1000    /* Counts per revolution (set per encoder) */

/* ── Operating Modes ─────────────────────────────────────────────────────── */
#define MODE_OPEN_LOOP      0       /* Direct PWM duty control */
#define MODE_VELOCITY       1       /* Velocity PID */
#define MODE_POSITION       2       /* Position PID */
#define MODE_TORQUE         3       /* Current / torque control */

#endif /* MS51_CONFIG_H */
