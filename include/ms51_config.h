/*
 * BL4818-Servo Hardware Configuration
 *
 * Pin assignments, clock settings, and feature toggles for the
 * BL4818 motor driver board (XT48-12 V02).
 *
 * *** PIN ASSIGNMENTS CONFIRMED BY MULTIMETER ***
 *
 * MCU Pin Mapping (Nuvoton datasheet + board probing):
 *
 *   Pin  Port  Board Connection
 *   ───  ────  ──────────────────────────────────────────
 *    1   P0.5  Tach output (N-FET inverted)
 *    2   P0.6  Current shunt ADC (direct, no amp) = ADC_CH3
 *    3   P0.7  Battery voltage divider (10k/10k) = ADC_CH2
 *    4   P2.0  nRESET / prog header "R"
 *    5   P3.0  Hall sensor 3 (INT0)
 *    6   P1.7  Hall sensor 2 (INT1)
 *    7   VSS   Ground
 *    8   P1.6  ICE_DAT / prog header "P"
 *    9   VDD   5V power
 *   10   P1.5  Hall sensor 1
 *   11   P1.4  Direction input
 *   12   P1.3  Possibly NC (available for expansion)
 *   13   P1.2  Phase U low-side gate  (PWM0_CH0)
 *   14   P1.1  Phase U high-side gate (PWM0_CH1)
 *   15   P1.0  Phase V high-side gate (PWM0_CH2)
 *   16   P0.0  Phase V low-side gate  (PWM0_CH3)
 *   17   P0.1  Phase W low-side gate  (PWM0_CH4)
 *   18   P0.2  ICE_CLK / prog header "S"
 *   19   P0.3  Phase W high-side gate (PWM0_CH5)
 *   20   P0.4  PWM speed input (unpop RC)
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

/* ── Pin Assignments — CONFIRMED ─────────────────────────────────────────── */

/*
 * Three-phase bridge gate drive outputs — ALL ACTIVE HIGH.
 *
 * Gate drive topology (confirmed):
 *   Each phase uses a complementary MOSFET pair (P-ch high / N-ch low)
 *   in a single 4-pin + tab package. Tab = phase output (U/V/W).
 *
 *   Low-side (N-ch):  MCU HIGH → gate resistor → FET on.
 *   High-side (P-ch): MCU HIGH → small N-FET on → pulls P-ch gate low → FET on.
 *
 * All 6 gate pins map to hardware PWM channels:
 *   Phase U: HI=P1.1 (CH1), LO=P1.2 (CH0)
 *   Phase V: HI=P1.0 (CH2), LO=P0.0 (CH3)
 *   Phase W: HI=P0.3 (CH5), LO=P0.1 (CH4)
 */
#define PHASE_U_HI_PIN  P11     /* P1.1 pin 14 - PWM0_CH1 */
#define PHASE_U_LO_PIN  P12     /* P1.2 pin 13 - PWM0_CH0 */
#define PHASE_V_HI_PIN  P10     /* P1.0 pin 15 - PWM0_CH2 */
#define PHASE_V_LO_PIN  P00     /* P0.0 pin 16 - PWM0_CH3 */
#define PHASE_W_HI_PIN  P03     /* P0.3 pin 19 - PWM0_CH5 */
#define PHASE_W_LO_PIN  P01     /* P0.1 pin 17 - PWM0_CH4 */

/* Aliases used by pwm.c (A/B/C → U/V/W) */
#define PHASE_A_HI_PIN  PHASE_U_HI_PIN
#define PHASE_A_LO_PIN  PHASE_U_LO_PIN
#define PHASE_B_HI_PIN  PHASE_V_HI_PIN
#define PHASE_B_LO_PIN  PHASE_V_LO_PIN
#define PHASE_C_HI_PIN  PHASE_W_HI_PIN
#define PHASE_C_LO_PIN  PHASE_W_LO_PIN

/* PWM channel numbers (for hardware PWM module, future use) */
#define PWM_CH_U_HI     1   /* PWM0_CH1 = P1.1 */
#define PWM_CH_U_LO     0   /* PWM0_CH0 = P1.2 */
#define PWM_CH_V_HI     2   /* PWM0_CH2 = P1.0 */
#define PWM_CH_V_LO     3   /* PWM0_CH3 = P0.0 */
#define PWM_CH_W_HI     5   /* PWM0_CH5 = P0.3 */
#define PWM_CH_W_LO     4   /* PWM0_CH4 = P0.1 */

/*
 * Hall sensor inputs (confirmed):
 *   Hall 1 = P1.5 (pin 10)
 *   Hall 2 = P1.7 (pin 6)  — has INT1
 *   Hall 3 = P3.0 (pin 5)  — has INT0
 *
 * Halls are on different ports — read individually.
 */
#define HALL_1_PIN      P15     /* P1.5 pin 10 - Hall sensor 1 */
#define HALL_2_PIN      P17     /* P1.7 pin 6  - Hall sensor 2 (INT1) */
#define HALL_3_PIN      P30     /* P3.0 pin 5  - Hall sensor 3 (INT0) */

/* Legacy aliases */
#define HALL_A_PIN      HALL_1_PIN
#define HALL_B_PIN      HALL_2_PIN
#define HALL_C_PIN      HALL_3_PIN

/*
 * ADC channels (confirmed):
 *   Current shunt: P0.6 (pin 2) = ADC_CH3, direct (no amplifier)
 *   Battery voltage: P0.7 (pin 3) = ADC_CH2, 10k/10k divider
 *   VDD = 5.0V (ADC reference)
 */
#define ADC_CH_CURRENT  3       /* ADC_CH3 = P0.6 (pin 2) — direct shunt */
#define ADC_CH_VOLTAGE  2       /* ADC_CH2 = P0.7 (pin 3) — 10k/10k divider */

/* Control inputs (confirmed) */
#define DIR_PIN         P14     /* P1.4 pin 11 - Direction input */
#define PWM_IN_PIN      P04     /* P0.4 pin 20 - PWM speed input (unpop RC) */
#define TACH_PIN        P05     /* P0.5 pin 1  - Tach output (N-FET inverted) */

/* Possibly NC / available for expansion */
#define SPARE_PIN       P13     /* P1.3 pin 12 - NC on stock board */

/* UART0 pins (directly from datasheet — shared with ADC) */
#define UART_TX_PIN     P06     /* P0.6 pin 2  - UART0_TXD / ADC_CH3 */
#define UART_RX_PIN     P07     /* P0.7 pin 3  - UART0_RXD / ADC_CH2 */

/* UART1 pins (on programming header — alternative serial port) */
#define UART1_TX_PIN    P16     /* P1.6 pin 8  - UART1_TXD / prog "P" */
#define UART1_RX_PIN    P02     /* P0.2 pin 18 - UART1_RXD / prog "S" */

/* LED — no dedicated LED pin identified; stock board may not have one */
/* #define LED_PIN      P13 */  /* Could use spare pin 12 if LED added */

/* ── Motor Parameters ────────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    7       /* BL4818 = 14 poles / 7 pole pairs */
#define HALL_STATES_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 42 states/mech rev */

/* ── Current Sensing — CONFIRMED: Direct shunt, no amplifier ─────────── */
#define SHUNT_RESISTANCE_MOHM   50      /* 50 milliohm — TODO: verify value */
#define CURRENT_AMP_GAIN        1       /* No amplifier — direct to ADC */
#define ADC_VREF_MV             5000    /* VDD = 5.0V confirmed */
#define ADC_RESOLUTION          4096    /* 12-bit ADC */
#define CURRENT_LIMIT_MA        5000    /* 5A overcurrent threshold */
#define CURRENT_WARN_MA         3000    /* Soft current limit for regulation */

/*
 * Convert ADC reading to milliamps (no amplifier):
 *   V_shunt = I × R_shunt
 *   ADC_val = V_shunt / V_ref × 4096
 *   I_mA = ADC_val × V_ref_mV / (4096 × R_shunt_ohm)
 *        = ADC_val × 5000 / (4096 × 0.050)
 *        = ADC_val × 5000 / 204.8
 *        ≈ ADC_val × 625 / 26    (simplified)
 *
 * Resolution: ~24mA per ADC LSB (with 50mΩ shunt, no amp)
 * Full scale: 5V / 0.050Ω = 100A (but FETs limit well before that)
 * 5A → 250mV → ADC ≈ 205
 *
 * NOTE: Update SHUNT_RESISTANCE_MOHM after reading the actual shunt value.
 */
#define ADC_TO_MA_NUM       625
#define ADC_TO_MA_DEN       26

/* ── Battery Voltage — CONFIRMED: 10k/10k divider ───────────────────── */
/*
 * V_battery = ADC_val / 4096 × 5.0 × 2
 *           = ADC_val × 10000 / 4096
 *           ≈ ADC_val × 625 / 256  (in millivolts)
 *
 * NOTE: At 12V nominal, the divider outputs 6V which exceeds the 5V
 * ADC reference. Useful range is only below ~10V (low battery detection).
 */
#define VDIV_RATIO_NUM      2       /* 10k/10k = 2:1 divider */
#define VDIV_RATIO_DEN      1

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
#define FEATURE_ENCODER     0       /* Disabled until encoder wired up */
#define FEATURE_UART        1       /* Enable UART command interface */
#define FEATURE_PID         1       /* Enable PID loops */
#define FEATURE_FLASH_SAVE  1       /* Enable parameter save to flash */
#define FEATURE_LED         0       /* No dedicated LED on stock board */

/* ── Encoder ─────────────────────────────────────────────────────────────── */
/* When adding encoder: A→P1.3 (pin 12), B→P0.4 (pin 20) */
#define ENC_A_PIN       P13     /* P1.3 pin 12 - repurpose NC pin */
#define ENC_B_PIN       P04     /* P0.4 pin 20 - repurpose PWM input */
#define ENCODER_CPR     1000    /* Counts per revolution (set per encoder) */

/* ── Operating Modes ─────────────────────────────────────────────────────── */
#define MODE_OPEN_LOOP      0       /* Direct PWM duty control */
#define MODE_VELOCITY       1       /* Velocity PID */
#define MODE_POSITION       2       /* Position PID */
#define MODE_TORQUE         3       /* Current / torque control */

#endif /* MS51_CONFIG_H */
