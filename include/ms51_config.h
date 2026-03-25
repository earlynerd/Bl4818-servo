/*
 * BL4818-Servo Hardware Configuration
 *
 * Pin assignments, clock settings, and feature toggles for the
 * BL4818 motor driver board.
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
 * Three-phase bridge gate drive outputs.
 *
 * Gate drive topology (from board inspection):
 *   Low-side:  MCU pin ──[R_gate]──> N-FET gate, with pulldown to GND.
 *              Active HIGH = low-side FET on.
 *   High-side: MCU pin ──[R]──> small N-FET gate (level shifter/inverter).
 *              The small N-FET pulls the high-side gate low (or drives it).
 *              Polarity TBD — may be ACTIVE LOW at MCU pin (inverted by N-FET).
 *
 * TODO: Confirm polarity after probing. If high-side is inverted,
 *       set PHASE_x_HI_INVERT to 1 and the PWM driver will invert.
 *
 * Pin assignments are PROVISIONAL — update after multimeter probing.
 * Gate pins are expected on pins 7-11 (P1.3-P1.7) and pin 1 (P0.5).
 */
#define PHASE_A_HI_PIN  P17     /* P1.7 - PWM CH0 — VERIFY */
#define PHASE_A_LO_PIN  P16     /* P1.6 - PWM CH1 — VERIFY */
#define PHASE_B_HI_PIN  P15     /* P1.5 - PWM CH2 — VERIFY */
#define PHASE_B_LO_PIN  P14     /* P1.4 - PWM CH3 — VERIFY */
#define PHASE_C_HI_PIN  P13     /* P1.3 - PWM CH4 — VERIFY */
#define PHASE_C_LO_PIN  P05     /* P0.5 - PWM CH5 — VERIFY */

/* Set to 1 if high-side MCU output is inverted by external N-FET driver */
#define PHASE_HI_INVERTED  0    /* TODO: determine after probing */

/* Hall sensor inputs */
#define HALL_A_PIN      P12     /* P1.2 */
#define HALL_B_PIN      P11     /* P1.1 */
#define HALL_C_PIN      P10     /* P1.0 */

/* Hall sensor port and mask */
#define HALL_PORT       P1
#define HALL_MASK       0x07    /* P1.0, P1.1, P1.2 */
#define HALL_SHIFT      0       /* Bits 0-2 of P1 */

/* ADC channels */
#define ADC_CH_CURRENT  6       /* P0.6 - current shunt */
#define ADC_CH_VOLTAGE  7       /* P0.7 - battery voltage divider */

/* Control inputs */
#define DIR_PIN         P00     /* P0.0 - direction input */
#define PWM_IN_PIN      P01     /* P0.1 - PWM speed input */
#define ENABLE_PIN      P03     /* P0.3 - enable/brake input */
#define LED_PIN         P04     /* P0.4 - status LED */

/* Encoder inputs (optional add-on) */
#define ENC_A_PIN       P3      /* P3.0 - encoder channel A */
#define ENC_B_PIN       P01     /* P0.1 - encoder channel B (replaces PWM in) */

/* UART pins */
#define UART_TX_PIN     P06     /* P0.6 - shared with ADC */
#define UART_RX_PIN     P07     /* P0.7 - shared with ADC */

/* ── Motor Parameters ────────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    7       /* BL4818 = 14 poles / 7 pole pairs */
#define HALL_STATES_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 42 states/mech rev */

/* ── Current Sensing ─────────────────────────────────────────────────────── */
#define SHUNT_RESISTANCE_MOHM   50      /* 50 milliohm shunt */
#define CURRENT_AMP_GAIN        10      /* Op-amp gain on shunt voltage */
#define ADC_VREF_MV             3300    /* ADC reference = VDD = 3.3V */
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
