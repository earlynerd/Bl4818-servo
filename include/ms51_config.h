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

#ifndef FLASH_TOTAL_SIZE
#define FLASH_TOTAL_SIZE    16384U
#endif

#ifndef LDROM_SIZE
#define LDROM_SIZE          0U
#endif

#ifndef APROM_SIZE
#define APROM_SIZE          (FLASH_TOTAL_SIZE - LDROM_SIZE)
#endif

#define FLASH_PAGE_SIZE     128U
#define PARAM_PAGE_ADDR     (APROM_SIZE - FLASH_PAGE_SIZE)

#if FLASH_TOTAL_SIZE != 16384U
#error "This firmware currently targets the 16 KB MS51FB9AE flash layout only."
#endif

#if (APROM_SIZE + LDROM_SIZE) != FLASH_TOTAL_SIZE
#error "APROM_SIZE + LDROM_SIZE must match FLASH_TOTAL_SIZE."
#endif

#if (APROM_SIZE < FLASH_PAGE_SIZE) || ((APROM_SIZE % FLASH_PAGE_SIZE) != 0)
#error "APROM_SIZE must be page-aligned and at least one flash page."
#endif

/* ── System Clock ────────────────────────────────────────────────────────── */
#define FSYS            24000000UL  /* 24 MHz HIRC selected at startup */
#define HIRC_TRIM_OFFSET     (-2)  /* Empirically tuned: -2 → 499.7 Hz tach (24 MHz ±0.06%) */

/* ── PWM Configuration ───────────────────────────────────────────────────── */
#define PWM_FREQUENCY   20000       /* 20 kHz switching frequency */
#define PWM_PERIOD      (FSYS / PWM_FREQUENCY)
#define PWM_MAX_DUTY    (PWM_PERIOD - 1)  /* Dead-time disabled; masks prevent shoot-through */

/* Hall-to-commutation alignment, in 60 electrical degree steps (0-5). */
#define COMMUTATION_OFFSET  1

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
#define UART_BAUD       250000UL

/* LED — no dedicated LED pin identified; stock board may not have one */
/* #define LED_PIN      P13 */  /* Could use spare pin 12 if LED added */

/* ── Motor Parameters ────────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    5       /* BL4818 = 10 poles / 5 pole pairs */
#define HALL_STATES_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 30 states/mech rev */

/* ── Current Sensing — CONFIRMED ──────────────────────────────────────── */
/* Shunt: 2512 package, marked "R020" = 20 milliohm (0.020Ω)             */
/* Direct to ADC (no amplifier). VDD = 5.0V reference.                    */
#define SHUNT_RESISTANCE_MOHM   20      /* 20 milliohm (R020 marking) */
#define CURRENT_AMP_GAIN        1       /* No amplifier — direct to ADC */
#define ADC_VREF_MV             5000    /* VDD = 5.0V confirmed */
#define ADC_RESOLUTION          4096    /* 12-bit ADC */
#define CURRENT_LIMIT_MA        5000    /* 5A overcurrent threshold */
#define CURRENT_WARN_MA         3000    /* Soft current limit for regulation */
#define DEFAULT_TORQUE_LIMIT_MA CURRENT_WARN_MA  /* Safer out-of-box soft limit */
#define BRAKE_COAST_CURRENT_MA  2500    /* Exit braking torque and coast above this current */

/*
 * Convert ADC reading to milliamps (no amplifier, 20mΩ shunt):
 *   V_shunt = I × R_shunt = I × 0.020
 *   ADC_val = V_shunt / 5.0 × 4096
 *   I_mA = ADC_val × 5000 / (4096 × 0.020)
 *        = ADC_val × 5000 / 81.92
 *        ≈ ADC_val × 5000 / 82
 *        ≈ ADC_val × 2500 / 41
 *
 * Resolution: ~61mA per ADC LSB
 * 1A  →  20mV → ADC ≈  16
 * 3A  →  60mV → ADC ≈  49
 * 5A  → 100mV → ADC ≈  82
 * 10A → 200mV → ADC ≈ 164
 */
#define ADC_TO_MA_NUM       2500
#define ADC_TO_MA_DEN       41

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

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ     1000    /* 1 kHz control loop */
#define FEATURE_WATCHDOG    1U      /* Reset if the main loop stops reaching the control tick */
#define WDT_PRESCALER_BITS  7U      /* Longest watchdog interval */

#if WDT_PRESCALER_BITS > 7U
#error "WDT_PRESCALER_BITS must be in the range 0..7."
#endif

/* ── Legacy Local Inputs ─────────────────────────────────────────────────── */
#define FEATURE_LOCAL_PWM_INPUT    1       /* PWM+DIR active until serial enumeration */
#define LOCAL_PWM_ACTIVE_LOW       1U      /* Pull-up on P0.4: idle/high = zero torque */
#define LOCAL_PWM_DC_FULLSCALE_MS  20U     /* Continuous active level -> full local command */
#define LOCAL_PWM_RAMP_UP_MS       100U    /* Local command rises from 0 to full over this time */
#define LOCAL_FAULT_RETRY_DELAY_MS 250U    /* Wait before each local auto-retry */
#define LOCAL_FAULT_RETRY_MAX      3U      /* Retries per nonzero local command episode */
#define LOCAL_PWM_TIMEOUT_MS       50U     /* Stop if PWM edges disappear */
#define LOCAL_PWM_MIN_DUTY_COUNTS  4U      /* Treat near-zero input as stop */
#define LOCAL_PWM_DIR_INVERT       0U      /* 0: DIR high = forward */

/* ── Feature Toggles ─────────────────────────────────────────────────────── */
#define FEATURE_UART        1       /* Enable UART command interface */
#define FEATURE_TACH_DEBUG  1       /* Toggle tach pin from control-loop timing */
#define TACH_DEBUG_DIVIDER  1U      /* Square wave = CONTROL_LOOP_HZ / (2 * divider) */

#endif /* MS51_CONFIG_H */
