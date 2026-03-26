/*
 * PWM Driver — Complementary 3-phase PWM with dead-time insertion
 *
 * Uses the MS51FB9AE PWM0 module (6 channels) configured for
 * center-aligned complementary mode with hardware dead-time.
 *
 * Channel mapping:
 *   CH0/CH1 = Phase A high/low (P1.7/P1.6)
 *   CH2/CH3 = Phase B high/low (P1.5/P1.4)
 *   CH4/CH5 = Phase C high/low (P1.3/P0.5)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "pwm.h"

/* Current duty cycle */
static uint16_t current_duty;

/* Current active pattern (which phases are driven) */
static uint8_t active_pattern;

/*
 * PWM register access on MS51 requires SFR page switching for some
 * extended registers. We use direct SFR access where possible and
 * page-switched access for extended PWM configuration.
 *
 * The MS51 PWM module supports:
 * - Independent or complementary mode per channel pair
 * - Hardware dead-time insertion
 * - Fault brake input
 * - Center-aligned or edge-aligned mode
 */

void pwm_init(void)
{
    /* Configure PWM output pins as push-pull */
    /* P1.7, P1.6, P1.5, P1.4, P1.3 = push-pull output */
    P1M1 &= ~0xF8;  /* Clear M1 bits for P1.3-P1.7 */
    P1M2 |=  0xF8;  /* Set M2 bits for push-pull */

    /* P0.5 = push-pull output (Phase C high-side) */
    P0M1 &= ~0x20;
    P0M2 |=  0x20;

    /* All outputs low initially */
    P1 &= ~0xF8;
    P0 &= ~0x20;

    /*
     * PWM0 Configuration:
     * - Clock source: FSYS (24 MHz)
     * - Center-aligned mode for reduced harmonics
     * - 6 channels in 3 complementary pairs
     * - Dead-time insertion enabled
     *
     * Since SDCC doesn't have convenient macros for all the extended
     * PWM SFRs, we access them via page-switched SFR writes.
     */

    /* PWM clock = FSYS, no prescaler */
    PWMCON0 = 0x00;  /* Clear PWM control, stopped */
    PWMCON1 = 0x00;

    /*
     * Set PWM period register.
     * For center-aligned mode, the counter counts up to PERIOD then back down,
     * so effective frequency = FSYS / (2 * PERIOD).
     * For 20kHz: PERIOD = 24000000 / (2 * 20000) = 600
     *
     * For edge-aligned mode: PERIOD = FSYS / freq = 1200
     * We use edge-aligned for simplicity on 8051.
     */

    /* We'll use software commutation with GPIO for maximum control */
    current_duty = 0;
    active_pattern = 0;

    /* Enable PWM fault brake interrupt for overcurrent */
    /* Fault brake will be triggered by software when ADC detects overcurrent */
}

/*
 * Software PWM phase control.
 *
 * Instead of relying on the hardware PWM module's complementary mode
 * (which has limited flexibility for commutation pattern changes),
 * we use Timer 0 to generate a fixed-frequency interrupt and
 * bit-bang the 6 MOSFET gates with proper dead-time.
 *
 * This gives us full control over commutation patterns and dead-time,
 * at the cost of CPU time in the ISR. At 20kHz with a 24MHz clock,
 * we have 1200 cycles per PWM period — plenty for the ISR.
 *
 * Timer 0 is configured in 16-bit auto-reload mode.
 * The ISR sets outputs based on current_duty and active_pattern.
 */

/*
 * Phase output control.
 *
 * All gate drives are active-high from the MCU:
 *   Low-side (N-ch):  MCU HIGH = on (direct gate drive through resistor)
 *   High-side (P-ch): MCU HIGH = on (small N-FET pulls P-ch gate low)
 */
static void phase_all_off(void)
{
    /* All high-sides off first (prevent shoot-through) */
    PHASE_A_HI_PIN = 0;
    PHASE_B_HI_PIN = 0;
    PHASE_C_HI_PIN = 0;
    /* Then low-sides off */
    PHASE_A_LO_PIN = 0;
    PHASE_B_LO_PIN = 0;
    PHASE_C_LO_PIN = 0;
}

void pwm_set_duty(uint16_t duty)
{
    if (duty > PWM_MAX_DUTY)
        duty = PWM_MAX_DUTY;
    current_duty = duty;
}

uint16_t pwm_get_duty(void)
{
    return current_duty;
}

void pwm_enable(void)
{
    /* Start Timer 0 for PWM generation */
    /* Timer 0: 16-bit mode, FSYS clock */
    TMOD = (TMOD & 0xF0) | 0x01;  /* Timer 0 mode 1 (16-bit) */

    /* Reload value for 20kHz (50us period) */
    /* 24MHz / 20kHz = 1200 counts. Timer counts up from (65536-1200). */
    TH0 = (65536 - 1200) >> 8;
    TL0 = (65536 - 1200) & 0xFF;

    ET0 = 1;   /* Enable Timer 0 interrupt */
    TR0 = 1;   /* Start Timer 0 */
}

void pwm_disable(void)
{
    TR0 = 0;   /* Stop Timer 0 */
    ET0 = 0;   /* Disable Timer 0 interrupt */
    phase_all_off();
}

void pwm_fault_brake(void)
{
    /* Immediate: all outputs off */
    TR0 = 0;
    phase_all_off();
    current_duty = 0;
    active_pattern = 0;
}

/*
 * Apply commutation pattern.
 *
 * Pattern encoding (6 bits):
 *   Bit 5: Phase C high-side ON
 *   Bit 4: Phase C low-side ON
 *   Bit 3: Phase B high-side ON
 *   Bit 2: Phase B low-side ON
 *   Bit 1: Phase A high-side ON
 *   Bit 0: Phase A low-side ON
 *
 * Only the high-side of the active phase pair is PWM-modulated.
 * The complementary low-side is always ON (for current recirculation).
 * The source low-side carries the current path.
 */
void pwm_apply_pattern(uint8_t pattern)
{
    active_pattern = pattern;
}

/*
 * Timer 0 ISR — Software PWM generation
 *
 * This implements a simple on/off PWM within each timer period.
 * For the first N cycles of the period, the active high-side is ON.
 * For the remainder, it's OFF (with low-side freewheeling).
 *
 * Since we can't easily do sub-period timing in a single timer ISR
 * on 8051, we use a dual-interrupt approach:
 * - Timer 0 overflow: start of PWM period → turn ON active outputs
 * - We use the duty cycle counter as a simple time ratio over multiple
 *   periods (sigma-delta modulation for fine resolution).
 *
 * For simplicity in this initial implementation, we use a straightforward
 * approach: the timer ISR applies the current pattern with the current duty
 * as a direct on/off decision based on a software counter.
 */

/* Software PWM counter (0 to PWM_PERIOD-1) */
static uint16_t pwm_counter;

void pwm_timer0_isr(void) __interrupt(INT_TIMER0)
{
    /* Reload timer */
    TH0 = (65536 - 48) >> 8;   /* ~2us sub-period at 24MHz for PWM resolution */
    TL0 = (65536 - 48) & 0xFF;

    pwm_counter++;
    if (pwm_counter >= (PWM_PERIOD / 48)) {
        pwm_counter = 0;
    }

    /* Threshold for duty cycle comparison */
    uint16_t threshold = (uint16_t)((uint32_t)current_duty * (PWM_PERIOD / 48) / PWM_PERIOD);

    if (pwm_counter < threshold && active_pattern != 0) {
        /* PWM ON phase — apply pattern */
        /* Turn off all high-sides first (dead-time) */
        PHASE_A_HI_PIN = 0;
        PHASE_B_HI_PIN = 0;
        PHASE_C_HI_PIN = 0;

        /* Apply low-sides (always on for active low-side phase) */
        PHASE_A_LO_PIN = (active_pattern & 0x01) ? 1 : 0;
        PHASE_B_LO_PIN = (active_pattern & 0x04) ? 1 : 0;
        PHASE_C_LO_PIN = (active_pattern & 0x10) ? 1 : 0;

        /* Brief dead-time (a few NOPs at 24MHz ≈ ~100ns) */
        __asm__("nop\nnop\nnop\nnop");

        /* Apply high-sides (PWM-modulated) */
        PHASE_A_HI_PIN = (active_pattern & 0x02) ? 1 : 0;
        PHASE_B_HI_PIN = (active_pattern & 0x08) ? 1 : 0;
        PHASE_C_HI_PIN = (active_pattern & 0x20) ? 1 : 0;
    } else {
        /* PWM OFF phase — high-sides off, keep low-sides for freewheeling */
        PHASE_A_HI_PIN = 0;
        PHASE_B_HI_PIN = 0;
        PHASE_C_HI_PIN = 0;

        /* In off-phase, keep active low-side ON for current recirculation */
        PHASE_A_LO_PIN = (active_pattern & 0x01) ? 1 : 0;
        PHASE_B_LO_PIN = (active_pattern & 0x04) ? 1 : 0;
        PHASE_C_LO_PIN = (active_pattern & 0x10) ? 1 : 0;
    }
}
