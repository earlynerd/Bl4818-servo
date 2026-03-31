/*
 * PWM Driver — Low-Side-Only Chopping
 *
 * Uses the MS51FB9AE PWM0 module in complementary mode (PWMMOD=01).
 * Dead-time is disabled — the commutation masks prevent shoot-through.
 *
 * Complementary pairing (even=primary, odd=complement):
 *   CH0/CH1 pair: CH0=P1.2 (Phase U low), CH1=P1.1 (Phase U high)
 *   CH2/CH3 pair: CH2=P1.0 (Phase V high), CH3=P0.0 (Phase V low)
 *   CH4/CH5 pair: CH4=P0.1 (Phase W low), CH5=P0.3 (Phase W high)
 *
 * Only low-side FETs switch at PWM rate (20 kHz).  High-side FETs are
 * held DC ON or OFF by the commutation mask registers (PMEN/PMD) and
 * only change state at commutation transitions.
 *
 * This matches the gate drive hardware: low-side N-FETs are driven
 * directly through 100 ohm (fast on/off), while high-side P-FETs
 * have only a 1.5k pullup for turn-off (too slow for 20 kHz switching).
 *
 * Duty register setup for low-side ON time control:
 *   Phase U (CH0 primary = low-side):  duty_reg = duty  (direct)
 *   Phase V (CH2 primary = high-side): duty_reg = PERIOD - duty
 *       (inverted so CH3 complement ON time = duty)
 *   Phase W (CH4 primary = low-side):  duty_reg = duty  (direct)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "pwm.h"

static uint16_t current_duty;

void pwm_init(void)
{
    /*
     * ── Pin Configuration ────────────────────────────────────────────
     * Configure all 6 gate drive pins as push-pull output.
     * All active-high: MCU HIGH = FET ON (both high-side and low-side).
     */

    /* Phase U: P1.1 (CH1, high), P1.2 (CH0, low) → push-pull */
    P1M1 &= ~0x06;   /* Clear M1 for P1.1, P1.2 */
    P1M2 |=  0x06;   /* Set M2 for push-pull */

    /* Phase V high: P1.0 (CH2) → push-pull */
    P1M1 &= ~0x01;   /* Clear M1 for P1.0 */
    P1M2 |=  0x01;   /* Set M2 for push-pull */

    /* Phase V low: P0.0 (CH3) → push-pull */
    P0M1 &= ~0x01;
    P0M2 |=  0x01;

    /* Phase W: P0.1 (CH4, low), P0.3 (CH5, high) → push-pull */
    P0M1 &= ~0x0A;   /* Clear M1 for P0.1, P0.3 */
    P0M2 |=  0x0A;   /* Set M2 for push-pull */

    /*
     * Drive the six gate pins with the fastest edge rate the MS51 GPIO can
     * provide. These pins feed the MOSFET gate-drive network directly and
     * benefit from a stronger digital edge than the default slew setting.
     */
    SFR_PAGE1();
    P1SR |= 0x07;    /* P1.0, P1.1, P1.2 -> high-speed slew */
    P0SR |= 0x0B;    /* P0.0, P0.1, P0.3 -> high-speed slew */
    SFR_PAGE0();

    /* All outputs LOW initially (all FETs off) */
    P11 = 0; P12 = 0;    /* Phase U */
    P10 = 0; P00 = 0;    /* Phase V */
    P01 = 0; P03 = 0;    /* Phase W */

    /*
     * ── PWM Module Configuration ─────────────────────────────────────
     * Stop PWM before configuring.
     */
    PWMCON0 = 0x00;       /* PWMRUN=0, clear all flags */

    /* PWM clock source: FSYS directly (24 MHz) */
    CKCON &= ~0x40;       /* PWMCKS=0: PWM clock = FSYS */

    /*
     * PWMCON1: mode and prescaler
     *   Bit 7:6 = PWMMOD = 01 (complementary mode)
     *   Bit 5   = GP = 0 (no group mode — independent duty per pair)
     *   Bit 4   = PWMTYP = 0 (edge-aligned)
     *   Bit 3   = FBINEN = 0 (no external fault brake pin)
     *   Bit 2:0 = PWMDIV = 000 (divider = 1)
     */
    PWMCON1 = 0x40;

    /*
     * ── Period ───────────────────────────────────────────────────────
     * Edge-aligned: freq = FSYS / PERIOD
     * 24 MHz / 1200 = 20 kHz
     */
    PWMPH = (PWM_PERIOD >> 8) & 0xFF;
    PWMPL = PWM_PERIOD & 0xFF;

    /*
     * ── Initial Duty = 0% ───────────────────────────────────────────
     * Phase U (CH0 primary = low-side): direct, 0 = low-side OFF
     * Phase V (CH2 primary = high-side): inverted, PERIOD = CH3 OFF
     * Phase W (CH4 primary = low-side): direct, 0 = low-side OFF
     */
    PWM0H = 0x00;
    PWM0L = 0x00;
    PWM2H = PWMPH;
    PWM2L = PWMPL;

    /* CH4/CH5 are on SFR page 1 */
    SFR_PAGE1();
    PWM4H = 0x00;
    PWM4L = 0x00;
    SFR_PAGE0();

    /*
     * ── Polarity ─────────────────────────────────────────────────────
     * All channels active-high (PNP=0): MCU HIGH = FET ON.
     *   Low-side (N-ch): 100 ohm direct gate drive, fast on/off.
     *   High-side (P-ch): small N-FET pulls P-gate low = ON;
     *       1.5k pullup for turn-off (slow — only switch at commutation rate).
     */
    PNP = 0x00;

    /*
     * ── Dead-Time ────────────────────────────────────────────────────
     * Disabled.  In the low-side-only chopping scheme no half-bridge
     * ever has both FETs switching at PWM rate — the commutation masks
     * guarantee at most one channel per half-bridge is unmasked.
     * Disabling dead-time also eliminates the asymmetry between
     * primary-driven phases (U/W) and complement-driven phase (V),
     * since the complement becomes a pure inverse of the primary.
     */
    TIMED_ACCESS();
    PDTCNT = 0;
    TIMED_ACCESS();
    PDTEN = 0x00;              /* Dead-time disabled — masks provide safety */

    /*
     * ── Mask: All Off Initially ──────────────────────────────────────
     * PMEN=0x3F masks all 6 channels, PMD=0x00 forces all LOW.
     * This ensures all FETs are OFF until commutation is applied.
     */
    PMEN = 0x3F;
    PMD  = 0x00;

    /*
     * ── Fault Brake Data ─────────────────────────────────────────────
     * On fault, all outputs go LOW (all FETs off).
     * FBD bits 5:0 = brake output level per channel.
     */
    FBD = 0x00;

    /*
     * ── Enable PWM Pin Output ────────────────────────────────────────
     * PIOCON0: Connect all 6 PWM channels to their primary pins.
     *   Bit 0: PIO00 = P1.2 → CH0 (Phase U low)
     *   Bit 1: PIO01 = P1.1 → CH1 (Phase U high)
     *   Bit 2: PIO02 = P1.0 → CH2 (Phase V high)
     *   Bit 3: PIO03 = P0.0 → CH3 (Phase V low)
     *   Bit 4: PIO04 = P0.1 → CH4 (Phase W low)
     *   Bit 5: PIO05 = P0.3 → CH5 (Phase W high)
     */
    PIOCON0 = 0x3F;

    current_duty = 0;
}

void pwm_set_duty(uint16_t duty)
{
    uint16_t inv;

    if (duty > PWM_MAX_DUTY)
        duty = PWM_MAX_DUTY;

    current_duty = duty;

    /*
     * Low-side ON time = duty counts.
     * Phases U and W: low-side is primary → register = duty (direct).
     * Phase V: high-side is primary, low-side is complement →
     *   register = PERIOD - duty (so complement ON time = duty).
     */
    inv = PWM_PERIOD - duty;

    /* Phase U: CH0 (low-side primary) — direct */
    PWM0H = (uint8_t)(duty >> 8);
    PWM0L = (uint8_t)(duty & 0xFF);

    /* Phase V: CH2 (high-side primary) — inverted for CH3 complement */
    PWM2H = (uint8_t)(inv >> 8);
    PWM2L = (uint8_t)(inv & 0xFF);

    /* Phase W: CH4 (low-side primary) — direct, page 1 */
    SFR_PAGE1();
    PWM4H = (uint8_t)(duty >> 8);
    PWM4L = (uint8_t)(duty & 0xFF);
    SFR_PAGE0();

    /* Trigger buffered load — takes effect at next PWM period boundary */
    LOAD = 1;
}

uint16_t pwm_get_duty(void)
{
    return current_duty;
}

void pwm_set_commutation(uint8_t pmen, uint8_t pmd)
{
    /*
     * Update mask registers for six-step commutation.
     *
     * PMEN: bit=1 → channel output forced to PMD value (masked)
     *       bit=0 → channel follows PWM generator (active)
     *
     * PMD:  forced output level when masked (0=low/off, 1=high/on)
     *
     * Channel bit mapping:
     *   Bit 0: CH0 = Phase U low-side  (P1.2)
     *   Bit 1: CH1 = Phase U high-side (P1.1)
     *   Bit 2: CH2 = Phase V high-side (P1.0)
     *   Bit 3: CH3 = Phase V low-side  (P0.0)
     *   Bit 4: CH4 = Phase W low-side  (P0.1)
     *   Bit 5: CH5 = Phase W high-side (P0.3)
     */
    PMEN = pmen;
    PMD  = pmd;
}

void pwm_enable(void)
{
    /* Clear counter, then start PWM */
    CLRPWM = 1;
    while (CLRPWM)
        ;   /* Wait for counter clear */

    LOAD = 1;           /* Load period/duty into buffers */
    PWMRUN = 1;         /* Start PWM generator */
}

void pwm_disable(void)
{
    PWMRUN = 0;         /* Stop PWM generator */

    /* Mask all channels LOW (all FETs off) */
    PMEN = 0x3F;
    PMD  = 0x00;

    /* Force pins low for safety */
    P11 = 0; P12 = 0;  /* Phase U */
    P10 = 0; P00 = 0;  /* Phase V */
    P01 = 0; P03 = 0;  /* Phase W */
}

void pwm_fault_brake(void)
{
    /*
     * Immediate emergency stop. All outputs forced LOW.
     * Hardware fault brake forces FBD values and clears PWMRUN.
     * We also manually stop and mask everything for belt-and-suspenders safety.
     */
    PWMRUN = 0;

    /* Mask all outputs LOW */
    PMEN = 0x3F;
    PMD  = 0x00;

    /* Force pins low */
    P11 = 0; P12 = 0;
    P10 = 0; P00 = 0;
    P01 = 0; P03 = 0;

    current_duty = 0;
}
