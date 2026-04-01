/*
 * BL4818-Servo — Custom Firmware for BL4818 Brushless Motor Drivers
 *
 * Direct duty-cycle control with hall-based commutation.
 * Host closes any higher-level control loops externally.
 *
 * Architecture:
 *   PWM0    — Low-side-only chopping (20 kHz, 6 channels)
 *   Timer 1 — 1 kHz tick for control loop timing
 *   Timer 2 — Free-running capture counter for hall timing and local PWM input
 *   Timer 3 — UART baud rate generator
 *   UART1   — Binary ring serial interface (115200 baud, on prog header)
 *   ADC     — Current sensing (polled in control loop)
 *   GPIO    — Hall sensors (polled)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "pwm.h"
#include "adc.h"
#include "hall.h"
#include "commutation.h"
#include "motor.h"
#include "uart.h"
#include "protocol.h"
#include <stdint.h>

/* Forward declarations */
static void clock_init(void);
static void sys_init(void);
static void timer1_init(void);
static void timer2_init(void);
static void wdt_init(void);
static void wdt_feed(void);
static void ta_write_rctrim1(uint8_t value);
static void ta_write_wdcon(uint8_t value);
static void wdt_set_bits(uint8_t mask);
static void wdt_clear_bits(uint8_t mask);
uint8_t wdt_reset_detected(void);
static void tach_init(void);
static void tach_debug_tick(void);
static void local_input_poll_fast(void);
static void local_input_update(void);
void capture_isr(void) __interrupt(INT_CAPTURE);

#if FEATURE_LOCAL_PWM_INPUT
static void local_input_init(void);
static void local_input_stop_motion(void);
static void local_input_release(void);
static void local_input_lock_out(void);
static void local_input_capture_enable(void);
static void local_input_capture_disable(void);
#endif

#if FEATURE_TACH_DEBUG
static uint8_t tach_debug_ticks;
#endif

#if FEATURE_LOCAL_PWM_INPUT
static volatile uint8_t local_input_locked_out;
static uint8_t local_pwm_dc_hold_ms;
static uint8_t local_fault_retry_count;
static uint16_t local_fault_retry_ms;
static uint16_t local_applied_abs_duty;
static volatile uint8_t local_pwm_seen_rise;
static volatile uint8_t local_pwm_valid_cycles;
static volatile uint8_t local_pwm_timeout_ms;
static volatile uint16_t local_pwm_last_rise;
static volatile uint16_t local_pwm_high_ticks;
static volatile uint16_t local_pwm_period_ticks;
static int8_t local_drive_direction;
static uint16_t local_reverse_coast_ms;
#endif

static void gate_pins_safe(void)
{
    /*
     * Force all 6 gate drive pins LOW before any other init.
     * On reset, pins default to quasi-bidirectional (weakly pulled high),
     * which could briefly enable FETs. Drive them push-pull LOW immediately.
     */
    P1M1 &= ~0x07;  P1M2 |=  0x07;  /* P1.0, P1.1, P1.2 push-pull */
    P0M1 &= ~0x0B;  P0M2 |=  0x0B;  /* P0.0, P0.1, P0.3 push-pull */
    P11 = 0; P12 = 0;  /* Phase U */
    P10 = 0; P00 = 0;  /* Phase V */
    P01 = 0; P03 = 0;  /* Phase W */
}

void main(void)
{
    gate_pins_safe();
    wdt_feed();
    sys_init();
    wdt_init();
    adc_init();
    hall_init();
    pwm_init();

#if FEATURE_UART
    uart_init(UART_BAUD);
    protocol_init();
#endif

    motor_init();

    timer1_init();
    timer2_init();
    wdt_feed();

    EA = 1;

    while (1) {
        motor_poll_fast();
        local_input_poll_fast();

        if (TF1) {
            TF1 = 0;
            TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
            TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;

            protocol_tick_1khz();
            tach_debug_tick();
            local_input_update();
            motor_update();
            wdt_feed();
        }

#if FEATURE_UART
        protocol_poll();
#endif
    }
}

static void sys_init(void)
{
    clock_init();

    /* Direction input pin: P1.4 */
    P1M1 |=  0x10;
    P1M2 &= ~0x10;

#if FEATURE_LOCAL_PWM_INPUT
    local_input_init();
#endif

    tach_init();
}

static void clock_init(void)
{
    uint8_t r1;

    /* Select the 24 MHz HIRC bank before applying any trim offset. */
    r1 = (uint8_t)(RCTRIM1 | 0x10u);
    ta_write_rctrim1(r1);

#if HIRC_TRIM_OFFSET != 0
    /*
     * Adjust RCTRIM0 directly — avoids 9-bit encode/decode assumptions.
     *
     * Pre-compute the value BEFORE the TA unlock so the protected write itself
     * is a single direct assignment inside the TA write window.
     */
    {
        uint8_t r0 = (uint8_t)(RCTRIM0 + (int8_t)HIRC_TRIM_OFFSET);
        TIMED_ACCESS();
        RCTRIM0 = r0;
    }
#endif

    TIMED_ACCESS();
    CKSWT = 0x00;
    CKDIV = 0x00;
}

static void ta_write_rctrim1(uint8_t value)
{
    uint8_t saved_ea = EA;

    EA = 0;
    TIMED_ACCESS();
    RCTRIM1 = value;
    EA = saved_ea;
}

static void ta_write_wdcon(uint8_t value)
{
    uint8_t saved_ea = EA;

    EA = 0;
    TIMED_ACCESS();
    WDCON = value;
    EA = saved_ea;
}

static void timer1_init(void)
{
    TMOD = (TMOD & 0x0F) | 0x10;  /* Timer 1 mode 1 (16-bit) */
    TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
    TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;
    TR1 = 1;
}

static void timer2_init(void)
{
    T2CON = 0x00;
#if FEATURE_LOCAL_PWM_INPUT
    /*
     * Timer2 capture mode, CAP0 load source, prescale /16.
     * Derived from the vendor TIMER2_CAP0_Capture_Mode + TIMER2_DIV_16 setup.
     */
    T2MOD = 0xA9;
#else
    T2MOD = 0x00;
#endif
    TL2 = 0;
    TH2 = 0;
    RCMP2L = 0;
    RCMP2H = 0;
    T2CON |= 0x04;  /* TR2 = 1 */
}

static void wdt_set_bits(uint8_t mask)
{
    uint8_t value = (uint8_t)(WDCON | mask);
    ta_write_wdcon(value);
}

static void wdt_clear_bits(uint8_t mask)
{
    uint8_t value = (uint8_t)(WDCON & (uint8_t)~mask);
    ta_write_wdcon(value);
}

static uint8_t wdt_reset_occurred;

static void wdt_init(void)
{
#if FEATURE_WATCHDOG
    /* Check if the MCU was reset by the WDT before we clear anything */
    wdt_reset_occurred = (WDCON & WDCON_WDTRF) ? 1u : 0u;

    /*
     * Follow the documented WDCON ceremony literally:
     * clear sticky flags, clear WDPS, set WDPS, then feed with WDCLR.
     * Avoid full-register writes because WDCON contains side-effect bits.
     */
    wdt_clear_bits((uint8_t)(WDCON_WDTF | WDCON_WDTRF | WDCON_WDPS_MASK));
    wdt_set_bits((uint8_t)(WDT_PRESCALER_BITS & WDCON_WDPS_MASK));
    wdt_feed();
#else
    wdt_reset_occurred = 0;
#endif
}

uint8_t wdt_reset_detected(void)
{
    return wdt_reset_occurred;
}

static void wdt_feed(void)
{
#if FEATURE_WATCHDOG
    wdt_set_bits(WDCON_WDCLR);
#endif
}

static void tach_init(void)
{
    P0M1 &= (uint8_t)~0x20;  /* P0.5 push-pull output */
    P0M2 |=  0x20;
    TACH_PIN = 0;

#if FEATURE_TACH_DEBUG
    tach_debug_ticks = 0;
#endif
}

static void tach_debug_tick(void)
{
#if FEATURE_TACH_DEBUG
    tach_debug_ticks++;
    if (tach_debug_ticks >= TACH_DEBUG_DIVIDER) {
        tach_debug_ticks = 0;
        TACH_PIN = !TACH_PIN;
    }
#endif
}

#if FEATURE_LOCAL_PWM_INPUT
static void local_input_stop_motion(void)
{
    local_applied_abs_duty = 0;
    local_drive_direction = 0;
    motor_set_duty(0);

    if (motor_get_state() == MOTOR_RUN)
        motor_stop();
}

static void local_input_release(void)
{
    uint8_t saved_ea = EA;

    EA = 0;
    local_pwm_last_rise = 0;
    local_pwm_seen_rise = 0;
    local_pwm_valid_cycles = 0;
    local_pwm_timeout_ms = 0;
    local_pwm_dc_hold_ms = 0;
    local_fault_retry_count = 0;
    local_fault_retry_ms = 0;
    local_applied_abs_duty = 0;
    local_drive_direction = 0;
    local_reverse_coast_ms = 0;
    local_pwm_high_ticks = 0;
    local_pwm_period_ticks = 0;
    EA = saved_ea;

    local_input_stop_motion();
}

static void local_input_lock_out(void)
{
    local_input_locked_out = 1;
    local_input_capture_disable();
    local_input_release();
}

static void local_input_capture_enable(void)
{
    /* CAP0 on IC3/P0.4, capture both edges, enable falling-edge latch. */
    CAPCON3 = (uint8_t)((CAPCON3 & (uint8_t)~0x0Fu) | 0x04u);
    CAPCON1 = (uint8_t)((CAPCON1 & (uint8_t)~0x03u) | 0x02u);
    CAPCON2 = (uint8_t)((CAPCON2 & (uint8_t)~0x70u) | 0x10u);
    CAPCON0 = (uint8_t)((CAPCON0 & (uint8_t)~0x77u) | 0x10u);
    TF2 = 0;
    EIE |= 0x04u;
}

static void local_input_capture_disable(void)
{
    EIE &= (uint8_t)~0x04u;
    CAPCON0 &= (uint8_t)~0x10u;
    CAPCON2 &= (uint8_t)~0x10u;
    CAPCON0 &= (uint8_t)~0x01u;
    TF2 = 0;
}

static void local_input_init(void)
{
    /* PWM speed input pin: P0.4 */
    P0M1 |=  0x10;
    P0M2 &= ~0x10;

    local_input_locked_out = 0;
    local_pwm_last_rise = 0;
    local_pwm_seen_rise = 0;
    local_pwm_valid_cycles = 0;
    local_pwm_timeout_ms = 0;
    local_pwm_dc_hold_ms = 0;
    local_fault_retry_count = 0;
    local_fault_retry_ms = 0;
    local_applied_abs_duty = 0;
    local_drive_direction = 0;
    local_reverse_coast_ms = 0;
    local_pwm_high_ticks = 0;
    local_pwm_period_ticks = 0;
    local_input_capture_enable();
}

static void local_input_poll_fast(void)
{
    if (local_input_locked_out)
        return;

    if (protocol_is_enumerated())
        local_input_lock_out();
}

static void local_input_update(void)
{
    uint8_t input_active;
    uint8_t saved_ea;
    uint8_t timeout_ms;
    uint8_t valid_cycles;
    uint16_t desired_abs_duty;
    uint32_t scaled_duty;
    uint16_t high_ticks;
    uint16_t period_ticks;
    uint16_t pulse_ticks;
    uint16_t abs_duty;
    int16_t signed_duty;
    int8_t requested_direction;
    uint8_t dir_positive;

    if (local_input_locked_out)
        return;

    if (protocol_is_enumerated()) {
        local_input_lock_out();
        return;
    }

    input_active = PWM_IN_PIN ? 0u : 1u;
#if !LOCAL_PWM_ACTIVE_LOW
    input_active = input_active ? 0u : 1u;
#endif

    if (input_active) {
        if (local_pwm_dc_hold_ms < LOCAL_PWM_DC_FULLSCALE_MS)
            local_pwm_dc_hold_ms++;
    } else {
        local_pwm_dc_hold_ms = 0;
    }

    saved_ea = EA;
    EA = 0;
    if (local_pwm_timeout_ms != 0u)
        local_pwm_timeout_ms--;
    timeout_ms = local_pwm_timeout_ms;
    valid_cycles = local_pwm_valid_cycles;
    high_ticks = local_pwm_high_ticks;
    period_ticks = local_pwm_period_ticks;
    EA = saved_ea;

    if (local_pwm_dc_hold_ms >= LOCAL_PWM_DC_FULLSCALE_MS) {
        desired_abs_duty = PWM_MAX_DUTY;
        goto apply_local_command;
    }

    if (timeout_ms == 0u || valid_cycles < 2u || period_ticks == 0u) {
        if (input_active) {
            local_input_stop_motion();
            return;
        }

        local_fault_retry_count = 0;
        local_fault_retry_ms = 0;
        local_input_release();
        return;
    }

    pulse_ticks = high_ticks;
#if LOCAL_PWM_ACTIVE_LOW
    pulse_ticks = (uint16_t)(period_ticks - high_ticks);
#endif

    scaled_duty = (uint32_t)pulse_ticks * (uint32_t)PWM_MAX_DUTY;
    desired_abs_duty = (uint16_t)(scaled_duty / period_ticks);

    if (desired_abs_duty <= LOCAL_PWM_MIN_DUTY_COUNTS) {
        local_fault_retry_count = 0;
        local_fault_retry_ms = 0;
        local_input_release();
        return;
    }

    if (desired_abs_duty > PWM_MAX_DUTY)
        desired_abs_duty = PWM_MAX_DUTY;

apply_local_command:
    if (local_applied_abs_duty < desired_abs_duty) {
        uint16_t ramp_step = (uint16_t)((PWM_MAX_DUTY + LOCAL_PWM_RAMP_UP_MS - 1u) / LOCAL_PWM_RAMP_UP_MS);

        local_applied_abs_duty = (uint16_t)(local_applied_abs_duty + ramp_step);
        if (local_applied_abs_duty > desired_abs_duty)
            local_applied_abs_duty = desired_abs_duty;
    } else {
        local_applied_abs_duty = desired_abs_duty;
    }

    if (motor_get_state() == MOTOR_FAULT) {
        if (local_fault_retry_ms != 0u) {
            local_fault_retry_ms--;
            return;
        }

        if (local_fault_retry_count >= LOCAL_FAULT_RETRY_MAX)
            return;

        local_applied_abs_duty = 0;
        motor_clear_fault();
        local_fault_retry_count++;
        local_fault_retry_ms = LOCAL_FAULT_RETRY_DELAY_MS;
    }

    abs_duty = local_applied_abs_duty;
    dir_positive = DIR_PIN ? 1u : 0u;
#if LOCAL_PWM_DIR_INVERT
    dir_positive = dir_positive ? 0u : 1u;
#endif

    requested_direction = dir_positive ? 1 : -1;

    if (local_reverse_coast_ms != 0u) {
        local_reverse_coast_ms--;
        local_input_stop_motion();
        return;
    }

    if (abs_duty != 0u &&
        local_drive_direction != 0 &&
        requested_direction != local_drive_direction) {
        local_reverse_coast_ms = LOCAL_DIR_REVERSAL_COAST_MS;
        local_input_stop_motion();
        return;
    }

    signed_duty = (requested_direction > 0) ? (int16_t)abs_duty : -(int16_t)abs_duty;
    motor_set_duty(signed_duty);

    if (motor_get_state() == MOTOR_IDLE)
        motor_start();

    local_drive_direction = requested_direction;
}
#else
static void local_input_poll_fast(void) { }
static void local_input_update(void) { }
#endif

void capture_isr(void) __interrupt(INT_CAPTURE)
{
#if FEATURE_LOCAL_PWM_INPUT
    if (CAPCON0 & 0x01u) {
        uint16_t captured = (uint16_t)(((uint16_t)C0H << 8) | C0L);
        uint16_t period;
        uint8_t level = PWM_IN_PIN ? 1u : 0u;

        CAPCON0 &= (uint8_t)~0x01u;
        TF2 = 0;

        if (local_input_locked_out)
            return;

        local_pwm_timeout_ms = LOCAL_PWM_TIMEOUT_MS;

        if (level) {
            if (local_pwm_seen_rise) {
                period = (uint16_t)(captured - local_pwm_last_rise);
                if (period != 0u && local_pwm_high_ticks < period) {
                    local_pwm_period_ticks = period;
                    if (local_pwm_valid_cycles < 2u)
                        local_pwm_valid_cycles++;
                } else {
                    local_pwm_valid_cycles = 0;
                }
            }

            local_pwm_last_rise = captured;
            local_pwm_seen_rise = 1u;
        } else if (local_pwm_seen_rise) {
            local_pwm_high_ticks = (uint16_t)(captured - local_pwm_last_rise);
        }
    }
#else
    CAPCON0 &= (uint8_t)~0x01u;
    TF2 = 0;
#endif
}
