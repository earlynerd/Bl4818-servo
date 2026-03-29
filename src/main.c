/*
 * BL4818-Servo — Custom Firmware for BL4818 Brushless Motor Drivers
 *
 * Direct duty-cycle control with hall-based commutation.
 * Host closes any higher-level control loops externally.
 *
 * Architecture:
 *   PWM0    — Low-side-only chopping (20 kHz, 6 channels)
 *   Timer 1 — 1 kHz tick for control loop timing
 *   Timer 2 — Free-running counter for speed measurement
 *   Timer 3 — UART baud rate generator
 *   UART1   — Binary ring serial interface (250000 baud, on prog header)
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
static void tach_init(void);
static void tach_debug_tick(void);

#if FEATURE_TACH_DEBUG
static uint8_t tach_debug_ticks;
#endif

void main(void)
{
    sys_init();
    adc_init();
    hall_init();
    commutation_init();
    pwm_init();

#if FEATURE_UART
    uart_init(UART_BAUD);
    protocol_init();
#endif

    motor_init();

    timer1_init();
    timer2_init();
    wdt_init();

    EA = 1;

    while (1) {
        motor_poll_fast();

        if (TF1) {
            TF1 = 0;
            TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
            TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;

            tach_debug_tick();
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

    tach_init();
}

static void clock_init(void)
{
    /* Select the 24 MHz HIRC bank before applying any trim offset. */
    TIMED_ACCESS();
    RCTRIM1 |= 0x10u;

#if HIRC_TRIM_OFFSET != 0
    /*
     * Adjust RCTRIM0 directly — avoids 9-bit encode/decode assumptions.
     *
     * CRITICAL: TIMED_ACCESS only holds the TA window open for 3 instructions.
     * Pre-compute the value BEFORE the TA unlock.
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
    T2MOD = 0x00;
    TL2 = 0;
    TH2 = 0;
    RCMP2L = 0;
    RCMP2H = 0;
    T2CON |= 0x04;  /* TR2 = 1 */
}

static void wdt_init(void)
{
    TIMED_ACCESS();
    WDCON = 0x07;
}

static void wdt_feed(void)
{
    TIMED_ACCESS();
    WDCON |= 0x40;
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
