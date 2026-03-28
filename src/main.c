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
 *   UART1   — Serial command interface (115200 baud, on prog header)
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

/* Forward declarations */
static void sys_init(void);
static void timer1_init(void);
static void timer2_init(void);
static void wdt_init(void);
static void wdt_feed(void);

void main(void)
{
    sys_init();
    adc_init();
    hall_init();
    commutation_init();
    pwm_init();

#if FEATURE_UART
    uart_init(115200);
    protocol_init();
#endif

    motor_init();

    timer1_init();
    timer2_init();
    wdt_init();

    EA = 1;

#if FEATURE_UART
    uart_puts("BL4818-Servo v0.2\n");
#endif

    while (1) {
        motor_poll_fast();

        if (TF1) {
            TF1 = 0;
            TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
            TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;

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
    SET_HIRC_24MHZ();

    /* Direction input pin: P1.4 */
    P1M1 |=  0x10;
    P1M2 &= ~0x10;
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
