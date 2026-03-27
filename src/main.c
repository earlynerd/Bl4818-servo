/*
 * BL4818-Servo — Custom Firmware for BL4818 Brushless Motor Drivers
 *
 * Main entry point. Initializes all peripherals, loads saved parameters,
 * and runs the main control loop.
 *
 * Architecture:
 *   PWM0    — Hardware complementary PWM (20 kHz, 6 channels, dead-time)
 *   Timer 1 — 1 kHz tick for control loop timing
 *   Timer 2 — Free-running counter for speed measurement
 *   Timer 3 — UART baud rate generator
 *   UART1   — Serial command interface (115200 baud, on prog header)
 *   ADC     — Current and voltage sensing (polled in control loop)
 *   GPIO    — Hall sensors (polled), encoder (polled/ISR)
 *
 * Control loop runs at 1 kHz from Timer 1 interrupt flag (polled in main loop).
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "pwm.h"
#include "adc.h"
#include "hall.h"
#include "commutation.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "uart.h"
#include "protocol.h"
#include "flash.h"

/* Saved parameters — in XRAM to save IRAM */
static params_t __xdata params;

/* LED blink state */
static uint16_t led_counter;

/* Forward declarations */
static void sys_init(void);
static void timer1_init(void);
static void timer2_init(void);
static void wdt_init(void);
static void wdt_feed(void);
static void led_update(void);
static void apply_params(void);

void main(void)
{
    /* ── System Initialization ───────────────────────────────────────── */
    sys_init();

    /* Initialize peripherals */
    adc_init();
    hall_init();
    commutation_init();

#if FEATURE_ENCODER
    encoder_init();
#endif

    pwm_init();

#if FEATURE_UART
    uart_init(115200);
    protocol_init();
#endif

    motor_init();

    /* Load saved parameters */
#if FEATURE_FLASH_SAVE
    flash_load_params(&params);
    apply_params();
#endif

    /* Start timers */
    timer1_init();  /* 1 kHz control loop tick */
    timer2_init();  /* Free-running for speed measurement */
    wdt_init();

    /* Enable global interrupts */
    EA = 1;

#if FEATURE_UART
    uart_puts("BL4818-Servo v0.1\n");
#endif

#if FEATURE_LED
    LED_PIN = 1;  /* LED on = powered up */
#endif

    /* ── Main Loop ───────────────────────────────────────────────────── */
    while (1) {
        /* Fast path: keep commutation aligned to hall edges between 1 kHz control ticks */
        motor_poll_fast();

        /* Wait for Timer 1 overflow (1 kHz tick) */
        if (TF1) {
            TF1 = 0;  /* Clear flag */

            /* Reload Timer 1 for next 1ms tick */
            TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
            TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;

            /* ── 1 kHz Control Tasks ─────────────────────────────── */

            /* Sample encoder (if not interrupt-driven) */
#if FEATURE_ENCODER
            encoder_isr();  /* Poll encoder state */
#endif

            /* Run motor control state machine */
            motor_update();

            /* LED status indication */
#if FEATURE_LED
            led_update();
#endif

            /* Feed watchdog */
            wdt_feed();
        }

        /* ── Non-time-critical Tasks (run every main loop iteration) ── */

#if FEATURE_UART
        /* Process serial commands */
        protocol_poll();
#endif
    }
}

/* ── System Clock and GPIO Initialization ────────────────────────────────── */
static void sys_init(void)
{
    /*
     * HIRC defaults to 16 MHz after reset.
     * TODO: switch to 24 MHz by reading factory trim from UID area.
     * For now, run at 16 MHz (FSYS must match in ms51_config.h).
     */
    CKDIV = 0x00;       /* No divider: FSYS = FOSC = 16 MHz */

    /* Configure direction input pin: P1.4 (pin 11) */
    P1M1 |=  0x10;   /* P1.4 M1=1 (input) */
    P1M2 &= ~0x10;   /* P1.4 M2=0 */

#if FEATURE_LED
    /* Configure LED pin as push-pull output (if LED added to spare pin) */
    /* LED_PIN = 0; */
#endif
}

/* ── Timer 1: 1 kHz Control Loop Tick ────────────────────────────────────── */
static void timer1_init(void)
{
    /*
     * Timer 1 in Mode 1 (16-bit timer), FSYS/12 clock.
     * At 24MHz: clock = 2 MHz, 1ms = 2000 counts.
     * Reload = 65536 - 2000 = 63536
     *
     * We use the overflow flag (TF1) polled in main loop,
     * NOT an interrupt, to trigger the control loop.
     * This keeps the timing simple and avoids ISR nesting issues.
     */
    TMOD = (TMOD & 0x0F) | 0x10;  /* Timer 1 mode 1 */
    TH1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) >> 8;
    TL1 = (65536 - (FSYS / 12 / CONTROL_LOOP_HZ)) & 0xFF;
    TR1 = 1;   /* Start Timer 1 */
    /* Note: ET1 = 0 — we poll TF1, not using interrupt */
}

/* ── Timer 2: Free-running Counter for Speed Measurement ─────────────────── */
static void timer2_init(void)
{
    /*
     * Timer 2 in auto-reload mode, free-running.
     * Used by hall.c to measure time between transitions.
     * Clock = FSYS/12 = 2 MHz → 0.5us per tick.
     * 16-bit counter overflows every ~32ms.
     */
    T2CON = 0x00;  /* Stop, auto-reload mode */
    T2MOD = 0x00;  /* Clock = FSYS/12 */
    TL2 = 0;
    TH2 = 0;
    RCMP2L = 0;    /* Reload value = 0 (free-running) */
    RCMP2H = 0;
    T2CON |= 0x04; /* TR2 = 1: start Timer 2 */
}

/* ── Watchdog Timer ──────────────────────────────────────────────────────── */
static void wdt_init(void)
{
    /*
     * WDT timeout ~1 second.
     * WDCON register controls the watchdog.
     * Prescaler: FSYS / 2^(WDT_PS+1)
     * For ~1s at 24MHz with PS=7: 24M / 2^21 ≈ 11.4 Hz → 87ms
     * Need longer: use WKT or just feed frequently.
     */
    TIMED_ACCESS();
    WDCON = 0x07;   /* Enable WDT, longest prescaler */
}

static void wdt_feed(void)
{
    /* Clear WDT by writing to WDCON with WDCLR bit */
    TIMED_ACCESS();
    WDCON |= 0x40;  /* WDCLR = 1 */
}

/* ── LED Status Indication ───────────────────────────────────────────────── */
static void led_update(void)
{
    led_counter++;
	#if FEATURE_LED
    switch (motor_get_state()) {
    case MOTOR_IDLE:
        /* Slow blink: 0.5 Hz */
        LED_PIN = (led_counter < 500) ? 1 : 0;
        if (led_counter >= 1000) led_counter = 0;
        break;

    case MOTOR_RUN:
        /* Solid on */
        LED_PIN = 1;
        led_counter = 0;
        break;

    case MOTOR_BRAKE:
    case MOTOR_REVERSING:
        /* Fast blink: 5 Hz */
        LED_PIN = (led_counter < 100) ? 1 : 0;
        if (led_counter >= 200) led_counter = 0;
        break;

    case MOTOR_FAULT:
        /* Very fast blink: 10 Hz */
        LED_PIN = (led_counter < 50) ? 1 : 0;
        if (led_counter >= 100) led_counter = 0;
        break;
    }
	#endif
}

/* ── Apply Loaded Parameters ─────────────────────────────────────────────── */
static void apply_params(void)
{
    motor_set_mode(params.mode);
    motor_set_torque_limit(params.torque_limit);
    commutation_set_offset(params.commutation_offset);
    /* PID gains would be applied here if we exposed setters */
}
