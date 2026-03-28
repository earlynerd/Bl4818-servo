/*
 * Minimal bench firmware for BL4818 gate-drive probing.
 *
 * This image intentionally avoids the motor state machine, ADC, halls, PWM
 * commutation, flash persistence, and watchdog. It only exposes direct GPIO
 * control of the six gate-drive pins over UART1 so board-level mapping can be
 * verified without the rest of the control stack in the loop.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "uart.h"
#include <stdint.h>

#define BENCH_BUF_SIZE  24

static char __xdata cmd_buf[BENCH_BUF_SIZE];
static uint8_t cmd_len;
static uint8_t gate_mask;
static uint8_t sector;
static uint8_t hall_raw;
static uint8_t hall_prev;
static uint16_t hall_changes;

static void sys_init(void)
{
    CKDIV = 0x00;
}

static void set_all_gates_low(void)
{
    P12 = 0;
    P11 = 0;
    P10 = 0;
    P00 = 0;
    P01 = 0;
    P03 = 0;
}

static void drive_gate_mask(uint8_t mask)
{
    P12 = (mask & 0x01) ? 1 : 0;
    P11 = (mask & 0x02) ? 1 : 0;
    P10 = (mask & 0x04) ? 1 : 0;
    P00 = (mask & 0x08) ? 1 : 0;
    P01 = (mask & 0x10) ? 1 : 0;
    P03 = (mask & 0x20) ? 1 : 0;
}

static uint8_t gate_mask_conflict(uint8_t mask)
{
    if ((mask & 0x03) == 0x03)
        return 1;

    if ((mask & 0x0C) == 0x0C)
        return 1;

    if ((mask & 0x30) == 0x30)
        return 1;

    return 0;
}

static void gate_init(void)
{
    /* Push-pull outputs on all six gate-drive pins. */
    P1M1 &= ~0x07;
    P1M2 |=  0x07;
    P0M1 &= ~0x0B;
    P0M2 |=  0x0B;

    /*
     * Make sure the PWM peripheral does not own the pins.
     * This image uses plain GPIO only.
     */
    PWMRUN = 0;
    PWMCON0 = 0x00;
    PWMCON1 = 0x00;
    PMEN = 0x3F;
    PMD = 0x00;
    PIOCON0 = 0x00;
    PNP = 0x00;

    set_all_gates_low();
    gate_mask = 0;
}

static void hall_init(void)
{
    /* Hall 1 = P1.5, Hall 2 = P1.7, Hall 3 = P3.0 */
    P1M1 |=  0x20;
    P1M2 &= (uint8_t)~0x20;

    P1M1 |=  0x80;
    P1M2 &= (uint8_t)~0x80;

    P3M1 |=  0x01;
    P3M2 &= (uint8_t)~0x01;

    hall_raw = 0;
    hall_prev = 0;
    hall_changes = 0;
}

static uint8_t hall_read_raw(void)
{
    uint8_t state = 0;

    if (HALL_1_PIN)
        state |= 0x01;
    if (HALL_2_PIN)
        state |= 0x02;
    if (HALL_3_PIN)
        state |= 0x04;

    return state;
}

static void hall_poll(void)
{
    uint8_t raw = hall_read_raw();

    if (raw != hall_raw) {
        hall_prev = hall_raw;
        hall_raw = raw;
        if (hall_changes < 0xFFFFu)
            hall_changes++;
    }
}

static const char *parse_u8(const char *s, uint8_t *val)
{
    uint16_t parsed = 0;

    if (*s == '+')
        s++;

    if (*s < '0' || *s > '9')
        return 0;

    while (*s >= '0' && *s <= '9') {
        parsed = (uint16_t)(parsed * 10u) + (uint16_t)(*s - '0');
        if (parsed > 255u)
            return 0;
        s++;
    }

    *val = (uint8_t)parsed;
    return s;
}

static void send_ok(void)
{
    uart_puts("OK\n");
}

static void send_err(void)
{
    uart_puts("ERR\n");
}

static void send_status(void)
{
    hall_poll();

    uart_puts("mask:");
    uart_put_int(gate_mask);
    uart_puts(",sector:");
    uart_put_int(sector);
    uart_puts(",hall:");
    uart_put_int(hall_raw);
    uart_puts(",hprev:");
    uart_put_int(hall_prev);
    uart_puts(",hcount:");
    uart_put_int(hall_changes);
    uart_puts(",h321:");
    uart_putc((hall_raw & 0x04) ? '1' : '0');
    uart_putc((hall_raw & 0x02) ? '1' : '0');
    uart_putc((hall_raw & 0x01) ? '1' : '0');
    uart_puts(",u_lo:");
    uart_put_int(P12 ? 1 : 0);
    uart_puts(",u_hi:");
    uart_put_int(P11 ? 1 : 0);
    uart_puts(",v_hi:");
    uart_put_int(P10 ? 1 : 0);
    uart_puts(",v_lo:");
    uart_put_int(P00 ? 1 : 0);
    uart_puts(",w_lo:");
    uart_put_int(P01 ? 1 : 0);
    uart_puts(",w_hi:");
    uart_put_int(P03 ? 1 : 0);
    uart_puts(",run:");
    uart_put_int(PWMRUN ? 1 : 0);
    uart_puts(",piocon0:");
    uart_put_int(PIOCON0);
    uart_puts(",pmen:");
    uart_put_int(PMEN);
    uart_puts(",pmd:");
    uart_put_int(PMD);
    uart_putc('\n');
}

static void release_gates(void)
{
    set_all_gates_low();
    gate_mask = 0;
    sector = 0;
}

static uint8_t sector_to_mask(uint8_t s, uint8_t *mask)
{
    switch (s) {
    case 1: *mask = 10; return 1; /* U_HI + V_LO */
    case 2: *mask = 18; return 1; /* U_HI + W_LO */
    case 3: *mask = 20; return 1; /* V_HI + W_LO */
    case 4: *mask = 5;  return 1; /* V_HI + U_LO */
    case 5: *mask = 33; return 1; /* W_HI + U_LO */
    case 6: *mask = 40; return 1; /* W_HI + V_LO */
    default:
        return 0;
    }
}

static void process_command(void)
{
    uint8_t mask;
    uint8_t requested_sector;
    const char *end;

    if (cmd_len == 0)
        return;

    switch (cmd_buf[0]) {
    case 'r':
    case 'R':
        release_gates();
        send_ok();
        break;

    case 'x':
    case 'X':
        end = parse_u8(&cmd_buf[1], &mask);
        if (!end || *end != '\0' || mask > 63u || gate_mask_conflict(mask)) {
            send_err();
            break;
        }

        release_gates();
        drive_gate_mask(mask);
        gate_mask = mask;
        sector = 0;
        send_ok();
        break;

    case 'j':
    case 'J':
        end = parse_u8(&cmd_buf[1], &requested_sector);
        if (!end || *end != '\0') {
            send_err();
            break;
        }

        if (requested_sector == 0) {
            release_gates();
            send_ok();
            break;
        }

        if (!sector_to_mask(requested_sector, &mask)) {
            send_err();
            break;
        }

        release_gates();
        drive_gate_mask(mask);
        gate_mask = mask;
        sector = requested_sector;
        send_ok();
        break;

    case '?':
        send_status();
        break;

    case 'h':
    case 'H':
        send_status();
        break;

    case 'z':
    case 'Z':
        hall_poll();
        hall_prev = hall_raw;
        hall_changes = 0;
        send_ok();
        break;

    default:
        send_err();
        break;
    }
}

static void protocol_poll(void)
{
    while (uart_available()) {
        int16_t c = uart_getc();

        if (c < 0)
            break;

        if (c == '\n' || c == '\r') {
            cmd_buf[cmd_len] = '\0';
            process_command();
            cmd_len = 0;
        } else if (cmd_len < BENCH_BUF_SIZE - 1) {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}

void main(void)
{
    sys_init();
    gate_init();
    hall_init();
    uart_init(115200);
    cmd_len = 0;
    sector = 0;
    hall_raw = hall_read_raw();
    hall_prev = hall_raw;
    EA = 0;

    uart_puts("BL4818-Bench v0.1\n");

    while (1) {
        hall_poll();
        protocol_poll();
    }
}
