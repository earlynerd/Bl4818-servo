/*
 * Serial Command Protocol — ASCII text commands over UART
 *
 * Commands are single-character prefixed, terminated by newline.
 *
 *   D<val>  Set duty cycle (-PWM_MAX to +PWM_MAX) and run
 *   T<val>  Set current limit in mA (1 to CURRENT_LIMIT_MA)
 *   S       Stop (coast, all outputs off)
 *   R       Release (same as stop)
 *   C       Clear fault
 *   ?       Status query
 *   H       Hall sensor debug
 *   K       Drive state debug (PMEN/PMD)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "protocol.h"
#include "uart.h"
#include "motor.h"
#include "hall.h"
#include "commutation.h"
#include "pwm.h"

static char __xdata cmd_buf[PROTO_BUF_SIZE];
static uint8_t cmd_len;

static const char *parse_int16(const char *s, int16_t *val)
{
    int16_t result = 0;
    uint8_t neg = 0;

    if (*s == '-') {
        neg = 1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        result = result * 10 + (*s - '0');
        s++;
    }

    *val = neg ? -result : result;
    return s;
}

static void send_ok(void)  { uart_puts("OK\n"); }
static void send_err(void) { uart_puts("ERR\n"); }

static void send_status(void)
{
    uart_puts("state:");
    uart_put_int(motor_get_state());
    uart_puts(",fault:");
    uart_put_int(motor_get_fault());
    uart_puts(",cur:");
    uart_put_int(motor_get_current());
    uart_puts(",duty:");
    uart_put_int(pwm_get_duty());
    uart_puts(",hall:");
    uart_put_int(hall_read());
    uart_putc('\n');
}

static void send_hall_status(void)
{
    uint8_t raw = hall_read_raw();
    uint8_t hall = hall_decode_state(raw);

    uart_puts("raw:");
    uart_put_int(raw);
    uart_puts(",hall:");
    uart_put_int(hall);
    uart_puts(",h321:");
    uart_putc((raw & 0x04) ? '1' : '0');
    uart_putc((raw & 0x02) ? '1' : '0');
    uart_putc((raw & 0x01) ? '1' : '0');
    uart_puts(",dir:");
    uart_put_int(hall_direction());
    uart_puts(",count:");
    uart_put_int(hall_count());
    uart_puts(",period:");
    uart_put_int(hall_period());
    uart_putc('\n');
}

static void send_drive_status(void)
{
    uart_puts("raw:");
    uart_put_int(hall_read_raw());
    uart_puts(",hall:");
    uart_put_int(hall_read());
    uart_puts(",state:");
    uart_put_int(motor_get_state());
    uart_puts(",fault:");
    uart_put_int(motor_get_fault());
    uart_puts(",duty:");
    uart_put_int(pwm_get_duty());
    uart_puts(",run:");
    uart_put_int(PWMRUN ? 1 : 0);
    uart_puts(",pmen:");
    uart_put_int(PMEN);
    uart_puts(",pmd:");
    uart_put_int(PMD);
    uart_putc('\n');
}

static void process_command(void)
{
    int16_t val;

    if (cmd_len == 0)
        return;

    switch (cmd_buf[0]) {
    case 'd':
    case 'D':
        parse_int16(&cmd_buf[1], &val);
        if (val >= -(int16_t)PWM_MAX_DUTY && val <= (int16_t)PWM_MAX_DUTY) {
            motor_set_duty(val);
            motor_start();
            send_ok();
        } else {
            send_err();
        }
        break;

    case 't':
    case 'T':
        parse_int16(&cmd_buf[1], &val);
        if (val > 0 && val <= (int16_t)CURRENT_LIMIT_MA) {
            motor_set_torque_limit((uint16_t)val);
            send_ok();
        } else {
            send_err();
        }
        break;

    case 's':
    case 'S':
    case 'r':
    case 'R':
        motor_stop();
        send_ok();
        break;

    case 'c':
    case 'C':
        motor_clear_fault();
        send_ok();
        break;

    case '?':
        send_status();
        break;

    case 'h':
    case 'H':
        send_hall_status();
        break;

    case 'k':
    case 'K':
        send_drive_status();
        break;

    default:
        send_err();
        break;
    }
}

void protocol_init(void)
{
    cmd_len = 0;
}

void protocol_poll(void)
{
    while (uart_available()) {
        int16_t c = uart_getc();
        if (c < 0)
            break;

        if (c == '\n' || c == '\r') {
            cmd_buf[cmd_len] = '\0';
            process_command();
            cmd_len = 0;
        } else if (cmd_len < PROTO_BUF_SIZE - 1) {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}
