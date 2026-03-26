/*
 * Serial Command Protocol — ASCII text commands over UART
 *
 * Commands are single-character prefixed, terminated by newline.
 * See README.md for the full command table.
 */

#include "ms51_config.h"
#include "protocol.h"
#include "uart.h"
#include "motor.h"
#include "flash.h"
#include "encoder.h"
#include <string.h>

static char __xdata cmd_buf[PROTO_BUF_SIZE];
static uint8_t cmd_len;

/* Parse a decimal integer from string, returns pointer past the number */
static const char *parse_int32(const char *s, int32_t *val)
{
    int32_t result = 0;
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

static void send_ok(void)
{
    uart_puts("OK\n");
}

static void send_err(void)
{
    uart_puts("ERR\n");
}

static void process_command(void)
{
    int32_t val;

    if (cmd_len == 0)
        return;

    switch (cmd_buf[0]) {
    case 'P':  /* Set target position */
        parse_int32(&cmd_buf[1], &val);
        motor_set_mode(MODE_POSITION);
        motor_set_position(val);
        motor_start();
        send_ok();
        break;

    case 'V':  /* Set target velocity (RPM) */
        parse_int32(&cmd_buf[1], &val);
        if (val >= -32000 && val <= 32000) {
            motor_set_mode(MODE_VELOCITY);
            motor_set_velocity((int16_t)val);
            motor_start();
            send_ok();
        } else {
            send_err();
        }
        break;

    case 'T':  /* Set torque limit (mA) */
        parse_int32(&cmd_buf[1], &val);
        if (val > 0 && val <= CURRENT_LIMIT_MA) {
            motor_set_torque_limit((uint16_t)val);
            send_ok();
        } else {
            send_err();
        }
        break;

    case 'D':  /* Direct PWM duty (-PWM_MAX to +PWM_MAX) */
        parse_int32(&cmd_buf[1], &val);
        if (val >= -(int32_t)PWM_MAX_DUTY && val <= (int32_t)PWM_MAX_DUTY) {
            motor_set_mode(MODE_OPEN_LOOP);
            motor_set_duty((int16_t)val);
            motor_start();
            send_ok();
        } else {
            send_err();
        }
        break;

    case 'S':  /* Stop (brake) */
        motor_stop();
        send_ok();
        break;

    case 'R':  /* Release (coast) */
        motor_release();
        send_ok();
        break;

    case '?':  /* Query status */
        protocol_send_status();
        break;

    case 'C':  /* Clear fault */
        motor_clear_fault();
        send_ok();
        break;

    case 'Z':  /* Zero encoder */
        encoder_set_position(0);
        send_ok();
        break;

    case 'W':  /* Write parameter: W<param>=<value> */
        {
            params_t p;
            flash_load_params(&p);
            /* Parse param name (single digit for now) */
            if (cmd_len > 3 && cmd_buf[2] == '=') {
                parse_int32(&cmd_buf[3], &val);
                switch (cmd_buf[1]) {
                case '0': p.mode = (uint8_t)val; break;
                case '1': p.torque_limit = (uint16_t)val; break;
                case '2': p.pid_pos_kp = (int16_t)val; break;
                case '3': p.pid_pos_ki = (int16_t)val; break;
                case '4': p.pid_pos_kd = (int16_t)val; break;
                case '5': p.pid_vel_kp = (int16_t)val; break;
                case '6': p.pid_vel_ki = (int16_t)val; break;
                case '7': p.pid_vel_kd = (int16_t)val; break;
                case '8': p.encoder_cpr = (uint16_t)val; break;
                default: send_err(); return;
                }
                flash_save_params(&p);
                send_ok();
            } else {
                send_err();
            }
        }
        break;

    case 'G':  /* Get parameter */
        {
            params_t p;
            flash_load_params(&p);
            if (cmd_len > 1) {
                int32_t pval = 0;
                switch (cmd_buf[1]) {
                case '0': pval = p.mode; break;
                case '1': pval = p.torque_limit; break;
                case '2': pval = p.pid_pos_kp; break;
                case '3': pval = p.pid_pos_ki; break;
                case '4': pval = p.pid_pos_kd; break;
                case '5': pval = p.pid_vel_kp; break;
                case '6': pval = p.pid_vel_ki; break;
                case '7': pval = p.pid_vel_kd; break;
                case '8': pval = p.encoder_cpr; break;
                default: send_err(); return;
                }
                uart_put_int(pval);
                uart_putc('\n');
            } else {
                send_err();
            }
        }
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
        /* else: overflow — ignore extra chars until newline */
    }
}

void protocol_send_status(void)
{
    uart_puts("pos:");
    uart_put_int(motor_get_position());
    uart_puts(",vel:");
    uart_put_int(motor_get_velocity());
    uart_puts(",cur:");
    uart_put_int(motor_get_current());
    uart_puts(",state:");
    uart_put_int(motor_get_state());
    uart_puts(",fault:");
    uart_put_int(motor_get_fault());
    uart_putc('\n');
}
