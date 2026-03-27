/*
 * Serial Command Protocol — ASCII text commands over UART
 *
 * Commands are single-character prefixed, terminated by newline.
 * See README.md for the full command table.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "protocol.h"
#include "uart.h"
#include "motor.h"
#include "flash.h"
#include "hall.h"
#include "commutation.h"
#include "encoder.h"
#include "pwm.h"
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
    uart_puts(",offset:");
    uart_put_int(commutation_get_offset());
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

static uint8_t params_set_by_id(params_t *p, uint8_t param_id, int32_t val)
{
    switch (param_id) {
    case 0: p->mode = (uint8_t)val; break;
    case 1: p->torque_limit = (uint16_t)val; break;
    case 2: p->pid_pos_kp = (int16_t)val; break;
    case 3: p->pid_pos_ki = (int16_t)val; break;
    case 4: p->pid_pos_kd = (int16_t)val; break;
    case 5: p->pid_vel_kp = (int16_t)val; break;
    case 6: p->pid_vel_ki = (int16_t)val; break;
    case 7: p->pid_vel_kd = (int16_t)val; break;
    case 8: p->encoder_cpr = (uint16_t)val; break;
    case 9:
        if (val < 0 || val > 5) {
            return 0;
        }
        p->commutation_offset = (uint8_t)val;
        commutation_set_offset(p->commutation_offset);
        break;
    default:
        return 0;
    }

    return 1;
}

static uint8_t params_get_by_id(const params_t *p, uint8_t param_id, int32_t *pval)
{
    switch (param_id) {
    case 0: *pval = p->mode; break;
    case 1: *pval = p->torque_limit; break;
    case 2: *pval = p->pid_pos_kp; break;
    case 3: *pval = p->pid_pos_ki; break;
    case 4: *pval = p->pid_pos_kd; break;
    case 5: *pval = p->pid_vel_kp; break;
    case 6: *pval = p->pid_vel_ki; break;
    case 7: *pval = p->pid_vel_kd; break;
    case 8: *pval = p->encoder_cpr; break;
    case 9: *pval = p->commutation_offset; break;
    default:
        return 0;
    }

    return 1;
}

static const char *parse_param_id(const char *s, uint8_t *param_id)
{
    int32_t parsed;
    const char *end = parse_int32(s, &parsed);

    if (end == s || parsed < 0 || parsed > 255) {
        return 0;
    }

    *param_id = (uint8_t)parsed;
    return end;
}

static void process_command(void)
{
    int32_t val;

    if (cmd_len == 0)
        return;

    switch (cmd_buf[0]) {
    case 'p':
    case 'P':  /* Set target position */
        parse_int32(&cmd_buf[1], &val);
        motor_set_mode(MODE_POSITION);
        motor_set_position(val);
        motor_start();
        send_ok();
        break;
    case 'v':
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
    case 't':
    case 'T':  /* Set torque limit (mA) */
        parse_int32(&cmd_buf[1], &val);
        if (val > 0 && val <= CURRENT_LIMIT_MA) {
            motor_set_torque_limit((uint16_t)val);
            send_ok();
        } else {
            send_err();
        }
        break;
    case 'd':
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
    case 's':
    case 'S':  /* Stop (brake) */
        motor_stop();
        send_ok();
        break;
    case 'r':
    case 'R':  /* Release (coast) */
        motor_release();
        send_ok();
        break;

    case '?':  /* Query status */
        protocol_send_status();
        break;
    case 'h':
    case 'H':  /* Query raw and decoded hall state */
        send_hall_status();
        break;
    case 'k':
    case 'K':  /* Query low-level commutation drive state */
        send_drive_status();
        break;
    case 'c':
    case 'C':  /* Clear fault */
        motor_clear_fault();
        send_ok();
        break;
    case 'z':
    case 'Z':  /* Zero encoder */
        encoder_set_position(0);
        send_ok();
        break;
    case 'w':
    case 'W':  /* Write parameter: W<param>=<value> */
        {
            params_t p;
            uint8_t param_id;
            const char *sep;
            flash_load_params(&p);
            sep = parse_param_id(&cmd_buf[1], &param_id);
            if (sep && *sep == '=' && *(sep + 1) != '\0') {
                parse_int32(sep + 1, &val);
                if (!params_set_by_id(&p, param_id, val)) {
                    send_err();
                    return;
                }
                flash_save_params(&p);
                send_ok();
            } else {
                send_err();
            }
        }
        break;
    case 'g':
    case 'G':  /* Get parameter */
        {
            params_t p;
            uint8_t param_id;
            int32_t pval;
            const char *end;
            flash_load_params(&p);
            end = parse_param_id(&cmd_buf[1], &param_id);
            if (end && *end == '\0' && params_get_by_id(&p, param_id, &pval)) {
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
    uart_puts(",hall:");
    uart_put_int(hall_read());
    uart_puts(",offset:");
    uart_put_int(commutation_get_offset());
    uart_putc('\n');
}
