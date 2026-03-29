/*
 * Binary Ring Protocol
 *
 * The production firmware accepts only the binary ring protocol:
 *
 *   0x7F [counter]
 *       Enumerate. Each device claims the current counter as its address,
 *       increments it, and forwards the packet.
 *
 *   0xFF [slots] [d0_hi] [d0_lo] ...
 *       Broadcast duty update. Each device decrements slots, consumes the
 *       first duty pair for itself, and forwards the remaining pairs.
 *
 *   [0x80 | addr] [cmd] [payload...]
 *       Addressed command. The addressed device swallows the command,
 *       executes it, and emits a fixed status response:
 *
 *   0x7E [state] [fault] [cur_hi] [cur_lo] [hall]
 *
 * The bench firmware retains the separate ASCII debug protocol.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "protocol.h"
#include "uart.h"
#include "motor.h"
#include "hall.h"

#define PROTO_SYNC_STATUS         0x7Eu
#define PROTO_SYNC_ENUM           0x7Fu
#define PROTO_SYNC_ADDR_BASE      0x80u
#define PROTO_SYNC_ADDR_MASK      0xF0u
#define PROTO_SYNC_BROADCAST      0xFFu

#define PROTO_ADDR_UNASSIGNED     0xFFu
#define PROTO_MAX_ADDRESSED_DEV   16u

#define PROTO_CMD_SET_DUTY        0x01u
#define PROTO_CMD_SET_TORQUE      0x02u
#define PROTO_CMD_STOP            0x03u
#define PROTO_CMD_CLEAR_FAULT     0x04u
#define PROTO_CMD_QUERY_STATUS    0x10u

#define PROTO_STATUS_PAYLOAD_SIZE 5u

typedef enum {
    PROTO_RX_IDLE = 0,
    PROTO_RX_ENUM_COUNTER,
    PROTO_RX_ADDR_CMD,
    PROTO_RX_ADDR_PAYLOAD,
    PROTO_RX_STATUS_FORWARD,
    PROTO_RX_BROADCAST_COUNT,
    PROTO_RX_BROADCAST_DUTY_HI,
    PROTO_RX_BROADCAST_DUTY_LO,
    PROTO_RX_BROADCAST_FORWARD
} proto_rx_state_t;

static uint8_t device_addr;
static proto_rx_state_t rx_state;
static uint8_t rx_addr;
static uint8_t rx_cmd;
static uint8_t rx_payload[2];
static uint8_t rx_payload_len;
static uint8_t rx_payload_pos;
static uint8_t rx_targeted;
static uint16_t rx_forward_remaining;

static void send_status_binary(void)
{
    uint16_t current = motor_get_current();

    uart_putc(PROTO_SYNC_STATUS);
    uart_putc((uint8_t)motor_get_state());
    uart_putc((uint8_t)motor_get_fault());
    uart_putc((uint8_t)(current >> 8));
    uart_putc((uint8_t)(current & 0xFFu));
    uart_putc(hall_read());
}

static uint8_t command_payload_len(uint8_t cmd)
{
    if (cmd == PROTO_CMD_SET_DUTY || cmd == PROTO_CMD_SET_TORQUE)
        return 2;

    return 0;
}

static void execute_set_duty(int16_t duty)
{
    if (duty < -(int16_t)PWM_MAX_DUTY || duty > (int16_t)PWM_MAX_DUTY)
        return;

    motor_set_duty(duty);
    motor_start();
}

static void execute_set_torque(uint16_t ma)
{
    if (ma == 0u || ma > CURRENT_LIMIT_MA)
        return;

    motor_set_torque_limit(ma);
}

static void execute_addressed_command(void)
{
    uint16_t value_u16;
    int16_t value_i16;

    if (rx_cmd == PROTO_CMD_SET_DUTY) {
        value_u16 = (uint16_t)(((uint16_t)rx_payload[0] << 8) | rx_payload[1]);
        value_i16 = (int16_t)value_u16;
        execute_set_duty(value_i16);
    } else if (rx_cmd == PROTO_CMD_SET_TORQUE) {
        value_u16 = (uint16_t)(((uint16_t)rx_payload[0] << 8) | rx_payload[1]);
        execute_set_torque(value_u16);
    } else if (rx_cmd == PROTO_CMD_STOP) {
        motor_stop();
    } else if (rx_cmd == PROTO_CMD_CLEAR_FAULT) {
        motor_clear_fault();
    }

    send_status_binary();
}

static void protocol_reset(void)
{
    rx_state = PROTO_RX_IDLE;
    rx_payload_len = 0;
    rx_payload_pos = 0;
    rx_targeted = 0;
    rx_forward_remaining = 0;
}

void protocol_init(void)
{
    device_addr = PROTO_ADDR_UNASSIGNED;
    uart_rx_flush();
    protocol_reset();
}

void protocol_poll(void)
{
    while (uart_available()) {
        uint8_t c = (uint8_t)uart_getc();

        switch (rx_state) {
        case PROTO_RX_IDLE:
            if (c == PROTO_SYNC_ENUM) {
                uart_putc(PROTO_SYNC_ENUM);
                rx_state = PROTO_RX_ENUM_COUNTER;
                break;
            }

            if (c == PROTO_SYNC_STATUS) {
                uart_putc(c);
                rx_forward_remaining = PROTO_STATUS_PAYLOAD_SIZE;
                rx_state = PROTO_RX_STATUS_FORWARD;
                break;
            }

            if (c == PROTO_SYNC_BROADCAST) {
                rx_state = PROTO_RX_BROADCAST_COUNT;
                break;
            }

            if ((c & PROTO_SYNC_ADDR_MASK) == PROTO_SYNC_ADDR_BASE) {
                rx_addr = c & 0x0Fu;
                rx_targeted = (device_addr != PROTO_ADDR_UNASSIGNED && rx_addr == device_addr) ? 1u : 0u;

                if (!rx_targeted)
                    uart_putc(c);

                rx_state = PROTO_RX_ADDR_CMD;
            }
            break;

        case PROTO_RX_ENUM_COUNTER:
            device_addr = (c < PROTO_MAX_ADDRESSED_DEV) ? c : PROTO_ADDR_UNASSIGNED;
            uart_putc((uint8_t)(c + 1u));
            protocol_reset();
            break;

        case PROTO_RX_STATUS_FORWARD:
            uart_putc(c);
            if (--rx_forward_remaining == 0u)
                protocol_reset();
            break;

        case PROTO_RX_ADDR_CMD:
            rx_cmd = c;
            rx_payload_len = command_payload_len(rx_cmd);
            rx_payload_pos = 0;

            if (!rx_targeted)
                uart_putc(c);

            if (rx_payload_len == 0u) {
                if (rx_targeted)
                    execute_addressed_command();
                protocol_reset();
            } else {
                rx_state = PROTO_RX_ADDR_PAYLOAD;
            }
            break;

        case PROTO_RX_ADDR_PAYLOAD:
            if (rx_payload_pos < sizeof(rx_payload))
                rx_payload[rx_payload_pos] = c;

            rx_payload_pos++;

            if (!rx_targeted)
                uart_putc(c);

            if (rx_payload_pos >= rx_payload_len) {
                if (rx_targeted)
                    execute_addressed_command();
                protocol_reset();
            }
            break;

        case PROTO_RX_BROADCAST_COUNT:
            uart_putc(PROTO_SYNC_BROADCAST);
            uart_putc((uint8_t)(c ? (c - 1u) : 0u));

            if (c == 0u) {
                protocol_reset();
            } else {
                rx_forward_remaining = (uint16_t)(2u * (uint16_t)(c - 1u));
                rx_state = PROTO_RX_BROADCAST_DUTY_HI;
            }
            break;

        case PROTO_RX_BROADCAST_DUTY_HI:
            rx_payload[0] = c;
            rx_state = PROTO_RX_BROADCAST_DUTY_LO;
            break;

        case PROTO_RX_BROADCAST_DUTY_LO:
            rx_payload[1] = c;
            execute_set_duty((int16_t)(((uint16_t)rx_payload[0] << 8) | rx_payload[1]));

            if (rx_forward_remaining == 0u) {
                protocol_reset();
            } else {
                rx_state = PROTO_RX_BROADCAST_FORWARD;
            }
            break;

        case PROTO_RX_BROADCAST_FORWARD:
            uart_putc(c);
            if (--rx_forward_remaining == 0u)
                protocol_reset();
            break;

        default:
            protocol_reset();
            break;
        }
    }
}
