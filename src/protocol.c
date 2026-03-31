/*
 * Binary Ring Protocol
 *
 * The production firmware accepts only the binary ring protocol:
 *
 *   0x7F [counter] [crc8]
 *       Enumerate. Each device claims the current counter as its address,
 *       increments it, and forwards the packet.
 *
 *   0xFF [slots] [d0_hi] [d0_lo] ... [crc8]
 *       Broadcast duty update. Each device decrements slots, consumes the
 *       first duty pair for itself, and forwards the remaining pairs.
 *
 *   [0x80 | addr] [cmd] [payload...] [crc8]
 *       Addressed command. The addressed device swallows the command,
 *       executes it, and emits a fixed status response:
 *
 *   0x7E [state] [fault] [cur_hi] [cur_lo] [hall] [crc8]
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

#define PROTO_CRC8_INIT           0x00u
#define PROTO_FRAME_TIMEOUT_MS    5u

#define PROTO_CMD_SET_DUTY        0x01u
#define PROTO_CMD_SET_TORQUE      0x02u
#define PROTO_CMD_STOP            0x03u
#define PROTO_CMD_CLEAR_FAULT     0x04u
#define PROTO_CMD_QUERY_STATUS    0x10u

#define PROTO_STATUS_DATA_SIZE    5u
#define PROTO_STATUS_TAIL_SIZE    (PROTO_STATUS_DATA_SIZE + 1u)
#define PROTO_BROADCAST_FRAME_MAX (2u + (2u * PROTO_MAX_ADDRESSED_DEV) + 1u)

typedef enum {
    PROTO_RX_IDLE = 0,
    PROTO_RX_ENUM_COUNTER,
    PROTO_RX_ENUM_CRC,
    PROTO_RX_ADDR_CMD,
    PROTO_RX_ADDR_PAYLOAD,
    PROTO_RX_ADDR_CRC,
    PROTO_RX_STATUS_FORWARD,
    PROTO_RX_BROADCAST_COUNT,
    PROTO_RX_BROADCAST_DUTY_HI,
    PROTO_RX_BROADCAST_DUTY_LO,
    PROTO_RX_BROADCAST_FORWARD,
    PROTO_RX_BROADCAST_CRC
} proto_rx_state_t;

static uint8_t device_addr = PROTO_ADDR_UNASSIGNED;
static proto_rx_state_t rx_state;
static uint8_t rx_addr;
static uint8_t rx_cmd;
static uint8_t rx_payload[2];
static uint8_t rx_payload_len;
static uint8_t rx_payload_pos;
static uint8_t rx_targeted;
static uint8_t rx_crc;
static uint8_t rx_broadcast_consume;
static uint8_t rx_broadcast_tx_pos;
static uint8_t rx_timeout_ms;
static uint16_t rx_forward_remaining;
static uint8_t __xdata rx_broadcast_tx[PROTO_BROADCAST_FRAME_MAX];

#if HOST_COMMS_TIMEOUT_MS
static uint16_t host_comms_countdown;
#endif

static void protocol_restart_timeout(void)
{
    if (rx_state != PROTO_RX_IDLE)
        rx_timeout_ms = PROTO_FRAME_TIMEOUT_MS;
}

#if HOST_COMMS_TIMEOUT_MS
static void host_comms_kick(void)
{
    host_comms_countdown = HOST_COMMS_TIMEOUT_MS;
}
#endif

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    uint8_t bit;

    crc ^= data;

    for (bit = 0; bit < 8u; bit++) {
        if (crc & 0x80u) {
            crc = (uint8_t)((crc << 1) ^ 0x07u);
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

static uint8_t broadcast_tx_crc(void)
{
    uint8_t crc = PROTO_CRC8_INIT;
    uint8_t i;

    for (i = 0; i < rx_broadcast_tx_pos; i++)
        crc = crc8_update(crc, rx_broadcast_tx[i]);

    return crc;
}

static void send_broadcast_tx(void)
{
    uint8_t i;

    for (i = 0; i < rx_broadcast_tx_pos; i++)
        uart_putc(rx_broadcast_tx[i]);

    uart_putc(broadcast_tx_crc());
}

static void send_status_binary(void)
{
    uint16_t current = motor_get_current();
    uint8_t state = (uint8_t)motor_get_state();
    uint8_t fault = (uint8_t)motor_get_fault();
    uint8_t hall = hall_read();
    uint8_t crc = PROTO_CRC8_INIT;

    crc = crc8_update(crc, PROTO_SYNC_STATUS);
    uart_putc(PROTO_SYNC_STATUS);
    crc = crc8_update(crc, state);
    uart_putc(state);
    crc = crc8_update(crc, fault);
    uart_putc(fault);
    crc = crc8_update(crc, (uint8_t)(current >> 8));
    uart_putc((uint8_t)(current >> 8));
    crc = crc8_update(crc, (uint8_t)(current & 0xFFu));
    uart_putc((uint8_t)(current & 0xFFu));
    crc = crc8_update(crc, hall);
    uart_putc(hall);
    uart_putc(crc);
}

static void send_status_binary_with_state(uint8_t state)
{
    uint16_t current = motor_get_current();
    uint8_t fault = (uint8_t)motor_get_fault();
    uint8_t hall = hall_read();
    uint8_t crc = PROTO_CRC8_INIT;

    crc = crc8_update(crc, PROTO_SYNC_STATUS);
    uart_putc(PROTO_SYNC_STATUS);
    crc = crc8_update(crc, state);
    uart_putc(state);
    crc = crc8_update(crc, fault);
    uart_putc(fault);
    crc = crc8_update(crc, (uint8_t)(current >> 8));
    uart_putc((uint8_t)(current >> 8));
    crc = crc8_update(crc, (uint8_t)(current & 0xFFu));
    uart_putc((uint8_t)(current & 0xFFu));
    crc = crc8_update(crc, hall);
    uart_putc(hall);
    uart_putc(crc);
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

    if (duty == 0) {
        motor_stop();
        return;
    }

    motor_set_duty(duty);
    motor_start();
}

static uint8_t prepare_set_duty(int16_t duty)
{
    if (duty < -(int16_t)PWM_MAX_DUTY || duty > (int16_t)PWM_MAX_DUTY)
        return 0u;

    if (duty == 0) {
        motor_stop();
        return 0u;
    }

    if (motor_get_state() == MOTOR_FAULT)
        return 0u;

    motor_set_duty(duty);
    return 1u;
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
    uint8_t start_after_reply = 0u;

#if HOST_COMMS_TIMEOUT_MS
    host_comms_kick();
#endif

    if (rx_cmd == PROTO_CMD_SET_DUTY) {
        value_u16 = (uint16_t)(((uint16_t)rx_payload[0] << 8) | rx_payload[1]);
        value_i16 = (int16_t)value_u16;
        start_after_reply = prepare_set_duty(value_i16);
        if (start_after_reply) {
            send_status_binary_with_state((uint8_t)MOTOR_RUN);
            motor_start();
        } else {
            send_status_binary();
        }
        return;
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
    rx_crc = PROTO_CRC8_INIT;
    rx_broadcast_consume = 0;
    rx_broadcast_tx_pos = 0;
    rx_timeout_ms = 0;
    rx_forward_remaining = 0;
}

void protocol_init(void)
{
    device_addr = PROTO_ADDR_UNASSIGNED;
    uart_rx_flush();
    protocol_reset();
#if HOST_COMMS_TIMEOUT_MS
    host_comms_countdown = HOST_COMMS_TIMEOUT_MS;
#endif
}

void protocol_tick_1khz(void)
{
    if (rx_state != PROTO_RX_IDLE && rx_timeout_ms != 0u) {
        rx_timeout_ms--;
        if (rx_timeout_ms == 0u)
            protocol_reset();
    }

#if HOST_COMMS_TIMEOUT_MS
    /* Stop motor if host goes silent after enumeration */
    if (device_addr != PROTO_ADDR_UNASSIGNED && host_comms_countdown != 0u) {
        host_comms_countdown--;
        if (host_comms_countdown == 0u)
            motor_stop();
    }
#endif
}

uint8_t protocol_is_enumerated(void)
{
    return (device_addr != PROTO_ADDR_UNASSIGNED) ? 1u : 0u;
}

void protocol_poll(void)
{
    if (uart_rx_overflowed()) {
        uart_rx_flush();
        protocol_reset();
        return;
    }

    while (uart_available()) {
        uint8_t c = (uint8_t)uart_getc();

        if (rx_state != PROTO_RX_IDLE)
            protocol_restart_timeout();

        switch (rx_state) {
        case PROTO_RX_IDLE:
            if (c == PROTO_SYNC_ENUM) {
                rx_crc = crc8_update(PROTO_CRC8_INIT, c);
                rx_state = PROTO_RX_ENUM_COUNTER;
                protocol_restart_timeout();
                break;
            }

            if (c == PROTO_SYNC_STATUS) {
                uart_putc(c);
                rx_forward_remaining = PROTO_STATUS_TAIL_SIZE;
                rx_state = PROTO_RX_STATUS_FORWARD;
                protocol_restart_timeout();
                break;
            }

            if (c == PROTO_SYNC_BROADCAST) {
                rx_crc = crc8_update(PROTO_CRC8_INIT, c);
                rx_state = PROTO_RX_BROADCAST_COUNT;
                protocol_restart_timeout();
                break;
            }

            if ((c & PROTO_SYNC_ADDR_MASK) == PROTO_SYNC_ADDR_BASE) {
                rx_addr = c & 0x0Fu;
                rx_targeted = (device_addr != PROTO_ADDR_UNASSIGNED && rx_addr == device_addr) ? 1u : 0u;
                rx_crc = crc8_update(PROTO_CRC8_INIT, c);

                if (!rx_targeted)
                    uart_putc(c);

                rx_state = PROTO_RX_ADDR_CMD;
                protocol_restart_timeout();
            }
            break;

        case PROTO_RX_ENUM_COUNTER:
            rx_payload[0] = c;
            rx_crc = crc8_update(rx_crc, c);
            rx_state = PROTO_RX_ENUM_CRC;
            break;

        case PROTO_RX_ENUM_CRC:
            if (c == rx_crc && rx_payload[0] < PROTO_MAX_ADDRESSED_DEV) {
                uint8_t next = (uint8_t)(rx_payload[0] + 1u);
                uint8_t forward_crc = PROTO_CRC8_INIT;

                device_addr = rx_payload[0];
                uart_putc(PROTO_SYNC_ENUM);
                uart_putc(next);
                forward_crc = crc8_update(forward_crc, PROTO_SYNC_ENUM);
                forward_crc = crc8_update(forward_crc, next);
                uart_putc(forward_crc);
            }
            protocol_reset();
            break;

        case PROTO_RX_STATUS_FORWARD:
            uart_putc(c);
            if (--rx_forward_remaining == 0u)
                protocol_reset();
            break;

        case PROTO_RX_ADDR_CMD:
            rx_cmd = c;
            rx_crc = crc8_update(rx_crc, c);
            rx_payload_len = command_payload_len(rx_cmd);
            rx_payload_pos = 0;

            if (!rx_targeted)
                uart_putc(c);

            if (rx_payload_len == 0u) {
                rx_state = PROTO_RX_ADDR_CRC;
            } else {
                rx_state = PROTO_RX_ADDR_PAYLOAD;
            }
            break;

        case PROTO_RX_ADDR_PAYLOAD:
            if (rx_payload_pos < sizeof(rx_payload))
                rx_payload[rx_payload_pos] = c;

            rx_payload_pos++;
            rx_crc = crc8_update(rx_crc, c);

            if (!rx_targeted)
                uart_putc(c);

            if (rx_payload_pos >= rx_payload_len)
                rx_state = PROTO_RX_ADDR_CRC;
            break;

        case PROTO_RX_ADDR_CRC:
            if (!rx_targeted)
                uart_putc(c);

            if (c == rx_crc && rx_targeted)
                execute_addressed_command();
            protocol_reset();
            break;

        case PROTO_RX_BROADCAST_COUNT:
            if (c > PROTO_MAX_ADDRESSED_DEV) {
                protocol_reset();
                break;
            }

            rx_crc = crc8_update(rx_crc, c);
            rx_broadcast_consume = (device_addr != PROTO_ADDR_UNASSIGNED && c != 0u) ? 1u : 0u;
            rx_broadcast_tx[0] = PROTO_SYNC_BROADCAST;
            rx_broadcast_tx[1] = rx_broadcast_consume ? (uint8_t)(c - 1u) : c;
            rx_broadcast_tx_pos = 2;

            if (c == 0u) {
                rx_state = PROTO_RX_BROADCAST_CRC;
            } else if (rx_broadcast_consume) {
                rx_forward_remaining = (uint16_t)(2u * (uint16_t)(c - 1u));
                rx_state = PROTO_RX_BROADCAST_DUTY_HI;
            } else {
                rx_forward_remaining = (uint16_t)(2u * (uint16_t)c);
                rx_state = PROTO_RX_BROADCAST_FORWARD;
            }
            break;

        case PROTO_RX_BROADCAST_DUTY_HI:
            rx_payload[0] = c;
            rx_crc = crc8_update(rx_crc, c);
            rx_state = PROTO_RX_BROADCAST_DUTY_LO;
            break;

        case PROTO_RX_BROADCAST_DUTY_LO:
            rx_payload[1] = c;
            rx_crc = crc8_update(rx_crc, c);

            if (rx_forward_remaining == 0u) {
                rx_state = PROTO_RX_BROADCAST_CRC;
            } else {
                rx_state = PROTO_RX_BROADCAST_FORWARD;
            }
            break;

        case PROTO_RX_BROADCAST_FORWARD:
            if (rx_broadcast_tx_pos < sizeof(rx_broadcast_tx))
                rx_broadcast_tx[rx_broadcast_tx_pos] = c;

            rx_broadcast_tx_pos++;
            rx_crc = crc8_update(rx_crc, c);

            if (--rx_forward_remaining == 0u)
                rx_state = PROTO_RX_BROADCAST_CRC;
            break;

        case PROTO_RX_BROADCAST_CRC:
            if (c == rx_crc) {
                uint8_t start_after_reply = 0u;

                if (rx_broadcast_consume) {
                    start_after_reply = prepare_set_duty((int16_t)(((uint16_t)rx_payload[0] << 8) | rx_payload[1]));
#if HOST_COMMS_TIMEOUT_MS
                    host_comms_kick();
#endif
                }

                send_broadcast_tx();

                if (start_after_reply)
                    motor_start();
            }
            protocol_reset();
            break;

        default:
            protocol_reset();
            break;
        }

        if (uart_rx_overflowed()) {
            uart_rx_flush();
            protocol_reset();
            return;
        }
    }
}
