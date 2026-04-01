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

#define PROTO_SYNC_DIAG           0x7Du
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
#define PROTO_CMD_QUERY_DIAG      0x11u

#define PROTO_STATUS_DATA_SIZE    5u
#define PROTO_STATUS_FRAME_SIZE   (1u + PROTO_STATUS_DATA_SIZE + 1u)
#define PROTO_DIAG_COUNTER_COUNT  6u
#define PROTO_DIAG_DATA_SIZE      (2u + (2u * PROTO_DIAG_COUNTER_COUNT))
#define PROTO_DIAG_FRAME_SIZE     (1u + PROTO_DIAG_DATA_SIZE + 1u)
#define PROTO_POLL_BYTE_BUDGET    32u
#define PROTO_ADDR_FRAME_MAX      5u
#define PROTO_BROADCAST_FRAME_MAX (2u + (2u * PROTO_MAX_ADDRESSED_DEV) + 1u)

#define PROTO_DIAG_FLAG_ENUMERATED  0x01u
#define PROTO_DIAG_FLAG_WDT_RESET   0x02u

#define PROTO_ABORT_NONE             0u
#define PROTO_ABORT_TIMEOUT          1u
#define PROTO_ABORT_RX_OVERFLOW      2u
#define PROTO_ABORT_ENUM_BAD_CRC     3u
#define PROTO_ABORT_ADDR_BAD_CRC     4u
#define PROTO_ABORT_BROADCAST_COUNT  5u
#define PROTO_ABORT_BROADCAST_BAD_CRC 6u
#define PROTO_ABORT_STATUS_BAD_CRC   7u
#define PROTO_ABORT_DIAG_BAD_CRC     8u
#define PROTO_ABORT_PARSER_DEFAULT   9u

typedef enum {
    PROTO_RX_IDLE = 0,
    PROTO_RX_ENUM_IGNORE_COUNTER,
    PROTO_RX_ENUM_IGNORE_CRC,
    PROTO_RX_ENUM_COUNTER,
    PROTO_RX_ENUM_CRC,
    PROTO_RX_ADDR_CMD,
    PROTO_RX_ADDR_PAYLOAD,
    PROTO_RX_ADDR_CRC,
    PROTO_RX_DIAG_FORWARD,
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
static uint8_t rx_addr_tx[PROTO_ADDR_FRAME_MAX];
static uint8_t rx_diag_tx_pos;
static uint8_t rx_status_tx_pos;
static uint8_t rx_broadcast_consume;
static uint8_t rx_broadcast_tx_pos;
static uint8_t rx_timeout_ms;
static uint16_t rx_forward_remaining;
static uint16_t diag_bad_crc_count;
static uint16_t diag_timeout_abort_count;
static uint16_t diag_rx_overflow_count;
static uint16_t diag_frame_abort_count;
static uint16_t diag_false_sync_count;
static uint16_t diag_addr_forward_count;
static uint8_t diag_last_abort_reason;
static uint8_t __xdata rx_diag_tx[PROTO_DIAG_FRAME_SIZE];
static uint8_t __xdata rx_status_tx[PROTO_STATUS_FRAME_SIZE];
static uint8_t __xdata rx_broadcast_tx[PROTO_BROADCAST_FRAME_MAX];

#if HOST_COMMS_TIMEOUT_MS
static uint16_t host_comms_countdown;
#endif

extern uint8_t wdt_reset_detected(void);

static void diag_count_inc(uint16_t *value)
{
    if (*value != 0xFFFFu)
        (*value)++;
}

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

static void send_fixed_frame(const uint8_t *frame, uint8_t length)
{
    uint8_t i;

    for (i = 0; i < length; i++)
        uart_putc(frame[i]);
}

static void send_addressed_tx(uint8_t length)
{
    send_fixed_frame(rx_addr_tx, length);
}

static void send_diag_binary(void)
{
    uint8_t flags = 0u;
    uint8_t crc = PROTO_CRC8_INIT;
    uint16_t counters[PROTO_DIAG_COUNTER_COUNT];
    uint8_t i;

    if (device_addr != PROTO_ADDR_UNASSIGNED)
        flags |= PROTO_DIAG_FLAG_ENUMERATED;
    if (wdt_reset_detected())
        flags |= PROTO_DIAG_FLAG_WDT_RESET;

    counters[0] = diag_bad_crc_count;
    counters[1] = diag_timeout_abort_count;
    counters[2] = diag_rx_overflow_count;
    counters[3] = diag_frame_abort_count;
    counters[4] = diag_false_sync_count;
    counters[5] = diag_addr_forward_count;

    crc = crc8_update(crc, PROTO_SYNC_DIAG);
    uart_putc(PROTO_SYNC_DIAG);
    crc = crc8_update(crc, flags);
    uart_putc(flags);
    crc = crc8_update(crc, diag_last_abort_reason);
    uart_putc(diag_last_abort_reason);

    for (i = 0; i < PROTO_DIAG_COUNTER_COUNT; i++) {
        uint8_t hi = (uint8_t)(counters[i] >> 8);
        uint8_t lo = (uint8_t)(counters[i] & 0xFFu);

        crc = crc8_update(crc, hi);
        uart_putc(hi);
        crc = crc8_update(crc, lo);
        uart_putc(lo);
    }

    uart_putc(crc);
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
    } else if (rx_cmd == PROTO_CMD_QUERY_DIAG) {
        send_diag_binary();
        return;
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
    rx_diag_tx_pos = 0;
    rx_status_tx_pos = 0;
    rx_broadcast_consume = 0;
    rx_broadcast_tx_pos = 0;
    rx_timeout_ms = 0;
    rx_forward_remaining = 0;
}

static void protocol_abort_frame(uint8_t reason)
{
    diag_count_inc(&diag_frame_abort_count);
    diag_last_abort_reason = reason;
    uart_rx_flush();
    protocol_reset();
}

void protocol_init(void)
{
    device_addr = PROTO_ADDR_UNASSIGNED;
    diag_bad_crc_count = 0;
    diag_timeout_abort_count = 0;
    diag_rx_overflow_count = 0;
    diag_frame_abort_count = 0;
    diag_false_sync_count = 0;
    diag_addr_forward_count = 0;
    diag_last_abort_reason = PROTO_ABORT_NONE;
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
        if (rx_timeout_ms == 0u) {
            diag_count_inc(&diag_timeout_abort_count);
            protocol_abort_frame(PROTO_ABORT_TIMEOUT);
        }
    }

#if HOST_COMMS_TIMEOUT_MS
    /* Stop motor if host goes silent after enumeration */
    if (device_addr != PROTO_ADDR_UNASSIGNED && host_comms_countdown != 0u) {
        host_comms_countdown--;
        if (host_comms_countdown == 0u) {
            if (motor_get_state() == MOTOR_RUN)
                motor_stop();
        }
    }
#endif
}

uint8_t protocol_is_enumerated(void)
{
    return (device_addr != PROTO_ADDR_UNASSIGNED) ? 1u : 0u;
}

void protocol_poll(void)
{
    uint8_t budget = PROTO_POLL_BYTE_BUDGET;

    if (uart_rx_overflowed()) {
        diag_count_inc(&diag_rx_overflow_count);
        protocol_abort_frame(PROTO_ABORT_RX_OVERFLOW);
        return;
    }

    while (budget != 0u && uart_available()) {
        uint8_t c = (uint8_t)uart_getc();
        budget--;

        if (rx_state != PROTO_RX_IDLE)
            protocol_restart_timeout();

        switch (rx_state) {
        case PROTO_RX_IDLE:
            if (c == PROTO_SYNC_DIAG) {
                rx_diag_tx[0] = c;
                rx_diag_tx_pos = 1u;
                rx_state = PROTO_RX_DIAG_FORWARD;
                protocol_restart_timeout();
                break;
            }

            if (c == PROTO_SYNC_ENUM) {
                if (device_addr != PROTO_ADDR_UNASSIGNED) {
                    /* Already enumerated — ignore the sync byte so it
                     * cannot consume the start of the next real packet.
                     * Re-enumerate requires a power cycle. */
                    /* Ignore the rest of the 3-byte enumerate frame. */
                    rx_state = PROTO_RX_ENUM_IGNORE_COUNTER;
                    protocol_restart_timeout();
                    break;
                }
                rx_crc = crc8_update(PROTO_CRC8_INIT, c);
                rx_state = PROTO_RX_ENUM_COUNTER;
                protocol_restart_timeout();
                break;
            }

            if (c == PROTO_SYNC_STATUS) {
                rx_status_tx[0] = c;
                rx_status_tx_pos = 1u;
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
                rx_addr_tx[0] = c;

                rx_state = PROTO_RX_ADDR_CMD;
                protocol_restart_timeout();
            }
            break;

        case PROTO_RX_ENUM_IGNORE_COUNTER:
            rx_state = PROTO_RX_ENUM_IGNORE_CRC;
            break;

        case PROTO_RX_ENUM_IGNORE_CRC:
            protocol_reset();
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

                if (device_addr == PROTO_ADDR_UNASSIGNED)
                    device_addr = rx_payload[0];
                uart_putc(PROTO_SYNC_ENUM);
                uart_putc(next);
                forward_crc = crc8_update(forward_crc, PROTO_SYNC_ENUM);
                forward_crc = crc8_update(forward_crc, next);
                uart_putc(forward_crc);
            } else {
                diag_count_inc(&diag_bad_crc_count);
                protocol_abort_frame(PROTO_ABORT_ENUM_BAD_CRC);
                return;
            }
            protocol_reset();
            break;

        case PROTO_RX_DIAG_FORWARD:
            if (rx_diag_tx_pos < sizeof(rx_diag_tx))
                rx_diag_tx[rx_diag_tx_pos] = c;

            rx_diag_tx_pos++;

            if (rx_diag_tx_pos >= sizeof(rx_diag_tx)) {
                uint8_t crc = PROTO_CRC8_INIT;
                uint8_t i;

                for (i = 0; i < (PROTO_DIAG_FRAME_SIZE - 1u); i++)
                    crc = crc8_update(crc, rx_diag_tx[i]);

                if (rx_diag_tx[PROTO_DIAG_FRAME_SIZE - 1u] == crc) {
                    send_fixed_frame(rx_diag_tx, PROTO_DIAG_FRAME_SIZE);
                } else {
                    diag_count_inc(&diag_bad_crc_count);
                    diag_count_inc(&diag_false_sync_count);
                    protocol_abort_frame(PROTO_ABORT_DIAG_BAD_CRC);
                    return;
                }

                protocol_reset();
            }
            break;

        case PROTO_RX_STATUS_FORWARD:
            if (rx_status_tx_pos < sizeof(rx_status_tx))
                rx_status_tx[rx_status_tx_pos] = c;

            rx_status_tx_pos++;

            if (rx_status_tx_pos >= sizeof(rx_status_tx)) {
                uint8_t crc = PROTO_CRC8_INIT;
                uint8_t i;

                for (i = 0; i < (PROTO_STATUS_FRAME_SIZE - 1u); i++)
                    crc = crc8_update(crc, rx_status_tx[i]);

                if (rx_status_tx[PROTO_STATUS_FRAME_SIZE - 1u] == crc) {
                    send_fixed_frame(rx_status_tx, PROTO_STATUS_FRAME_SIZE);
                } else {
                    diag_count_inc(&diag_bad_crc_count);
                    diag_count_inc(&diag_false_sync_count);
                    protocol_abort_frame(PROTO_ABORT_STATUS_BAD_CRC);
                    return;
                }

                protocol_reset();
            }
            break;

        case PROTO_RX_ADDR_CMD:
            rx_cmd = c;
            rx_crc = crc8_update(rx_crc, c);
            rx_payload_len = command_payload_len(rx_cmd);
            rx_payload_pos = 0;
            rx_addr_tx[1] = c;

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
            if ((uint8_t)(2u + rx_payload_pos - 1u) < sizeof(rx_addr_tx))
                rx_addr_tx[(uint8_t)(2u + rx_payload_pos - 1u)] = c;

            if (rx_payload_pos >= rx_payload_len)
                rx_state = PROTO_RX_ADDR_CRC;
            break;

        case PROTO_RX_ADDR_CRC:
            if ((uint8_t)(2u + rx_payload_len) < sizeof(rx_addr_tx))
                rx_addr_tx[(uint8_t)(2u + rx_payload_len)] = c;

            if (c == rx_crc) {
                if (rx_targeted) {
                    execute_addressed_command();
                } else {
                    diag_count_inc(&diag_addr_forward_count);
                    send_addressed_tx((uint8_t)(3u + rx_payload_len));
                }
            } else {
                diag_count_inc(&diag_bad_crc_count);
                protocol_abort_frame(PROTO_ABORT_ADDR_BAD_CRC);
                return;
            }
            protocol_reset();
            break;

        case PROTO_RX_BROADCAST_COUNT:
            if (c > PROTO_MAX_ADDRESSED_DEV) {
                protocol_abort_frame(PROTO_ABORT_BROADCAST_COUNT);
                return;
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
            } else {
                diag_count_inc(&diag_bad_crc_count);
                protocol_abort_frame(PROTO_ABORT_BROADCAST_BAD_CRC);
                return;
            }
            protocol_reset();
            break;

        default:
            protocol_abort_frame(PROTO_ABORT_PARSER_DEFAULT);
            return;
        }

        if (uart_rx_overflowed()) {
            diag_count_inc(&diag_rx_overflow_count);
            protocol_abort_frame(PROTO_ABORT_RX_OVERFLOW);
            return;
        }
    }
}
