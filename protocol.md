# BL4818 Ring Protocol

This document describes the UART protocol implemented by the firmware in
[`src/protocol.c`](src/protocol.c).

Default line settings:

- `115200` baud
- `8N1`
- UART1 on the programming header pads: `P = TX`, `S = RX`

## Transport Model

The firmware is designed for a one-way UART ring:

```text
Master TX -> Dev0 RX -> Dev0 TX -> Dev1 RX -> ... -> DevN TX -> Master RX
```

Only one transaction should be in flight at a time. The master sends one packet,
waits for the transformed packet or response to return, then sends the next.

The firmware does not emit unsolicited protocol bytes at boot. The line stays
quiet until the master starts a transaction.

Legacy local PWM+direction control remains available only while the node is
still unassigned. The first valid enumeration packet claims a serial address,
stops any locally driven motion, and hands command ownership to the serial
protocol until the next reboot. On the stock board, the legacy PWM input is
treated as active-low because `P0.4` has a pull-up to 5 V, so idle/high maps to
zero torque.

## Scope

This document describes the production firmware protocol only. The production
image is binary-only.

The separate bench image keeps its own local ASCII debug commands for bring-up
and gate-drive probing.

## Binary Packet Types

| First byte | Meaning | Device behavior |
|---|---|---|
| `0x7D` | Diagnostic response | Always forward unchanged after CRC validation |
| `0x7F` | Enumerate | Claim current counter as address, increment, forward |
| `0xFF` | Broadcast duty | Consume first duty slot, decrement slot count, forward remaining slots |
| `0x80`-`0x8F` | Addressed command | If address matches me: swallow, execute, reply with status |
| `0x7E` | Status response | Always forward unchanged after CRC validation |

The current implementation supports addressed device IDs `0..15`.

All production frames end with a CRC-8 byte. The CRC uses polynomial `0x07`,
initial value `0x00`, no reflection, and no final XOR. Bad frames are dropped,
the current parse is aborted, and queued RX bytes are flushed so the next clean
transaction can recover deterministically.

## Enumeration

Master sends:

```text
[0x7F] [0x00] [crc]
```

Each device:

1. Stores the current counter as its address.
2. Increments the counter by one.
3. Recomputes the CRC.
4. Forwards the updated 3-byte packet.

Example with four devices:

```text
Master TX: [7F] [00] [crc]
Dev0 TX:   [7F] [01] [crc]
Dev1 TX:   [7F] [02] [crc]
Dev2 TX:   [7F] [03] [crc]
Dev3 TX:   [7F] [04] [crc]
Master RX: [7F] [04] [crc]
```

`0x04` means there are four devices addressed `0..3`.

## Broadcast Duty Update

The fast path is a duty broadcast:

```text
[0xFF] [slot_count] [d0_hi] [d0_lo] [d1_hi] [d1_lo] ... [crc]
```

`slot_count` is the number of remaining duty slots in the packet, including the
current device. This one-byte count is the only deviation from the original
discussion sketch; it makes forwarding deterministic and avoids relying on idle
gaps to find packet boundaries.

Per-device behavior:

1. Read `slot_count`.
2. If the device has been enumerated, consume the first signed 16-bit duty pair
   for itself, decrement `slot_count`, recompute CRC, and forward the remaining
   packet.
3. If the device is still unassigned, forward the broadcast unchanged. That
   prevents startup noise from commanding motion before enumeration.

Example for four devices:

```text
Master TX: [FF] [04] [d0h] [d0l] [d1h] [d1l] [d2h] [d2l] [d3h] [d3l] [crc]
Dev0 TX:   [FF] [03] [d1h] [d1l] [d2h] [d2l] [d3h] [d3l] [crc]
Dev1 TX:   [FF] [02] [d2h] [d2l] [d3h] [d3l] [crc]
Dev2 TX:   [FF] [01] [d3h] [d3l] [crc]
Dev3 TX:   [FF] [00] [crc]
Master RX: [FF] [00] [crc]
```

Notes:

- Duty is big-endian signed `int16`.
- Valid range is `-PWM_MAX_DUTY .. +PWM_MAX_DUTY`.
- The firmware applies the duty and calls `motor_start()`, matching the existing
  ASCII `D<val>` behavior.
- Devices beyond `slot_count` just forward `[0xFF] [0x00] [crc]` unchanged.

## Addressed Commands

Format:

```text
[0x80 | addr] [cmd] [payload...] [crc]
```

Supported commands:

| Cmd | Payload | Meaning |
|---|---|---|
| `0x01` | `int16` | Set duty and run |
| `0x02` | `uint16` | Set torque limit in mA |
| `0x03` | none | Stop / coast |
| `0x04` | none | Clear fault |
| `0x10` | none | Query status |
| `0x11` | none | Query transport diagnostics |

Payload integers are big-endian.

For every addressed command, the addressed device emits the same fixed-length
status response after executing the command. That gives the master a positive
completion marker for both queries and configuration writes.

## Status Response

Format:

```text
[0x7E] [state] [fault] [cur_hi] [cur_lo] [hall] [crc]
```

Fields:

- `state`: `motor_state_t`
- `fault`: `fault_code_t`
- `current`: measured current in mA, unsigned big-endian
- `hall`: decoded hall state (`1..6` valid, `0` or `7` invalid)

Example:

```text
Master TX: [83] [10] [crc]
Master RX: [7E] [01] [00] [01] [F4] [03] [crc]
```

That means:

- address `3`
- `state = 1`
- `fault = 0`
- `current = 0x01F4 = 500 mA`
- `hall = 3`

## Diagnostic Response

Format:

```text
[0x7D] [flags] [last_abort] [bad_crc_hi] [bad_crc_lo]
       [timeout_hi] [timeout_lo] [overflow_hi] [overflow_lo]
       [frame_abort_hi] [frame_abort_lo] [false_sync_hi] [false_sync_lo]
       [addr_fwd_hi] [addr_fwd_lo] [crc]
```

Fields:

- `flags bit0`: node is enumerated
- `flags bit1`: last reset was caused by watchdog
- `last_abort`: most recent parser abort reason code
- `bad_crc`: count of CRC failures seen by the parser or forwarded-frame validator
- `timeout_abort`: count of frame timeouts
- `overflow`: count of UART RX overflows
- `frame_abort`: total parser abort count
- `false_sync`: count of bad forwarded `0x7D`/`0x7E` frames
- `addr_fwd`: count of valid non-local addressed frames forwarded onward

## Recommended Master Loop

A simple host control loop looks like:

1. Send enumeration once after boot.
2. Send one broadcast duty packet every cycle.
3. Send one addressed status query each cycle in round-robin order.
4. Wait for each packet to return before sending the next.

This keeps the fast path small while still refreshing every device's status
regularly.

## Not Implemented

The firmware does not implement a broadcast "status all" packet. That variant
requires a growing response frame, which complicates the forwarding state
machine. The shipped protocol uses individual addressed queries instead.
