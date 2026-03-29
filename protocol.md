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

The firmware never transmits unsolicited bytes at boot. That matters in a ring:
the line stays quiet until the master starts a transaction.

## Scope

This document describes the production firmware protocol only. The production
image is binary-only.

The separate bench image keeps its own local ASCII debug commands for bring-up
and gate-drive probing.

## Binary Packet Types

| First byte | Meaning | Device behavior |
|---|---|---|
| `0x7F` | Enumerate | Claim current counter as address, increment, forward |
| `0xFF` | Broadcast duty | Consume first duty slot, decrement slot count, forward remaining slots |
| `0x80`-`0x8F` | Addressed command | If address matches me: swallow, execute, reply with status |
| `0x7E` | Status response | Always forward unchanged |

The current implementation supports addressed device IDs `0..15`.

## Enumeration

Master sends:

```text
[0x7F] [0x00]
```

Each device:

1. Stores the current counter as its address.
2. Increments the counter by one.
3. Forwards the updated 2-byte packet.

Example with four devices:

```text
Master TX: [7F] [00]
Dev0 TX:   [7F] [01]
Dev1 TX:   [7F] [02]
Dev2 TX:   [7F] [03]
Dev3 TX:   [7F] [04]
Master RX: [7F] [04]
```

`0x04` means there are four devices addressed `0..3`.

## Broadcast Duty Update

The fast path is a duty broadcast:

```text
[0xFF] [slot_count] [d0_hi] [d0_lo] [d1_hi] [d1_lo] ...
```

`slot_count` is the number of remaining duty slots in the packet, including the
current device. This one-byte count is the only deviation from the original
discussion sketch; it makes forwarding deterministic and avoids relying on idle
gaps to find packet boundaries.

Per-device behavior:

1. Read `slot_count`.
2. Forward `[0xFF] [slot_count - 1]`.
3. Consume the first signed 16-bit duty pair for this device.
4. Forward the remaining `2 * (slot_count - 1)` bytes unchanged.

Example for four devices:

```text
Master TX: [FF] [04] [d0h] [d0l] [d1h] [d1l] [d2h] [d2l] [d3h] [d3l]
Dev0 TX:   [FF] [03] [d1h] [d1l] [d2h] [d2l] [d3h] [d3l]
Dev1 TX:   [FF] [02] [d2h] [d2l] [d3h] [d3l]
Dev2 TX:   [FF] [01] [d3h] [d3l]
Dev3 TX:   [FF] [00]
Master RX: [FF] [00]
```

Notes:

- Duty is big-endian signed `int16`.
- Valid range is `-PWM_MAX_DUTY .. +PWM_MAX_DUTY`.
- The firmware applies the duty and calls `motor_start()`, matching the existing
  ASCII `D<val>` behavior.
- Devices beyond `slot_count` just forward `[0xFF] [0x00]` unchanged.

## Addressed Commands

Format:

```text
[0x80 | addr] [cmd] [payload...]
```

Supported commands:

| Cmd | Payload | Meaning |
|---|---|---|
| `0x01` | `int16` | Set duty and run |
| `0x02` | `uint16` | Set torque limit in mA |
| `0x03` | none | Stop / coast |
| `0x04` | none | Clear fault |
| `0x10` | none | Query status |

Payload integers are big-endian.

For every addressed command, the addressed device emits the same fixed-length
status response after executing the command. That gives the master a positive
completion marker for both queries and configuration writes.

## Status Response

Format:

```text
[0x7E] [state] [fault] [cur_hi] [cur_lo] [hall]
```

Fields:

- `state`: `motor_state_t`
- `fault`: `fault_code_t`
- `current`: measured current in mA, unsigned big-endian
- `hall`: decoded hall state (`1..6` valid, `0` or `7` invalid)

Example:

```text
Master TX: [83] [10]
Master RX: [7E] [01] [00] [01] [F4] [03]
```

That means:

- address `3`
- `state = 1`
- `fault = 0`
- `current = 0x01F4 = 500 mA`
- `hall = 3`

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
