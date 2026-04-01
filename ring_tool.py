#!/usr/bin/env python3
"""
ring_tool.py - Manual binary protocol helper for BL4818 production firmware

Examples:
    python ring_tool.py -p COM7 enumerate
    python ring_tool.py -p COM7 probe
    python ring_tool.py -p COM7 status 0
    python ring_tool.py -p COM7 diag 0
    python ring_tool.py -p COM7 set-duty 0 250
    python ring_tool.py -p COM7 torque 0 2500
    python ring_tool.py -p COM7 broadcast 200 0 0
    python ring_tool.py -p COM7 validate --cycles 500 --duty 150
    python ring_tool.py -p COM7 stress-invalid --pattern addr-bad-crc --cycles 20

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import dataclasses
import random
import struct
import sys
import time
from typing import Iterable, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


SYNC_DIAG = 0x7D
SYNC_STATUS = 0x7E
SYNC_ENUMERATE = 0x7F
SYNC_ADDRESS_BASE = 0x80
SYNC_BROADCAST = 0xFF

CMD_SET_DUTY = 0x01
CMD_SET_TORQUE = 0x02
CMD_STOP = 0x03
CMD_CLEAR_FAULT = 0x04
CMD_QUERY_STATUS = 0x10
CMD_QUERY_DIAG = 0x11

MAX_DEVICES = 16
DIAG_FRAME_SIZE = 16
STATUS_FRAME_SIZE = 7
DEFAULT_TIMEOUT_MS = 100
DEFAULT_OPEN_SETTLE_MS = 250
DEFAULT_READ_SLICE_MS = 10


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class RingError(Exception):
    pass


class RingTimeout(RingError):
    pass


@dataclasses.dataclass
class MotorStatus:
    state: int
    fault: int
    current_ma: int
    hall: int


@dataclasses.dataclass
class ProtocolDiag:
    flags: int
    last_abort_reason: int
    bad_crc_count: int
    timeout_abort_count: int
    rx_overflow_count: int
    frame_abort_count: int
    false_sync_count: int
    addr_forward_count: int


class BL4818RingClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_ms: int = DEFAULT_TIMEOUT_MS,
        open_settle_ms: int = DEFAULT_OPEN_SETTLE_MS,
        trace: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout_ms = timeout_ms
        self.open_settle_ms = open_settle_ms
        self.trace = trace
        self.ser: Optional[serial.Serial] = None
        self.device_count: Optional[int] = None
        self.link_primed = False

    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=max(min(self.timeout_ms / 1000.0, DEFAULT_READ_SLICE_MS / 1000.0), 0.001),
            inter_byte_timeout=DEFAULT_READ_SLICE_MS / 1000.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            write_timeout=max(self.timeout_ms / 1000.0, 0.1),
        )
        self.ser.setRTS(False)
        self.ser.setDTR(True)
        if self.open_settle_ms > 0:
            time.sleep(self.open_settle_ms / 1000.0)
        self.link_primed = False
        self.clear_input()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def clear_input(self) -> None:
        if not self.ser:
            return

        dropped = bytearray()
        while True:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                break
            chunk = self.ser.read(waiting)
            if not chunk:
                break
            dropped.extend(chunk)
            time.sleep(0.001)

        if dropped:
            self._trace_bytes("drop", bytes(dropped))

    def enumerate(self) -> int:
        packet = bytes((SYNC_ENUMERATE, 0x00))
        response = self._transaction(
            packet + bytes((crc8(packet),)),
            lambda frame: self._validate_fixed_response(frame, bytes((SYNC_ENUMERATE,))),
            3,
        )

        self.device_count = response[1]
        return self.device_count

    def broadcast_duty(self, duties: Iterable[int]) -> None:
        duty_list = list(duties)
        if not duty_list:
            raise RingError("broadcast requires at least one duty")
        if len(duty_list) > MAX_DEVICES:
            raise RingError(f"broadcast supports at most {MAX_DEVICES} duty slots")

        payload = bytearray((SYNC_BROADCAST, len(duty_list)))
        for duty in duty_list:
            payload.extend(self._pack_i16(duty))
        payload.append(crc8(bytes(payload)))

        response = self._transaction(
            bytes(payload),
            lambda frame: self._validate_fixed_response(frame, bytes((SYNC_BROADCAST, 0x00))),
            3,
        )

    def set_duty(self, address: int, duty: int) -> MotorStatus:
        return self._send_addressed(address, CMD_SET_DUTY, self._pack_i16(duty))

    def set_torque_limit(self, address: int, torque_ma: int) -> MotorStatus:
        if torque_ma < 0 or torque_ma > 0xFFFF:
            raise RingError("torque limit must fit in uint16")
        return self._send_addressed(address, CMD_SET_TORQUE, struct.pack(">H", torque_ma))

    def stop(self, address: int) -> MotorStatus:
        return self._send_addressed(address, CMD_STOP, b"")

    def clear_fault(self, address: int) -> MotorStatus:
        return self._send_addressed(address, CMD_CLEAR_FAULT, b"")

    def query_status(self, address: int) -> MotorStatus:
        return self._send_addressed(address, CMD_QUERY_STATUS, b"")

    def query_diag(self, address: int) -> ProtocolDiag:
        self._validate_address(address)
        frame = bytes((SYNC_ADDRESS_BASE | address, CMD_QUERY_DIAG))
        packet = frame + bytes((crc8(frame),))
        response = self._transaction(packet, self._validate_diag_frame, DIAG_FRAME_SIZE)
        return self._parse_diag(response)

    def inject_raw(self, payload: bytes, settle_ms: float = 10.0) -> bytes:
        if not self.ser:
            raise RingError("serial port is not open")

        self.clear_input()
        written = self.ser.write(payload)
        if written != len(payload):
            raise RingError(f"short raw write: wrote {written} of {len(payload)} bytes")

        self.ser.flush()
        self._trace_bytes("tx-raw", payload)

        observed = bytearray()
        deadline = time.monotonic() + max(settle_ms, 0.0) / 1000.0
        while time.monotonic() < deadline:
            waiting = max(self.ser.in_waiting, 1)
            chunk = self.ser.read(waiting)
            if chunk:
                observed.extend(chunk)
                continue
            time.sleep(0.001)

        if observed:
            self._trace_bytes("rx-raw", bytes(observed))

        return bytes(observed)

    def _send_addressed(self, address: int, cmd: int, payload: bytes) -> MotorStatus:
        self._validate_address(address)
        frame = bytes((SYNC_ADDRESS_BASE | address, cmd)) + payload
        packet = frame + bytes((crc8(frame),))
        response = self._transaction(packet, self._validate_status_frame, STATUS_FRAME_SIZE)
        return self._parse_status(response)

    def _validate_address(self, address: int) -> None:
        if address < 0 or address >= MAX_DEVICES:
            raise RingError(f"address must be in range 0..{MAX_DEVICES - 1}")
        if self.device_count is not None and address >= self.device_count:
            raise RingError(f"address {address} is outside enumerated range 0..{self.device_count - 1}")

    def _transaction(self, packet: bytes, validator, response_len: int) -> bytes:
        if not self.ser:
            raise RingError("serial port is not open")

        for attempt in range(1):
            self.clear_input()

            written = self.ser.write(packet)
            if written != len(packet):
                raise RingError(f"short write: wrote {written} of {len(packet)} bytes")

            self.ser.flush()
            self._trace_bytes("tx", packet)

            try:
                response = self._read_valid_frame(response_len, validator)
                self.link_primed = True
                return response
            except RingTimeout:
                if self.link_primed or attempt > 0:
                    raise
                self._trace_note("retry", "first transaction after open")
                time.sleep(0.02)

        raise RingTimeout("timeout waiting for response after startup retry")

    def _read_valid_frame(self, length: int, validator) -> bytes:
        assert self.ser is not None

        received = bytearray()
        observed = bytearray()
        last_byte_at = time.monotonic()
        timeout_s = self.timeout_ms / 1000.0

        while True:
            chunk = self.ser.read(max(length, 1))
            if chunk:
                observed.extend(chunk)
                received.extend(chunk)
                last_byte_at = time.monotonic()

                while len(received) >= length:
                    candidate = bytes(received[:length])
                    if validator(candidate):
                        self._trace_bytes("rx", candidate)
                        return candidate
                    self._trace_bytes("rx-bad", candidate)
                    del received[0]
                continue

            if time.monotonic() - last_byte_at >= timeout_s:
                self._trace_bytes("rx-seen", bytes(observed))
                self._trace_bytes("rx-timeout", bytes(received))
                raise RingTimeout(
                    "timeout waiting for valid "
                    f"{length}-byte frame (tail={received.hex(' ')} seen={observed.hex(' ')})"
                )

            time.sleep(0.001)

    def _trace_bytes(self, label: str, data: bytes) -> None:
        if not self.trace:
            return
        if data:
            print(f"ring {label}: {data.hex(' ').upper()}")
        else:
            print(f"ring {label}:")

    def _trace_note(self, label: str, note: str) -> None:
        if not self.trace:
            return
        print(f"ring {label}: {note}")

    @staticmethod
    def _pack_i16(value: int) -> bytes:
        if value < -0x8000 or value > 0x7FFF:
            raise RingError("signed duty payload must fit in int16")
        return struct.pack(">h", value)

    @staticmethod
    def _parse_status(frame: bytes) -> MotorStatus:
        if len(frame) != STATUS_FRAME_SIZE:
            raise RingError(f"expected {STATUS_FRAME_SIZE} status bytes, got {len(frame)}")
        if not BL4818RingClient._validate_status_frame(frame):
            raise RingError(f"bad status frame: {frame.hex(' ')}")

        return MotorStatus(
            state=frame[1],
            fault=frame[2],
            current_ma=struct.unpack(">H", frame[3:5])[0],
            hall=frame[5],
        )

    @staticmethod
    def _parse_diag(frame: bytes) -> ProtocolDiag:
        if len(frame) != DIAG_FRAME_SIZE:
            raise RingError(f"expected {DIAG_FRAME_SIZE} diagnostic bytes, got {len(frame)}")
        if not BL4818RingClient._validate_diag_frame(frame):
            raise RingError(f"bad diagnostic frame: {frame.hex(' ')}")

        return ProtocolDiag(
            flags=frame[1],
            last_abort_reason=frame[2],
            bad_crc_count=struct.unpack(">H", frame[3:5])[0],
            timeout_abort_count=struct.unpack(">H", frame[5:7])[0],
            rx_overflow_count=struct.unpack(">H", frame[7:9])[0],
            frame_abort_count=struct.unpack(">H", frame[9:11])[0],
            false_sync_count=struct.unpack(">H", frame[11:13])[0],
            addr_forward_count=struct.unpack(">H", frame[13:15])[0],
        )

    @staticmethod
    def _validate_fixed_response(frame: bytes, prefix: bytes) -> bool:
        if len(frame) < len(prefix) + 1:
            return False
        if frame[: len(prefix)] != prefix:
            return False
        return frame[-1] == crc8(frame[:-1])

    @staticmethod
    def _validate_status_frame(frame: bytes) -> bool:
        if len(frame) != STATUS_FRAME_SIZE:
            return False
        if frame[0] != SYNC_STATUS:
            return False
        return frame[6] == crc8(frame[:6])

    @staticmethod
    def _validate_diag_frame(frame: bytes) -> bool:
        if len(frame) != DIAG_FRAME_SIZE:
            return False
        if frame[0] != SYNC_DIAG:
            return False
        return frame[-1] == crc8(frame[:-1])


def auto_detect_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    preferred = []
    fallback = []

    for port in ports:
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        if (
            port.vid in (0x2E8A, 0x1A86, 0x10C4, 0x0403)
            or "usb serial" in description
            or "wch" in manufacturer
            or "silicon labs" in manufacturer
            or "ftdi" in manufacturer
            or "pico" in description
            or "acm" in description
        ):
            preferred.append(port.device)
        else:
            fallback.append(port.device)

    return preferred[0] if preferred else fallback[0]


def list_ports() -> int:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1

    for port in ports:
        print(f"{port.device}: {port.description}")
    return 0


def format_status(address: int, status: MotorStatus) -> str:
    return (
        f"addr={address} state={status.state} fault={status.fault} "
        f"current_ma={status.current_ma} hall={status.hall}"
    )


def format_diag(address: int, diag: ProtocolDiag) -> str:
    flags = []
    if diag.flags & 0x01:
        flags.append("enumerated")
    if diag.flags & 0x02:
        flags.append("wdt-reset")
    flag_text = ",".join(flags) if flags else "none"

    return (
        f"addr={address} flags={flag_text} last_abort={diag.last_abort_reason} "
        f"bad_crc={diag.bad_crc_count} timeout_abort={diag.timeout_abort_count} "
        f"rx_overflow={diag.rx_overflow_count} frame_abort={diag.frame_abort_count} "
        f"false_sync={diag.false_sync_count} addr_forward={diag.addr_forward_count}"
    )


def build_invalid_pattern(name: str, address: int) -> bytes:
    if name == "enum-bad-crc":
        frame = bytes((SYNC_ENUMERATE, 0x00))
        return frame + bytes(((crc8(frame) ^ 0xFF) & 0xFF,))
    if name == "enum-truncated":
        return bytes((SYNC_ENUMERATE, 0x00))
    if name == "status-false-sync":
        frame = bytes((SYNC_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00))
        return frame + bytes(((crc8(frame) ^ 0x55) & 0xFF,))
    if name == "addr-bad-crc":
        frame = bytes((SYNC_ADDRESS_BASE | address, CMD_QUERY_STATUS))
        return frame + bytes(((crc8(frame) ^ 0xA5) & 0xFF,))
    if name == "broadcast-bad-crc":
        frame = bytes((SYNC_BROADCAST, 0x01, 0x00, 0x00))
        return frame + bytes(((crc8(frame) ^ 0x5A) & 0xFF,))
    if name == "random3":
        return bytes(random.getrandbits(8) for _ in range(3))

    raise RingError(f"unknown invalid pattern: {name}")


def require_enumeration(client: BL4818RingClient) -> int:
    count = client.enumerate()
    print(f"devices={count}")
    if count == 0:
        raise RingError("no devices enumerated")
    return count


def run_validate(client: BL4818RingClient, args: argparse.Namespace) -> None:
    count = require_enumeration(client)

    if args.target_addr < 0 or args.target_addr >= count:
        raise RingError(f"target address must be in range 0..{count - 1}")

    if args.query_addr is not None and (args.query_addr < 0 or args.query_addr >= count):
        raise RingError(f"query address must be in range 0..{count - 1}")

    duties = [0] * count
    duties[args.target_addr] = args.duty

    if args.torque_ma is not None:
        status = client.set_torque_limit(args.target_addr, args.torque_ma)
        print(f"torque_limit {format_status(args.target_addr, status)}")

    start = time.perf_counter()
    last_status = None
    last_addr = 0

    for cycle in range(args.cycles):
        client.broadcast_duty(duties)
        last_addr = args.query_addr if args.query_addr is not None else (cycle % count)
        last_status = client.query_status(last_addr)

        if args.print_every > 0 and (
            cycle == 0
            or (cycle + 1) % args.print_every == 0
            or (cycle + 1) == args.cycles
        ):
            print(
                f"cycle={cycle + 1}/{args.cycles} duty={duties[args.target_addr]} "
                f"{format_status(last_addr, last_status)}"
            )

        if args.sleep_ms > 0:
            time.sleep(args.sleep_ms / 1000.0)

    elapsed = time.perf_counter() - start
    rate_hz = (args.cycles / elapsed) if elapsed > 0 else 0.0
    print(f"validate complete cycles={args.cycles} elapsed_s={elapsed:.3f} rate_hz={rate_hz:.1f}")
    if last_status is not None:
        print(f"last {format_status(last_addr, last_status)}")


def run_stress_invalid(client: BL4818RingClient, args: argparse.Namespace) -> None:
    count = require_enumeration(client)

    if args.address < 0 or args.address >= count:
        raise RingError(f"address must be in range 0..{count - 1}")

    recoveries = 0
    failures = 0

    for cycle in range(args.cycles):
        pattern = build_invalid_pattern(args.pattern, args.address)
        raw_rx = client.inject_raw(pattern, args.gap_ms)

        try:
            diag = client.query_diag(args.address)
            recoveries += 1
            if args.print_every > 0 and (
                cycle == 0
                or (cycle + 1) % args.print_every == 0
                or (cycle + 1) == args.cycles
            ):
                print(
                    f"cycle={cycle + 1}/{args.cycles} pattern={args.pattern} "
                    f"raw_rx={raw_rx.hex(' ').upper() if raw_rx else '-'} "
                    f"{format_diag(args.address, diag)}"
                )
        except RingError as exc:
            failures += 1
            print(
                f"cycle={cycle + 1}/{args.cycles} pattern={args.pattern} "
                f"raw_rx={raw_rx.hex(' ').upper() if raw_rx else '-'} "
                f"recovery_failed={exc}"
            )

    print(
        f"stress-invalid complete cycles={args.cycles} recoveries={recoveries} "
        f"failures={failures}"
    )


def parse_baud_list(spec: str) -> list[int]:
    baud_list = []
    for part in spec.split(","):
        part = part.strip()
        if part:
            baud_list.append(int(part))
    if not baud_list:
        raise RingError("probe baud list is empty")
    return baud_list


def run_probe(
    port: str,
    timeout_ms: int,
    baud_list: list[int],
    open_settle_ms: int,
    trace: bool,
) -> int:
    print(f"probing port={port} bauds={','.join(str(v) for v in baud_list)}")
    any_success = False

    for baud in baud_list:
        client = BL4818RingClient(
            port=port,
            baudrate=baud,
            timeout_ms=timeout_ms,
            open_settle_ms=open_settle_ms,
            trace=trace,
        )
        try:
            client.open()
            count = client.enumerate()
            print(f"baud={baud} enumerate_ok devices={count}")
            any_success = True
        except RingTimeout as exc:
            print(f"baud={baud} timeout {exc}")
        except (RingError, serial.SerialException) as exc:
            print(f"baud={baud} error {exc}")
        finally:
            client.close()

    return 0 if any_success else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Direct binary protocol helper for BL4818 production firmware"
    )
    parser.add_argument("-p", "--port", help="Serial port (auto-detected if omitted)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=DEFAULT_TIMEOUT_MS,
        help=f"Per-transaction inactivity timeout in milliseconds (default: {DEFAULT_TIMEOUT_MS})",
    )
    parser.add_argument(
        "--open-settle-ms",
        type=int,
        default=DEFAULT_OPEN_SETTLE_MS,
        help=f"Delay after opening the port before first I/O (default: {DEFAULT_OPEN_SETTLE_MS})",
    )
    parser.add_argument("--trace", action="store_true", help="Print raw TX/RX bytes for each transaction")

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ports", help="List available serial ports")
    probe_parser = subparsers.add_parser(
        "probe",
        help="Try binary enumeration at one or more baud rates and print what responds",
    )
    probe_parser.add_argument(
        "--bauds",
        default="115200,250000",
        help="Comma-separated baud rates to try (default: 115200,250000)",
    )
    subparsers.add_parser("enumerate", help="Enumerate devices and print the count")

    status_parser = subparsers.add_parser("status", help="Query status from one addressed device")
    status_parser.add_argument("address", type=int, help="Device address")

    diag_parser = subparsers.add_parser("diag", help="Query transport diagnostics from one addressed device")
    diag_parser.add_argument("address", type=int, help="Device address")

    duty_parser = subparsers.add_parser("set-duty", help="Set signed duty on one addressed device")
    duty_parser.add_argument("address", type=int, help="Device address")
    duty_parser.add_argument("duty", type=int, help="Signed duty int16")

    torque_parser = subparsers.add_parser("torque", help="Set torque limit in mA on one addressed device")
    torque_parser.add_argument("address", type=int, help="Device address")
    torque_parser.add_argument("milliamps", type=int, help="Torque limit in mA")

    stop_parser = subparsers.add_parser("stop", help="Stop one addressed device")
    stop_parser.add_argument("address", type=int, help="Device address")

    clear_parser = subparsers.add_parser("clear-fault", help="Clear fault on one addressed device")
    clear_parser.add_argument("address", type=int, help="Device address")

    broadcast_parser = subparsers.add_parser(
        "broadcast", help="Send one broadcast duty packet with one signed duty per device"
    )
    broadcast_parser.add_argument("duties", nargs="+", type=int, help="Signed duty values")

    validate_parser = subparsers.add_parser(
        "validate",
        help="Run repeated broadcast + status transactions to validate ring comms",
    )
    validate_parser.add_argument("--cycles", type=int, default=200, help="Number of cycles to run")
    validate_parser.add_argument("--duty", type=int, default=0, help="Duty to apply to the target address")
    validate_parser.add_argument(
        "--target-addr", type=int, default=0, help="Address to receive the test duty (default: 0)"
    )
    validate_parser.add_argument(
        "--query-addr",
        type=int,
        default=None,
        help="Address to query every cycle (default: round-robin across devices)",
    )
    validate_parser.add_argument(
        "--torque-ma",
        type=int,
        default=None,
        help="Optional torque limit to set on the target address before the loop",
    )
    validate_parser.add_argument("--sleep-ms", type=float, default=0.0, help="Optional delay between cycles")
    validate_parser.add_argument(
        "--print-every",
        type=int,
        default=25,
        help="Print one progress line every N cycles (default: 25)",
    )

    stress_parser = subparsers.add_parser(
        "stress-invalid",
        help="Inject malformed traffic and verify that the next diagnostic query still recovers",
    )
    stress_parser.add_argument(
        "--cycles", type=int, default=50, help="Number of malformed-injection cycles to run"
    )
    stress_parser.add_argument("--address", type=int, default=0, help="Address to query after each injection")
    stress_parser.add_argument(
        "--pattern",
        choices=(
            "enum-bad-crc",
            "enum-truncated",
            "status-false-sync",
            "addr-bad-crc",
            "broadcast-bad-crc",
            "random3",
        ),
        default="addr-bad-crc",
        help="Malformed payload pattern to inject",
    )
    stress_parser.add_argument(
        "--gap-ms",
        type=float,
        default=10.0,
        help="Idle time to allow after raw injection before querying diagnostics",
    )
    stress_parser.add_argument(
        "--print-every",
        type=int,
        default=1,
        help="Print one line every N cycles (default: 1)",
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "ports":
        return list_ports()

    port = args.port or auto_detect_port()
    if not port:
        print("ERROR: no serial port detected. Use -p to specify one.")
        return 1
    if args.port is None:
        print(f"Auto-detected port: {port}")

    if args.command == "probe":
        try:
            return run_probe(
                port,
                args.timeout_ms,
                parse_baud_list(args.bauds),
                args.open_settle_ms,
                args.trace,
            )
        except RingError as exc:
            print(f"ERROR: {exc}")
            return 1

    client = BL4818RingClient(
        port=port,
        baudrate=args.baud,
        timeout_ms=args.timeout_ms,
        open_settle_ms=args.open_settle_ms,
        trace=args.trace,
    )

    try:
        client.open()

        if args.command == "enumerate":
            count = client.enumerate()
            print(f"devices={count}")
            return 0

        if args.command == "broadcast":
            count = require_enumeration(client)
            if len(args.duties) != count:
                raise RingError(
                    f"broadcast duty count ({len(args.duties)}) must match enumerated device count ({count})"
                )
            client.broadcast_duty(args.duties)
            print("broadcast ok")
            return 0

        require_enumeration(client)

        if args.command == "status":
            print(format_status(args.address, client.query_status(args.address)))
            return 0

        if args.command == "diag":
            print(format_diag(args.address, client.query_diag(args.address)))
            return 0

        if args.command == "set-duty":
            print(format_status(args.address, client.set_duty(args.address, args.duty)))
            return 0

        if args.command == "torque":
            print(format_status(args.address, client.set_torque_limit(args.address, args.milliamps)))
            return 0

        if args.command == "stop":
            print(format_status(args.address, client.stop(args.address)))
            return 0

        if args.command == "clear-fault":
            print(format_status(args.address, client.clear_fault(args.address)))
            return 0

        if args.command == "validate":
            run_validate(client, args)
            return 0

        if args.command == "stress-invalid":
            run_stress_invalid(client, args)
            return 0

        raise RingError(f"unknown command: {args.command}")

    except RingError as exc:
        print(f"ERROR: {exc}")
        return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    except KeyboardInterrupt:
        print("Cancelled.")
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
