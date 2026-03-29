#!/usr/bin/env python3
"""
ring_tool.py - Manual binary protocol helper for BL4818 production firmware

Examples:
    python ring_tool.py -p COM7 enumerate
    python ring_tool.py -p COM7 probe
    python ring_tool.py -p COM7 status 0
    python ring_tool.py -p COM7 set-duty 0 250
    python ring_tool.py -p COM7 torque 0 2500
    python ring_tool.py -p COM7 broadcast 200 0 0
    python ring_tool.py -p COM7 validate --cycles 500 --duty 150

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import dataclasses
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


SYNC_STATUS = 0x7E
SYNC_ENUMERATE = 0x7F
SYNC_ADDRESS_BASE = 0x80
SYNC_BROADCAST = 0xFF

CMD_SET_DUTY = 0x01
CMD_SET_TORQUE = 0x02
CMD_STOP = 0x03
CMD_CLEAR_FAULT = 0x04
CMD_QUERY_STATUS = 0x10

MAX_DEVICES = 16
STATUS_FRAME_SIZE = 6


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


class BL4818RingClient:
    def __init__(self, port: str, baudrate: int = 250000, timeout_ms: int = 20):
        self.port = port
        self.baudrate = baudrate
        self.timeout_ms = timeout_ms
        self.ser: Optional[serial.Serial] = None
        self.device_count: Optional[int] = None

    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0,
            write_timeout=max(self.timeout_ms / 1000.0, 0.1),
        )
        self.clear_input()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def clear_input(self) -> None:
        if not self.ser:
            return
        self.ser.reset_input_buffer()

    def enumerate(self) -> int:
        response = self._transaction(bytes((SYNC_ENUMERATE, 0x00)), 2)
        if response[0] != SYNC_ENUMERATE:
            raise RingError(f"bad enumerate response sync: 0x{response[0]:02X}")

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

        response = self._transaction(bytes(payload), 2)
        if response != bytes((SYNC_BROADCAST, 0x00)):
            raise RingError(f"bad broadcast echo: {response.hex(' ')}")

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

    def _send_addressed(self, address: int, cmd: int, payload: bytes) -> MotorStatus:
        self._validate_address(address)
        packet = bytes((SYNC_ADDRESS_BASE | address, cmd)) + payload
        response = self._transaction(packet, STATUS_FRAME_SIZE)
        return self._parse_status(response)

    def _validate_address(self, address: int) -> None:
        if address < 0 or address >= MAX_DEVICES:
            raise RingError(f"address must be in range 0..{MAX_DEVICES - 1}")
        if self.device_count is not None and address >= self.device_count:
            raise RingError(f"address {address} is outside enumerated range 0..{self.device_count - 1}")

    def _transaction(self, packet: bytes, response_len: int) -> bytes:
        if not self.ser:
            raise RingError("serial port is not open")

        self.clear_input()

        written = self.ser.write(packet)
        if written != len(packet):
            raise RingError(f"short write: wrote {written} of {len(packet)} bytes")

        return self._read_exact(response_len)

    def _read_exact(self, length: int) -> bytes:
        assert self.ser is not None

        received = bytearray()
        last_byte_at = time.monotonic()
        timeout_s = self.timeout_ms / 1000.0

        while len(received) < length:
            chunk = self.ser.read(length - len(received))
            if chunk:
                received.extend(chunk)
                last_byte_at = time.monotonic()
                continue

            if time.monotonic() - last_byte_at >= timeout_s:
                raise RingTimeout(
                    f"timeout waiting for {length} bytes (received {len(received)})"
                )

            time.sleep(0.001)

        return bytes(received)

    @staticmethod
    def _pack_i16(value: int) -> bytes:
        if value < -0x8000 or value > 0x7FFF:
            raise RingError("signed duty payload must fit in int16")
        return struct.pack(">h", value)

    @staticmethod
    def _parse_status(frame: bytes) -> MotorStatus:
        if len(frame) != STATUS_FRAME_SIZE:
            raise RingError(f"expected {STATUS_FRAME_SIZE} status bytes, got {len(frame)}")
        if frame[0] != SYNC_STATUS:
            raise RingError(f"bad status sync: 0x{frame[0]:02X}")

        return MotorStatus(
            state=frame[1],
            fault=frame[2],
            current_ma=struct.unpack(">H", frame[3:5])[0],
            hall=frame[5],
        )


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


def parse_baud_list(spec: str) -> list[int]:
    baud_list = []
    for part in spec.split(","):
        part = part.strip()
        if part:
            baud_list.append(int(part))
    if not baud_list:
        raise RingError("probe baud list is empty")
    return baud_list


def run_probe(port: str, timeout_ms: int, baud_list: list[int]) -> int:
    print(f"probing port={port} bauds={','.join(str(v) for v in baud_list)}")
    any_success = False

    for baud in baud_list:
        client = BL4818RingClient(port=port, baudrate=baud, timeout_ms=timeout_ms)
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
    parser.add_argument("-b", "--baud", type=int, default=250000, help="Baud rate (default: 250000)")
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=20,
        help="Per-transaction inactivity timeout in milliseconds (default: 20)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ports", help="List available serial ports")
    probe_parser = subparsers.add_parser(
        "probe",
        help="Try binary enumeration at one or more baud rates and print what responds",
    )
    probe_parser.add_argument(
        "--bauds",
        default="250000,115200",
        help="Comma-separated baud rates to try (default: 250000,115200)",
    )
    subparsers.add_parser("enumerate", help="Enumerate devices and print the count")

    status_parser = subparsers.add_parser("status", help="Query status from one addressed device")
    status_parser.add_argument("address", type=int, help="Device address")

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
            return run_probe(port, args.timeout_ms, parse_baud_list(args.bauds))
        except RingError as exc:
            print(f"ERROR: {exc}")
            return 1

    client = BL4818RingClient(port=port, baudrate=args.baud, timeout_ms=args.timeout_ms)

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
