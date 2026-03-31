#!/usr/bin/env python3
"""
flash.py - Standalone ISP flasher for MS51FB9AE via NuMicro-8051-prog bridge

Implements the Nuvoton NuMicro 8051 ISP-over-UART protocol to program
APROM on the MS51FB9AE through a Pi Pico (or Arduino) running the
nikitalita/NuMicro-8051-prog firmware.

Usage:
    python flash.py                          # Flash build/bl4818-servo.bin on auto-detected port
    python flash.py -p COM5                  # Specify serial port
    python flash.py -f firmware.bin          # Specify firmware file
    python flash.py -p COM5 -r dump.bin      # Read flash to file
    python flash.py -p COM5 -i              # Print device info only

Requires: pyserial (pip install pyserial)
"""

import argparse
import os
import re
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

# ── ISP Protocol Constants ──────────────────────────────────────────────────

PACKSIZE = 64

# Standard commands
CMD_UPDATE_APROM      = 0xA0
CMD_UPDATE_CONFIG     = 0xA1
CMD_READ_CONFIG       = 0xA2
CMD_ERASE_ALL         = 0xA3
CMD_SYNC_PACKNO       = 0xA4
CMD_READ_ROM          = 0xA5
CMD_GET_FWVER         = 0xA6
CMD_RUN_APROM         = 0xAB
CMD_RUN_LDROM         = 0xAC
CMD_RESET             = 0xAD
CMD_CONNECT           = 0xAE
CMD_GET_DEVICEID      = 0xB1
CMD_GET_UID           = 0xB2
CMD_GET_CID           = 0xB3
CMD_GET_PID           = 0xEB
CMD_ISP_PAGE_ERASE    = 0xD5
CMD_ISP_MASS_ERASE    = 0xD6
CMD_TARGET_POWER      = 0xD7
CMD_POWER_CYCLE_CONNECT = 0xD8
CMD_FORMAT2_CONTINUATION = 0x00

# Firmware version thresholds
EXTENDED_CMDS_FW_VER = 0xD0
ICP_BRIDGE_FW_VER    = 0xE0
POWER_CTRL_FW_VER    = 0xE1

# Timeouts (seconds)
CONNECT_TIMEOUT  = 0.05
SLOW_TIMEOUT     = 0.25
ERASE_TIMEOUT    = 8.5
FORMAT2_TIMEOUT  = 0.2
READ_ROM_TIMEOUT = 2.0
RESET_TIMEOUT    = 0.5
POLL_INTERVAL    = 0.01

DEFAULT_POWER_OFF_MS = 50
DEFAULT_RECOVERY_DELAYS_US = [100, 250, 500, 1000, 2000, 5000, 10000, 20000]
BOD_THRESHOLD_BITS = {"4.4": 0x00, "3.7": 0x10, "2.7": 0x20, "2.2": 0x30}
WDT_MODE_BITS = {"software": 0xF0, "idle-stop": 0x50, "always": 0x00}

DEFAULT_CONFIG_PROFILES = {
    "factory-safe": {
        "bytes": bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        "description": (
            "APROM boot, reset pin enabled, unlocked, no LDROM, "
            "BOD/BOR enabled at 2.2V, WDT software-controlled"
        ),
    },
}
DEFAULT_PROJECT_DEFAULT_PROFILE = "factory-safe"
DEFAULT_PROJECT_RECOVERY_PROFILE = "factory-safe"

try:
    from ms51_flash_config import (  # type: ignore
        CONFIG_PROFILES,
        PROJECT_DEFAULT_PROFILE,
        PROJECT_RECOVERY_PROFILE,
    )
except ImportError:
    CONFIG_PROFILES = DEFAULT_CONFIG_PROFILES
    PROJECT_DEFAULT_PROFILE = DEFAULT_PROJECT_DEFAULT_PROFILE
    PROJECT_RECOVERY_PROFILE = DEFAULT_PROJECT_RECOVERY_PROFILE

SAFE_FACTORY_CONFIG = bytes(DEFAULT_CONFIG_PROFILES["factory-safe"]["bytes"])

# MS51FB9AE specifics
MS51FB9AE_DEVIDS = {0x4B20, 0x4B21}
MS51FB9AE_FLASH_SIZE = 16 * 1024  # 16 KB APROM

# ── Helpers ─────────────────────────────────────────────────────────────────

def pack_u32(val):
    return bytes([val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF])

def pack_u16(val):
    return bytes([val & 0xFF, (val >> 8) & 0xFF])

def unpack_u16(data):
    return (data[0] & 0xFF) | ((data[1] & 0xFF) << 8)

def unpack_u32(data):
    return ((data[0] & 0xFF) | ((data[1] & 0xFF) << 8) |
            ((data[2] & 0xFF) << 16) | ((data[3] & 0xFF) << 24))

def calc_checksum(data):
    return sum(data) & 0xFFFF

def progress_bar(label, current, total, width=40):
    pct = current / total if total > 0 else 1.0
    filled = int(width * pct)
    bar = "#" * filled + "-" * (width - filled)
    print(f"\r  {label}: [{bar}] {int(pct * 100):3d}%", end="", flush=True)

def decode_ldrom_size_kb(config_bytes):
    if len(config_bytes) < 2:
        return 0
    lds = config_bytes[1] & 0x07
    return {7: 0, 6: 1, 5: 2, 4: 3, 3: 4, 2: 4, 1: 4, 0: 4}.get(lds, 0)

def decode_aprom_size_bytes(config_bytes):
    return MS51FB9AE_FLASH_SIZE - (decode_ldrom_size_kb(config_bytes) * 1024)

def is_safe_factory_config(config_bytes):
    if len(config_bytes) != 5:
        return False
    c0, c1, _, _, _ = config_bytes
    return (
        config_bytes == SAFE_FACTORY_CONFIG and
        (c0 & 0x80) and  # boot APROM
        (c0 & 0x04) and  # keep reset pin enabled
        (c0 & 0x02) and  # stay unlocked
        ((c1 & 0x07) == 0x07)  # reserve no LDROM
    )

# ── ISP Programmer ──────────────────────────────────────────────────────────

class ISPError(Exception):
    pass


def normalize_config_bytes(config_bytes):
    config_bytes = bytes(config_bytes)
    if len(config_bytes) != 5:
        raise ValueError("Config must contain exactly 5 bytes")
    return config_bytes


def format_config_bytes(config_bytes):
    return " ".join(f"{b:02X}" for b in normalize_config_bytes(config_bytes))


def get_config_profile(profile_name):
    profile = CONFIG_PROFILES.get(profile_name)
    if profile is None:
        raise ValueError(f"Unknown config profile: {profile_name}")
    config_bytes = normalize_config_bytes(profile["bytes"])
    description = profile.get("description", "")
    return config_bytes, description


def print_config_profiles():
    print("Checked-in config profiles:")
    print(f"  default : {PROJECT_DEFAULT_PROFILE}")
    print(f"  recovery: {PROJECT_RECOVERY_PROFILE}")
    for name in sorted(CONFIG_PROFILES.keys()):
        config_bytes, description = get_config_profile(name)
        suffix = f" - {description}" if description else ""
        print(f"  {name:18s} [{format_config_bytes(config_bytes)}]{suffix}")


def parse_config_bytes(text):
    tokens = [tok for tok in re.split(r"[\s,]+", text.strip()) if tok]
    if len(tokens) != 5:
        raise ValueError("Expected exactly 5 config bytes, e.g. 'FF FF FF FF FF'")

    config = []
    for token in tokens:
        token = token[2:] if token.lower().startswith("0x") else token
        if len(token) == 0 or len(token) > 2:
            raise ValueError(f"Invalid config byte: {token}")
        config.append(int(token, 16))
    return bytes(config)


def config_override_requested(args):
    return any((
        args.bod_threshold is not None,
        args.bod is not None,
        args.bor_reset is not None,
        args.boiap is not None,
        args.wdt_config is not None,
    ))


def apply_config_overrides(config_bytes, args):
    config = bytearray(normalize_config_bytes(config_bytes))

    if args.bod is not None:
        if args.bod == "on":
            config[2] |= 0x80
        else:
            config[2] &= ~0x80

    if args.bor_reset is not None:
        if args.bor_reset == "on":
            config[2] |= 0x04
        else:
            config[2] &= ~0x04

    if args.boiap is not None:
        if args.boiap == "inhibit":
            config[2] |= 0x08
        else:
            config[2] &= ~0x08

    if args.bod_threshold is not None:
        config[2] = (config[2] & ~0x30) | BOD_THRESHOLD_BITS[args.bod_threshold]

    if args.wdt_config is not None:
        config[4] = (config[4] & 0x0F) | WDT_MODE_BITS[args.wdt_config]

    return bytes(config)


def validate_config_bytes(config_bytes, allow_unsafe=False):
    config_bytes = normalize_config_bytes(config_bytes)
    c0 = config_bytes[0]
    ldrom_size = decode_ldrom_size_kb(config_bytes)
    problems = []

    if not (c0 & 0x04):
        problems.append("RPD=0 disables the reset pin and can block ICP entry")
    if not (c0 & 0x80) and ldrom_size == 0:
        problems.append("CBS=0 selects LDROM boot but no LDROM is reserved")
    if not (c0 & 0x02):
        problems.append("LOCK=0 locks flash and breaks config readback verification")

    if problems and not allow_unsafe:
        raise ISPError("Refusing unsafe config:\n  - " + "\n  - ".join(problems))

    return problems


def resolve_requested_config(args, default_profile_name):
    if args.config_profile and args.config_bytes:
        raise ISPError("Use either --config-profile or --config-bytes, not both")

    if args.config_bytes:
        config_bytes = parse_config_bytes(args.config_bytes)
        label = "raw config bytes"
    else:
        profile_name = args.config_profile or default_profile_name
        config_bytes, description = get_config_profile(profile_name)
        label = f"profile '{profile_name}'"
        if description:
            label += f" ({description})"

    return apply_config_overrides(config_bytes, args), label

class ISPProgrammer:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.seq_num = 0
        self.fw_ver = 0

    def open(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.flush()

    def require_icp_bridge(self, feature):
        if self.fw_ver < ICP_BRIDGE_FW_VER:
            raise ISPError(f"{feature} requires the custom ICP bridge firmware (FW >= 0x{ICP_BRIDGE_FW_VER:02X})")

    def require_power_control(self, feature):
        if self.fw_ver < POWER_CTRL_FW_VER:
            raise ISPError(f"{feature} requires bridge FW >= 0x{POWER_CTRL_FW_VER:02X} with target power control")

    def close(self, run_aprom=True):
        if self.ser and self.ser.is_open:
            if run_aprom:
                try:
                    self._send_raw(self._build_packet(CMD_RUN_APROM))
                    self.seq_num += 1
                except (serial.SerialException, OSError) as exc:
                    print(f"WARNING: failed to send RUN_APROM during close: {exc}", file=sys.stderr)
                time.sleep(RESET_TIMEOUT)
            self.ser.close()
            self.ser = None

    def _build_packet(self, cmd, data=b""):
        """Build a 64-byte ISP packet: [cmd:4][seq:4][data:56], zero-padded."""
        pkt = pack_u32(cmd) + pack_u32(self.seq_num) + data
        pkt = pkt + bytes(PACKSIZE - len(pkt))
        return pkt

    def _send_raw(self, pkt):
        self.ser.write(pkt)

    def _wait_for_data(self, timeout):
        elapsed = 0.0
        while self.ser.in_waiting < PACKSIZE:
            if elapsed >= timeout:
                return False
            time.sleep(POLL_INTERVAL)
            elapsed += POLL_INTERVAL
        return True

    def _read_response(self, timeout):
        if not self._wait_for_data(timeout):
            return None
        rx = self.ser.read(PACKSIZE)
        # Drain any extra bytes (from redundant connect packets, etc.)
        if self.ser.in_waiting > 0:
            extra = self.ser.read(self.ser.in_waiting)
            # Use the last complete packet
            combined = rx + extra
            if len(combined) >= PACKSIZE:
                rx = combined[len(combined) - PACKSIZE:]
        return rx

    def _send_bootstrap_cmd(self, cmd, data=b"", timeout=1.0, retries=5):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.seq_num = 0
        pkt = self._build_packet(cmd, data)

        for attempt in range(retries):
            self._send_raw(pkt)
            rx = self._read_response(timeout)
            if rx and len(rx) == PACKSIZE:
                if unpack_u16(rx[0:2]) == calc_checksum(pkt):
                    return rx
            self.ser.flush()

        raise ISPError(f"No valid response for bootstrap command 0x{cmd:02X} after {retries} attempts")

    def send_cmd(self, cmd, data=b"", timeout=None):
        """Send a command and receive the ACK. Returns the 64-byte response."""
        if timeout is None:
            timeout = 0.2
        self.seq_num += 1
        pkt = self._build_packet(cmd, data)
        expected_rx_seq = self.seq_num + 1
        self._send_raw(pkt)

        for attempt in range(5):
            rx = self._read_response(timeout)
            if rx and len(rx) == PACKSIZE:
                # Verify checksum: rx[0:2] should equal sum of our sent packet
                rx_checksum = unpack_u16(rx[0:2])
                rx_seq = unpack_u32(rx[4:8])
                tx_checksum = calc_checksum(pkt)
                if rx_checksum == tx_checksum and rx_seq == expected_rx_seq:
                    self.seq_num = expected_rx_seq
                    return rx
            # Retry
            self.ser.flush()
            self._send_raw(pkt)

        raise ISPError(f"No valid response for command 0x{cmd:02X} after 5 attempts")

    # ── Connection ──────────────────────────────────────────────────────

    def _finish_connect(self):
        self.seq_num = 0
        sync_data = pack_u32(1)
        self.send_cmd(CMD_SYNC_PACKNO, sync_data, timeout=1.0)

        rx = self.send_cmd(CMD_GET_FWVER)
        self.fw_ver = rx[8]
        fw_type = ""
        if self.fw_ver >= POWER_CTRL_FW_VER:
            fw_type = " (ICP bridge + power control)"
        elif self.fw_ver >= ICP_BRIDGE_FW_VER:
            fw_type = " (ICP bridge)"
        elif self.fw_ver >= EXTENDED_CMDS_FW_VER:
            fw_type = " (extended cmds)"
        print(f"  Connected! ISP FW version: 0x{self.fw_ver:02X}{fw_type}")

    def connect(self):
        """Establish ISP connection with the target."""
        print(f"  Connecting on {self.port}...")
        print("  (Reset the target board now, or the bridge handles it)")
        self._send_bootstrap_cmd(CMD_CONNECT, timeout=1.0, retries=30)
        self._finish_connect()

    def connect_with_power_cycle(self, off_ms=DEFAULT_POWER_OFF_MS, delays_us=None):
        """Attempt ISP connection by power-cycling the target and sweeping entry delays."""
        if delays_us is None:
            delays_us = DEFAULT_RECOVERY_DELAYS_US

        print(f"  Recovery connect on {self.port} with target power control...")
        for delay_us in delays_us:
            print(f"  Trying power-cycle connect: off {off_ms} ms, entry delay {delay_us} us")
            timeout = max(1.0, (off_ms / 1000.0) + 0.5)
            data = pack_u16(off_ms) + pack_u16(delay_us)
            try:
                self._send_bootstrap_cmd(CMD_POWER_CYCLE_CONNECT, data=data, timeout=timeout, retries=2)
                self._finish_connect()
                print(f"  Recovery connect succeeded at {delay_us} us.")
                return delay_us
            except ISPError:
                continue

        raise ISPError("Could not connect using power-cycle recovery. Check VDD switch wiring and timing.")

    def set_target_power(self, on):
        """Turn switched target power on or off via the bridge and wait for the ACK."""
        if self.fw_ver and self.fw_ver < POWER_CTRL_FW_VER:
            self.require_power_control("Target power control")
        state = 1 if on else 0
        self._send_bootstrap_cmd(CMD_TARGET_POWER, bytes([state]), timeout=0.5, retries=3)
        print(f"  Target power {'ON' if on else 'OFF'}.")

    # ── Device Info ─────────────────────────────────────────────────────

    def get_device_id(self):
        rx = self.send_cmd(CMD_GET_DEVICEID)
        return unpack_u32(rx[8:12])

    def get_cid(self):
        if self.fw_ver < EXTENDED_CMDS_FW_VER:
            return None
        rx = self.send_cmd(CMD_GET_CID)
        return rx[8]

    def get_uid(self):
        if self.fw_ver < EXTENDED_CMDS_FW_VER:
            return None
        rx = self.send_cmd(CMD_GET_UID)
        return rx[8:20]

    def get_pid(self):
        if self.fw_ver < EXTENDED_CMDS_FW_VER:
            return None
        rx = self.send_cmd(CMD_GET_PID)
        return unpack_u32(rx[8:12])

    def read_config(self):
        rx = self.send_cmd(CMD_READ_CONFIG)
        return rx[8:13]  # 5 config bytes

    def print_device_info(self):
        dev_id = self.get_device_id()
        print(f"  Device ID:  0x{dev_id:04X}", end="")
        if dev_id in MS51FB9AE_DEVIDS:
            print(" (MS51FB9AE)")
        else:
            print(" (unknown - expected 0x4B20 or 0x4B21 for MS51FB9AE)")

        pid = self.get_pid()
        if pid is not None:
            print(f"  Product ID: 0x{pid:08X}")

        cid = self.get_cid()
        if cid is not None:
            locked = "LOCKED" if cid == 0xFF else "unlocked"
            print(f"  Company ID: 0x{cid:02X} ({locked})")

        uid = self.get_uid()
        if uid is not None:
            print(f"  Unique ID:  {uid.hex()}")

        config = self.read_config()
        self._print_config(config)

    def _print_config(self, config):
        """Decode and display MS51FB9AE config bytes per TRM section 6.1.4."""
        print(f"  Config raw: [{' '.join(f'0x{b:02X}' for b in config)}]")
        c0 = config[0]
        c1 = config[1]
        c2 = config[2] if len(config) > 2 else 0xFF
        c3 = config[3] if len(config) > 3 else 0xFF
        c4 = config[4] if len(config) > 4 else 0xFF

        # CONFIG0 bit layout (TRM page 25):
        #   [7] CBS     1=APROM boot, 0=LDROM boot
        #   [6] -       reserved (keep 1)
        #   [5] OCDPWM  1=tri-state PWM on OCD halt, 0=PWM continues
        #   [4] OCDEN   1=OCD disabled, 0=OCD enabled
        #   [3] -       reserved
        #   [2] RPD     1=P2.0 is reset pin, 0=P2.0 is GPIO (!! disables ICP entry !!)
        #   [1] LOCK    1=unlocked, 0=LOCKED (reads all FF via ICP)
        #   [0] -       reserved
        cbs    = "APROM" if (c0 & 0x80) else "LDROM"
        ocdpwm = "tri-state" if (c0 & 0x20) else "continues"
        ocden  = "disabled" if (c0 & 0x10) else "ENABLED"
        rpd    = "reset pin" if (c0 & 0x04) else "GPIO (!! ICP entry blocked !!)"
        lock   = "unlocked" if (c0 & 0x02) else "LOCKED"

        print(f"  CONFIG0:    0x{c0:02X}")
        print(f"    Boot:     {cbs}")
        print(f"    Lock:     {lock}")
        print(f"    P2.0/RST: {rpd}")
        print(f"    OCD:      {ocden}, PWM on halt: {ocdpwm}")

        # CONFIG1 — LDROM size (TRM page 26)
        #   [2:0] LDSIZE  111=no LDROM, 110=1KB, 101=2KB, 100=3KB, 0xx=4KB
        lds = c1 & 0x07
        ldrom_kb = {7: 0, 6: 1, 5: 2, 4: 3, 3: 4, 2: 4, 1: 4, 0: 4}
        ldrom_size = ldrom_kb.get(lds, 0)
        aprom_size = 16 - ldrom_size
        print(f"  CONFIG1:    0x{c1:02X}")
        print(f"    LDROM:    {ldrom_size} KB, APROM: {aprom_size} KB")

        # CONFIG2 (TRM page 27):
        #   [7]   CBODEN  1=BOD enabled, 0=BOD off
        #   [6:4] CBOV    brown-out voltage select
        #   [3]   BOIAP   1=inhibit IAP on brownout, 0=allow
        #   [2]   CBORST  1=BOD reset enabled, 0=BOD reset off
        cboden = "enabled" if (c2 & 0x80) else "off"
        cbov = (c2 >> 4) & 0x07
        bov_mv = {7: 2200, 6: 2700, 5: 3700, 4: 4400, 3: 2200, 2: 2700, 1: 3700, 0: 4400}
        boiap  = "inhibit" if (c2 & 0x08) else "allow"
        cborst = "reset" if (c2 & 0x04) else "no reset"
        print(f"  CONFIG2:    0x{c2:02X}")
        print(f"    BOD:      {cboden}, {bov_mv.get(cbov, '?')}mV, {cborst}, IAP on BOD: {boiap}")

        # CONFIG4 — WDT (TRM page 28)
        #   [7:4] WDTEN   1111=disabled/GP, 0101=timeout+stops in idle, others=timeout+runs
        wdten = (c4 >> 4) & 0x0F
        if wdten == 0x0F:
            wdt_mode = "disabled (GP timer)"
        elif wdten == 0x05:
            wdt_mode = "timeout reset, stops in idle"
        else:
            wdt_mode = f"timeout reset, runs always (0x{wdten:X})"
        print(f"  CONFIG3:    0x{c3:02X}")
        print(f"  CONFIG4:    0x{c4:02X}")
        print(f"    WDT:      {wdt_mode}")

        # Safety warnings
        problems = []
        if not (c0 & 0x80):
            problems.append("CBS=0: boots from LDROM, not APROM!")
        if not (c0 & 0x04):
            problems.append("RPD=0: reset pin DISABLED — ICP entry may be blocked!")
        if not (c0 & 0x02):
            problems.append("LOCK=0: flash is LOCKED — ICP reads return all 0xFF!")
        if ldrom_size > 0:
            problems.append(f"LDROM={ldrom_size}KB: APROM is only {aprom_size}KB")
        if problems:
            print("  *** WARNINGS ***")
            for p in problems:
                print(f"    - {p}")

    def write_config(self, config_bytes):
        """Write config bytes to device."""
        self.require_icp_bridge("Config writes")
        if len(config_bytes) != 5:
            raise ISPError("Config write requires exactly 5 raw config bytes")
        # The Pico bridge expects the raw config bytes at packet offset 8.
        pkt_data = bytes(config_bytes)
        rx = self.send_cmd(CMD_UPDATE_CONFIG, pkt_data, timeout=ERASE_TIMEOUT)
        return rx

    def write_safe_factory_config(self):
        """Restore the known-safe factory config and verify it by readback."""
        if not is_safe_factory_config(SAFE_FACTORY_CONFIG):
            raise ISPError("Internal error: safe factory config validation failed")
        self.write_verified_config(SAFE_FACTORY_CONFIG, "safe factory config")

    def write_verified_config(self, config_bytes, label="config"):
        """Write config bytes, read them back, and verify an exact match."""
        self.require_icp_bridge("Config writes")
        config_bytes = normalize_config_bytes(config_bytes)
        old_config = self.read_config()
        print(f"\n  Writing {label}")
        print(f"  Old config: [{format_config_bytes(old_config)}]")
        print(f"  New config: [{format_config_bytes(config_bytes)}]")
        self.write_config(config_bytes)
        verify = self.read_config()
        print(f"  Readback:   [{format_config_bytes(verify)}]")
        self._print_config(verify)
        if verify[:5] != config_bytes[:5]:
            raise ISPError("Config readback doesn't match requested config")
        print("  Config write verified OK.")

    # ── Flash Operations ────────────────────────────────────────────────

    def program_aprom(self, data):
        """Write data to APROM. Automatically erases affected pages."""
        size = len(data)
        addr = 0
        pos = 0

        print(f"  Programming {size} bytes to APROM...")

        # First packet: CMD_UPDATE_APROM with address + length + first 48 bytes
        chunk = data[0:48]
        pkt_data = pack_u32(addr) + pack_u32(size) + chunk
        tx_datasum = calc_checksum(chunk)

        rx = self.send_cmd(CMD_UPDATE_APROM, pkt_data, timeout=ERASE_TIMEOUT)
        rx_checksum = unpack_u16(rx[8:10])
        if rx_checksum != tx_datasum:
            raise ISPError(f"Data checksum mismatch at offset 0: expected 0x{tx_datasum:04X}, got 0x{rx_checksum:04X}")
        pos = 48
        progress_bar("Flash", pos, size)

        # Continuation packets: 56 bytes each
        while pos < size:
            remaining = size - pos
            chunk_size = min(56, remaining)
            chunk = data[pos:pos + chunk_size]
            if len(chunk) < 56:
                chunk = chunk + bytes(56 - len(chunk))

            tx_datasum = (tx_datasum + calc_checksum(chunk[:chunk_size])) & 0xFFFF

            rx = self.send_cmd(CMD_FORMAT2_CONTINUATION, chunk, timeout=FORMAT2_TIMEOUT)
            rx_checksum = unpack_u16(rx[8:10])
            if rx_checksum != tx_datasum:
                raise ISPError(f"Data checksum mismatch at offset {pos}: expected 0x{tx_datasum:04X}, got 0x{rx_checksum:04X}")

            pos += chunk_size
            progress_bar("Flash", min(pos, size), size)

        print()  # newline after progress bar
        print("  Programming complete.")

    def read_flash(self, start_addr=0, length=None):
        """Read flash contents. Requires extended command support (FW >= 0xD0)."""
        if self.fw_ver < EXTENDED_CMDS_FW_VER:
            raise ISPError("Flash read requires extended commands (FW >= 0xD0)")

        if length is None:
            length = MS51FB9AE_FLASH_SIZE

        print(f"  Reading {length} bytes from address 0x{start_addr:04X}...")

        data = bytearray()
        addr = start_addr
        end_addr = start_addr + length

        # First packet
        pkt_data = (pack_u16(start_addr) + bytes(2) + pack_u16(length))
        rx = self.send_cmd(CMD_READ_ROM, pkt_data, timeout=READ_ROM_TIMEOUT)
        remaining = min(56, end_addr - addr)
        data.extend(rx[8:8 + remaining])
        addr += 56
        progress_bar("Read", len(data), length)

        # Continuation packets
        while addr < end_addr:
            rx = self.send_cmd(CMD_FORMAT2_CONTINUATION, timeout=FORMAT2_TIMEOUT)
            remaining = min(56, end_addr - addr)
            data.extend(rx[8:8 + remaining])
            addr += 56
            progress_bar("Read", min(len(data), length), length)

        print()
        data = bytes(data[:length])
        print(f"  Read {len(data)} bytes.")
        return data

    def erase_all(self):
        """Erase APROM only. On the Pico bridge this preserves LDROM and config bytes."""
        print("  Sending ERASE_ALL (APROM only)...")
        rx = self.send_cmd(CMD_ERASE_ALL, timeout=ERASE_TIMEOUT)
        print("  Erase complete.")
        return rx

    def chip_erase(self):
        """Mass erase the whole device, including LDROM and config, via the custom bridge."""
        if self.fw_ver < ICP_BRIDGE_FW_VER:
            raise ISPError("Full-chip erase requires the custom ICP bridge firmware (FW >= 0xE0)")
        print("  Sending ISP_MASS_ERASE (full chip)...")
        rx = self.send_cmd(CMD_ISP_MASS_ERASE, timeout=ERASE_TIMEOUT)
        print("  Full-chip erase complete.")
        return rx

    def verify_flash(self, expected_data):
        """Read back flash and compare to expected data."""
        print("  Verifying...")
        actual = self.read_flash(0, len(expected_data))
        mismatches = 0
        first_mismatch = -1
        for i in range(len(expected_data)):
            if i < len(actual) and actual[i] != expected_data[i]:
                mismatches += 1
                if first_mismatch < 0:
                    first_mismatch = i

        if mismatches == 0:
            print("  Verification passed!")
            return True
        else:
            print(f"  Verification FAILED: {mismatches} byte errors (first at 0x{first_mismatch:04X})")
            return False


# ── Port Detection ──────────────────────────────────────────────────────────

def find_pico_port():
    """Try to auto-detect the Pi Pico serial port."""
    ports = list(serial.tools.list_ports.comports())
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        vid = p.vid
        # Pi Pico shows up as USB VID 0x2E8A (Raspberry Pi)
        if vid == 0x2E8A:
            candidates.insert(0, p)
        elif "pico" in desc or "usbmodem" in desc or "acm" in desc:
            candidates.append(p)

    if candidates:
        return candidates[0].device

    # Fallback: return first available port
    if ports:
        return ports[0].device
    return None


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Flash MS51FB9AE APROM via NuMicro ISP (Pi Pico bridge)")
    parser.add_argument("-p", "--port",
        help="Serial port (auto-detected if omitted)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
        help="Baud rate (default: 115200)")
    parser.add_argument("-f", "--file", default="build/bl4818-servo.bin",
        help="Binary file to flash (default: build/bl4818-servo.bin)")
    parser.add_argument("-r", "--read",
        help="Read flash to file instead of writing")
    parser.add_argument("-i", "--info", action="store_true",
        help="Print device info and exit")
    parser.add_argument("-v", "--verify", action="store_true",
        help="Verify after programming")
    parser.add_argument("--no-verify", action="store_true",
        help="Skip auto-verification (when FW supports it)")
    parser.add_argument("--write-config", action="store_true",
        help=f"Write the checked-in default config profile ({PROJECT_DEFAULT_PROFILE})")
    parser.add_argument("--list-config-profiles", action="store_true",
        help="List checked-in config profiles and exit")
    parser.add_argument("--config-profile", choices=sorted(CONFIG_PROFILES.keys()),
        help="Config profile name from ms51_flash_config.py")
    parser.add_argument("--config-bytes",
        help="Five raw config bytes, e.g. 'FF FF FF FF FF'")
    parser.add_argument("--bod-threshold", choices=("4.4", "3.7", "2.7", "2.2"),
        help="Override CONFIG2 brown-out threshold in volts")
    parser.add_argument("--bod", choices=("on", "off"),
        help="Override CONFIG2 brown-out detect enable")
    parser.add_argument("--bor-reset", choices=("on", "off"),
        help="Override CONFIG2 brown-out reset enable")
    parser.add_argument("--boiap", choices=("inhibit", "allow"),
        help="Override CONFIG2 IAP behavior during brown-out")
    parser.add_argument("--wdt-config", choices=("software", "idle-stop", "always"),
        help="Override CONFIG4 WDT mode")
    parser.add_argument("--allow-unsafe-config", action="store_true",
        help="Allow config writes that disable reset pin, lock flash, or boot LDROM with no LDROM")
    parser.add_argument("--erase", action="store_true",
        help="Erase APROM only (preserves LDROM/config on the custom bridge)")
    parser.add_argument("--chip-erase", action="store_true",
        help="Full-chip erase via the custom bridge (erases APROM, LDROM, and config)")
    parser.add_argument("--recover", action="store_true",
        help=f"Recover a bricked target: power-cycle connect, full-chip erase, restore the recovery config profile ({PROJECT_RECOVERY_PROFILE}), then exit")
    parser.add_argument("--power-cycle-connect", action="store_true",
        help="If normal connect fails, try recovery connect using switched target power")
    parser.add_argument("--power-off-ms", type=int, default=DEFAULT_POWER_OFF_MS,
        help=f"Target power-off time for recovery connect (default: {DEFAULT_POWER_OFF_MS} ms)")
    parser.add_argument("--power-on-delay-us", type=int, action="append",
        help="Power-up delay before sending ICP entry bits during recovery; repeat to sweep multiple values")
    parser.add_argument("--target-power", choices=("on", "off"),
        help="Switch target VDD through the bridge and exit")
    args = parser.parse_args()

    if args.list_config_profiles:
        print_config_profiles()
        return 0

    if (args.config_profile or args.config_bytes or config_override_requested(args)) and not (args.write_config or args.recover):
        print("ERROR: config selection/override options require --write-config or --recover")
        return 1

    # Resolve port
    port = args.port
    if port is None:
        port = find_pico_port()
        if port is None:
            print("ERROR: No serial port detected. Use -p to specify.")
            return 1
        print(f"Auto-detected port: {port}")

    prog = ISPProgrammer(port, args.baud)
    run_aprom_on_close = False

    try:
        prog.open()
        recovery_delays = args.power_on_delay_us or DEFAULT_RECOVERY_DELAYS_US

        if args.target_power:
            prog.set_target_power(args.target_power == "on")
            return 0

        try:
            prog.connect()
        except ISPError:
            if not (args.power_cycle_connect or args.recover):
                raise
            print("  Normal connect failed; trying power-cycle recovery...")
            prog.connect_with_power_cycle(args.power_off_ms, recovery_delays)

        prog.print_device_info()

        # Warn if device ID doesn't match MS51FB9AE
        dev_id = prog.get_device_id()
        if dev_id not in MS51FB9AE_DEVIDS:
            print(f"\n  WARNING: Device ID 0x{dev_id:04X} is not MS51FB9AE.")
            print("  Proceeding anyway...\n")

        if args.chip_erase:
            prog.chip_erase()
            prog.print_device_info()
            run_aprom_on_close = True
            if not args.write_config and args.info:
                return 0

        if args.recover:
            if not args.chip_erase:
                prog.chip_erase()
                prog.print_device_info()
            if dev_id not in MS51FB9AE_DEVIDS:
                print(f"ERROR: refusing to write config to unknown device 0x{dev_id:04X}")
                return 1
            requested_config, requested_label = resolve_requested_config(args, PROJECT_RECOVERY_PROFILE)
            validate_config_bytes(requested_config, args.allow_unsafe_config)
            print(f"\n  Restoring {requested_label} after recovery")
            prog.write_verified_config(requested_config, requested_label)
            run_aprom_on_close = True
            return 0

        if args.erase:
            prog.erase_all()
            # Re-read device info after erase
            prog.print_device_info()
            run_aprom_on_close = True
            if not args.write_config and args.info:
                return 0

        if args.write_config:
            if dev_id not in MS51FB9AE_DEVIDS:
                print(f"ERROR: refusing to write config to unknown device 0x{dev_id:04X}")
                return 1
            requested_config, requested_label = resolve_requested_config(args, PROJECT_DEFAULT_PROFILE)
            validate_config_bytes(requested_config, args.allow_unsafe_config)
            prog.write_verified_config(requested_config, requested_label)
            run_aprom_on_close = True

        if args.info:
            run_aprom_on_close = True
            return 0

        if args.read:
            # Read flash to file
            data = prog.read_flash()
            with open(args.read, "wb") as f:
                f.write(data)
            print(f"  Saved to {args.read}")
            run_aprom_on_close = True
            return 0

        # Write flash
        if not os.path.isfile(args.file):
            print(f"ERROR: File not found: {args.file}")
            return 1

        with open(args.file, "rb") as f:
            firmware = f.read()

        print(f"  Firmware: {args.file} ({len(firmware)} bytes)")

        config = prog.read_config()
        aprom_size = decode_aprom_size_bytes(config)
        if len(firmware) > aprom_size:
            print(f"ERROR: Firmware ({len(firmware)} bytes) exceeds configured APROM size ({aprom_size} bytes)")
            return 1
        if aprom_size != MS51FB9AE_FLASH_SIZE:
            print(f"  NOTE: Config reserves {decode_ldrom_size_kb(config)} KB of LDROM; APROM is {aprom_size} bytes.")

        prog.program_aprom(firmware)

        # Auto-verify if FW supports extended commands (unless --no-verify)
        do_verify = args.verify or (prog.fw_ver >= EXTENDED_CMDS_FW_VER and not args.no_verify)
        if do_verify:
            if prog.fw_ver < EXTENDED_CMDS_FW_VER:
                print("  Skipping verify: ISP firmware doesn't support read-back.")
            else:
                if not prog.verify_flash(firmware):
                    return 1

        print("\n  Done! Resetting to APROM...")
        run_aprom_on_close = True
        return 0

    except ISPError as e:
        print(f"\nERROR: {e}")
        return 1
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130
    finally:
        prog.close(run_aprom=run_aprom_on_close)


if __name__ == "__main__":
    sys.exit(main())
