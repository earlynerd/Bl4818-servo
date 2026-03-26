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
CMD_FORMAT2_CONTINUATION = 0x00

# Firmware version thresholds
EXTENDED_CMDS_FW_VER = 0xD0
ICP_BRIDGE_FW_VER    = 0xE0

# Timeouts (seconds)
CONNECT_TIMEOUT  = 0.05
SLOW_TIMEOUT     = 0.25
ERASE_TIMEOUT    = 8.5
FORMAT2_TIMEOUT  = 0.2
READ_ROM_TIMEOUT = 2.0
RESET_TIMEOUT    = 0.5
POLL_INTERVAL    = 0.01

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

# ── ISP Programmer ──────────────────────────────────────────────────────────

class ISPError(Exception):
    pass

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

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self._send_raw(self._build_packet(CMD_RUN_APROM))
                self.seq_num += 1
            except Exception:
                pass
            time.sleep(RESET_TIMEOUT)
            self.ser.close()

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

    def send_cmd(self, cmd, data=b"", timeout=None):
        """Send a command and receive the ACK. Returns the 64-byte response."""
        if timeout is None:
            timeout = 0.2
        self.seq_num += 1
        pkt = self._build_packet(cmd, data)
        self._send_raw(pkt)

        for attempt in range(5):
            rx = self._read_response(timeout)
            if rx and len(rx) == PACKSIZE:
                # Verify checksum: rx[0:2] should equal sum of our sent packet
                rx_checksum = unpack_u16(rx[0:2])
                tx_checksum = calc_checksum(pkt)
                if rx_checksum == tx_checksum:
                    self.seq_num += 1
                    return rx
            # Retry
            self.ser.flush()
            self._send_raw(pkt)

        raise ISPError(f"No valid response for command 0x{cmd:02X} after 5 attempts")

    # ── Connection ──────────────────────────────────────────────────────

    def connect(self):
        """Establish ISP connection with the target."""
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.seq_num = 0

        print(f"  Connecting on {self.port}...")
        print("  (Reset the target board now, or the Pico bridge handles it)")

        connected = False
        for attempt in range(30):  # ~30 attempts
            self.ser.reset_input_buffer()
            self.seq_num = 0
            pkt = self._build_packet(CMD_CONNECT)
            self._send_raw(pkt)

            # ICP entry on the bridge takes ~300ms (reset sequence + entry bits
            # + device ID read), so wait long enough for it to complete
            if not self._wait_for_data(1.0):
                continue

            rx = self.ser.read(self.ser.in_waiting)
            if len(rx) < PACKSIZE:
                continue
            # Take last complete packet
            rx = rx[len(rx) - PACKSIZE:]

            rx_checksum = unpack_u16(rx[0:2])
            tx_checksum = calc_checksum(pkt)
            if rx_checksum == tx_checksum:
                connected = True
                break

        if not connected:
            raise ISPError("Could not connect to target. Check wiring and reset.")

        # Sync packet number
        self.seq_num = 0
        sync_data = pack_u32(1)
        rx = self.send_cmd(CMD_SYNC_PACKNO, sync_data, timeout=1.0)

        # Get firmware version
        rx = self.send_cmd(CMD_GET_FWVER)
        self.fw_ver = rx[8]
        fw_type = ""
        if self.fw_ver >= ICP_BRIDGE_FW_VER:
            fw_type = " (ICP bridge)"
        elif self.fw_ver >= EXTENDED_CMDS_FW_VER:
            fw_type = " (extended cmds)"
        print(f"  Connected! ISP FW version: 0x{self.fw_ver:02X}{fw_type}")

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
        # CMD_UPDATE_CONFIG sends 5 config bytes starting at data offset 8
        # The protocol packs them as: [start_addr:4][total_len:4][config0..4]
        pkt_data = pack_u32(0) + pack_u32(len(config_bytes))
        for b in config_bytes:
            pkt_data += bytes([b, 0x00, 0x00, 0x00])  # each config byte is 32-bit word
        rx = self.send_cmd(CMD_UPDATE_CONFIG, pkt_data, timeout=ERASE_TIMEOUT)
        return rx

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
        """Mass erase: erases all flash and resets config to defaults."""
        print("  Sending ERASE_ALL...")
        rx = self.send_cmd(CMD_ERASE_ALL, timeout=ERASE_TIMEOUT)
        print("  Erase complete.")
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
        help="Write factory-default config (APROM boot, unlocked, no LDROM)")
    parser.add_argument("--erase", action="store_true",
        help="Mass erase all flash and reset config (recovers locked chips)")
    args = parser.parse_args()

    # Resolve port
    port = args.port
    if port is None:
        port = find_pico_port()
        if port is None:
            print("ERROR: No serial port detected. Use -p to specify.")
            return 1
        print(f"Auto-detected port: {port}")

    prog = ISPProgrammer(port, args.baud)

    try:
        prog.open()
        prog.connect()
        prog.print_device_info()

        # Warn if device ID doesn't match MS51FB9AE
        dev_id = prog.get_device_id()
        if dev_id not in MS51FB9AE_DEVIDS:
            print(f"\n  WARNING: Device ID 0x{dev_id:04X} is not MS51FB9AE.")
            print("  Proceeding anyway...\n")

        if args.erase:
            prog.erase_all()
            # Re-read device info after erase
            prog.print_device_info()
            if not args.write_config and args.info:
                return 0

        if args.write_config:
            # Factory defaults: APROM boot, unlocked, no LDROM, BOD off, WDT stopped
            new_config = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
            print("\n  Writing config: APROM boot, unlocked, full 16KB APROM")
            old_config = prog.read_config()
            print(f"  Old config: [{' '.join(f'0x{b:02X}' for b in old_config)}]")
            print(f"  New config: [{' '.join(f'0x{b:02X}' for b in new_config)}]")
            prog.write_config(new_config)
            verify = prog.read_config()
            print(f"  Readback:   [{' '.join(f'0x{b:02X}' for b in verify)}]")
            if verify[:5] == new_config[:5]:
                print("  Config write verified OK.")
            else:
                print("  WARNING: Config readback doesn't match!")
                return 1

        if args.info:
            return 0

        if args.read:
            # Read flash to file
            data = prog.read_flash()
            with open(args.read, "wb") as f:
                f.write(data)
            print(f"  Saved to {args.read}")
            return 0

        # Write flash
        if not os.path.isfile(args.file):
            print(f"ERROR: File not found: {args.file}")
            return 1

        with open(args.file, "rb") as f:
            firmware = f.read()

        print(f"  Firmware: {args.file} ({len(firmware)} bytes)")

        if len(firmware) > MS51FB9AE_FLASH_SIZE:
            print(f"ERROR: Firmware ({len(firmware)} bytes) exceeds APROM size ({MS51FB9AE_FLASH_SIZE} bytes)")
            return 1

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
        prog.close()


if __name__ == "__main__":
    sys.exit(main())
