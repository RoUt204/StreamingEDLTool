import os
import shlex
import shutil
import struct
import sys
import time
import uuid
import zlib
import ctypes
import serial

SAHARA_HELLO_REQ               = 0x01
SAHARA_HELLO_RSP                = 0x02
SAHARA_READ_DATA               = 0x03
SAHARA_END_TRANSFER            = 0x04
SAHARA_DONE_REQ                = 0x05
SAHARA_DONE_RSP                = 0x06
SAHARA_64BIT_MEMORY_READ_DATA  = 0x12
SAHARA_VERSION                 = 2
SAHARA_VERSION_SUPPORTED       = 2
SAHARA_MODE_IMAGE_TX_PENDING   = 0

CMD_WRITE_32BIT = 0x0F
CMD_GO          = 0x05
CMD_ACK         = 0x02
PACKET_TIMEOUT  = 60.0

RECTYPE_DATA     = 0
RECTYPE_EOF      = 1
RECTYPE_EXT_ADDR = 4
RECTYPE_GO       = 5

_PROGRESS_LINE_WIDTH = 120
_BAR_LENGTH          = 20
_SECTOR_SIZE         = 512
_NO_DETAILS          = object()

_MBR_SIGNATURE       = b'\x55\xAA'
_GPT_SIGNATURE       = b'EFI PART'
_MBR_EXTENDED_TYPES  = (0x05, 0x0F, 0x85)
_MBR_PROTECTIVE_TYPE = 0xEE
_MAX_EBR_COUNT       = 256
_MAX_GPT_TABLE       = 1024 * 1024

_EMMC_VENDORS = {
    0x90: "SK Hynix",
    0x15: "Samsung",
    0x45: "SanDisk",
    0x11: "Toshiba",
    0x13: "Micron",
    0x70: "Kingston",
    0xFE: "Micron",
    0x2C: "Micron",
}


RESPONSE_DESCRIPTIONS = {
    0x02: "Response to hello",
    0x04: "Response to successful EMMC read",
    0x08: "Response to successful stream write",
    0x0A: "Response when phone is idle (NOP)",
    0x0C: "Response to successful phone reboot",
    0x0D: "Error",
    0x0E: "Information",
    0x16: "Response to successful close and flush of last partial flash write",
    0x18: "Response to successful protection bypass",
    0x1C: "Response to successful flash chip open",
    0x1E: "Response to successful flash erase (CMD38)",
    0x24: "Response to successful flash info read",
}

_HELP = [
    ("exit", "Quit"),
    ("help", "Show this help"),
    ("load -f <file> -n (newline)", "Load a HEX/MBN/ELF programmer file"),
    ("raw <bytes in HEX>", "Send RAW hex bytes to device"),
    ("nop", "Ask what device is doing (NOP)"),
    ("hello", "Get hello response"),
    ("bypass", "Bypass security"),
    ("close", "Close after init storage"),
    ("reboot", "Reboot phone"),
    ("initstorage", "Open eMMC card"),
    ("getstorageinfo", "Get eMMC information"),
    ("readpartitiontable", "Read partition table (short)"),
    ("readfullpartitiontable", "Read partition table (all fields)"),
    ("read  -s <start_sector> -l <length> | -p <partition> -f <file> -n (newline)", "Read from eMMC"),
    ("readhex -s <start_sector> -l <length> | -p <partition> [-l <length>]", "Read from eMMC and print as HEX"),
    ("write -s <start_sector> | -p <partition> -f <file> -n (newline)", "Write data to eMMC"),
    ("erase -s <start_sector> -l <length> | -p <partition> -n (newline)", "Write 00 to eMMC"),
    ("eraseall", "Erase all eMMC (CMD38)"),
    ("storagetest -s <start_sector> -l <length> | -p <partition> -n (newline)", "Write random data to eMMC and verify it"),
    ("safestoragetest -s <start_sector> -l <length> | -p <partition> -n (newline)", "Same, then restore the original data"),
]
_HELP_WIDTH = max(len(cmd) for cmd, _ in _HELP)
HELP_TEXT = (
    "Streaming EDL Tool v1.1 Alpha:\n\n"
    + "\n".join(f"  {cmd:<{_HELP_WIDTH}} - {desc}" for cmd, desc in _HELP)
)


class LazySerial:
    def __init__(self, port, baudrate=115200, timeout=5.0):
        self._port     = port
        self._baudrate = baudrate
        self._timeout  = timeout
        self._ser      = None

    def _ensure_open(self):
        if self._ser is None or not self._ser.is_open:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
        return self._ser

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def __getattr__(self, name):
        return getattr(self._ensure_open(), name)


def _build_crc16_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
        table.append(crc)
    return table


_CRC16_TABLE = _build_crc16_table()


def calc_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc = _CRC16_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFF


def hdlc_encode(payload):
    data_with_crc = bytes(payload) + struct.pack('<H', calc_crc16(payload))
    escaped = data_with_crc.replace(b'\x7d', b'\x7d\x5d').replace(b'\x7e', b'\x7d\x5e')
    return b'\x7e' + escaped + b'\x7e'


def hdlc_decode(packet):
    decoded = bytearray()
    escape = False
    for byte in packet:
        if byte == 0x7E:
            continue
        if byte == 0x7D:
            escape = True
            continue
        if escape:
            decoded.append(byte ^ 0x20)
            escape = False
        else:
            decoded.append(byte)
    if len(decoded) < 2:
        return None
    payload, crc_bytes = bytes(decoded[:-2]), bytes(decoded[-2:])
    if struct.pack('<H', calc_crc16(payload)) != crc_bytes:
        return None
    return payload


def _read_hdlc_packet(port, deadline, check_open=False, skip_info=False):
    buf = bytearray()
    started = False
    while time.monotonic() < deadline:
        if check_open and not port.is_open:
            return None
        waiting = port.in_waiting
        raw = port.read(waiting if waiting > 0 else 1)
        if not raw:
            continue
        for byte in raw:
            if byte == 0x7E:
                if not started:
                    started = True
                elif len(buf) > 0:
                    pkt = hdlc_decode(buf)
                    buf = bytearray()
                    if pkt is None:
                        continue
                    if skip_info and (len(pkt) == 0 or pkt[0] == 0x0E):
                        continue
                    return pkt
            elif started:
                buf.append(byte)
    return None


def send_sync(port, payload, timeout=PACKET_TIMEOUT, skip_info=False, check_open=True):
    try:
        port.reset_input_buffer()
        port.write(hdlc_encode(payload))
    except (serial.SerialException, OSError) as e:
        print(f"Device   > Port error: {e}")
        return None
    try:
        return _read_hdlc_packet(
            port, time.monotonic() + timeout,
            check_open=check_open, skip_info=skip_info,
        )
    except (serial.SerialException, OSError) as e:
        print(f"Device   > Port error: {e}")
        return None


def format_time(seconds):
    seconds = int(seconds)
    d, rem = divmod(seconds, 86400)
    h, rem = divmod(rem, 3600)
    m, s = divmod(rem, 60)
    if d: return f"{d}d {h:02}h {m:02}m {s:02}s"
    if h: return f"{h}h {m:02}m {s:02}s"
    if m: return f"{m}m {s:02}s"
    return f"{s}s"


def _hex_ascii_lines(data, bytes_per_line=16):
    for i in range(0, len(data), bytes_per_line):
        chunk = data[i:i + bytes_per_line]
        hex_part = " ".join(f"{b:02X}" for b in chunk).ljust(bytes_per_line * 3 - 1)
        txt_part = "".join(chr(b) if 32 <= b <= 126 else "." for b in chunk)
        yield f"{hex_part}  |  {txt_part}"


def print_response(resp):
    bytes_per_line = 16
    code = resp[0] if resp else None
    desc = RESPONSE_DESCRIPTIONS.get(code, "Unknown response code") if code is not None else ""
    header = f"0x{code:02X} - {desc}" if code is not None else "empty"
    print(f"Device   > Response ({len(resp)} bytes)  [{header}]")
    for line in _hex_ascii_lines(resp, bytes_per_line):
        print(line)
    print("-" * (bytes_per_line * 3 + bytes_per_line + 5))


def print_status(ok, msg_ok, msg_fail):
    print(f"Device   > {msg_ok if ok else msg_fail}")


def check_response(resp, expected_code, name, silent=False):
    if not resp:
        print(f"Device   > Timeout (No Response to {name})")
        return False
    ok = resp[0] == expected_code
    if not silent:
        print_status(ok, f"{name} OK", f"{name} failed (0x{resp[0]:02X})")
        if not ok:
            print_response(resp)
    return ok


def _print_progress(label, done_bytes, total_bytes, offset, start_time, line_end):
    percent = min(done_bytes / total_bytes, 1.0)
    filled = int(_BAR_LENGTH * percent)
    bar = '█' * filled + '-' * (_BAR_LENGTH - filled)
    elapsed = time.monotonic() - start_time
    speed_kb = (done_bytes / 1024) / elapsed if elapsed > 0 else 0
    eta = (total_bytes - done_bytes) / 1024 / speed_kb if speed_kb > 0 else 0
    line = (
        f"Device   > {label}: [{bar}] {percent * 100:5.1f}% "
        f"|{offset:9d} Sector |{done_bytes / 1024:11.1f} KB "
        f"|{speed_kb:6.1f} KB/s | ETA: {format_time(eta)}"
    )
    width = min(_PROGRESS_LINE_WIDTH, shutil.get_terminal_size(fallback=(_PROGRESS_LINE_WIDTH, 24)).columns)
    print(f"{line:<{width}}", end=line_end, flush=True)


def _report_failure(newline_needed, message, resp):
    prefix = "\n" if newline_needed else ""
    print(f"{prefix}Device   > {message}")
    if resp is _NO_DETAILS:
        return
    if resp:
        print_response(resp)
    else:
        print("Device   > Timeout (No Response)")


def parse_storage_info(resp):
    if not resp or len(resp) < 49 or resp[0] != 0x24:
        print("Device   > Failed to parse storage info (invalid response)")
        return

    block_count  = struct.unpack_from('<I', resp, 5)[0]
    manufacturer = resp[39]
    name         = resp[43:49].decode('ascii', errors='replace').strip().rstrip('\x00')

    vendor = _EMMC_VENDORS.get(manufacturer, f"Unknown (0x{manufacturer:02X})")

    total_kb = block_count * _SECTOR_SIZE / 1024
    total_mb = total_kb / 1024
    total_gb = total_mb / 1024

    print(f"Device   > Storage Vendor: {vendor}")
    print(f"Device   > Storage Name: {name}")
    print(f"Device   > Storage Size: {block_count} sectors  ({total_kb:.0f} KB / {total_mb:.0f} MB / {total_gb:.2f} GB)")


def make_hello():
    magic = b'QCOM fast download protocol host'
    return bytearray([0x01]) + magic + bytes([3, 2, 1 | 0x10])


def do_hello(ser, silent=False):
    resp = send_sync(ser, make_hello())
    ok = check_response(resp, 0x02, "Hello", silent)
    return ok, resp


def do_presequence(ser, silent=False):
    ok, _ = do_hello(ser, silent)
    if not ok:
        return False
    if not check_response(send_sync(ser, bytearray([0x17, 0x00])), 0x18, "Bypass", silent):
        return False
    if not check_response(send_sync(ser, bytearray([0x1B, 0x21])), 0x1C, "EMMC initialization", silent):
        return False
    return True


def _close_flush_check(ser):
    resp = send_sync(ser, bytearray([0x15]))
    check_response(resp, 0x16, "Close")


def _parse_args(user_input, required, usage, posix=True, allow_partition=False,
                 allow_partition_length=False):
    try:
        parts = shlex.split(user_input, posix=posix)
    except ValueError as e:
        print(f"Device   > Error: {e}. Usage: {usage}")
        return None
    if not posix:
        parts = [p.strip('"\'') for p in parts]
    result = {}
    if allow_partition and '-p' in parts:
        blocked_with_p = ('-s',) if allow_partition_length else ('-s', '-l')
        if any(flag in parts for flag in blocked_with_p):
            joined = "/".join(blocked_with_p)
            print(f"Device   > Error: -p cannot be combined with {joined}. Usage: {usage}")
            return None
        try:
            result['partition'] = parts[parts.index('-p') + 1]
        except IndexError:
            result['partition'] = ''
        if not result['partition'] or result['partition'].startswith('-'):
            print(f"Device   > Usage: {usage}")
            return None
        if allow_partition_length and '-l' in parts:
            try:
                part_length = int(parts[parts.index('-l') + 1])
            except (ValueError, IndexError):
                print(f"Device   > Usage: {usage}")
                return None
            if part_length < 0:
                print(f"Device   > Usage: {usage}")
                return None
            result['partition_length'] = part_length
        required = [r for r in required if r[1] not in ('-s', '-l')]
    try:
        for key, flag, cast in required:
            value = cast(parts[parts.index(flag) + 1])
            if cast is int and value < 0:
                raise ValueError(f"{flag} must not be negative")
            result[key] = value
    except (ValueError, IndexError):
        print(f"Device   > Usage: {usage}")
        return None
    result['newline'] = '-n' in parts
    return result

class _COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", ctypes.c_ulong),
        ("ReadTotalTimeoutMultiplier", ctypes.c_ulong),
        ("ReadTotalTimeoutConstant", ctypes.c_ulong),
        ("WriteTotalTimeoutMultiplier", ctypes.c_ulong),
        ("WriteTotalTimeoutConstant", ctypes.c_ulong),
    ]


class RawSaharaPort:
    _GENERIC_READ = 0x80000000
    _GENERIC_WRITE = 0x40000000
    _OPEN_EXISTING = 3
    _INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    def __init__(self, port_name, timeout=10.0):
        create_file_w = ctypes.windll.kernel32.CreateFileW
        create_file_w.restype = ctypes.c_void_p
        create_file_w.argtypes = [
            ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ]
        name = port_name if port_name.startswith('\\\\.\\') else '\\\\.\\' + port_name
        self._h = create_file_w(
            name, self._GENERIC_READ | self._GENERIC_WRITE, 0, None,
            self._OPEN_EXISTING, 0, None
        )
        if self._h in (self._INVALID_HANDLE_VALUE, 0, None):
            raise OSError(f"CreateFile failed for {port_name} (err {ctypes.GetLastError()})")

        timeout_ms = max(int(timeout * 1000), 1)
        timeouts = _COMMTIMEOUTS(0, 0, timeout_ms, 0, timeout_ms)
        if not ctypes.windll.kernel32.SetCommTimeouts(self._h, ctypes.byref(timeouts)):
            err = ctypes.GetLastError()
            self.close()
            raise OSError(f"SetCommTimeouts failed (err {err})")

    def read(self, n):
        buf = ctypes.create_string_buffer(n)
        got = ctypes.c_ulong(0)
        if not ctypes.windll.kernel32.ReadFile(self._h, buf, n, ctypes.byref(got), None):
            return b""
        return buf.raw[:got.value]

    def write(self, data):
        sent = ctypes.c_ulong(0)
        if not ctypes.windll.kernel32.WriteFile(self._h, bytes(data), len(data), ctypes.byref(sent), None):
            raise OSError(f"WriteFile failed (err {ctypes.GetLastError()})")
        return sent.value

    def close(self):
        if getattr(self, "_h", None):
            ctypes.windll.kernel32.CloseHandle(self._h)
            self._h = None


class RawSaharaPortPosix:
    def __init__(self, port_name, timeout=10.0):
        import termios

        self._timeout = timeout

        self._fd = os.open(
            port_name,
            os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        )

        try:

            attrs = termios.tcgetattr(self._fd)

            attrs[0] &= ~(
                termios.IGNBRK |
                termios.BRKINT |
                termios.PARMRK |
                termios.ISTRIP |
                termios.INLCR |
                termios.IGNCR |
                termios.ICRNL |
                termios.IXON |
                termios.IXOFF |
                getattr(termios, 'IXANY', 0)
            )

            attrs[1] &= ~termios.OPOST

            attrs[2] &= ~(
                termios.CSIZE |
                termios.PARENB |
                getattr(termios, 'CSTOPB', 0) |
                getattr(termios, 'CRTSCTS', 0)
            )
            attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD

            attrs[3] &= ~(
                termios.ECHO |
                termios.ECHONL |
                termios.ICANON |
                termios.ISIG |
                getattr(termios, 'IEXTEN', 0)
            )

            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0

            try:
                termios.cfsetispeed(attrs, termios.B115200)
                termios.cfsetospeed(attrs, termios.B115200)
            except AttributeError:
                attrs[4] = attrs[5] = termios.B115200

            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)

        except Exception:
            os.close(self._fd)
            self._fd = None
            raise

    @property
    def is_open(self):
        return getattr(self, "_fd", None) is not None

    def read(self, n):
        import select

        deadline = time.monotonic() + self._timeout
        buf = bytearray()

        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            try:
                r, _, _ = select.select([self._fd], [], [], remaining)
            except (InterruptedError, OSError):
                continue

            if not r:
                break

            try:
                chunk = os.read(self._fd, n - len(buf))
            except BlockingIOError:
                continue
            except OSError:
                break

            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.001)

        return bytes(buf)

    def write(self, data):
        import select

        data = bytes(data)
        deadline = time.monotonic() + self._timeout
        off = 0

        while off < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            try:
                _, w, _ = select.select([], [self._fd], [], remaining)
            except (InterruptedError, OSError):
                continue

            if not w:
                break

            try:
                sent = os.write(self._fd, data[off:])
            except BlockingIOError:
                continue
            except OSError:
                break

            if sent <= 0:
                break

            off += sent

        return off

    def close(self):
        fd = getattr(self, "_fd", None)
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
            self._fd = None


def _open_sahara_port(com_port):
    if sys.platform.startswith('win'):
        return RawSaharaPort(com_port, timeout=10.0)
    return RawSaharaPortPosix(com_port, timeout=10.0)


def _detect_loader_format(path):
    try:
        with open(path, 'rb') as f:
            head = f.read(4)
    except FileNotFoundError:
        return None, f"File not found: '{path}'"
    except OSError as e:
        return None, f"Cannot open file: {e}"
    if head[:4] == b'\x7fELF':
        return 'sahara', None
    if head[:3] == b'\xD1\xDC\x4B':
        return 'sahara', None
    if head[:1] == b':':
        return 'dmss_hex', None
    return None, f"Unknown loader file format (header bytes: {head.hex().upper()})"


def _ihex_line_bytes(line):
    try:
        raw = bytes.fromhex(line[1:])
    except ValueError:
        return None
    if len(raw) < 5 or len(raw) != 5 + raw[0] or (sum(raw) & 0xFF) != 0:
        return None
    return raw


def load_programmer(com_port, prog_filename, fmt, bNewline=False):
    if fmt == 'dmss_hex':
        try:
            port = serial.Serial(com_port, 115200, timeout=5.0)
            port.reset_input_buffer()
        except Exception as e:
            print(f"Device   > Could not open port: {e}")
            return

        try:
            hex_file = open(prog_filename, 'r')
        except FileNotFoundError:
            print(f"Device   > File not found: '{prog_filename}'")
            port.close()
            return
        except Exception as e:
            print(f"Device   > Cannot open file: {e}")
            port.close()
            return

        current_high_addr = 0
        go_addr = 0
        newline_needed = [False]
        line_end = '\n' if bNewline else '\r'

        print(f"Device   > Loading file : {prog_filename}...")

        def _dmss_send_chunk(target_addr, data):
            payload = bytearray([CMD_WRITE_32BIT])
            payload += struct.pack('>I', target_addr)
            payload += struct.pack('>H', len(data))
            payload += data
            rsp = send_sync(port, payload)
            if rsp and rsp[0] == CMD_ACK:
                print(f"Device   > Loading at address: 0x{target_addr:08X}", end=line_end, flush=True)
                if line_end != '\n':
                    newline_needed[0] = True
                return True
            _report_failure(newline_needed[0], f"Error during loading at 0x{target_addr:08X}", rsp)
            return False

        with hex_file as f:
            write_buf = bytearray()
            target_addr = 0

            for line in f:
                line = line.strip()
                if not line.startswith(':') or len(line) < 11:
                    continue
                raw = _ihex_line_bytes(line)
                if raw is None:
                    print(f"Device   > Warning: bad checksum/format in HEX line, skipping: {line}")
                    continue
                length, addr_hi, addr_lo, rectype = raw[0], raw[1], raw[2], raw[3]
                addr = (addr_hi << 8) | addr_lo
                data = raw[4:4 + length]
                if len(data) != length:
                    print(f"Device   > Warning: truncated HEX line, skipping: {line}")
                    continue

                if rectype == RECTYPE_DATA:
                    record_addr = current_high_addr + addr
                    if write_buf and record_addr != target_addr + len(write_buf):
                        if not _dmss_send_chunk(target_addr, bytes(write_buf)):
                            port.close()
                            return
                        write_buf = bytearray()
                    if not write_buf:
                        target_addr = record_addr
                    write_buf.extend(data)
                    while len(write_buf) >= 256:
                        chunk = write_buf[:256]
                        if not _dmss_send_chunk(target_addr, chunk):
                            port.close()
                            return
                        target_addr += 256
                        write_buf = write_buf[256:]

                elif rectype == RECTYPE_EOF:
                    break

                elif rectype == RECTYPE_EXT_ADDR:
                    if write_buf:
                        if not _dmss_send_chunk(target_addr, bytes(write_buf)):
                            port.close()
                            return
                        write_buf = bytearray()
                    current_high_addr = int.from_bytes(data[:2], 'big') << 16

                elif rectype == RECTYPE_GO:
                    go_addr = int.from_bytes(data[:4], 'big')

            if write_buf:
                if not _dmss_send_chunk(target_addr, bytes(write_buf)):
                    port.close()
                    return

        prefix = "\n" if newline_needed[0] else ""
        print(f"{prefix}Device   > Executing GO at 0x{go_addr:08X}")
        go_payload = bytearray([CMD_GO]) + struct.pack('>I', go_addr)
        port.reset_input_buffer()
        final_rsp = send_sync(port, go_payload)
        time.sleep(5)

        if not (final_rsp and final_rsp[0] == CMD_ACK):
            _report_failure(False, f"GO failed at 0x{go_addr:08X}", final_rsp)
        port.close()
        return

    if fmt == 'sahara':
        try:
            port = _open_sahara_port(com_port)
        except Exception as e:
            print(f"Device   > Could not open port: {e}")
            return

        try:
            prog_file = open(prog_filename, 'rb')
        except FileNotFoundError:
            print(f"Device   > File not found: '{prog_filename}'")
            port.close()
            return
        except Exception as e:
            print(f"Device   > Cannot open file: {e}")
            port.close()
            return

        print(f"Device   > Loading file : {prog_filename}...")

        hello = port.read(48)
        if len(hello) < 48:
            print("Device   > Sahara Hello handshake failed (no/invalid HELLO_REQ)")
            port.close()
            return
        cmd, _length = struct.unpack_from('<II', hello, 0)
        if cmd != SAHARA_HELLO_REQ:
            print("Device   > Sahara Hello handshake failed (no/invalid HELLO_REQ)")
            port.close()
            return
        hello_resp = struct.pack(
            '<12I',
            SAHARA_HELLO_RSP, 0x30,
            SAHARA_VERSION, SAHARA_VERSION_SUPPORTED,
            0, SAHARA_MODE_IMAGE_TX_PENDING,
            0, 0, 0, 0, 0, 0,
        )
        port.write(hello_resp)

        total_bytes = 0

        with prog_file as f:
            while True:
                hdr = port.read(8)
                if len(hdr) < 8:
                    print("Device   > Timeout waiting for Sahara read request")
                    port.close()
                    return
                cmd, length = struct.unpack('<II', hdr)

                if cmd == SAHARA_READ_DATA:
                    body = port.read(12)
                    if len(body) < 12:
                        print("Device   > Timeout reading Sahara read_data body")
                        port.close()
                        return
                    _id, offset, data_len = struct.unpack('<III', body)

                elif cmd == SAHARA_64BIT_MEMORY_READ_DATA:
                    body = port.read(24)
                    if len(body) < 24:
                        print("Device   > Timeout reading Sahara read_data_64 body")
                        port.close()
                        return
                    _id, offset, data_len = struct.unpack('<QQQ', body)

                elif cmd == SAHARA_END_TRANSFER:
                    break

                else:
                    print(f"Device   > Unexpected Sahara command: 0x{cmd:02X}")
                    port.close()
                    return

                f.seek(offset)
                chunk = f.read(data_len)
                port.write(chunk)
                total_bytes += len(chunk)
                print(f"Device   > Sent {total_bytes} bytes (offset 0x{offset:X}, len {len(chunk)})", end='\r', flush=True)

        print()

        end_body = port.read(8)
        if len(end_body) < 8:
            print("Device   > Timeout waiting for END_TRANSFER status")
            port.close()
            return
        _id, status = struct.unpack('<II', end_body)
        if status != 0:
            print(f"Device   > Sahara image load failed, status: {status}")
            port.close()
            return

        port.write(struct.pack('<II', SAHARA_DONE_REQ, 8))
        done_resp = port.read(12)
        if len(done_resp) >= 4 and struct.unpack('<I', done_resp[:4])[0] == SAHARA_DONE_RSP:
            print("Device   > Sending OK, flash programmer running")
        else:
            print("Device   > Sahara Done failed (no/invalid DONE_RSP)")

        port.close()
        return

    print(f"Device   > Unknown loader format: {fmt!r}")



def _stream_loop(ser, start, length_sectors, bNewline, label, per_sector_fn):
    line_end = '\n' if bNewline else '\r'
    offset = start
    total_bytes = length_sectors * _SECTOR_SIZE
    done_bytes = 0
    start_time = time.monotonic()
    newline_needed = False

    while done_bytes < total_bytes:
        ok, inc_sectors, err_msg, resp = per_sector_fn(ser, offset)
        if not ok:
            _report_failure(newline_needed, f"{err_msg} at sector: {offset}", resp)
            return False, newline_needed, start_time

        offset += inc_sectors
        done_bytes += inc_sectors * _SECTOR_SIZE
        _print_progress(label, done_bytes, total_bytes, offset, start_time, line_end)
        if not bNewline:
            newline_needed = True

    return True, newline_needed, start_time


def _finish(success, length_sectors, newline_needed, start_time, done_msg):
    prefix = "\n" if newline_needed else ""
    if success and length_sectors > 0:
        elapsed = time.monotonic() - start_time
        print(f"{prefix}Device   > {done_msg} | Time: {format_time(elapsed)}")


def _cmd_nop(ser, user_input):
    resp = send_sync(ser, bytearray([0x09]))
    if resp:
        print_response(resp)
    else:
        print("Device   > Device Timeout (No Response)")


def _cmd_hello(ser, user_input):
    do_hello(ser)


def _cmd_bypass(ser, user_input):
    check_response(send_sync(ser, bytearray([0x17, 0x00])), 0x18, "Bypass")


def _cmd_close(ser, user_input):
    _close_flush_check(ser)


def _cmd_reboot(ser, user_input):
    if not do_hello(ser)[0]:
        return
    if not check_response(send_sync(ser, bytearray([0x17, 0x00])), 0x18, "Bypass"):
        return
    _close_flush_check(ser)
    check_response(send_sync(ser, bytearray([0x0B])), 0x0C, "Reboot")


def _cmd_initstorage(ser, user_input):
    if do_presequence(ser):
        _close_flush_check(ser)


def _cmd_getstorageinfo(ser, user_input):
    if not do_presequence(ser):
        return
    resp = send_sync(ser, bytes.fromhex("23" + "00" * 33 + "addeb21c"))
    if resp and resp[0] == 0x24:
        print("Device   > Get Storage Information OK")
        parse_storage_info(resp)
    elif resp:
        print(f"Device   > Get Storage Information failed (0x{resp[0]:02X})")
        print_response(resp)
    else:
        print("Device   > Timeout (No Response to Get Storage Information)")
    _close_flush_check(ser)


def _read_sector(ser, offset):
    if not 0 <= offset <= 0xFFFFFFFF:
        return False, None, None
    cmd = bytearray([0x03]) + struct.pack('<I', offset) + struct.pack('>H', 2)
    resp = send_sync(ser, cmd)
    if (resp and len(resp) >= 517 and resp[0] == 0x04
            and struct.unpack_from('<I', resp, 1)[0] == offset):
        return True, resp[5:517], resp
    return False, None, resp


def _write_sector(ser, offset, data):
    if not 0 <= offset <= 0xFFFFFFFF:
        return False, None
    cmd = bytearray([0x07]) + struct.pack('<I', offset) + bytes(data)
    resp = send_sync(ser, cmd, skip_info=True)
    return (resp and resp[0] == 0x08), resp


class _SectorReadError(Exception):
    @property
    def lba(self):
        return self.args[0]


_MBR_ENTRY = struct.Struct('<B3sB3sII')
_MBR_KEYS = ('boot', 'chs_start', 'type', 'chs_end', 'start', 'count')
_GPT_HEADER = struct.Struct('<8sIIIIQQQQ16sQIII')
_GPT_KEYS = ('sig', 'rev', 'size', 'crc', 'reserved', 'my', 'alt', 'first', 'last', 'guid', 'elba', 'count', 'esize', 'ecrc')
_GPT_ENTRY = struct.Struct('<16s16sQQQ72s')
_ATTR_RESERVED = 0x0000FFFFFFFFFFF8


def _read_lba(ser, lba):
    ok, data, _ = _read_sector(ser, lba) if lba <= 0xFFFFFFFF else (False, None, None)
    if not ok:
        raise _SectorReadError(lba)
    return bytes(data)


def _get_total_sectors(ser):
    resp = send_sync(ser, bytes.fromhex("23" + "00" * 33 + "addeb21c"), timeout=10.0)
    return struct.unpack_from('<I', resp, 5)[0] if resp and len(resp) >= 9 and resp[0] == 0x24 else None


def _format_size(num_bytes):
    size = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024:
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.2f} {unit}"
        size /= 1024
    return f"{size:.2f} TB"


def _crc_status(stored, calculated):
    if calculated == stored:
        return f"OK ({stored:08X})"
    return f"BAD (stored {stored:08X}, calculated {'n/a' if calculated is None else f'{calculated:08X}'})"


def _chs(raw):
    return f"{((raw[1] & 0xC0) << 2) | raw[2]}/{raw[0]}/{raw[1] & 0x3F}"


def _parse_mbr_entries(sector):
    return [dict(zip(_MBR_KEYS, _MBR_ENTRY.unpack_from(sector, 446 + 16 * i))) for i in range(4)]


def _parse_gpt_header(hdr):
    h = dict(zip(_GPT_KEYS, _GPT_HEADER.unpack_from(hdr)))
    h['crc_calc'] = zlib.crc32(hdr[:16] + bytes(4) + hdr[20:h['size']]) if 92 <= h['size'] <= _SECTOR_SIZE else None
    return h


def _is_empty_mbr_entry(entry):
    return entry['type'] == 0 and entry['count'] == 0


def _special_entry(name, start, count):
    return {'number': None, 'name': name, 'type': '', 'start': start, 'count': count}


def _mbr_partition(number, e, start):
    return {
        'number': number, 'name': '', 'type': f"0x{e['type']:02X}", 'start': start, 'count': e['count'],
        'flag': "*" if e['boot'] == 0x80 else ("?" if e['boot'] else ""),
        'container': e['type'] in _MBR_EXTENDED_TYPES,
    }


def _mbr_header(mbr, entries):
    nonzero = sum(map(bool, mbr[:440]))
    return {
        'info': [
            ("Disk signature", f"{struct.unpack_from('<I', mbr, 440)[0]:08X}"),
            ("Reserved", mbr[444:446].hex().upper()),
            ("Boot signature", mbr[510:512].hex().upper()),
            ("Bootstrap code", f"present ({nonzero} of 440 bytes non-zero)" if nonzero else "empty (all 440 bytes are zero)"),
        ],
        'entries': entries,
        'bootstrap': bytes(mbr[:440]) if nonzero else None,
    }


def _scan_ebr_chain(ser, ext_start, number, notes):
    partitions, chain, ebr_lba, visited = [], [], ext_start, {ext_start}
    for _ in range(_MAX_EBR_COUNT):
        try:
            sector = _read_lba(ser, ebr_lba)
        except _SectorReadError as e:
            notes.append(f"Read Error at sector: {e.lba}")
            break
        if sector[510:512] != _MBR_SIGNATURE:
            notes.append(f"Invalid EBR signature at sector: {ebr_lba} (found {sector[510:512].hex().upper()})")
            break
        entries = _parse_mbr_entries(sector)
        logical, link = entries[:2]
        item = {
            'lba': ebr_lba, 'logical': None, 'next': None, 'signature': sector[510:512].hex().upper(),
            'unused': sum(map(bool, sector[:446])),
            'entries': entries,
        }
        chain.append(item)
        if not all(map(_is_empty_mbr_entry, entries[2:])):
            notes.append(f"EBR at sector {ebr_lba} has non-empty entries 3/4")
        if not _is_empty_mbr_entry(logical):
            partitions.append(_mbr_partition(number, logical, ebr_lba + logical['start']))
            item['logical'] = number
            number += 1
        if link['type'] not in _MBR_EXTENDED_TYPES or link['count'] == 0:
            break
        ebr_lba = item['next'] = ext_start + link['start']
        if ebr_lba in visited:
            notes.append(f"EBR chain loop detected at sector: {ebr_lba}")
            break
        visited.add(ebr_lba)
    else:
        notes.append(f"EBR chain too long, stopped after {_MAX_EBR_COUNT} entries")
    return partitions, chain


def _ebr_entries(chain):
    runs = []
    for lba in sorted({item['lba'] for item in chain}):
        if runs and lba == runs[-1][0] + runs[-1][1]:
            runs[-1][1] += 1
        else:
            runs.append([lba, 1])
    return [_special_entry(f"EBR{i}", start, count) for i, (start, count) in enumerate(runs, 1)]


def _scan_mbr(ser, entries, notes):
    partitions = [_special_entry("MBR", 0, 1)]
    ext = None
    for number, e in enumerate(entries, 1):
        if not _is_empty_mbr_entry(e):
            partitions.append(_mbr_partition(number, e, e['start']))
            if e['type'] in _MBR_EXTENDED_TYPES and ext is None:
                ext = e
    chain = []
    if ext:
        logicals, chain = _scan_ebr_chain(ser, ext['start'], 5, notes)
        partitions += logicals + _ebr_entries(chain)
    return {'kind': 'MBR', 'info': [], 'partitions': partitions, 'ebr_chain': chain, 'notes': notes}


def _verify_backup_gpt(ser, h, sectors, primary_raw):
    try:
        bh = _read_lba(ser, h['alt'])
    except _SectorReadError:
        return f"not readable (sector {h['alt']})"
    if not any(bh):
        return f"missing (sector {h['alt']} is empty)"
    if bh[:8] != _GPT_SIGNATURE:
        return f"BAD (no GPT signature at sector {h['alt']})"
    b = _parse_gpt_header(bh)
    problems = [text for bad, text in (
        (b['crc_calc'] != b['crc'], "header CRC"),
        (b['my'] != h['alt'], "header LBA"),
        (b['alt'] != h['my'], "backup LBA"),
        (b['ecrc'] != h['ecrc'], "entries CRC differs from primary"),
    ) if bad]
    try:
        b_raw = b"".join(_read_lba(ser, b['elba'] + i) for i in range(sectors))
        if len(b_raw) != len(primary_raw) or b_raw[:len(primary_raw)] != primary_raw:
            problems.append("entries differ from primary")
    except _SectorReadError:
        problems.append("entries not readable")
    return f"BAD ({', '.join(problems)})" if problems else "OK (header CRC valid, entries identical to primary)"


def _scan_gpt(ser, hdr, notes, full):
    h = _parse_gpt_header(hdr)
    if h['esize'] < 128 or h['count'] == 0 or h['count'] * h['esize'] > _MAX_GPT_TABLE:
        notes.append(f"Invalid GPT header (entries: {h['count']}, entry size: {h['esize']})")
        return None

    size = h['count'] * h['esize']
    sectors = -(-size // _SECTOR_SIZE)
    raw = b"".join(_read_lba(ser, h['elba'] + i) for i in range(sectors))[:size]

    partitions = []
    for i in range(h['count']):
        type_guid, unique, first, last, attrs, name = _GPT_ENTRY.unpack_from(raw, i * h['esize'])
        if any(type_guid):
            partitions.append({
                'number': i + 1, 'name': name.decode('utf-16-le', errors='replace').split('\x00')[0],
                'type': str(uuid.UUID(bytes_le=type_guid)).upper(), 'unique': str(uuid.UUID(bytes_le=unique)).upper(),
                'attrs': attrs, 'start': first, 'count': max(last - first + 1, 0),
            })
    used = len(partitions)

    entries_end = h['elba'] + sectors
    primary_end = h['first'] if entries_end <= h['first'] <= h['last'] else entries_end
    partitions.append(_special_entry("PrimaryGPT", 0, primary_end))
    backup_start = h['last'] + 1 if primary_end <= h['last'] + 1 <= h['alt'] else h['alt'] - sectors
    backup_ok = primary_end <= backup_start <= h['alt']
    if backup_ok:
        partitions.append(_special_entry("BackupGPT", backup_start, h['alt'] - backup_start + 1))
    else:
        notes.append(f"Backup GPT location looks invalid (backup header LBA {h['alt']})")

    pad = hdr[h['size']:] if 92 <= h['size'] <= _SECTOR_SIZE else None
    nonzero = sum(map(bool, pad or b""))
    padding = ("n/a (invalid header size)" if pad is None else
               f"{nonzero} of {len(pad)} bytes non-zero" if nonzero else f"empty (all {len(pad)} bytes are zero)")
    info = [  
        ("Signature", h['sig'].decode('ascii', errors='replace')),
        ("Revision", f"{h['rev'] >> 16}.{h['rev'] & 0xFFFF}"),
        ("Header size", str(h['size'])),
        ("Header CRC", _crc_status(h['crc'], h['crc_calc'])),
        ("Reserved", f"{h['reserved']:08X}"),
        ("Header LBA", str(h['my'])),
        ("Backup LBA", str(h['alt'])),
        ("Usable LBA", f"{h['first']} - {h['last']}"),
        ("Disk GUID", str(uuid.UUID(bytes_le=h['guid'])).upper()),
        ("Entries LBA", str(h['elba'])),
        ("Entries", f"{used} used / {h['count']} slots"),
        ("Entry size", str(h['esize'])),
        ("Entries CRC", _crc_status(h['ecrc'], zlib.crc32(raw))),
        ("Header padding", padding),
    ]
    if full and backup_ok:
        info.append(("Backup GPT", _verify_backup_gpt(ser, h, sectors, raw)))
    return {'kind': 'GPT', 'info': info, 'partitions': partitions, 'ebr_chain': [], 'notes': notes}


def _scan_layout(ser, full):
    notes = []
    mbr = _read_lba(ser, 0)
    valid = mbr[510:512] == _MBR_SIGNATURE
    entries = _parse_mbr_entries(mbr) if valid else []
    protective = any(e['type'] == _MBR_PROTECTIVE_TYPE for e in entries)

    table = None
    hdr = _read_lba(ser, 1)
    if hdr[:8] == _GPT_SIGNATURE:
        table = _scan_gpt(ser, hdr, notes, full)
        if valid and not protective:
            notes.append("GPT header found at sector 1, but the MBR at sector 0 has no protective (0xEE) entry")
    elif protective:
        notes.append("Protective MBR found, but GPT header is invalid")
    if table is None and valid:
        table = _scan_mbr(ser, entries, notes)
    table = table or {'kind': None, 'info': [], 'partitions': [], 'ebr_chain': [], 'notes': notes}
    table['mbr'] = _mbr_header(mbr, entries) if valid else None
    return table


def _add_gaps(table, total_sectors):
    partitions = table['partitions']
    extra, cursor = [], 0
    for p in sorted((p for p in partitions if not p.get('container')), key=lambda p: p['start']):
        if p['start'] > cursor:
            extra.append(_special_entry(f"GAP{len(extra) + 1}", cursor, p['start'] - cursor))
        cursor = max(cursor, p['start'] + p['count'])
    if total_sectors is None:
        table['notes'].append("Disk size unknown, Remainder is not shown")
    elif cursor < total_sectors:
        extra.append(_special_entry("Remainder", cursor, total_sectors - cursor))
    partitions += extra
    partitions.sort(key=lambda p: (p['start'], -p['count']))


def _scan_partition_table(ser, full=False):
    table = _scan_layout(ser, full)
    if table['kind'] is not None:
        _add_gaps(table, _get_total_sectors(ser))
    return table

def _print_table(cols, rows):
    fmt = lambda values: "  ".join(f"{v!s:{align}{width}}" for v, (_, width, align) in zip(values, cols)).rstrip()
    head = "  ".join(f"{name:{align}{width}}" for name, width, align in cols)
    print(head.rstrip())
    print("-" * len(head))
    for row in rows:
        print(fmt(row))


def _table(title, cols, rows):
    def show():
        if title:
            print(title)
        _print_table(cols, rows)
    return show


def _info(title, info):
    def show():
        print(title)
        for label, value in info:
            print(f"{label:<14} : {value}")
    return show


def _hexdump(title, data):
    def show():
        print(title)
        for i, line in enumerate(_hex_ascii_lines(data)):
            print(f"  {i * 16:04X}  {line}")
    return show


def _partition_end(part):
    return part['start'] + part['count'] - 1 if part['count'] else part['start']


_NUM = ("#", 3, '>')
_EBR = ("EBR LBA", 10, '>')
_CHS = [("Boot", 4, '>'), ("Start CHS", 11, '>'), ("Type", 4, '>'), ("End CHS", 11, '>')]


def _chs_values(e):
    return f"0x{e['boot']:02X}", _chs(e['chs_start']), f"0x{e['type']:02X}", _chs(e['chs_end'])


def _names_width(partitions):
    return max([4] + [len(p['name']) for p in partitions])


def _partitions_section(table, full):
    parts, gpt = table['partitions'], table['kind'] == 'GPT'
    cols = [_NUM, ("Name", _names_width(parts), '<')]
    cols += [("Boot", 4, '^')] * (full and not gpt)
    cols += [("Start LBA", 12, '>'), ("End LBA", 12, '>'), ("Sectors", 12, '>'), ("Size", 10, '>')]
    cols += [("Attributes", 18, '<')] * (full and gpt) + [("Type", 4, '<')] * (not gpt)
    rows = []
    for p in parts:
        row = ['-' if p['number'] is None else p['number'], p['name']]
        row += [p.get('flag', '')] * (full and not gpt)
        row += [p['start'], _partition_end(p), p['count'], _format_size(p['count'] * _SECTOR_SIZE)]
        row += [f"0x{p['attrs']:016X}" if p.get('attrs') is not None else ''] * (full and gpt) + [p['type']] * (not gpt)
        rows.append(row)
    return _table("Partitions:" if full else None, cols, rows)


def _mbr_sections(mbr):
    sections = [_info("MBR header:", mbr['info'])]
    if mbr['bootstrap']:
        sections.append(_hexdump("Bootstrap code:", mbr['bootstrap']))
    rows = [(i, *_chs_values(e), e['start'], e['count']) for i, e in enumerate(mbr['entries'], 1)]
    return sections + [_table("MBR entries:", [_NUM, *_CHS, ("Start LBA", 12, '>'), ("Sectors", 12, '>')], rows)]


def _ebr_sections(chain):
    numbered = list(enumerate(chain, 1))
    empty34 = lambda it: all(map(_is_empty_mbr_entry, it['entries'][2:]))
    dash = lambda v: '-' if v is None else v
    sections = [
        _table("EBR sectors:", [_NUM, _EBR, ("Signature", 9, '>'), ("Bytes 0-445", 15, '>'), ("Entries 3-4", 11, '>')],
               [(i, it['lba'], it['signature'], f"{it['unused']} non-zero" if it['unused'] else "empty",
                 "empty" if empty34(it) else "NOT empty") for i, it in numbered]),
        _table("EBR entry 1 (logical partition):",
               [_NUM, _EBR, ("Logical", 7, '>'), *_CHS, ("Rel Start", 10, '>'), ("Start LBA", 10, '>'), ("Sectors", 10, '>')],
               [(i, it['lba'], dash(it['logical']), *_chs_values(e), e['start'],
                 '-' if _is_empty_mbr_entry(e) else it['lba'] + e['start'], e['count'])
                for i, it in numbered for e in it['entries'][:1]]),
        _table("EBR entry 2 (link to the next EBR):",
               [_NUM, _EBR, *_CHS, ("Rel Start", 10, '>'), ("Next EBR", 10, '>'), ("Sectors", 10, '>')],
               [(i, it['lba'], *_chs_values(e), e['start'], dash(it['next']), e['count'])
                for i, it in numbered for e in it['entries'][1:2]]),
    ]
    extra = [(i, it['lba'], n, *_chs_values(e), e['start'], e['count'])
             for i, it in numbered for n, e in enumerate(it['entries'][2:], 3) if not _is_empty_mbr_entry(e)]
    if extra:
        sections.append(_table("EBR entries 3-4 (must be empty):",
                               [_NUM, _EBR, ("Entry", 5, '>'), *_CHS, ("Start LBA", 10, '>'), ("Sectors", 10, '>')], extra))
    return sections


def _gpt_sections(partitions):
    real = [p for p in partitions if p['number'] is not None]
    if not real:
        return []
    name = ("Name", _names_width(real), '<')
    attrs = [(p['number'], p['name'], a & 1, a >> 1 & 1, a >> 2 & 1,
              f"0x{a & _ATTR_RESERVED:012X}" if a & _ATTR_RESERVED else '-', f"0x{a >> 48:04X}" if a >> 48 else '-')
             for p in real for a in [p['attrs']]]
    return [
        _table("Partition attributes:",
               [_NUM, name, ("Required", 8, '>'), ("No BlockIO", 10, '>'), ("Legacy Boot", 11, '>'),
                ("Bits 3-47", 14, '>'), ("Bits 48-63", 10, '>')], attrs),
        _table("Partition GUIDs:", [_NUM, name, ("Type GUID", 36, '<'), ("Unique GUID", 36, '<')],
               [(p['number'], p['name'], p['type'], p['unique']) for p in real]),
    ]


def _print_partition_table(table, full=False):
    if table['kind'] is None:
        print("Device   > Reading Partition table failed (MBR/EBR and MBR/GPT not found)")
    else:
        print(f"Partition table: {table['kind']}\n")
        sections = [_partitions_section(table, False)]
        if full:
            sections = (_mbr_sections(table['mbr']) if table['mbr'] else []) + \
                       ([_info("GPT header:", table['info'])] if table['kind'] == 'GPT' else []) + \
                       (_ebr_sections(table['ebr_chain']) if table['ebr_chain'] else []) + \
                       [_partitions_section(table, True)] + \
                       (_gpt_sections(table['partitions']) if table['kind'] == 'GPT' else [])
        for i, show in enumerate(sections):
            if i:
                print()
            show()
        print()
    for note in table['notes']:
        print(f"Device   > {note}")

def _partition_label(part):
    name = f"'{part['name']}'" if part['name'] else ""
    return name if part['number'] is None else f"{part['number']} {name}".strip()


def _find_partition(table, ident):
    partitions = table['partitions']
    if ident.isdecimal():
        for p in partitions:
            if p['number'] == int(ident):
                return p

    named = [p for p in partitions if p['name']]
    matches = [p for p in named if p['name'] == ident] or [p for p in named if p['name'].lower() == ident.lower()]
    if len(matches) == 1:
        return matches[0]
    if matches:
        where = ", ".join(f"#{p['number']}" if p['number'] is not None else f"LBA {p['start']}" for p in matches)
        print(f"Device   > Partition name '{ident}' is ambiguous ({where}), use its number")
        return None

    hint = ("see readpartitiontable; regular MBR partitions can only be selected by number"
            if table['kind'] == 'MBR' and not ident.isdecimal() else "see readpartitiontable")
    print(f"Device   > Partition '{ident}' not found ({hint})")
    return None


def _resolve_partition(ser, ident):
    try:
        table = _scan_partition_table(ser)
    except _SectorReadError as e:
        print(f"Device   > Read Error at sector: {e.lba}")
        return None
    if table['kind'] is None:
        _print_partition_table(table)
        return None

    part = _find_partition(table, ident)
    if part is None:
        return None
    label = _partition_label(part)
    if part['count'] == 0:
        print(f"Device   > Partition {label} is empty (0 sectors)")
        return None
    print(
        f"Device   > Partition {label}: sectors {part['start']} - {_partition_end(part)} "
        f"({part['count']} sectors, {_format_size(part['count'] * _SECTOR_SIZE)})"
    )
    return part


def _resolve_range(ser, args):
    if 'partition' not in args:
        return args.get('start'), args.get('length')
    part = _resolve_partition(ser, args['partition'])
    if part is None:
        _close_flush_check(ser)
        return None
    length = part['count']
    if 'partition_length' in args:
        length = min(length, args['partition_length'])
    return part['start'], length


_RANGE = [('start', '-s', int), ('length', '-l', int)]


def _begin(ser, args):
    if not do_presequence(ser):
        return None
    return _resolve_range(ser, args)


def _run_stream(ser, start, length, args, label, done_msg, per_sector):
    if length == 0:
        print(f"Device   > {label}: nothing to do (0 sectors)")
        _close_flush_check(ser)
        return True
    success, nl_needed, t0 = _stream_loop(ser, start, length, args['newline'], label, per_sector)
    _finish(success, length, nl_needed, t0, done_msg)
    _close_flush_check(ser)
    return success


def _cmd_read(ser, user_input):
    args = _parse_args(
        user_input, _RANGE + [('file', '-f', str)],
        "read (-s <start_sector> -l <length> | -p <partition>) -f <file> -n (newline)",
        posix=False, allow_partition=True,
    )
    ctx = args and _begin(ser, args)
    if not ctx:
        return
    path = args['file']
    try:
        f = open(path, 'wb')
    except OSError as e:
        print(f"Device   > Cannot open file: {e}")
        _close_flush_check(ser)
        return

    try:
        def per_sector(ser, offset):
            ok, data, resp = _read_sector(ser, offset)
            if ok:
                f.write(data)
            return (True, 1, "", None) if ok else (False, 0, "Read Error", resp)

        success = _run_stream(ser, *ctx, args, "Reading", f"Reading Done Saved to {path}", per_sector)
    finally:
        f.close()

def _cmd_readhex(ser, user_input):
    args = _parse_args(
        user_input, _RANGE,
        "readhex (-s <start_sector> -l <length> | -p <partition> [-l <length>])",
        allow_partition=True, allow_partition_length=True,
    )
    ctx = args and _begin(ser, args)
    if not ctx:
        return
    start, length = ctx

    offset = start
    for i in range(length):
        ok, data, resp = _read_sector(ser, offset)
        if not ok:
            print(f"Device   > Read Error at sector: {offset}")
            if resp:
                print_response(resp)
            break

        print(f"Device   > Sector {offset} ({len(data)} bytes)")
        for line in _hex_ascii_lines(data):
            print(line)
        print("-" * 53)

        offset += 1
    else:
        print(f"Device   > Reading Done ({length} sectors)")

    _close_flush_check(ser)

def _cmd_write(ser, user_input):
    args = _parse_args(
        user_input, [('start', '-s', int), ('file', '-f', str)],
        "write (-s <start_sector> | -p <partition>) -f <file> -n (newline)",
        posix=False, allow_partition=True,
    )
    if args is None:
        return
    try:
        f = open(args['file'], 'rb')
    except FileNotFoundError:
        print(f"Device   > File not found: '{args['file']}'")
        return
    except OSError as e:
        print(f"Device   > Cannot open file: {e}")
        return

    try:
        file_size = os.fstat(f.fileno()).st_size
        total_sectors = -(-file_size // _SECTOR_SIZE)
        ctx = _begin(ser, args)
        if not ctx:
            return
        start, part_sectors = ctx
        if 'partition' in args and total_sectors > part_sectors:
            print(f"Device   > File is larger than the partition ({total_sectors} > {part_sectors} sectors), nothing written")
            _close_flush_check(ser)
            return

        def per_sector(ser, offset):
            ok, resp = _write_sector(ser, offset, f.read(_SECTOR_SIZE))
            return (True, 1, "", None) if ok else (False, 0, "Write failed", resp)

        _run_stream(ser, start, total_sectors, args, "Writing", "Writing done", per_sector)
    finally:
        f.close()


def _cmd_erase(ser, user_input):
    args = _parse_args(
        user_input, _RANGE, "erase (-s <start_sector> -l <length> | -p <partition>) -n (newline)", allow_partition=True,
    )
    ctx = args and _begin(ser, args)
    if not ctx:
        return
    zero_sector = bytes(_SECTOR_SIZE)

    def per_sector(ser, offset):
        ok, resp = _write_sector(ser, offset, zero_sector)
        return (True, 1, "", None) if ok else (False, 0, "Erasing failed", resp)

    _run_stream(ser, *ctx, args, "Erasing", "Erasing done", per_sector)


_ERASEALL_TIMEOUT = 600.0


def _cmd_eraseall(ser, user_input):
    if not do_presequence(ser):
        return
    check_response(send_sync(ser, bytearray([0x1D]), timeout=_ERASEALL_TIMEOUT, skip_info=True), 0x1E, "Erase all")
    _close_flush_check(ser)


def _write_verify(ser, offset, pattern):
    ok, resp = _write_sector(ser, offset, pattern)
    if not ok:
        return "Write failed", resp
    ok, data, resp = _read_sector(ser, offset)
    if not ok:
        return "Read failed", resp
    return None if data == pattern else ("EMMC verification failed", _NO_DETAILS)


def _restore_sector(ser, offset, original):
    ok, _ = _write_sector(ser, offset, original)
    if not ok:
        return False
    ok, data, _ = _read_sector(ser, offset)
    return bool(ok and data == original)


def _save_original_sector(sector, original):
    path = f"safestoragetest_sector_{sector}.bin"
    try:
        with open(path, 'wb') as f:
            f.write(original)
        return path
    except OSError:
        return "(could not save the file)"


def _cmd_storagetest(ser, user_input):
    args = _parse_args(
        user_input, _RANGE, "storagetest (-s <start_sector> -l <length> | -p <partition>) -n (newline)",
        allow_partition=True,
    )
    ctx = args and _begin(ser, args)
    if not ctx:
        return

    def per_sector(ser, offset):
        error = _write_verify(ser, offset, os.urandom(_SECTOR_SIZE))
        return (False, 0, *error) if error else (True, 1, "", None)

    _run_stream(ser, *ctx, args, "Write & Verify", "Write & Verify done", per_sector)


def _cmd_safestoragetest(ser, user_input):
    args = _parse_args(
        user_input, _RANGE, "safestoragetest (-s <start_sector> -l <length> | -p <partition>) -n (newline)",
        allow_partition=True,
    )
    ctx = args and _begin(ser, args)
    if not ctx:
        return

    def per_sector(ser, offset):
        ok, original, resp = _read_sector(ser, offset)
        if not ok:
            return False, 0, "Read failed", resp
        original = bytes(original)
        try:
            error = _write_verify(ser, offset, os.urandom(_SECTOR_SIZE))
        except BaseException:
            _restore_sector(ser, offset, original)
            raise
        if not _restore_sector(ser, offset, original):
            saved = _save_original_sector(offset, original)
            return False, 0, f"RESTORE FAILED, original sector saved to {saved}", _NO_DETAILS
        return (False, 0, f"{error[0]}, original data restored", error[1]) if error else (True, 1, "", None)

    _run_stream(ser, *ctx, args, "Safe Write & Verify", "Safe Write & Verify done, original data restored", per_sector)


def _read_partition_table(ser, full):
    if not do_presequence(ser):
        return
    try:
        table = _scan_partition_table(ser, full=full)
        if table['kind'] is not None:
            print("Device   > Reading Partition table OK\n")
        _print_partition_table(table, full=full)
    except _SectorReadError as e:
        print(f"Device   > Read Error at sector: {e.lba}")
    _close_flush_check(ser)


def _cmd_readpartitiontable(ser, user_input):
    _read_partition_table(ser, full=False)


def _cmd_readfullpartitiontable(ser, user_input):
    _read_partition_table(ser, full=True)


EXACT_COMMANDS = {
    'nop':            _cmd_nop,
    'hello':          _cmd_hello,
    'bypass':         _cmd_bypass,
    'close':          _cmd_close,
    'reboot':         _cmd_reboot,
    'initstorage':    _cmd_initstorage,
    'getstorageinfo': _cmd_getstorageinfo,
    'readpartitiontable': _cmd_readpartitiontable,
    'readfullpartitiontable': _cmd_readfullpartitiontable,
    'eraseall':       _cmd_eraseall,
}

PREFIX_COMMANDS = [
    ('readhex',     _cmd_readhex),
    ('read',        _cmd_read),
    ('write',       _cmd_write),
    ('erase',       _cmd_erase),
    ('storagetest', _cmd_storagetest),
    ('safestoragetest', _cmd_safestoragetest),
]


def _resolve_handler(cmd_lower):
    words = cmd_lower.split(None, 1)
    cmd_word = words[0] if words else ''
    handler = EXACT_COMMANDS.get(cmd_word)
    if handler:
        return handler
    for prefix, fn in PREFIX_COMMANDS:
        if cmd_word == prefix:
            return fn
    return None


def _handle_load(port, ser, user_input):
    try:
        parts = shlex.split(user_input, posix=False)
    except ValueError as e:
        print(f"Device   > Error: {e}")
        return ser
    try:
        hex_path = parts[parts.index('-f') + 1].strip('"\'')
    except (ValueError, IndexError):
        print("Device   > Usage: load -f <path> -n (newline)")
        return ser
    bNewline = '-n' in parts
    ser.close()

    fmt, err = _detect_loader_format(hex_path)
    if fmt is None:
        print(f"Device   > {err}")
    else:
        load_programmer(port, hex_path, fmt, bNewline)

    return ser


def _handle_raw(ser, user_input):
    hex_str = user_input[4:].strip().replace(" ", "")
    try:
        cmd_body = bytearray(bytes.fromhex(hex_str))
    except ValueError:
        print("Device   > Error: Invalid HEX string after 'raw'")
        return
    resp = send_sync(ser, cmd_body)
    if resp:
        print_response(resp)
    else:
        print("Device   > Device Timeout (No Response)")


def _run_command(ser, handler, user_input):
    try:
        handler(ser, user_input)
    except Exception as e:
        print(f"Device   > Unexpected error: {e}")
    finally:
        ser.close()


def _run_load(port, ser, user_input):
    try:
        return _handle_load(port, ser, user_input)
    except Exception as e:
        print(f"Device   > Unexpected error: {e}")
        return ser
    finally:
        ser.close()


def terminal(port):
    print(HELP_TEXT)
    ser = LazySerial(port)

    try:
        while True:
            try:
                user_input = input("\nTerminal > ").strip()
                print()
            except EOFError:
                print()
                break
            except (serial.SerialException, OSError):
                continue

            if not user_input:
                continue

            cmd_lower = user_input.lower()

            if cmd_lower == 'exit':
                break
            if cmd_lower == 'help':
                print(HELP_TEXT)
                continue

            try:
                words = user_input.split(None, 1)
                if words and words[0].lower() == 'load':
                    ser = _run_load(port, ser, user_input)
                    continue
                if cmd_lower.startswith('raw '):
                    _run_command(ser, _handle_raw, user_input)
                    continue

                handler = _resolve_handler(cmd_lower)
                if handler:
                    _run_command(ser, handler, user_input)
                else:
                    print("Device   > Unknown command. Type 'help' or use 'raw <hex>'")
            except KeyboardInterrupt:
                print("\nDevice   > Interrupted")
                ser.close()

    finally:
        ser.close()


if __name__ == "__main__":
    port = input("Enter Device Serial Port (e.g. COM1 or /dev/ttyUSB0): ").strip()
    print()
    terminal(port)