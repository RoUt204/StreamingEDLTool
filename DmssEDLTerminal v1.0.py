import serial
import struct
import time

def format_time(seconds):
    seconds = int(seconds)
    d = seconds // 86400
    h = (seconds % 86400) // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    if d > 0:
        return f"{d}d {h:02}h {m:02}m {s:02}s"
    elif h > 0:
        return f"{h}h {m:02}m {s:02}s"
    elif m > 0:
        return f"{m}m {s:02}s"
    else:
        return f"{s}s"

CMD_WRITE_32BIT = 0x0F
CMD_GO          = 0x05
CMD_ACK         = 0x02
PACKET_TIMEOUT  = 60.0

RESPONSE_DESCRIPTIONS = {
    0x02: "Response to hello",
    0x04: "Response to successful RAM/EMMC read",
    0x06: "Response to successful write without erasing",
    0x08: "Response to successful stream write",
    0x0A: "Response when phone is idle (NOP)",
    0x0C: "Response to successful phone reboot",
    0x0D: "Error",
    0x0E: "Information",
    0x10: "Response to successful phone unlock",
    0x12: "Response to successful power off",
    0x14: "Response to successful image write open",
    0x16: "Response to successful close and flush of last partial flash write",
    0x18: "Response to successful protection bypass",
    0x1A: "Partition table successfully accepted",
    0x1C: "Response to successful flash chip open",
    0x1E: "Response to successful flash erase (CMD38)",
    0x20: "ECC generation status info",
    0x22: "Successful ECC generation enable/disable",
    0x24: "Response to successful flash info read",
    0x31: "Successful memory write via new method",
}

HELP_TEXT = """DMSS EDL Terminal v1.0 Help:

  exit                                          - Quit
  help                                          - Show this help
  load -f <file>                                - Load a HEX programmer file
  raw <bytes in HEX>                            - Send RAW hex bytes to device
  nop                                           - Ask what device is doing
  reboot                                        - Reboot phone
  initstorage                                   - Open eMMC card
  getstorageinfo                                - Get eMMC information
  read  -s <start_sector> -l <length> -f <file> - Read from RAM/eMMC
  write -s <start_sector> -f <file>             - Write data to eMMC
  erase -s <start_sector> -l <length>           - Write 00 to eMMC
  eraseall                                      - Erase all eMMC (CMD38)"""


def calc_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc ^ 0xFFFF

def hdlc_encode(payload):
    crc = calc_crc16(payload)
    data_with_crc = payload + struct.pack('<H', crc)
    encoded = bytearray([0x7E])
    for byte in data_with_crc:
        if byte == 0x7E:
            encoded.extend([0x7D, 0x5E])
        elif byte == 0x7D:
            encoded.extend([0x7D, 0x5D])
        else:
            encoded.append(byte)
    encoded.append(0x7E)
    return encoded

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
    return decoded[:-2] if len(decoded) > 2 else decoded

def send_sync(port, payload, timeout=PACKET_TIMEOUT):
    try:
        port.reset_input_buffer()
        port.write(hdlc_encode(payload))
        resp_packet = bytearray()
        started     = False
        start_time  = time.time()
        while (time.time() - start_time) < timeout:
            if not port.is_open:
                return None
            b = port.read(1)
            if not b:
                continue
            if b[0] == 0x7E:
                if not started:
                    started = True
                elif len(resp_packet) > 0:
                    return hdlc_decode(resp_packet)
            elif started:
                resp_packet.append(b[0])
    except (serial.SerialException, OSError, PermissionError) as e:
        return None
    return None

def recv_next_packet(port, timeout=PACKET_TIMEOUT):
    resp_packet = bytearray()
    started     = False
    start_time  = time.time()
    while (time.time() - start_time) < timeout:
        b = port.read(1)
        if not b:
            continue
        if b[0] == 0x7E:
            if not started:
                started = True
            elif len(resp_packet) > 0:
                return hdlc_decode(resp_packet)
        elif started:
            resp_packet.append(b[0])
    return None

def send_sync_data(port, payload, expect, timeout=PACKET_TIMEOUT):
    try:
        port.reset_input_buffer()
        port.write(hdlc_encode(payload))
        deadline    = time.time() + timeout
        resp_packet = bytearray()
        started     = False
        while time.time() < deadline:
            b = port.read(1)
            if not b:
                continue
            if b[0] == 0x7E:
                if not started:
                    started = True
                elif len(resp_packet) > 0:
                    pkt = hdlc_decode(resp_packet)
                    resp_packet = bytearray()
                    started     = False
                    if len(pkt) == 0:
                        continue
                    if pkt[0] == 0x0E:
                        continue
                    return pkt
            elif started:
                resp_packet.append(b[0])
    except (serial.SerialException, OSError, PermissionError) as e:
        return None
    return None

def print_response(resp):
    bytes_per_line = 16
    code   = resp[0] if resp else None
    desc   = RESPONSE_DESCRIPTIONS.get(code, "Unknown response code") if code is not None else ""
    header = f"0x{code:02X} - {desc}" if code is not None else "empty"
    print(f"Device   > Response ({len(resp)} bytes)  [{header}]")
    for i in range(0, len(resp), bytes_per_line):
        chunk    = resp[i:i + bytes_per_line]
        hex_part = " ".join([f"{b:02X}" for b in chunk])
        hex_part = hex_part.ljust(bytes_per_line * 3 - 1)
        txt_part = "".join([chr(b) if 32 <= b <= 126 else "." for b in chunk])
        print(f"{hex_part}  |  {txt_part}")
    print("-" * (bytes_per_line * 3 + bytes_per_line + 5))

def print_status(ok, msg_ok, msg_fail):
    if ok:
        print(f"Device   > {msg_ok}")
    else:
        print(f"Device   > {msg_fail}")

def parse_storage_info(resp):
    if not resp or len(resp) < 57 or resp[0] != 0x24:
        print("Device   > Failed to parse storage info (invalid response)")
        return
    emmc_type       = struct.unpack_from('<I', resp, 1)[0]
    block_count     = struct.unpack_from('<I', resp, 5)[0]
    page_size       = struct.unpack_from('<I', resp, 13)[0]
    pages_per_block = struct.unpack_from('<I', resp, 17)[0]
    superblock_size = page_size * pages_per_block
    vendor_id       = struct.unpack_from('<I', resp, 39)[0]
    name            = resp[43:49].decode('ascii', errors='replace').strip().rstrip('\x00')
    serial_num      = struct.unpack_from('<I', resp, 53)[0]
    total_bytes = block_count * page_size
    total_kb    = total_bytes / 1024
    total_mb    = total_kb / 1024
    total_gb    = total_mb / 1024
    print("eMMC Information:")
    print(f"eMMC Type                 : {emmc_type:08X}")
    print(f"eMMC Userarea size        : {block_count} sectors  ({total_kb:.0f} KB / {total_mb:.0f} MB / {total_gb:.2f} GB)")
    print(f"eMMC Superblock size      : {superblock_size}")
    print(f"eMMC Page size            : {page_size}")
    print(f"eMMC Vendor id            : {vendor_id:08X}")
    print(f"eMMC Name                 : {name}")
    print(f"eMMC Serial               : {serial_num:08X}")

def make_hello():
    ver   = 3
    magic = b'QCOM fast download protocol host'
    return bytearray([0x01]) + magic + bytes([ver | 0x10, 2, 1])

def do_hello(ser, silent=False):
    resp = send_sync(ser, make_hello())
    if not resp:
        print("Device   > Timeout (No Response to Hello)")
        return False, None
    if not silent:
        print_status(resp[0] == 0x02, "Hello OK", f"Hello failed (0x{resp[0]:02X})")
        if resp[0] != 0x02:
            print_response(resp)
    bSectorAddress = False
    try:
        if resp[0] == 0x02 and len(resp) > 44:
            flash_id_len = resp[43]
            base = 44 + flash_id_len
            if len(resp) > base + 3:
                num_sectors = struct.unpack_from('<H', resp, base + 2)[0]
                feature_offset = base + 4 + num_sectors * 4
                if feature_offset < len(resp):
                    bSectorAddress = bool(resp[feature_offset] & 0x10)
    except (IndexError, struct.error):
        bSectorAddress = False
    return bSectorAddress, resp

def do_bypass(ser, silent=False):
    resp = send_sync(ser, bytearray([0x17]))
    if not resp:
        print("Device   > Timeout (No Response to Bypass)")
        return None
    if not silent:
        print_status(resp[0] == 0x18, "Bypass OK", f"Bypass failed (0x{resp[0]:02X})")
        if resp[0] != 0x18:
            print_response(resp)
    return resp

def do_open_flash(ser, silent=False):
    resp = send_sync(ser, bytearray([0x1B, 0x21]))
    if not resp:
        print("Device   > Timeout (No Response to Open Flash)")
        return None
    if not silent:
        print_status(resp[0] == 0x1C, "EMMC initilization OK", f"EMMC initilization failed (0x{resp[0]:02X})")
        if resp[0] != 0x1C:
            print_response(resp)
    return resp

def do_close_flush(ser, silent=False):
    resp = send_sync(ser, bytearray([0x15]))
    if not resp:
        print("Device   > Timeout (No Response to Close)")
        return None
    if not silent:
        print_status(resp[0] == 0x16, "Close OK", f"Close failed (0x{resp[0]:02X})")
    return resp

def do_presequence(ser, silent=False):
    bSectorAddress, resp = do_hello(ser, silent)
    if resp is None:
        return False, False
    if do_bypass(ser, silent) is None:
        return bSectorAddress, False
    if do_open_flash(ser, silent) is None:
        return bSectorAddress, False
    return bSectorAddress, True

def send_write_chunk(port, target_addr, data, newline_needed):
    payload  = bytearray([CMD_WRITE_32BIT])
    payload += struct.pack('>I', target_addr)
    payload += struct.pack('>H', len(data))
    payload += data
    rsp = send_sync(port, payload)
    if rsp and len(rsp) > 0 and rsp[0] == CMD_ACK:
        print(f"Device   > Loading at address: 0x{target_addr:08X}", end='\r', flush=True)
        newline_needed[0] = True
        return True
    else:
        prefix = "\n" if newline_needed[0] else ""
        print(f"{prefix}Device   > Error during loading at 0x{target_addr:08X}")
        if rsp:
            print_response(rsp)
        else:
            print(f"{prefix}Device   > Timeout (No Response)")
        return False

def load_flash_prog(com_port, hex_filename):
    try:
        port = serial.Serial(com_port, 115200, timeout=5.0)
        port.reset_input_buffer()
    except Exception as e:
        print(f"Device   > Timeout (No Response)")
        return

    current_high_addr = 0
    go_addr           = 0
    newline_needed    = [False]

    print(f"Device   > Loading file {hex_filename}...")
    try:
        hex_file = open(hex_filename, 'r')
    except FileNotFoundError:
        print(f"Device   > File not found: '{hex_filename}'")
        port.close()
        return
    except Exception as e:
        print(f"Device   > Cannot open file: {e}")
        port.close()
        return

    with hex_file as f:
        write_buf   = bytearray()
        target_addr = 0
        bFirst      = True

        for line in f:
            line = line.strip()
            if not line.startswith(':') or len(line) < 11:
                continue
            length  = int(line[1:3],  16)
            addr    = int(line[3:7],  16)
            rectype = int(line[7:9],  16)

            if rectype == 0:
                if bFirst:
                    target_addr = current_high_addr + addr
                    bFirst      = False
                write_buf.extend(bytes.fromhex(line[9:9 + (length * 2)]))
                while len(write_buf) >= 256:
                    if not send_write_chunk(port, target_addr, write_buf[:256], newline_needed):
                        port.close()
                        return
                    target_addr += 256
                    write_buf    = write_buf[256:]

            elif rectype == 1:
                break

            elif rectype == 4:
                if write_buf:
                    if not send_write_chunk(port, target_addr, write_buf, newline_needed):
                        port.close()
                        return
                    write_buf = bytearray()
                current_high_addr = int(line[9:13], 16) << 16
                bFirst = True

            elif rectype == 5:
                go_addr = int(line[9:17], 16)

        if write_buf:
            send_write_chunk(port, target_addr, write_buf, newline_needed)

    prefix = "\n" if newline_needed[0] else ""
    print(f"{prefix}Device   > Executing GO at 0x{go_addr:08X}")
    go_payload = bytearray([CMD_GO]) + struct.pack('>I', go_addr)
    port.reset_input_buffer()
    final_rsp = send_sync(port, go_payload)
    time.sleep(5)

    if final_rsp and len(final_rsp) > 0 and final_rsp[0] == CMD_ACK:
        pass
    else:
        print(f"Device   > GO failed at 0x{go_addr:08X}")
        if final_rsp:
            print_response(final_rsp)
        else:
            print("Device   > Device Timeout (No Response)")
    port.close()

def terminal(port):
    print(HELP_TEXT)

    try:
        ser = serial.Serial(port, 115200, timeout=5.0)

        while True:
            try:
                user_input = input("\nTerminal > ").strip()
                print()
            except (serial.SerialException, OSError):
                continue

            if user_input.lower() == 'exit':
                break
            if not user_input:
                continue

            if user_input.lower() == 'help':
                print(HELP_TEXT)
                continue

            if user_input.lower().startswith('load'):
                import shlex
                parts = shlex.split(user_input)
                try:
                    f_idx = parts.index('-f') + 1
                    hex_path = parts[f_idx]
                except (ValueError, IndexError):
                    print("Usage: load -f <path>")
                    continue
                ser.close()
                load_flash_prog(port, hex_path)
                try:
                    ser = serial.Serial(port, 115200, timeout=5.0)
                except Exception as e:
                    pass
                continue

            if user_input.lower().startswith('raw '):
                hex_str = user_input[4:].strip().replace(" ", "")
                try:
                    cmd_body = bytearray(bytes.fromhex(hex_str))
                except ValueError:
                    print("Error: Invalid HEX string after 'raw'")
                    continue
                if cmd_body == bytearray([0x01]):
                    cmd_body = make_hello()
                resp = send_sync(ser, cmd_body)
                if resp:
                    print_response(resp)
                else:
                    print("Device   > Device Timeout (No Response)")
                continue

            cmd_lower = user_input.lower()

            if cmd_lower == 'nop':
                resp = send_sync(ser, bytearray([0x09]))
                if resp:
                    print_response(resp)
                else:
                    print("Device   > Device Timeout (No Response)")
                continue

            elif cmd_lower == 'reboot':
                bSectorAddress, ok = do_hello(ser)
                if not ok:
                    continue
                if do_bypass(ser) is None:
                    continue
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                resp = send_sync(ser, bytearray([0x0B]))
                if resp:
                    print_status(resp[0] == 0x0C, "Reboot OK", f"Reboot failed (0x{resp[0]:02X})")
                    if resp[0] != 0x0C:
                        print_response(resp)
                else:
                    print("Device   > Timeout (No Response to Reboot)")
                continue

            elif cmd_lower == 'initstorage':
                _, ok = do_presequence(ser)
                if not ok:
                    continue
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            elif cmd_lower == 'getstorageinfo':
                _, ok = do_presequence(ser)
                if not ok:
                    continue
                resp = send_sync(ser, bytearray([0x23]))
                if resp and resp[0] == 0x24:
                    parse_storage_info(resp)
                    print("Device   > Reading eMMC info OK")
                elif resp:
                    print(f"Device   > Reading eMMC Info failed (0x{resp[0]:02X})")
                    print_response(resp)
                else:
                    print("Device   > Timeout (No Response to Storage Info)")
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            elif cmd_lower.startswith('read'):
                import shlex
                parts = shlex.split(user_input)
                try:
                    s_idx         = parts.index('-s') + 1
                    l_idx         = parts.index('-l') + 1
                    o_idx         = parts.index('-f') + 1
                    start_val     = int(parts[s_idx])
                    total_sectors = int(parts[l_idx])
                    outfile       = parts[o_idx]
                except (ValueError, IndexError):
                    print("Usage: read -s <start_sector> -l <length> -f <file>")
                    continue
                bSectorAddress, ok = do_presequence(ser)
                if not ok:
                    continue
                offset        = start_val if bSectorAddress else start_val * 512
                total_bytes   = total_sectors * 512
                pos           = 0
                start_time    = time.time()
                bar_length    = 20
                success       = True
                newline_needed = False
                with open(outfile, 'wb') as f:
                    while pos < total_bytes:
                        cmd_body = bytearray([0x03]) + struct.pack('<I', offset) + struct.pack('>H', 2)
                        resp = send_sync(ser, cmd_body)
                        if resp and len(resp) >= 517 and resp[0] == 0x04:
                            data_chunk = resp[5:517]
                            f.write(data_chunk)
                            offset += 1 if bSectorAddress else 512
                            pos    += 512
                            percent  = min(pos / total_bytes, 1.0)
                            filled   = int(bar_length * percent)
                            bar      = '█' * filled + '-' * (bar_length - filled)
                            elapsed  = time.time() - start_time
                            speed    = (pos / 1024) / elapsed if elapsed > 0 else 0
                            cur_sect = offset if bSectorAddress else offset // 512
                            eta      = (total_bytes - pos) / 1024 / speed if speed > 0 else 0
                            print(f"Device   > Reading: [{bar}] {percent*100:3.1f}% | Sect: {cur_sect} |  {pos/1024:7.1f} KB | {speed:6.1f} KB/s | ETA: {format_time(eta)}               ", end='\r')
                            newline_needed = True
                        else:
                            cur_sect = offset if bSectorAddress else offset // 512
                            prefix = "\n" if newline_needed else ""
                            print(f"{prefix}Device   > Read Error at sector: {cur_sect}")
                            if resp:
                                print_response(resp)
                            else:
                                print(f"Device   > Timeout (No Response)")
                            success = False
                            break
                prefix = "\n" if newline_needed else ""
                if success and total_sectors > 0:
                    elapsed = time.time() - start_time
                    print(f"{prefix}Device   > Reading Done Saved to {outfile} | Time: {format_time(elapsed)}")
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            elif cmd_lower.startswith('write'):
                import shlex
                parts = shlex.split(user_input)
                try:
                    s_idx    = parts.index('-s') + 1
                    p_idx    = parts.index('-f') + 1
                    start    = int(parts[s_idx])
                    filepath = parts[p_idx]
                except (ValueError, IndexError):
                    print("Usage: write -s <start_sector> -f <file>")
                    continue
                bSectorAddress, ok = do_presequence(ser)
                if not ok:
                    continue
                try:
                    with open(filepath, 'rb') as f:
                        data = f.read()
                except FileNotFoundError:
                    print(f"Device   > File not found: '{filepath}'")
                    continue
                except Exception as e:
                    print(f"Device   > Cannot open file: {e}")
                    continue
                total          = len(data)
                offset         = start if bSectorAddress else start * 512
                pos            = 0
                start_time     = time.time()
                bar_length     = 20
                success        = True
                newline_needed = False
                while pos < total:
                    chunk = bytearray(data[pos:pos + 512])
                    if len(chunk) % 512 != 0:
                        chunk += b'\x00' * (512 - (len(chunk) % 512))
                    cmd_body = bytearray([0x07]) + struct.pack('<I', offset) + chunk
                    resp     = send_sync_data(ser, cmd_body, expect=0x08)
                    if resp and len(resp) > 0 and resp[0] == 0x08:
                        inc      = len(chunk) // 512 if bSectorAddress else len(chunk)
                        offset  += inc
                        pos     += len(chunk)
                        percent  = min(pos / total, 1.0)
                        filled   = int(bar_length * percent)
                        bar      = '█' * filled + '-' * (bar_length - filled)
                        elapsed  = time.time() - start_time
                        speed    = (pos / 1024) / elapsed if elapsed > 0 else 0
                        cur_sect = offset if bSectorAddress else offset // 512
                        eta      = (total - pos) / 1024 / speed if speed > 0 else 0
                        print(f"Device   > Writing: [{bar}] {percent*100:3.1f}% | Sect: {cur_sect} |  {pos/1024:7.1f} KB | {speed:6.1f} KB/s | ETA: {format_time(eta)}               ", end='\r')
                        newline_needed = True
                    else:
                        cur_sect = offset if bSectorAddress else offset // 512
                        prefix = "\n" if newline_needed else ""
                        print(f"{prefix}Device   > Write failed at sector: {cur_sect}")
                        if resp:
                            print_response(resp)
                        else:
                            print(f"Device   > Timeout (No Response)")
                        success = False
                        break
                prefix = "\n" if newline_needed else ""
                if success and total > 0:
                    elapsed = time.time() - start_time
                    print(f"{prefix}Device   > Writing done | Time: {format_time(elapsed)}")
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            elif cmd_lower.startswith('erase') and cmd_lower != 'eraseall':
                parts = user_input.split()
                try:
                    s_idx  = parts.index('-s') + 1
                    l_idx  = parts.index('-l') + 1
                    start  = int(parts[s_idx])
                    length = int(parts[l_idx])
                except (ValueError, IndexError):
                    print("Usage: erase -s <start_sector> -l <length>")
                    continue
                bSectorAddress, ok = do_presequence(ser)
                if not ok:
                    continue
                offset         = start if bSectorAddress else start * 512
                remaining      = length
                start_time     = time.time()
                bar_length     = 20
                success        = True
                newline_needed = False
                while remaining > 0:
                    sectors_now = min(1, remaining)
                    chunk_bytes = sectors_now * 512
                    cmd_body    = bytearray([0x07]) + struct.pack('<I', offset) + bytes(chunk_bytes)
                    resp        = send_sync(ser, cmd_body)
                    if resp and len(resp) > 0 and resp[0] == 0x08:
                        offset    += sectors_now if bSectorAddress else chunk_bytes
                        remaining -= sectors_now
                        done       = length - remaining
                        percent    = done / length
                        filled     = int(bar_length * percent)
                        bar        = '█' * filled + '-' * (bar_length - filled)
                        elapsed    = time.time() - start_time
                        speed      = (done * 0.5) / elapsed if elapsed > 0 else 0
                        cur_sect   = offset if bSectorAddress else offset // 512
                        eta        = (remaining * 0.5) / speed if speed > 0 else 0
                        print(f"Device   > Erasing: [{bar}] {percent*100:3.1f}% | Sect: {cur_sect} |  {done*0.5:7.1f} KB | {speed:6.1f} KB/s | ETA: {format_time(eta)}               ", end='\r')
                        newline_needed = True
                    else:
                        cur_sect = offset if bSectorAddress else offset // 512
                        prefix = "\n" if newline_needed else ""
                        print(f"{prefix}Device   > Erasing failed at sector: {cur_sect}")
                        if resp:
                            print_response(resp)
                        else:
                            print(f"Device   > Timeout (No Response)")
                        success = False
                        break
                prefix = "\n" if newline_needed else ""
                if success and length > 0:
                    elapsed = time.time() - start_time
                    print(f"{prefix}Device   > Erasing done | Time: {format_time(elapsed)}")
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            elif cmd_lower == 'eraseall':
                _, ok = do_presequence(ser)
                if not ok:
                    continue
                resp = send_sync(ser, bytearray([0x1D]))
                if resp:
                    print_status(resp[0] == 0x1E, "Erase all OK", f"Erase all failed (0x{resp[0]:02X})")
                    if resp[0] != 0x1E:
                        print_response(resp)
                else:
                    print("Device   > Timeout (No Response to Erase All)")
                close_resp = do_close_flush(ser)
                if close_resp and close_resp[0] != 0x16:
                    print_response(close_resp)
                continue

            else:
                print("Unknown command. Type 'help' or use 'raw <hex>'")
                continue

        ser.close()
    except Exception as e:
        print(f"Serial Port Error: {e}")

if __name__ == "__main__":
    port = input("Enter Device Serial Port: ").strip().upper()
    print()
    terminal(port)