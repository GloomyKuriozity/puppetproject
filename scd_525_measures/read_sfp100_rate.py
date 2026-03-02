#!/usr/bin/env python3
import argparse
import binascii
import serial
import struct
import sys
import time


# ---------------------------
# CRC16 (MODBUS-RTU) over ASCII characters
# Count from first char of recipient to last char of transaction message.
# ---------------------------
def crc16_modbus_ascii(ascii_bytes: bytes) -> int:
    crc = 0xFFFF
    for b in ascii_bytes:
        crc ^= b
        for _ in range(8):
            lsb = crc & 0x0001
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc & 0xFFFF

def b2hex(b: int) -> str:
    return f"{b:02X}"

def word_le_hex(val: int) -> str:
    # 16-bit little-endian into two ASCII hex bytes (LSB first on the bus, but encoded as hex pairs)
    lo = val & 0xFF
    hi = (val >> 8) & 0xFF
    return f"{lo:02X}{hi:02X}"

def encode_transaction_bytes(tx_bytes: bytes) -> str:
    return binascii.hexlify(tx_bytes).upper().decode("ascii")

def decode_hex_bytes(ascii_hex: str) -> bytes:
    return binascii.unhexlify(ascii_hex)

# ---------------------------
# Build a single Read-Request transaction (one PDU)
# PDU header byte bits: [7..5]=001 (read), [4]=RO/RW selector, [3..0]=(N-1)
# ---------------------------
def build_read_transaction(trans_id: int, ro_space: bool, start_addr: int, reg_count: int) -> bytes:
    if not (1 <= reg_count <= 16):
        raise ValueError("reg_count must be 1..16 for a single PDU")
    pdu_header = ((0b001 & 0x7) << 5) | ((0 if ro_space else 1) << 4) | ((reg_count - 1) & 0x0F)
    # address little-endian
    addr_lo = start_addr & 0xFF
    addr_hi = (start_addr >> 8) & 0xFF
    return bytes([trans_id & 0xFF, pdu_header & 0xFF, addr_lo, addr_hi])

# ---------------------------
# Frame VFBUS SL packet: '#' + recipient(2 hex) + sender(2 hex) + TX(hex) + CRC(4 hex) + '\r'
# CRC is over the ASCII bytes from recipient through end of transaction message (hex chars).
# ---------------------------
def frame_vfbus_sl(recipient: int, sender: int, tx_message: bytes) -> bytes:
    rec_hex = b2hex(recipient)
    snd_hex = b2hex(sender)
    tx_hex = encode_transaction_bytes(tx_message)

    crc_input_ascii = (rec_hex + snd_hex + tx_hex).encode("ascii")
    crc = crc16_modbus_ascii(crc_input_ascii)
    crc_hex = f"{crc & 0xFF:02X}{(crc >> 8) & 0xFF:02X}"  # low byte first

    frame = "#" + rec_hex + snd_hex + tx_hex + crc_hex + "\r"
    return frame.encode("ascii")

# ---------------------------
# Parse response, verify CRC, return transaction bytes (decoded) and addresses
# ---------------------------
def parse_vfbus_sl_response(resp: bytes):
    if not resp or resp[0:1] != b"#" or resp[-1:] != b"\r":
        raise ValueError("Bad frame: missing start '#' or end <CR>")

    ascii_payload = resp[1:-1].decode("ascii")  # exclude '#' and '\r'
    if len(ascii_payload) < 2 + 2 + 4:  # rec(2) + snd(2) + crc(4) minimal
        raise ValueError("Bad frame: too short")

    rec_hex = ascii_payload[0:2]
    snd_hex = ascii_payload[2:4]
    # CRC is last 4 characters
    crc_hex = ascii_payload[-4:]
    tx_hex = ascii_payload[4:-4]

    # verify CRC
    crc_input_ascii = (rec_hex + snd_hex + tx_hex).encode("ascii")
    calc_crc = crc16_modbus_ascii(crc_input_ascii)
    rx_crc = int(crc_hex[2:4] + crc_hex[0:2], 16)  # received is lo-hi; convert to int
    if calc_crc != rx_crc:
        raise ValueError(f"CRC mismatch: calc=0x{calc_crc:04X}, recv=0x{rx_crc:04X}")

    # decode transaction bytes
    tx_bytes = decode_hex_bytes(tx_hex)
    recipient = int(rec_hex, 16)
    sender = int(snd_hex, 16)
    return recipient, sender, tx_bytes

# ---------------------------
# Simple reader for a single PDU with read-data
# Return: (first_addr, [register_values as 16-bit list])
# ---------------------------
def parse_read_data_pdu(tx_bytes: bytes):
    if len(tx_bytes) < 2:
        raise ValueError("Transaction too short")

    trans_id = tx_bytes[0]
    pdu = tx_bytes[1:]
    if len(pdu) < 3:
        raise ValueError("PDU too short")

    header = pdu[0]
    pdu_type = (header >> 5) & 0x07  # 010 for read data, 011 for read error
    is_ro = ((header >> 4) & 0x01) == 0
    n_regs = (header & 0x0F) + 1

    if pdu_type == 0b011:
        raise RuntimeError("Read error PDU returned by device")

    if pdu_type != 0b010:
        raise RuntimeError(f"Unexpected PDU type 0b{pdu_type:03b} (expected read data)")

    if len(pdu) < 3 + 2 * n_regs:
        raise ValueError("PDU length inconsistent with register count")

    first_addr = pdu[1] | (pdu[2] << 8)
    reg_bytes = pdu[3:3 + 2 * n_regs]

    regs = []
    for i in range(n_regs):
        lo = reg_bytes[2 * i]
        hi = reg_bytes[2 * i + 1]
        regs.append(lo | (hi << 8))
    return trans_id, is_ro, first_addr, regs

def decode_auto_cps_from_frame(line: bytes):
    """
    Decode one auto-send VFBUS SL frame and return (ch1_cps, ch2_cps).

    Raises ValueError if the frame is not a valid auto CPS frame.
    """
    # Reuse your existing frame parser
    recipient, sender, tx = parse_vfbus_sl_response(line)

    if len(tx) < 4:
        raise ValueError("Transaction too short")

    tid = tx[0]
    hdr = tx[1]
    start = tx[2] | (tx[3] << 8)
    nregs = (hdr & 0x0F) + 1
    data = tx[4:]

    # This matches the auto-send frame layout from the doc / earlier code
    if start != 0x00EA or nregs < 12:
        raise ValueError("Not an auto-send CPS frame")

    if len(data) < 2 * nregs:
        raise ValueError("Data length inconsistent with nregs")

    def u16(i: int) -> int:
        return data[2 * i] | (data[2 * i + 1] << 8)

    def f32_at(reg_idx: int) -> float:
        w0 = u16(reg_idx)
        w1 = u16(reg_idx + 1)
        raw = (w1 << 16) | w0
        return struct.unpack("<f", struct.pack("<I", raw))[0]

    # Channel mapping from earlier:
    #   Ch1 cps → RES_RATE_VAL_1 → words at indices 4 & 5 (0x00EE–0x00EF)
    #   Ch2 cps → RES_RATE_VAL_2 → words at indices 10 & 11 (0x00F4–0x00F5)
    ch1_cps = f32_at(4)   # beta+gamma
    ch2_cps = f32_at(10)  # alpha

    return ch1_cps, ch2_cps


# ---------------------------
def main():
    ap = argparse.ArgumentParser(description="Read SFP-100 medium count rate (cps) over VFBUS SL")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=57600, help="Baud rate (default 57600)")
    ap.add_argument("--recipient", type=lambda x: int(x, 0), default=0x01, help="Device address (recipient)")
    ap.add_argument("--sender", type=lambda x: int(x, 0), default=0x02, help="Our address (sender)")
    ap.add_argument("--trans-id", type=lambda x: int(x, 0), default=0x01, help="Transaction ID byte")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x00FA, help="Start register address")
    ap.add_argument("--count", type=int, default=2, help="Register count (RES_RATE_VAL_* is 32-bit => 2 regs)")
    ap.add_argument("--rw", choices=["ro", "rw"], default="ro", help="RO vs RW address space")
    ap.add_argument("--timeout", type=float, default=0.6, help="Serial timeout seconds")
    args = ap.parse_args()

    # Build transaction
    tx = build_read_transaction(
        trans_id=args.trans_id,
        ro_space=(args.rw == "ro"),
        start_addr=args.addr,
        reg_count=args.count,
    )
    frame = frame_vfbus_sl(args.recipient, args.sender, tx)

    # Open serial
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout,
        write_timeout=args.timeout,
    )

    try:
        print("Listening for auto-send frames from probe...\n(Press Ctrl+C to stop)\n")

        # optional: shorter timeout makes the loop responsive
        ser.timeout = 0.1

        buf = bytearray()

        def extract_frames(buffer: bytearray):
            frames = []
            while True:
                # find start
                i_hash = buffer.find(b"#")
                if i_hash == -1:
                    buffer.clear()
                    break
                if i_hash > 0:
                    del buffer[:i_hash]  # drop garbage before '#'

                # find terminator
                i_cr = buffer.find(b"\r", 1)
                if i_cr == -1:
                    # incomplete frame; wait for more bytes
                    break

                frames.append(bytes(buffer[:i_cr+1]))  # include '\r'
                del buffer[:i_cr+1]  # consume
            return frames

        while True:
            # read what's available; fall back to small reads if in_waiting is 0
            n = ser.in_waiting
            chunk = ser.read(n if n else 64)
            if chunk:
                buf.extend(chunk)

            for line in extract_frames(buf):
                # Parse full VFBUS SL frame
                try:
                    recipient, sender, tx = parse_vfbus_sl_response(line)
                except Exception as e:
                    # Comment out next line if you want it completely silent
                    # print(f"Frame parse error: {e}")
                    continue

                # Basic fields
                tid = tx[0]                    # usually 0x80 in auto-send mode
                hdr = tx[1]
                start = tx[2] | (tx[3] << 8)
                nregs = (hdr & 0x0F) + 1
                data = tx[4:]

                if start != 0x00EA or nregs < 12:
                    # Unexpected auto frame type; ignore silently
                    continue

                # Helpers
                def u16(i): 
                    return data[2*i] | (data[2*i+1] << 8)
                def f32_at(reg_idx):
                    w0 = u16(reg_idx)
                    w1 = u16(reg_idx+1)
                    raw = (w1 << 16) | w0
                    import struct
                    return struct.unpack("<f", struct.pack("<I", raw))[0]

                # Decode rates
                cps_ch1 = f32_at(4)   # 0x00EE–0x00EF
                cps_ch2 = f32_at(10)  # 0x00F4–0x00F5
                print(f"Ch1 = {cps_ch1:8.3f} cps   Ch2 = {cps_ch2:8.3f} cps")

    except KeyboardInterrupt:
        print("\nStopped listening.")
    finally:
        ser.close()

def read_rate_at_addr(
    ser,
    recipient: int,
    sender: int,
    trans_id: int,
    start_addr: int,
) -> float:
    """
    Read ONE 32-bit float rate from 'start_addr' (2 regs) and return it as cps.
    """

    tx = build_read_transaction(
        trans_id=trans_id,
        ro_space=True,          # RO space
        start_addr=start_addr,
        reg_count=2,            # just 2 regs -> one 32-bit float
    )
    frame = frame_vfbus_sl(recipient, sender, tx)

    ser.write(frame)
    ser.flush()

    # --- read one full VFBUS SL frame: sync '#' ... '\r' ---
    resp = bytearray()

    # sync to '#'
    while True:
        ch = ser.read(1)
        if not ch:
            raise TimeoutError("Timeout waiting for start of frame '#'")
        if ch == b"#":
            resp += ch
            break

    # read until '\r'
    while True:
        ch = ser.read(1)
        if not ch:
            raise TimeoutError("Timeout waiting for end of frame '\\r'")
        resp += ch
        if ch == b"\r":
            break

    _, _, tx_bytes = parse_vfbus_sl_response(bytes(resp))

    if tx_bytes[0] != (trans_id & 0xFF):
        raise RuntimeError(f"Unexpected transaction ID 0x{tx_bytes[0]:02X}")

    _, _, first_addr, regs = parse_read_data_pdu(tx_bytes)

    if len(regs) < 2:
        raise ValueError(f"Expected 2 regs, got {len(regs)}")

    w0, w1 = regs[0], regs[1]
    raw = (w1 << 16) | w0
    cps = struct.unpack("<f", struct.pack("<I", raw))[0]
    return cps


def read_two_rates_once(
    ser,
    recipient: int = 0x01,
    sender: int = 0x02,
    trans_id: int = 0x01,
):
    """
    Do ONE measurement and return (beta_gamma_cps, alpha_cps).

    Ch1: RES_RATE_VAL_1 @ 0x00FA–0x00FB
    Ch2: RES_RATE_VAL_2 @ 0x0100–0x0101
    """

    # Channel 1 (beta+gamma)
    ch1_cps = read_rate_at_addr(
        ser=ser,
        recipient=recipient,
        sender=sender,
        trans_id=trans_id,
        start_addr=0x00FA,
    )

    # Use a different TID just to keep responses clearly separated (optional)
    ch2_cps = read_rate_at_addr(
        ser=ser,
        recipient=recipient,
        sender=sender,
        trans_id=(trans_id + 1) & 0xFF,
        start_addr=0x0100,
    )

    return ch1_cps, ch2_cps


if __name__ == "__main__":
    main()

