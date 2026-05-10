#!/usr/bin/env python3

import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import serial

# =============================================================================
# USER CONFIG
# =============================================================================

PORT = "/dev/ttyUSB0"
BAUD = 921600
SERIAL_TIMEOUT_S = 0.05

UPDATE_RATE = 0               # CMD 76 value: 0 = 20010 Hz (persists across power cycles)
RICH_POLL_INTERVAL_S = 0.25   # Poll CMD 44 at this interval (~4 Hz)
STARTUP_DELAY_S = 2.0         # Wait after opening port before handshake (interface autodetect)

CMD_PRODUCT_NAME = 0
CMD_DISTANCE_OUTPUT = 29    # Bitmask: which fields appear in CMD 44 packets
CMD_STREAM = 30
CMD_FULL_SPEED_DIST = 40    # Batched distance; auto-streams when CMD_STREAM = 11
CMD_DISTANCE_DATA = 44      # Rich per-field data; polled periodically
CMD_LASER_FIRING = 50
CMD_UPDATE_RATE = 76        # Controls measurement rate; 1/uint8, persists

STREAM_MODE_FULL_SPEED = 11  # CMD 30 value: stream CMD 40

STREAM_DISABLED = 0

# CMD 29 bitmask bits — each enabled bit adds one Int16 to the CMD 44 payload.
DIST_OUT_FIRST_RAW      = (1 << 0)   # First return raw (cm)
DIST_OUT_FIRST_FILTER   = (1 << 1)   # First return filtered (cm)
DIST_OUT_FIRST_STRENGTH = (1 << 2)   # First return strength (%)
DIST_OUT_LAST_RAW       = (1 << 3)   # Last return raw (cm)
DIST_OUT_LAST_FILTER    = (1 << 4)   # Last return filtered (cm)
DIST_OUT_LAST_STRENGTH  = (1 << 5)   # Last return strength (%)
DIST_OUT_BACKGROUND     = (1 << 6)   # Background noise
DIST_OUT_TEMPERATURE    = (1 << 7)   # Temperature

DISTANCE_OUTPUT_MASK = (
    DIST_OUT_FIRST_RAW | DIST_OUT_FIRST_FILTER |
    DIST_OUT_LAST_RAW  | DIST_OUT_LAST_FILTER
)


# =============================================================================
# Packet helpers
# =============================================================================

@dataclass
class Packet:
    command_id: int
    write: bool
    data: bytes
    raw: bytes


def crc16_ccitt_lightware(data: bytes) -> int:
    crc = 0
    for byte in data:
        code = (crc >> 8) & 0xFFFF
        code ^= byte
        code ^= (code >> 4)
        crc = ((crc << 8) & 0xFFFF)
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 7) & 0xFFFF
        crc ^= code
        crc &= 0xFFFF
    return crc


def build_packet(command_id: int, write: bool, payload: bytes = b"") -> bytes:
    payload_length = 1 + len(payload)
    flags = (payload_length << 6) | (1 if write else 0)

    pkt = bytearray()
    pkt.append(0xAA)
    pkt.append(flags & 0xFF)
    pkt.append((flags >> 8) & 0xFF)
    pkt.append(command_id & 0xFF)
    pkt.extend(payload)

    crc = crc16_ccitt_lightware(pkt)
    pkt.append(crc & 0xFF)
    pkt.append((crc >> 8) & 0xFF)
    return bytes(pkt)


class PacketReader:
    def __init__(self) -> None:
        self.invalid_crc_count = 0
        self.invalid_length_count = 0
        self.reset()

    def reset(self) -> None:
        self.state = 0
        self.buf = bytearray()
        self.remaining = 0

    def feed(self, byte: int):
        if self.state == 0:
            if byte == 0xAA:
                self.buf = bytearray([byte])
                self.state = 1
            return None

        if self.state == 1:
            self.buf.append(byte)
            self.state = 2
            return None

        if self.state == 2:
            self.buf.append(byte)
            flags = self.buf[1] | (self.buf[2] << 8)
            payload_length = flags >> 6
            if not (1 <= payload_length <= 1023):
                self.invalid_length_count += 1
                self.reset()
                return None
            self.remaining = payload_length + 2
            self.state = 3
            return None

        self.buf.append(byte)
        self.remaining -= 1
        if self.remaining > 0:
            return None

        raw = bytes(self.buf)
        self.reset()

        rx_crc = raw[-2] | (raw[-1] << 8)
        calc_crc = crc16_ccitt_lightware(raw[:-2])
        if rx_crc != calc_crc:
            self.invalid_crc_count += 1
            return None

        flags = raw[1] | (raw[2] << 8)
        payload_length = flags >> 6
        write = bool(flags & 0x1)
        payload = raw[3:3 + payload_length]
        if len(payload) < 1:
            return None

        return Packet(
            command_id=payload[0],
            write=write,
            data=payload[1:],
            raw=raw,
        )


def pack_u32(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=False)


def decode_c_string(data: bytes) -> str:
    return data.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def unpack_u32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=False)


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT_S,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    ser.dtr = False
    ser.rts = False
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_packet_until(port: serial.Serial, reader: PacketReader, deadline: float):
    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        pkt = reader.feed(chunk[0])
        if pkt is not None:
            return pkt
    return None


def execute_command(
    port: serial.Serial,
    reader: PacketReader,
    command_id: int,
    write: bool,
    payload: bytes = b"",
    timeout_s: float = 0.30,
    retries: int = 3,
):
    request = build_packet(command_id, write, payload)
    last_error = None

    for _ in range(retries):
        port.write(request)
        port.flush()

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pkt = read_packet_until(port, reader, deadline)
            if pkt is None:
                continue
            if pkt.command_id == command_id:
                return pkt

        last_error = TimeoutError(f"No response for command {command_id}")

    if last_error is not None:
        raise last_error
    raise TimeoutError(f"No response for command {command_id}")


def handshake(port: serial.Serial, reader: PacketReader) -> str:
    request = build_packet(CMD_PRODUCT_NAME, False, b"")
    port.reset_input_buffer()
    port.reset_output_buffer()
    reader.reset()

    for _ in range(4):
        port.write(request)
        port.flush()

        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            pkt = read_packet_until(port, reader, deadline)
            if pkt is None:
                continue
            if pkt.command_id == CMD_PRODUCT_NAME:
                return decode_c_string(pkt.data)

        time.sleep(0.05)

    raise TimeoutError("Handshake failed")


def stop_stream_best_effort(port: serial.Serial, reader: PacketReader) -> None:
    try:
        execute_command(
            port,
            reader,
            CMD_STREAM,
            True,
            pack_u32(STREAM_DISABLED),
            timeout_s=0.20,
            retries=2,
        )
    except Exception:
        pass


def read_stream_mode(port: serial.Serial, reader: PacketReader):
    pkt = execute_command(port, reader, CMD_STREAM, False, timeout_s=0.20, retries=2)
    if len(pkt.data) >= 4:
        return unpack_u32(pkt.data, 0)
    return None


def main() -> None:
    output_dir = Path.home() / "lidar_collect"
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fast_csv = output_dir / f"sf30d_fast_{timestamp}.csv"
    rich_csv = output_dir / f"sf30d_rich_{timestamp}.csv"

    _rich_field_names = [
        "first_raw", "first_filt", "first_str%",
        "last_raw",  "last_filt",  "last_str%",
        "bg_noise",  "temp",
    ]

    with open_serial(PORT, BAUD) as ser:
        reader = PacketReader()

        print(f"Opened {PORT} @ {BAUD} 8N1")
        print(f"Waiting {STARTUP_DELAY_S:.1f}s for interface autodetect...")
        time.sleep(STARTUP_DELAY_S)
        ser.reset_input_buffer()
        product = handshake(ser, reader)
        print(f"Handshake OK: {product}")

        stop_stream_best_effort(ser, reader)
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        reader.reset()

        # CMD 29 controls which fields appear in CMD 44 packets.
        dist_out_pkt = execute_command(ser, reader, CMD_DISTANCE_OUTPUT, False, timeout_s=0.3)
        dist_out_val = unpack_u32(dist_out_pkt.data) if len(dist_out_pkt.data) >= 4 else 0
        print(f"Distance output mask (CMD 29): 0x{dist_out_val:02X}")
        if dist_out_val == 0:
            execute_command(ser, reader, CMD_DISTANCE_OUTPUT, True, pack_u32(DISTANCE_OUTPUT_MASK))
            dist_out_val = DISTANCE_OUTPUT_MASK
            print(f"  Was 0 — set to 0x{dist_out_val:02X}")

        rich_field_names = [
            name for bit, name in enumerate(_rich_field_names) if (dist_out_val >> bit) & 1
        ]

        # CMD 76 persists across power cycles.
        execute_command(ser, reader, CMD_UPDATE_RATE, True, bytes([UPDATE_RATE]))
        print(f"Update rate: {UPDATE_RATE} (20010 Hz)")

        # Start CMD 40 full-speed stream.
        execute_command(ser, reader, CMD_STREAM, True, pack_u32(STREAM_MODE_FULL_SPEED))
        print(f"Stream mode: {STREAM_MODE_FULL_SPEED} (CMD 40 full-speed distance)")

        # Precompute the CMD 44 read request to inject mid-stream.
        rich_request = build_packet(CMD_DISTANCE_DATA, False)

        fast_readings = 0
        fast_packets = 0
        rich_packets = 0
        crc_errors_start = reader.invalid_crc_count

        print(f"Capturing until Ctrl-C → {fast_csv.name} + {rich_csv.name}")
        print(f"  CMD 40 fast distance  |  CMD 44 rich @ ~{1/RICH_POLL_INTERVAL_S:.0f} Hz")

        t_start = time.monotonic()

        with open(fast_csv, "w") as ff, open(rich_csv, "w") as rf:
            ff.write("t_s,dist_cm\n")
            rf.write("t_s," + ",".join(rich_field_names) + "\n")

            t_next_rich = t_start + RICH_POLL_INTERVAL_S

            try:
                while True:
                    now = time.monotonic()

                    # Inject CMD 44 read request without interrupting the stream.
                    if now >= t_next_rich:
                        ser.write(rich_request)
                        ser.flush()
                        t_next_rich += RICH_POLL_INTERVAL_S

                    # Read whatever the device has sent since last iteration.
                    chunk = ser.read(4096)
                    if not chunk:
                        continue

                    t_pkt = time.monotonic() - t_start

                    for b in chunk:
                        pkt = reader.feed(b)
                        if pkt is None:
                            continue

                        if pkt.command_id == CMD_FULL_SPEED_DIST:
                            # pkt.data[0] = n (number of readings), then n × Int16
                            if not pkt.data:
                                continue
                            n = pkt.data[0]
                            if n == 0:
                                continue
                            for i in range(n):
                                off = 1 + i * 2
                                if off + 2 > len(pkt.data):
                                    break
                                d = int.from_bytes(pkt.data[off:off + 2], "little", signed=True)
                                ff.write(f"{t_pkt:.6f},{d}\n")
                            fast_readings += n
                            fast_packets += 1
                            if fast_packets <= 5:
                                first = int.from_bytes(pkt.data[1:3], "little", signed=True)
                                print(f"  CMD40 pkt={fast_packets} n={n} first={first} cm")

                        elif pkt.command_id == CMD_DISTANCE_DATA:
                            values = []
                            off = 0
                            for bit in range(8):
                                if (dist_out_val >> bit) & 1:
                                    if off + 2 <= len(pkt.data):
                                        values.append(
                                            int.from_bytes(pkt.data[off:off + 2], "little", signed=True)
                                        )
                                        off += 2
                            rf.write(f"{t_pkt:.6f}," + ",".join(str(v) for v in values) + "\n")
                            rich_packets += 1
                            pairs = " ".join(f"{n}={v}" for n, v in zip(rich_field_names, values))
                            print(f"  RICH [{rich_packets}] t={t_pkt:.3f}s {pairs}")

            except KeyboardInterrupt:
                print("\nCapture interrupted — finalizing CSV.")

        elapsed = time.monotonic() - t_start
        stop_stream_best_effort(ser, reader)

        print()
        print("Results")
        print("-------")
        print(f"Fast readings:     {fast_readings}  ({fast_readings / elapsed:.0f} Hz effective)")
        print(f"Rich packets:      {rich_packets}")
        print(f"CRC errors:        {reader.invalid_crc_count - crc_errors_start}")
        print(f"Fast CSV:          {fast_csv}")
        print(f"Rich CSV:          {rich_csv}")


if __name__ == "__main__":
    main()