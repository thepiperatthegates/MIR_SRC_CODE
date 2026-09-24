import struct
import sys
import os

IN_FILE = "20260723_133155,286.bin"       # Binary input
OUT_FILE_BIN = "unpacked_byte.bin"        # Output: raw reduced frame
OUT_FILE_CSV = "unpacked_byte.csv"        # Output: decoded CSV
CSV_SEP = ";"

SAMPLE_FREQ = 1000
TIME_STAMP = 1.0 / SAMPLE_FREQ

SENSOR_HEADER_BYTES = 8
BYTE_INT16T = 2

# channel indices in the FULL frame
CH_DAC1, CH_DAC2 = 2, 3
CH_HALL_A, CH_HALL_B = 4, 5
CH_COIL_A, CH_COIL_B = 6, 7

# channel indices in the frame
R_HALL_A, R_HALL_B, R_COIL_A, R_COIL_B = 0, 1, 2, 3

# ---------------------------------------------------------------------------
# SD recorder format (REC*.BIN)
#
# The file is a sequence of images, each IMAGE_STEPS * IMAGE_LINES * 2 bytes
# (1000 samples * 8 lines * 2 = 16000 bytes), all little-endian uint16/int16.
# Every image slot is one of:
#
#   header image : TestImage_CreateImageHeader() output, no samples
#       word 0      0x55AA  SYNC1
#       word 1      siX     samples per line (1000)
#       word 2      siY     lines (8)
#       word 3      ImageNr
#       word 4..5   tick count t1 (LO, HI) in ms
#       word 6      DataSource
#       word 7      CommChannel
#       word 8..10  rfu
#       word 11     0xCAFE  SYNC2
#       word 12..139  RMAP register copy (128 words)
#       word 140..142 rfu
#       word 143    0xFFFE  SYNC3
#
#   data image   : no header, lines DAC1..COIL_B hold samples, but the whole
#                  image sits DATA_SHIFT bytes earlier than the line grid
#
#   empty image  : all zeros
# ---------------------------------------------------------------------------
IMAGE_STEPS = 1000
IMAGE_LINES = 8
LINE_BYTES = IMAGE_STEPS * BYTE_INT16T
IMAGE_BYTES = LINE_BYTES * IMAGE_LINES

SYNC1, SYNC2, SYNC3 = 0x55AA, 0xCAFE, 0xFFFE
HEADER_WORDS = 144
RMAP_FIRST_WORD, RMAP_WORDS = 12, 128
DATA_SHIFT = 8


class SensorFrame:
    """Validate + bind to a whole frame buffer (zero-copy view, like the C struct)."""

    def __init__(self, buf, length):
        if length < SENSOR_HEADER_BYTES:
            raise ValueError("frame too short")

        self.channel_count = sensor_frame_read_be32(buf, 0)
        self.samples_per_channel = sensor_frame_read_be32(buf, 4)

        if self.channel_count == 0 or self.samples_per_channel == 0:
            raise ValueError("sanity check failed")
        if length < self.channel_count * self.samples_per_channel * 2:
            raise ValueError("buffer smaller than channel_count * samples_per_channel * 2")

        self.base = buf


def sensor_frame_read_be32(buf, offset):
    return struct.unpack_from(">I", buf, offset)[0]


def sensor_frame_write_be32(buf, offset, value):
    struct.pack_into(">I", buf, offset, value)


def sensor_sample(f, ch, i):
    """One sample as signed int16 (big-endian -> host)."""
    offset = SENSOR_HEADER_BYTES + (ch * f.samples_per_channel + i) * BYTE_INT16T
    return struct.unpack_from(">h", f.base, offset)[0]


def sensor_reduce_to_4(in_buf, in_len):
    """Returns the reduced buffer (bytes) on success, or None on failure."""
    try:
        f = SensorFrame(in_buf, in_len)
    except ValueError:
        return None

    if f.channel_count <= CH_COIL_B:
        return None  # source must contain ch 4..7

    block = f.samples_per_channel * BYTE_INT16T
    need = block * 4 + SENSOR_HEADER_BYTES

    out = bytearray(need)
    sensor_frame_write_be32(out, 0, 4)                        # new channel count
    sensor_frame_write_be32(out, 4, f.samples_per_channel)    # unchanged sample count

    keep = (CH_HALL_A, CH_HALL_B, CH_COIL_A, CH_COIL_B)
    w = SENSOR_HEADER_BYTES
    for ch in keep:
        src_off = SENSOR_HEADER_BYTES + ch * block
        out[w:w + block] = f.base[src_off:src_off + block]
        w += block

    return bytes(out)


# ------------------------------ SD recorder --------------------------------

def parse_image_header(buf, off):
    """Decode a TestImage header at byte offset off, or None if the syncs don't match."""
    if off + HEADER_WORDS * 2 > len(buf):
        return None
    w = struct.unpack_from(f"<{HEADER_WORDS}H", buf, off)
    if w[0] != SYNC1 or w[11] != SYNC2 or w[143] != SYNC3:
        return None
    return {
        "siX": w[1],
        "siY": w[2],
        "image_nr": w[3],
        "tick_ms": w[4] | (w[5] << 16),
        "data_source": w[6],
        "comm_channel": w[7],
        "rmap": w[RMAP_FIRST_WORD:RMAP_FIRST_WORD + RMAP_WORDS],
    }


def data_image_lines(buf, off):
    """Return the 8 lines of a headerless data image as lists of int16, or None if empty."""
    start = off - DATA_SHIFT
    img = bytearray(IMAGE_BYTES)
    src_from = max(start, 0)
    src_to = min(start + IMAGE_BYTES, len(buf))
    img[src_from - start:src_to - start] = buf[src_from:src_to]

    hall_coil = img[CH_HALL_A * LINE_BYTES:(CH_COIL_B + 1) * LINE_BYTES]
    if not any(hall_coil):
        return None

    samples = struct.unpack(f"<{IMAGE_STEPS * IMAGE_LINES}h", img)
    return [samples[ch * IMAGE_STEPS:(ch + 1) * IMAGE_STEPS] for ch in range(IMAGE_LINES)]


def unpack_sd_recording(in_buf):
    """Split a REC*.BIN file into header images and data images.

    Returns (headers, frames) where headers is a list of (slot, header dict) and
    frames is a list of (slot, est_tick_ms, lines)."""
    n_slots = len(in_buf) // IMAGE_BYTES
    headers, data_slots = [], []

    for slot in range(n_slots):
        off = slot * IMAGE_BYTES
        hdr = parse_image_header(in_buf, off)
        if hdr is not None:
            headers.append((slot, hdr))
            continue
        lines = data_image_lines(in_buf, off)
        if lines is not None:
            data_slots.append((slot, lines))

    # data images carry no tick count -> estimate it from the header images,
    # which advance one image (1 s at 1 kHz) per slot
    ms_per_slot = IMAGE_STEPS * 1000 // SAMPLE_FREQ
    if headers:
        ref_slot, ref_hdr = headers[0]
        t0 = ref_hdr["tick_ms"] - ref_slot * ms_per_slot
    else:
        t0 = 0

    frames =[]
    for slot, lines in data_slots:
        tick_ms = t0 + slot * ms_per_slot
        frames.append((slot, tick_ms, lines))
    return headers, frames


def write_headers_csv(path, headers):
    with open(path, "w") as fptr:
        cols = ["slot", "offset", "image_nr", "tick_ms", "siX", "siY", "data_source", "comm_channel"]
        cols += [f"rmap_{i}" for i in range(RMAP_WORDS)]
        fptr.write(CSV_SEP.join(cols) + "\n")
        for slot, h in headers:
            row = [slot, slot * IMAGE_BYTES, h["image_nr"], h["tick_ms"], h["siX"], h["siY"],
                   h["data_source"], h["comm_channel"], *h["rmap"]]
            fptr.write(CSV_SEP.join(str(v) for v in row) + "\n")


def main_sd_recording(in_buf, out_file_csv):
    headers, frames = unpack_sd_recording(in_buf)
    
    print(f"SD recording: {len(in_buf) // IMAGE_BYTES} image slots, " f"{len(headers)} header images, {len(frames)} data images")

    if not frames:
        print("no data images found in SD recording", file=sys.stderr)
        return 1

    keep = (CH_HALL_A, CH_HALL_B, CH_COIL_A, CH_COIL_B)
    total = len(frames) * IMAGE_STEPS

    t_first = frames[0][1]
    with open(out_file_csv, "w") as fptr:
        fptr.write(f"i{CSV_SEP}HALL_A{CSV_SEP}HALL_B{CSV_SEP}COIL_A{CSV_SEP}COIL_B\n")
        for slot, tick_ms, lines in frames:
            t_img = (tick_ms - t_first) / 1000.0
            for i in range(IMAGE_STEPS):
                fptr.write(f"{t_img + i * TIME_STAMP:f}" + "".join(
                    f"{CSV_SEP}{lines[ch][i]}" for ch in keep) + "\n")
    print(f"wrote {out_file_csv} ({total} rows)")
    return 0


    return 0
# ------------------------------ legacy frame -------------------------------

# def main_legacy_frame(in_buf, out_file_bin, out_file_csv):
#     in_len = len(in_buf)

#     # --- reduce to the 4 channels ---
#     reduced_buf = sensor_reduce_to_4(in_buf, in_len)
#     if not reduced_buf:
#         print("not a valid full frame", file=sys.stderr)
#         return 1

#     rlen = len(reduced_buf)
#     print(f"reduced_buf to {rlen} bytes (dropped internals + both DACs)")

#     # --- decode the reduced_buf buffer ---
#     try:
#         r = SensorFrame(reduced_buf, rlen)
#     except ValueError:
#         print("bad reduced_buf frame", file=sys.stderr)
#         return 1

#     print(f"reduced_buf frame: channel_count={r.channel_count} samples_per_channel={r.samples_per_channel}")

#     # --- write the raw reduced_buf binary file ---
#     try:
#         with open(out_file_bin, "wb") as fptr:
#             fptr.write(reduced_buf)
#     except OSError:
#         print(f"open {out_file_bin}")
#         return 1

#     print(f"wrote {out_file_bin} ({rlen} bytes)")

#     # --- open the CSV file ---
#     try:
#         fptr = open(out_file_csv, "w")
#     except OSError:
#         print(f"open {out_file_csv}")
#         return 1

#     with fptr:
#         time_stamp = 0.0
#         # Write the header
#         fptr.write(f"i{CSV_SEP}HALL_A{CSV_SEP}HALL_B{CSV_SEP}COIL_A{CSV_SEP}COIL_B\n")
#         for i in range(r.samples_per_channel):
#             fptr.write(
#                 f"{time_stamp:f}{CSV_SEP}{sensor_sample(r, R_HALL_A, i)}{CSV_SEP}"
#                 f"{sensor_sample(r, R_HALL_B, i)}{CSV_SEP}{sensor_sample(r, R_COIL_A, i)}{CSV_SEP}"
#                 f"{sensor_sample(r, R_COIL_B, i)}\n"
#             )
#             time_stamp += TIME_STAMP

#     print(f"wrote {out_file_csv} ({r.samples_per_channel} rows)")

#     # --- print a few samples to stdout for a quick sanity check ---
#     print(" i  HALL_A HALL_B COIL_A COIL_B")
#     for i in range(3):
#         print(f"{i:3d} {sensor_sample(r, R_HALL_A, i):6d} {sensor_sample(r, R_HALL_B, i):6d} "
#               f"{sensor_sample(r, R_COIL_A, i):6d} {sensor_sample(r, R_COIL_B, i):6d}")

#     last = r.samples_per_channel - 1
#     print(f"... last sample {last}: {sensor_sample(r, R_HALL_A, last)} {sensor_sample(r, R_HALL_B, last)} "
#           f"{sensor_sample(r, R_COIL_A, last)} {sensor_sample(r, R_COIL_B, last)}")

#     return 0


# def is_legacy_frame(in_buf):
#     """Legacy frames start with a BE (channel_count, samples_per_channel) prefix."""
#     try:
#         f = SensorFrame(in_buf, len(in_buf))
#     except ValueError:
#         return False
#     return f.channel_count <= 64 and len(in_buf) == SENSOR_HEADER_BYTES + f.channel_count * f.samples_per_channel * 2


def main():
    # Sort out the passed arguments
    in_file = sys.argv[1] if len(sys.argv) > 1 else IN_FILE
    out_file_bin = sys.argv[2] if len(sys.argv) > 2 else OUT_FILE_BIN
    out_file_csv = sys.argv[3] if len(sys.argv) > 3 else OUT_FILE_CSV

    # --- read the whole file into memory ---
    try:
        with open(in_file, "rb") as fptr:
            in_buf = fptr.read()
    except OSError:
        print("Failed to open file!")
        return 1

    print(f"read {len(in_buf)} bytes from {in_file}")

    return main_sd_recording(in_buf, out_file_csv)


if __name__ == "__main__":
    sys.exit(main())
