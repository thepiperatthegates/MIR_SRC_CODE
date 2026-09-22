import struct
import sys

IN_FILE = "20260723_133155,286.bin"       # Binary input
OUT_FILE_BIN = "unpacked_byte.bin"        # Output: raw reduced frame
OUT_FILE_CSV = "unpacked_byte.csv"        # Output: decoded CSV
CSV_SEP = ";"

SAMPLE_FREQ = 1000
TIME_STAMP = 1.0 / SAMPLE_FREQ

SENSOR_HEADER_BYTES = 8
BYTE_UINT16T = 2

# channel indices in the FULL frame
CH_DAC1, CH_DAC2 = 2, 3
CH_HALL_A, CH_HALL_B = 4, 5
CH_COIL_A, CH_COIL_B = 6, 7

# channel indices in the REDUCED (4-ch) frame
R_HALL_A, R_HALL_B, R_COIL_A, R_COIL_B = 0, 1, 2, 3


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
    offset = SENSOR_HEADER_BYTES + (ch * f.samples_per_channel + i) * BYTE_UINT16T
    return struct.unpack_from(">h", f.base, offset)[0]


def sensor_reduce_to_4(in_buf, in_len):
    """Returns the reduced buffer (bytes) on success, or None on failure."""
    try:
        f = SensorFrame(in_buf, in_len)
    except ValueError:
        print("Feeding inner buffer to reduce produced an error!")
        return None

    if f.channel_count <= CH_COIL_B:
        return None  # source must contain ch 4..7

    block = f.samples_per_channel * BYTE_UINT16T
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

    in_len = len(in_buf)
    print(f"read {in_len} bytes from {in_file}")

    # --- reduce to the 4 channels ---
    reduced_buf = sensor_reduce_to_4(in_buf, in_len)
    if not reduced_buf:
        print("not a valid full frame", file=sys.stderr)
        return 1

    rlen = len(reduced_buf)
    print(f"reduced_buf to {rlen} bytes (dropped internals + both DACs)")

    # --- decode the reduced_buf buffer ---
    try:
        r = SensorFrame(reduced_buf, rlen)
    except ValueError:
        print("bad reduced_buf frame", file=sys.stderr)
        return 1

    print(f"reduced_buf frame: channel_count={r.channel_count} samples_per_channel={r.samples_per_channel}")

    # --- write the raw reduced_buf binary file ---
    try:
        with open(out_file_bin, "wb") as fptr:
            fptr.write(reduced_buf)
    except OSError:
        print(f"open {out_file_bin}")
        return 1

    print(f"wrote {out_file_bin} ({rlen} bytes)")

    # --- open the CSV file ---
    try:
        fptr = open(out_file_csv, "w")
    except OSError:
        print(f"open {out_file_csv}")
        return 1

    with fptr:
        time_stamp = 0.0
        # Write the header
        fptr.write(f"i{CSV_SEP}HALL_A{CSV_SEP}HALL_B{CSV_SEP}COIL_A{CSV_SEP}COIL_B\n")
        for i in range(r.samples_per_channel):
            fptr.write(
                f"{time_stamp:f}{CSV_SEP}{sensor_sample(r, R_HALL_A, i)}{CSV_SEP}"
                f"{sensor_sample(r, R_HALL_B, i)}{CSV_SEP}{sensor_sample(r, R_COIL_A, i)}{CSV_SEP}"
                f"{sensor_sample(r, R_COIL_B, i)}\n"
            )
            time_stamp += TIME_STAMP

    print(f"wrote {out_file_csv} ({r.samples_per_channel} rows)")

    # END

    # --- print a few samples to stdout for a quick sanity check ---
    print(" i  HALL_A HALL_B COIL_A COIL_B")
    for i in range(3):
        print(f"{i:3d} {sensor_sample(r, R_HALL_A, i):6d} {sensor_sample(r, R_HALL_B, i):6d} "
              f"{sensor_sample(r, R_COIL_A, i):6d} {sensor_sample(r, R_COIL_B, i):6d}")

    last = r.samples_per_channel - 1
    print(f"... last sample {last}: {sensor_sample(r, R_HALL_A, last)} {sensor_sample(r, R_HALL_B, last)} "
          f"{sensor_sample(r, R_COIL_A, last)} {sensor_sample(r, R_COIL_B, last)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
