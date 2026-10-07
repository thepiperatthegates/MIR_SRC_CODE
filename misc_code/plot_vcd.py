"""Draw a timing diagram (PNG) of selected signals from a .vcd file."""

import re

import matplotlib.pyplot as plt

TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9, "ps": 1e-12, "fs": 1e-15}


def read_vcd(path):
    """Return {full_signal_name: (width, [(time_s, value_str), ...])} and the list of names."""
    scope = []
    code_to_names = {}
    widths = {}
    changes = {}
    timescale = 1e-12
    t = 0.0

    with open(path, "r", errors="replace") as f:
        text = f.read()

    # ---- header: timescale, scopes and variables ----
    header, _, body = text.partition("$enddefinitions")
    m = re.search(r"\$timescale\s+(\d+)\s*([a-z]+)\s+\$end", header)
    if m:
        timescale = int(m.group(1)) * TIME_UNITS[m.group(2)]
    # walk the header word by word (ID codes can themselves be '$', so no regex on '$...$end')
    words = header.split()
    i = 0
    while i < len(words):
        w = words[i]
        if w == "$scope":                       # $scope module <name> $end
            scope.append(words[i + 2])
            i += 4
        elif w == "$upscope":                   # $upscope $end
            scope.pop()
            i += 2
        elif w == "$var":                       # $var wire <width> <code> <name> [range] $end
            width, code, name = int(words[i + 2]), words[i + 3], words[i + 4]
            full = ".".join(scope + [name])
            code_to_names.setdefault(code, []).append(full)
            widths[full] = width
            changes[full] = []
            i += 5
            while words[i] != "$end":
                i += 1
            i += 1
        else:
            i += 1

    # ---- body: #time and value changes ----
    for line in body.split("\n")[1:]:
        line = line.strip()
        if not line or line.startswith("$"):
            continue
        if line[0] == "#":
            t = int(line[1:]) * timescale
            continue
        if line[0] in "bBrR":
            value, code = line[1:].split()
        else:
            value, code = line[0], line[1:]
        for name in code_to_names.get(code, []):
            # the simulator also logs re-assignments of the same value -> keep real changes only
            if not changes[name] or changes[name][-1][1] != value:
                changes[name].append((t, value))

    return {n: (widths[n], changes[n]) for n in changes}


def find_signal(signals, short_name):
    """Match a signal by the end of its full name, e.g. 'spi_cs' or 'SPI_MASTER_inst.rx_shift_1'."""
    hits = [n for n in signals if n == short_name or n.endswith("." + short_name)]
    if not hits:
        raise KeyError(f"signal '{short_name}' not in VCD")
    return min(hits, key=len)


def value_at_start(change_list, t0):
    """Value a signal has at time t0 (last change at or before t0)."""
    v = None
    for t, val in change_list:
        if t > t0:
            break
        v = val
    return v


def to_hex(value):
    """Turn a VCD bit string into hex text (keeps x/u if present)."""
    if any(c not in "01" for c in value):
        return value
    return f"{int(value, 2):X}"


def plot_timing(vcd_path, signal_names, t_start_us, t_end_us, out_png, title="",
                mark_edges_of=None, mark_edge="falling"):
    """mark_edges_of: optional 1-bit signal (e.g. "spi_sclk") whose edges get a dotted line across all rows."""
    signals = read_vcd(vcd_path)
    t0, t1 = t_start_us * 1e-6, t_end_us * 1e-6

    fig, ax = plt.subplots(figsize=(14, 0.6 * len(signal_names) + 1))

    # ---- dotted line + number at every (falling/rising) edge of the marker signal ----
    if mark_edges_of is not None:
        _, chg = signals[find_signal(signals, mark_edges_of)]
        wanted = "0" if mark_edge == "falling" else "1"
        n = 0
        for t, val in chg:
            if t0 < t < t1 and val == wanted:
                n += 1
                ax.axvline(t * 1e6, color="tab:red", linestyle=":", lw=0.9, alpha=0.7)
                ax.text(t * 1e6, len(signal_names) - 0.15, str(n), color="tab:red",
                        ha="center", va="bottom", fontsize=7)
    for row, short in enumerate(signal_names):
        name = find_signal(signals, short)
        width, chg = signals[name]
        y = len(signal_names) - 1 - row          # first signal on top

        # ---- segments [start, end, value] inside the window ----
        segs = []
        v = value_at_start(chg, t0)
        start = t0
        for t, val in chg:
            if t <= t0:
                continue
            if t >= t1:
                break
            segs.append((start, t, v))
            start, v = t, val
        segs.append((start, t1, v))

        for s, e, val in segs:
            xs, xe = s * 1e6, e * 1e6
            if val is None:
                continue
            if width == 1:
                level = 0.7 if val == "1" else 0.0
                ax.plot([xs, xe], [y + level, y + level], color="tab:green", lw=1.5)
                if val == "1":
                    ax.fill_between([xs, xe], y, y + level, color="tab:green", alpha=0.15, lw=0)
            else:
                # bus: two lines with a label in the middle
                ax.plot([xs, xe], [y + 0.7, y + 0.7], color="tab:blue", lw=1.2)
                ax.plot([xs, xe], [y, y], color="tab:blue", lw=1.2)
                ax.plot([xs, xs], [y, y + 0.7], color="tab:blue", lw=1.2)
                if (xe - xs) > (t_end_us - t_start_us) / 40:      # only label wide enough segments
                    ax.text((xs + xe) / 2, y + 0.35, to_hex(val), ha="center", va="center", fontsize=8)

        # vertical edges for 1-bit signals
        if width == 1:
            for t, val in chg:
                if t0 < t < t1:
                    ax.plot([t * 1e6, t * 1e6], [y, y + 0.7], color="tab:green", lw=1.5)

    ax.set_yticks([len(signal_names) - 1 - i + 0.35 for i in range(len(signal_names))])
    ax.set_yticklabels(signal_names)
    ax.set_xlim(t_start_us, t_end_us)
    ax.set_xlabel("Time [µs]")
    ax.set_title(title)
    ax.grid(True, axis="x", alpha=0.3)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"Saved {out_png}")


if __name__ == "__main__":
    plot_timing(
        vcd_path=r"C:\Users\Hijazi\Downloads\tb_spi_workflow.vcd",
        signal_names=[
            "spi_cs",
            "spi_sclk",
            "spi_mosi",
            "spi_miso_1",
            "SPI_MASTER_inst.rx_shift_1",
            "spi_miso_2",
            "SPI_MASTER_inst.rx_shift_2",
            "spi_done",
            "raw_valid",
        ],
        t_start_us=70.5,
        t_end_us=89.0,
        out_png=r"C:\Users\Hijazi\Downloads\spi_frame2_sample1.png",
        title="Frame 2 of sample 1: ADC sends 0x91F0 (A) and 0x4E84 (B)",
        mark_edges_of="spi_sclk",       # red dotted line at each SCLK falling edge = moment a bit is read
        mark_edge="falling",
    )
