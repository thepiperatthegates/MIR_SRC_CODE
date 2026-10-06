"""Extract the V1 (Hall 1) column from saved recording CSVs, convert it back to raw uint16 ADC codes, and compare the noise of a raw and a filtered recording."""

import os

import matplotlib.pyplot as plt
import numpy as np

# Normalisation constants that were active when the recording was made.
# save_to_bin did: V1 = (-code - ZERO_OFFSET_1) / AMP_1   ->   code = -(V1 * AMP_1 + ZERO_OFFSET_1)
AMP_1 = 2.5 / 4096              # voltage_amp_1_V
ZERO_OFFSET_1 = 0.45361328125   # voltage_zero_offset_1_V

V1_COLUMN = 1   # 0 = time, 1 = V1, 2 = V2, 3 = I1, 4 = I2
SAMPLE_FREQ = 5000   # Hz, used for the time axis and the noise spectrum
N_HARMONICS = 5      # harmonics of the sine removed before looking at the noise


def extract_v1_uint16(input_csv, output_csv=None):
    """Read the recording, undo the V1 normalisation, and save the codes as uint16 to a new CSV."""
    if output_csv is None:
        root, _ = os.path.splitext(input_csv)
        output_csv = root + "_V1_uint16.csv"

    data = np.loadtxt(input_csv, delimiter=";")
    v1 = data[:, V1_COLUMN]

    codes = -(v1 * AMP_1 + ZERO_OFFSET_1)

    # ---- sanity checks: codes should be whole numbers inside the uint16 range ----
    max_frac_error = np.abs(codes - np.round(codes)).max()
    if max_frac_error > 0.01:
        print(f"WARNING: codes are not whole numbers (max error {max_frac_error:.3f}), check AMP_1 / ZERO_OFFSET_1")
    if codes.min() < 0 or codes.max() > 65535:
        print(f"WARNING: codes outside uint16 range ({codes.min():.1f} .. {codes.max():.1f}), values will be clipped")

    codes_u16 = np.clip(np.round(codes), 0, 65535).astype(np.uint16)

    np.savetxt(output_csv, codes_u16, delimiter=";", fmt="%d")
    print(f"Saved {len(codes_u16)} samples to {output_csv} (min {codes_u16.min()}, max {codes_u16.max()})")
    return codes_u16


def remove_sine(codes):
    """Fit the sine (plus harmonics) by least squares and return what is left over, i.e. the noise."""
    x = codes.astype(float)
    t = np.arange(len(x)) / SAMPLE_FREQ

    # ---- fundamental frequency from the FFT peak ----
    spectrum = np.abs(np.fft.rfft(x - x.mean()))
    f0 = np.fft.rfftfreq(len(x), 1 / SAMPLE_FREQ)[np.argmax(spectrum)]

    columns = [np.ones_like(t)]
    for k in range(1, N_HARMONICS + 1):
        columns += [np.sin(2 * np.pi * k * f0 * t), np.cos(2 * np.pi * k * f0 * t)]
    model = np.column_stack(columns)

    coeffs, *_ = np.linalg.lstsq(model, x, rcond=None)
    return x - model @ coeffs, f0


def plot_comparison(csv_paths, labels):
    """Plot the signals, their noise after removing the sine, and the noise spectrum, all in one figure."""
    fig, (ax_signal, ax_noise, ax_psd) = plt.subplots(3, 1, figsize=(11, 10))

    for csv_path, label in zip(csv_paths, labels):
        codes = np.loadtxt(csv_path, delimiter=";", dtype=np.uint16)
        time = np.arange(len(codes)) / SAMPLE_FREQ
        noise, f0 = remove_sine(codes)
        rms = noise.std()
        print(f"{label}: f0 = {f0:.2f} Hz, noise RMS = {rms:.2f} counts, peak-to-peak = {np.ptp(noise):.0f} counts")

        ax_signal.plot(time, codes, linewidth=0.8, label=label)
        ax_noise.plot(time, noise, linewidth=0.5, alpha=0.7, label=f"{label} (RMS {rms:.2f})")
        ax_psd.psd(noise, NFFT=1024, Fs=SAMPLE_FREQ, label=label)

    ax_signal.set_title("V1 raw ADC codes")
    ax_signal.set_xlabel("Time [s]")
    ax_signal.set_ylabel("ADC code (uint16)")

    ax_noise.set_title("Noise (signal minus fitted sine)")
    ax_noise.set_xlabel("Time [s]")
    ax_noise.set_ylabel("Counts")

    ax_psd.set_title("Noise spectrum")
    ax_psd.set_xlabel("Frequency [Hz]")

    for ax in (ax_signal, ax_noise, ax_psd):
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    recordings = [
        # (input csv, output csv, label)
        (r"C:\Users\Hijazi\Downloads\ASDADA_before_filter.csv",
         r"C:\Users\Hijazi\Downloads\ASDADA_before_filter_V1_uint16.csv",
         "Before filter"),
        (r"C:\Users\Hijazi\Downloads\ASDADA_after_filter.csv",
         r"C:\Users\Hijazi\Downloads\ASDADA_after_filter_V1_uint16.csv",
         "After filter (MA N=4)"),
    ]

    for input_csv, output_csv, _ in recordings:
        extract_v1_uint16(input_csv, output_csv)

    plot_comparison([out for _, out, _ in recordings], [label for _, _, label in recordings])
