"""
Converts raw RGB888 .BIN frame dumps (written by SD_SaveFrame() in main.c)
into JPG images.
"""

import sys
import os
import glob

import numpy as np
import cv2

#------------- CHOSEN RESOLUTION --------------------#
WIDTH = 1920
HEIGHT = 1080
BPP = 3
EXPECTED_SIZE = WIDTH * HEIGHT * BPP



def black_level_and_white_patch_balance(img, black_percentile=2, white_percentile=95):
    """Two-stage correction, matching a typical ISP order:

    1. Black-level subtraction: measure each channel's mean over the
       DARKEST pixels (the sensor's per-channel bias/pedestal) and
       subtract it. Plain white-balance gain can't fix this -- scaling a
       near-zero signal is still near-zero, but the fixed offset dominates
       there, which is exactly why the purple tint was concentrated in
       shadows while bright areas already looked fine.
    2. White-patch gain: scales the brightest pixels (post-offset) to
       neutral, to catch any remaining highlight tint.
    """
    img = img.astype(np.float32)
    flat = img.reshape(-1, img.shape[2])
    brightness = flat.sum(axis=1)

    black_threshold = np.percentile(brightness, black_percentile)
    black_level = flat[brightness <= black_threshold].mean(axis=0)
    img = img - black_level
    img = np.clip(img, 0, 255)

    flat = img.reshape(-1, img.shape[2])
    brightness = flat.sum(axis=1)
    white_threshold = np.percentile(brightness, white_percentile)
    white_level = flat[brightness >= white_threshold].mean(axis=0)
    target = white_level.mean()
    scale = target / np.maximum(white_level, 1e-6)
    img = img * scale

    return np.clip(img, 0, 255).astype(np.uint8)


def start_convert_one(bin_path, out_dir):
    with open(bin_path, "rb") as f:
        data = f.read()

    if len(data) != EXPECTED_SIZE:
        print(f"  skip {os.path.basename(bin_path)}: {len(data)} bytes, expected {EXPECTED_SIZE} "
              f"(wrong WIDTH/HEIGHT/BPP in this script, or a truncated/corrupt file)")
        return False

    img = np.frombuffer(data, dtype=np.uint8).reshape((HEIGHT, WIDTH, BPP))
    img = black_level_and_white_patch_balance(img)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # sensor stream is RGB888

    name = os.path.splitext(os.path.basename(bin_path))[0] + ".jpg"
    out_path = os.path.join(out_dir, name)
    cv2.imwrite(out_path, img_bgr)
    print(f"  {os.path.basename(bin_path)} -> {out_path}")
    return True


def cli_start_convert_bin(INPUT_PATH, OUTPUT_DIR):
    
    #---- check for argument in the terminal, if no then use the var up there ----#
    if len(sys.argv) > 1:
        src = sys.argv[1]
    else:
        src = INPUT_PATH

    if len(sys.argv) > 2:
        out_dir = sys.argv[2]
    else:
        out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    #-----------------------------------------------------------------------------#

    if os.path.isdir(src):
        bin_files = sorted(set(glob.glob(os.path.join(src, "*.BIN"))) |
                            set(glob.glob(os.path.join(src, "*.bin"))))
        if not bin_files:
            print(f"No .BIN files found in {src}")
            sys.exit(1)

        print(f"Converting {len(bin_files)} file(s) from {src} -> {out_dir}/")
        ok = 0
        for path in bin_files:
            if start_convert_one(path, out_dir):
                ok += 1
        print(f"Done: {ok}/{len(bin_files)} converted")
    else:
        return start_convert_one(src, out_dir)


if __name__ == "__main__":
    cli_start_convert_bin(None, None)
