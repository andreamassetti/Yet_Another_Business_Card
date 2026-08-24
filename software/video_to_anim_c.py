#!/usr/bin/env python3
"""Headless: video/GIF -> compressed (delta+RLE) animation .c for the LED matrix.

Does the whole pipeline in one step (no GUI): decode -> grayscale -> resize to the
matrix -> tone-map/quantize to 2^depth levels -> bit planes -> temporal delta + RLE.
Output is a ready-to-compile `anim_rle[]` file (same format the firmware decodes).

Example:
  python video_to_anim_c.py "C:/path/clip.mp4" ../firmware_v2_hw_v2/Core/Src/my_anim_4_1_out.c --depth 4
"""
import argparse
import sys
import cv2
import numpy as np

TARGET_W, TARGET_H = 19, 20
MAX_RUN = 128


def quantize_to_levels(gray, levels, black, white, gamma, contrast):
    g = gray.astype(np.float32)
    norm = np.clip((g - black) / max(1, (white - black)), 0.0, 1.0)
    if gamma != 1.0:
        norm = norm ** gamma
    if contrast > 1.0:
        s = np.tanh(contrast * (norm - 0.5)) / np.tanh(contrast * 0.5)
        norm = np.clip(0.5 * (s + 1.0), 0.0, 1.0)
    return np.round(norm * (levels - 1)).astype(np.uint8)


def frame_to_words(levels_img, depth):
    """Return COLOR_DEPTH*Y_RES bit-plane column masks (plane-major)."""
    words = []
    for p in range(depth):
        for y in range(TARGET_H):
            row = 0
            for x in range(TARGET_W):
                if (int(levels_img[y, x]) >> p) & 1:
                    row |= (1 << x)
            words.append(row)
    return words


def encode_delta_rle(frames, words_per_frame):
    out = bytearray()
    prev = [0] * words_per_frame
    for fr in frames:
        delta = [fr[i] ^ prev[i] for i in range(words_per_frame)]
        i = 0
        while i < words_per_frame:
            if delta[i] == 0:
                j = i
                while j < words_per_frame and delta[j] == 0 and (j - i) < MAX_RUN:
                    j += 1
                out.append((j - i) - 1)
                i = j
            else:
                j = i
                while j < words_per_frame and delta[j] != 0 and (j - i) < MAX_RUN:
                    j += 1
                out.append(0x80 | ((j - i) - 1))
                for k in range(i, j):
                    d = delta[k]
                    if d > 0xFFFFFF:
                        raise SystemExit("delta word > 24 bits; widen literal encoding")
                    out += bytes((d & 0xFF, (d >> 8) & 0xFF, (d >> 16) & 0xFF))
                i = j
        prev = fr
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("out_c")
    ap.add_argument("--depth", type=int, default=4, help="bit planes (4 -> 16 levels)")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--black", type=int, default=40)
    ap.add_argument("--white", type=int, default=235)
    ap.add_argument("--gamma", type=float, default=2.2)
    ap.add_argument("--contrast", type=float, default=1.8)
    a = ap.parse_args()

    levels = 1 << a.depth
    wpf = a.depth * TARGET_H
    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        sys.exit(f"could not open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS)

    frames = []
    while True:
        if a.max_frames is not None and len(frames) >= a.max_frames:
            break
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        q = quantize_to_levels(resized, levels, a.black, a.white, a.gamma, a.contrast)
        frames.append(frame_to_words(q, a.depth))
    cap.release()

    blob = encode_delta_rle(frames, wpf)
    raw = len(frames) * wpf * 4
    with open(a.out_c, "w") as f:
        f.write(f"/* Delta+RLE compressed animation (from {a.video}). */\n")
        f.write(f"/* {levels}-level greyscale, {a.depth} bit planes. Decoded per-frame by the firmware. */\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define ANIMATION_FRAMES {len(frames)}\n")
        f.write(f"#define COLOR_DEPTH {a.depth}\n")
        f.write(f"#define Y_RES {TARGET_H}\n\n")
        f.write(f"const uint32_t anim_rle_len = {len(blob)};\n")
        f.write("const uint8_t anim_rle[] = {\n")
        for i in range(0, len(blob), 20):
            f.write("\t" + "".join(f"0x{b:02X}," for b in blob[i:i + 20]) + "\n")
        f.write("};\n")

    print(f"frames        : {len(frames)}   source fps: {fps:.3f}  (set ANIM_FPS = {round(fps)})")
    print(f"levels        : {levels} ({a.depth} planes)")
    print(f"raw           : {raw} bytes")
    print(f"compressed    : {len(blob)} bytes  ({raw/len(blob):.2f}x, {len(blob)/len(frames):.1f} B/frame)")
    print(f"set ANIMATION_FRAMES = {len(frames)} in animations.h")
    print(f"wrote         : {a.out_c}")


if __name__ == "__main__":
    main()
