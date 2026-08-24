#!/usr/bin/env python3
"""Compress a greyscale bit-plane animation with temporal-delta + RLE.

Input : a generated `animation_data[F][COLOR_DEPTH][Y_RES]` .c file (bit-plane format).
Output: a .c file holding a compact byte stream (`anim_rle[]`) that the firmware
        decodes one frame at a time into a RAM buffer.

Format
------
Each frame is COLOR_DEPTH*Y_RES = 60 uint32 "words" (plane-major: idx = plane*Y_RES + row).
We XOR each frame against the previous one (the frame before frame 0 is all-zero), giving
a 60-word delta that is mostly zero for video. The delta is then run-length encoded:

  control byte:
    bit7 = 0 -> ZERO RUN  : skip (count) unchanged words      (count = bits6..0 + 1, 1..128)
    bit7 = 1 -> LITERAL   : (count) changed words follow, each as 3 little-endian bytes
                            (24 bits; LED data only uses bits 0..18, so 3 bytes is lossless)

A frame ends when 60 words have been accounted for (no explicit delimiter needed).
Decoding is sequential; on loop the firmware re-zeroes the buffer and restarts at byte 0.
"""
import re
import sys

COLOR_DEPTH = 4
Y_RES = 20
WORDS_PER_FRAME = COLOR_DEPTH * Y_RES  # 80
MAX_RUN = 128


def parse_frames(path):
    with open(path, "r") as f:
        text = f.read()
    brace = text.index("{", text.index("animation_data"))
    body = text[brace + 1:text.rindex("}")]
    vals = [int(v, 16) for v in re.findall(r"0x[0-9A-Fa-f]+", body)]
    if len(vals) % WORDS_PER_FRAME != 0:
        raise SystemExit(f"{len(vals)} words not a multiple of {WORDS_PER_FRAME}")
    frames = [vals[i:i + WORDS_PER_FRAME] for i in range(0, len(vals), WORDS_PER_FRAME)]
    return frames


def encode(frames):
    out = bytearray()
    prev = [0] * WORDS_PER_FRAME
    for fr in frames:
        delta = [fr[i] ^ prev[i] for i in range(WORDS_PER_FRAME)]
        for d in delta:
            if d > 0xFFFFFF:
                raise SystemExit("delta word exceeds 24 bits; widen literal encoding")
        i = 0
        while i < WORDS_PER_FRAME:
            if delta[i] == 0:
                j = i
                while j < WORDS_PER_FRAME and delta[j] == 0 and (j - i) < MAX_RUN:
                    j += 1
                out.append((j - i) - 1)            # bit7=0 zero-run
                i = j
            else:
                j = i
                while j < WORDS_PER_FRAME and delta[j] != 0 and (j - i) < MAX_RUN:
                    j += 1
                out.append(0x80 | ((j - i) - 1))   # bit7=1 literal-run
                for k in range(i, j):
                    d = delta[k]
                    out.append(d & 0xFF)
                    out.append((d >> 8) & 0xFF)
                    out.append((d >> 16) & 0xFF)
                i = j
        prev = fr
    return out


def write_c(out_path, blob, frame_count, src):
    with open(out_path, "w") as f:
        f.write(f"/* Delta+RLE compressed animation (from {src}). */\n")
        f.write("/* Decoded one frame at a time by the firmware; see GREYSCALE.md. */\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define ANIMATION_FRAMES {frame_count}\n")
        f.write(f"#define COLOR_DEPTH {COLOR_DEPTH}\n")
        f.write(f"#define Y_RES {Y_RES}\n\n")
        f.write(f"const uint32_t anim_rle_len = {len(blob)};\n")
        f.write("const uint8_t anim_rle[] = {\n")
        for i in range(0, len(blob), 20):
            f.write("\t" + "".join(f"0x{b:02X}," for b in blob[i:i + 20]) + "\n")
        f.write("};\n")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]
    frames = parse_frames(in_path)
    blob = encode(frames)

    raw = len(frames) * WORDS_PER_FRAME * 4
    print(f"frames           : {len(frames)}")
    print(f"raw (uint32)     : {raw} bytes")
    print(f"compressed       : {len(blob)} bytes")
    print(f"ratio            : {raw / len(blob):.2f}x")
    print(f"bytes/frame      : {len(blob) / len(frames):.1f}  (raw 240)")
    write_c(out_path, blob, len(frames), in_path)
    print(f"wrote            : {out_path}")


if __name__ == "__main__":
    main()
