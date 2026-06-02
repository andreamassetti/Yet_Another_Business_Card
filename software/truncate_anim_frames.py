#!/usr/bin/env python3
"""Truncate a greyscale bit-plane animation_data .c file to the first N frames.

Reuses the existing values; no video re-export needed. Rewrites the array
dimension and the local ANIMATION_FRAMES #define to match.

Usage:  python truncate_anim_frames.py <file.c> <N> [out.c]
        (if out.c omitted, the input file is overwritten in place)
"""
import re
import sys

COLOR_DEPTH = 3
Y_RES = 20
PER_FRAME = COLOR_DEPTH * Y_RES  # hex values per frame


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    in_path = sys.argv[1]
    n = int(sys.argv[2])
    out_path = sys.argv[3] if len(sys.argv) > 3 else in_path

    with open(in_path, "r") as f:
        text = f.read()

    brace_start = text.index("{", text.index("animation_data"))
    body = text[brace_start + 1:text.rindex("}")]
    values = [int(v, 16) for v in re.findall(r"0x[0-9A-Fa-f]+", body)]

    total_frames = len(values) // PER_FRAME
    if n > total_frames:
        print(f"Requested {n} > available {total_frames}; clamping to {total_frames}")
        n = total_frames
    values = values[:n * PER_FRAME]

    with open(out_path, "w") as f:
        f.write(f"/* Truncated to first {n} frames (was {total_frames}). */\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define ANIMATION_FRAMES {n}\n")
        f.write(f"#define COLOR_DEPTH {COLOR_DEPTH}\n")
        f.write(f"#define Y_RES {Y_RES}\n\n")
        f.write(f"const uint32_t animation_data[{n}][{COLOR_DEPTH}][{Y_RES}] = {{\n")
        idx = 0
        for fr in range(n):
            f.write(f"\t{{ /* Frame {fr} */\n")
            for _ in range(COLOR_DEPTH):
                row = values[idx:idx + Y_RES]
                idx += Y_RES
                f.write("\t\t{ " + ", ".join(f"0x{v:08X}" for v in row) + " },\n")
            f.write("\t},\n")
        f.write("};\n")

    print(f"Wrote {n} frames to {out_path} "
          f"({n * PER_FRAME * 4} bytes of flash for the data array)")


if __name__ == "__main__":
    main()
