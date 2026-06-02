#!/usr/bin/env python3
"""
One-shot converter: takes an existing 1-bit-per-pixel animation_data .c file
(format: animation_data[FRAMES][Y_RES], each value a column bitmask) and rewrites
it in the new 8-level greyscale BIT-PLANE format expected by the v2/hw_v2 firmware:

    animation_data[FRAMES][COLOR_DEPTH][Y_RES]

Because the source is only on/off, every lit pixel is emitted at full brightness
(level 7) -> its column bit is set on ALL COLOR_DEPTH planes. The result looks
identical to the original on hardware but now exercises the greyscale engine, so
you can confirm the rewrite before feeding it true greyscale data from the
updated video converter.

Usage:  python convert_1bit_to_greyscale_planes.py input.c output.c
"""
import re
import sys

COLOR_DEPTH = 3   # must match animations.h
Y_RES = 20        # must match animations.h


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]

    with open(in_path, "r") as f:
        text = f.read()

    # Pull the body of the array initializer (between the first '{' after '=' and final '}')
    eq = text.index("animation_data")
    brace_start = text.index("{", eq)
    body = text[brace_start + 1:text.rindex("}")]

    # Grab every hex literal in order; they are the row masks, 20 per frame.
    values = [int(v, 16) for v in re.findall(r"0x[0-9A-Fa-f]+", body)]
    if len(values) % Y_RES != 0:
        print(f"WARNING: {len(values)} values not a multiple of Y_RES={Y_RES}")
    frames = len(values) // Y_RES
    print(f"Parsed {frames} frames x {Y_RES} rows")

    with open(out_path, "w") as f:
        f.write("/* Greyscale bit-plane data (auto-converted from 1-bit silhouette). */\n")
        f.write("/* Full brightness: each lit pixel is set on all bit planes (level 7). */\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define ANIMATION_FRAMES {frames}\n")
        f.write(f"#define COLOR_DEPTH {COLOR_DEPTH}\n")
        f.write(f"#define Y_RES {Y_RES}\n\n")
        f.write(f"const uint32_t animation_data[{frames}][{COLOR_DEPTH}][{Y_RES}] = {{\n")
        for fr in range(frames):
            rows = values[fr * Y_RES:(fr + 1) * Y_RES]
            f.write(f"\t{{ /* Frame {fr} */\n")
            for p in range(COLOR_DEPTH):
                f.write("\t\t{ ")
                # full brightness -> mask identical on every plane
                f.write(", ".join(f"0x{r:08X}" for r in rows))
                f.write(" },\n")
            f.write("\t},\n")
        f.write("};\n")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
