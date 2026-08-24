# Greyscale (Binary Code Modulation)

This firmware drives the 19x18 charlieplexed matrix with **16 brightness levels per
pixel** (`COLOR_DEPTH = 4`) using Binary Code Modulation (BCM). Each pixel's brightness
is split across `COLOR_DEPTH` bit planes; plane `p` is shown for a time proportional to
its weight (`2^p`), so the eye integrates the planes into a grey level.

> Depth is set by one constant (`COLOR_DEPTH` in `animations.h`); the firmware is fully
> parameterized. **Cost of depth:** total display time per frame scales as `2^D - 1`, so
> each extra plane ~halves the refresh rate. To keep ~60 Hz at `D=4` the TIM1 interrupt
> uses a **lean bare-metal ISR** (see below) and a shorter LSB tick (`base=5`).

## How it works

- **Pixel format:** each frame is `COLOR_DEPTH=3` bit planes x `Y_RES=20` rows = 60
  uint32 words (plane-major: `idx = plane*Y_RES + row`). Each word is a column bitmask
  of the pixels lit during that plane. Plane 0 = LSB (shortest on-time), plane 2 = MSB.
- **Storage:** frames are **compressed** (temporal delta + RLE) into `anim_rle[]` and
  decoded one frame at a time into the RAM buffer `framebuf[COLOR_DEPTH][Y_RES]`.
  See the "Compression" section below.
- **Render** (`Core/Src/main.c`):
  - Each frame tick the main loop calls `DecodeNextFrame()` (fills `framebuf`) then
    `PrecomputeFrame()` (builds the GPIO register states). `frameBufferLUT` is
    **double-buffered**: PrecomputeFrame fills the back copy and publishes it with one
    atomic pointer write (`displayBuf`), so the ISR never shows a half-built frame (no tearing).
  - `TIM1_DisplayUpdate()` walks every `(row, plane)` and holds each for `bitDuration[plane]`.
    It runs from a **lean bare-metal `TIM1_UP_IRQHandler`** (`stm32f1xx_it.c`) that bypasses
    `HAL_TIM_IRQHandler` -- per-interrupt overhead is a few us instead of ~15-25 us at 4 MHz.
    That low overhead keeps the short LSB plane linear and frees the time budget for 4 planes.
  - `initBitDurationLUT()` uses `base=5` at `D=4` -> planes last 62.5/125/250/500 us;
    a full 4-plane frame is ~17.8 ms => **~56 Hz** (no visible flicker). (`base=10` at `D=3`.)
- **Game mode** (Snake) renders at full brightness: every lit pixel is set on all planes.

## Compression (temporal delta + RLE)

Consecutive video frames barely differ, so each frame is stored as the XOR difference
from the previous frame (frame -1 is all-zero), then run-length encoded. The decoder is
sequential: `DecodeNextFrame()` advances `framebuf` by one frame; on loop it clears the
buffer and rewinds to byte 0. Cost: ~240 B RAM + a tiny decoder; produces 60 words per
33 ms tick (trivial at 4 MHz). Random seek is not supported (fine for a looping player).

- **Byte stream format** (`anim_rle[]`): a sequence of control bytes.
  - `bit7=0`: ZERO RUN of `(low7 + 1)` unchanged words (1..128).
  - `bit7=1`: LITERAL RUN of `(low7 + 1)` changed words; each follows as 3 little-endian
    bytes (24-bit; LED data uses only bits 0..18, so 3 bytes is lossless).
- **Measured:** the 796-frame (26.5 s @ 30 fps) clip = 191 KB raw -> **64 KB (2.97x)**.
- Roundtrip is verified losslessly by the encoder; the C decoder mirrors it exactly.

## Files

| File | Role |
|------|------|
| `Core/Inc/animations.h` | Data contract: `ANIMATION_FRAMES`, `COLOR_DEPTH`, `Y_RES`, `anim_rle` decl |
| `Core/Src/main.c` | BCM engine + `DecodeNextFrame`/`DecodeReset`, `PrecomputeFrame`, TIM1 ISR |
| `Core/Src/my_anim_1.c` | Active animation data: compressed `anim_rle[]` (the file the build compiles) |
| `Core/Src/my_anim_1.c.796.bak` | Uncompressed 796-frame bit-plane data, kept for re-encoding |
| `../software/video_to_anim_c.py` | **Headless one-step**: video -> compressed `anim_rle[]` (no GUI). Recommended. |
| `../software/video_convertion_v2.py` | GUI: generates greyscale bit-plane data + live tone-map preview |
| `../software/encode_delta_rle.py` | Compresses a bit-plane `.c` -> delta+RLE `anim_rle[]` |
| `../software/truncate_anim_frames.py` | One-shot: trim a bit-plane `.c` to the first N frames |
| `../software/convert_1bit_to_greyscale_planes.py` | One-shot: old 1-bit data -> bit-plane format |

## Steps to use

**Shortcut (no GUI):** do steps 1+2 in one command --
`python video_to_anim_c.py clip.mp4 ../firmware_v2_hw_v2/Core/Src/my_anim_4_1_out.c --depth 4`
It prints the frame count, compression ratio, and the `ANIM_FPS` to use. Then jump to step 3.
Use the GUI flow below instead when you want to tune contrast/cutoff against a live preview.

### 1. Generate greyscale bit-plane data from a video
1. In `software/`:
   ```
   python video_convertion_v2.py
   ```
   (needs `opencv-python`, `numpy`, `pillow`)
2. **Load Video/GIF** and pick a clip.
3. Scrub frames; adjust the **Black-level cutoff** slider so the background stays off
   but the subject keeps its shades. The right-hand preview shows the true 8-level result.
4. **GENERATE C CODE** and save (e.g. `my_anim_raw.c`). This is the *uncompressed*
   bit-plane file. Note the frame count printed at the top.

### 2. Compress it (delta + RLE)
```
python encode_delta_rle.py my_anim_raw.c my_anim_1.c
```
This prints the ratio and writes the compressed `anim_rle[]`. Because it's compressed,
you can usually keep the **whole** clip (the 796-frame example fits in 64 KB).

### 3. Put it into the build
The build compiles `Core/Src/my_anim_1.c` as the animation source. Either overwrite that
file with your compressed output (keep the name), or add a new file and exclude the old
one (right-click -> **Resource Configurations -> Exclude from Build**). Exactly one file
may define `anim_rle[]`.

Then set in `Core/Inc/animations.h`:
- `ANIMATION_FRAMES` = the frame count from step 1 (must match, or playback loops early/late).
- `COLOR_DEPTH 3` and `Y_RES 20` already match.

Also set `#define ANIM_FPS` in `main.c` to your video's frame rate (see Playback speed).

### 4. Build & flash
**Project -> Clean -> Build -> Run.** You should see real 8-level shading at the right speed.

## Flash budget (IMPORTANT)

Raw greyscale is **240 bytes/frame** (`COLOR_DEPTH * Y_RES * 4`), 3x the old 1-bit data.
But frames are stored **compressed** (delta+RLE, ~3x on video -> ~80 bytes/frame), so the
effective capacity is roughly tripled. With ~13 KB of code/HAL:

| Flash | Data budget | Frames (raw 240 B) | Frames (compressed ~80 B) |
|-------|-------------|--------------------|---------------------------|
| 64 KB  | ~52 KB  | ~200 | ~650 |
| 128 KB | ~118 KB | ~450 | ~1500 |

(Compression ratio is content-dependent — `encode_delta_rle.py` prints the real number.)

The chip is an STM32F103**C8** (rated 64 KB), but the firmware is built with
`-DSTM32F103xB` and most C8 dies physically contain 128 KB. The linker script
`STM32F103C8TX_FLASH.ld` has been set to `FLASH ... LENGTH = 128K` to use it.

- If a build/flash **succeeds and runs**, the extra flash is really there.
- If it **fails to flash past 64 KB or hangs**, this die only has 64 KB: set the
  linker `LENGTH` back to `64K`. The current 64 KB compressed clip still won't fit
  under 64 KB alongside code, so you'd shorten the clip and re-encode.

To trim before compressing, cap the export with `MAX_FRAMES` at the top of
`video_convertion_v2.py`, or run `truncate_anim_frames.py` on the raw bit-plane file.

## Playback speed

Playback rate is set by `#define ANIM_FPS` in `main.c`. **Set it to the FPS of the
source video** so the animation runs at the original speed. TIM2 (`MX_TIM2_Init`) is
configured from it: 40 kHz tick, `Period = 40000/ANIM_FPS - 1`, advancing one frame per
interrupt. (Don't reintroduce the old "first 150 frames slower" divider in the TIM2
callback -- that was specific to the original strato animation.)

Find a video's FPS with:
`python -c "import cv2;c=cv2.VideoCapture('clip.mp4');print(c.get(cv2.CAP_PROP_FPS))"`

## Sleep / power

After playback the board enters STANDBY (woken by the PA0 button = reboot). Policy in
`main.c`'s main loop:
- **Animation mode:** plays one full playthrough, then shows a filled 6x6 heart in the
  bottom-right for ~2 s (`HEART_HOLD_TICKS`) that pulses between half and full brightness
  (`HEART_PULSE_PERIOD`, `HEART_MIN/MAX_LEVEL`; see `RenderHeart`), then sleeps. Driven by
  `animation_complete` -> heart -> `animation_done_shown`, not a timer.
- **Snake mode:** during play, sleeps after 10 s of no button activity. After a game over,
  blinks (`GAME_OVER_BLINK_MS`) then enters standby ~2 s after death (`GAME_OVER_SLEEP_MS`),
  so the next button press wakes back into the video. Presses after death are ignored (no
  in-game restart); to play Snake again, wake into the video and press a button.

If you want the animation to **loop forever** instead, remove the animation branch from
the sleep policy (or just don't set `animation_done_shown`).

## Tuning

- **Brightness depth (`COLOR_DEPTH` in `animations.h`):** 3 = 8 levels, 4 = 16 levels.
  Display time per frame scales as `2^D - 1`, so each added plane ~halves refresh. The lean
  TIM1 ISR + `base` in `initBitDurationLUT()` buy that time back; `D=4` runs ~56 Hz. `D=5`
  (32 levels) is not viable at 4 MHz (LSB would be ~25 us). Changing depth requires
  regenerating data at the same depth (`--depth`) and re-flashing.
- **Contrast / appearance on the matrix** — tone mapping lives in `video_convertion_v2.py`
  (`quantize_to_levels`), applied per frame as: levels stretch -> gamma -> S-curve -> quantize.
  - **Contrast slider** (GUI): S-curve strength around mid-grey (1.0 = off, higher = punchier).
  - **Black-level cutoff slider** = `BLACK_POINT`: input at/below it -> off. Raise to kill background.
  - `WHITE_POINT` constant: input at/above it -> full brightness. **Lower it to brighten** and
    map the brightest *useful* tone to level 7 (the biggest contrast lever for only 8 levels).
  - `GAMMA` constant: >1 darkens mid-tones, <1 brightens them. If shadows crush, move toward 1.0.
  Tune with the live preview, then regenerate + re-encode (`encode_delta_rle.py`).
- **Flicker:** lower the base in `initBitDurationLUT()` (`main.c`). This also shortens
  the dim planes, so there is a practical floor (~60 Hz is already comfortable).
- **Frame stutter:** `PrecomputeFrame()` now does 1.5x the work (3 planes) and the
  project builds at `-O0`. It runs ~47x/s in the main loop and should keep up at 4 MHz;
  if needed, raise optimization to `-O2` for that file.
- **LSB linearity:** at 125 us the fixed ISR blank-and-swap overhead is a slightly larger
  fraction of the dimmest plane, so level 1 can read marginally brighter than a perfect
  1/7. The gamma curve compensates in practice.

## Notes / gotchas

- The matrix is 19 rows x 18 columns; the data arrays use `Y_RES = 20` rows (row 19 unused)
  and column bits 0..17.
- The data `.c` file carries its own local `#define ANIMATION_FRAMES/COLOR_DEPTH/Y_RES`
  for readability; `main.c` uses the values from `animations.h`, so keep the header in sync.
- Several other animation `.c` files exist in `Core/Src/`, but only the one listed in
  `Debug/Core/Src/subdir.mk` (`strato_sliding_2.c`) is actually compiled.
