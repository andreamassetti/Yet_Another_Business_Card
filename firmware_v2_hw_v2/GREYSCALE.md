# 8-Level Greyscale (Binary Code Modulation)

This firmware drives the 19x18 charlieplexed matrix with **8 brightness levels per
pixel** using Binary Code Modulation (BCM). Each pixel's 3-bit brightness (0..7) is
split across `COLOR_DEPTH = 3` bit planes; plane `p` is shown for a time proportional
to its weight (`2^p`), so the eye integrates the planes into a grey level.

## How it works

- **Data format** (`Core/Inc/animations.h`):
  `animation_data[frame][plane][row]` — each entry is a column bitmask of the pixels
  lit during that plane. Plane 0 = LSB (shortest on-time), plane 2 = MSB (longest).
- **Render** (`Core/Src/main.c`):
  - `PrecomputeFrame()` builds `frameBufferLUT[row][plane]` (GPIO register states).
  - The TIM1 ISR walks every `(row, plane)` and holds each for `bitDuration[plane]`.
  - `initBitDurationLUT()` sets `bitDuration[i] = 10 * (1 << i)` -> planes last
    125 / 250 / 500 us. A full 3-plane frame is ~16.6 ms => **~60 Hz** (no flicker).
- **Game mode** (Snake) renders at full brightness: every lit pixel is set on all planes.

## Files

| File | Role |
|------|------|
| `Core/Inc/animations.h` | Data contract: `ANIMATION_FRAMES`, `COLOR_DEPTH`, `Y_RES`, array decl |
| `Core/Src/main.c` | BCM engine: `PrecomputeFrame`, TIM1 ISR, `initBitDurationLUT` |
| `Core/Src/strato_sliding_2.c` | Active animation data (the file the build compiles) |
| `Core/Src/strato_sliding_2.c.1bit.bak` | Original 1-bit silhouette data, kept for reference |
| `../software/video_convertion_v2.py` | Generates greyscale bit-plane data from a video/GIF |
| `../software/convert_1bit_to_greyscale_planes.py` | One-shot: old 1-bit data -> bit-plane format (full brightness) |

## Steps to use

### 1. Flash what's there now (sanity check)
The bundled data was converted from the old 1-bit silhouette to the new format at full
brightness, so it builds and runs identically — this confirms the engine before you
feed it real greyscale.

1. STM32CubeIDE -> open `firmware_v2_hw_v2`.
2. **Project -> Clean...**, then **Build** (the data array tripled in size; a clean
   rebuild avoids stale objects). Confirm it links without a flash-overflow error
   (~54 KB data of 128 KB).
3. Plug in the board, **Run** to flash.
4. Power-cycle: the animation should play at a steady ~60 Hz, no flicker.

### 2. Generate real greyscale data
1. In `software/`:
   ```
   python video_convertion_v2.py
   ```
   (needs `opencv-python`, `numpy`, `pillow`)
2. **Load Video/GIF** and pick a clip.
3. Scrub frames; adjust the **Black-level cutoff** slider so the background stays off
   but the subject keeps its shades. The right-hand preview shows the true 8-level
   result.
4. **GENERATE C CODE** and save (e.g. `my_anim.c`).

### 3. Put the new data into the build
The build compiles `Core/Src/strato_sliding_2.c` as the animation source. Either:
- **Easiest:** overwrite `strato_sliding_2.c` with your generated file's contents
  (keep the filename), **or**
- **Cleaner:** add `my_anim.c` to `Core/Src/`, then exclude the old file:
  right-click `strato_sliding_2.c` -> **Resource Configurations -> Exclude from Build**.
  Only one file may define `animation_data`.

Then check `Core/Inc/animations.h`:
- `ANIMATION_FRAMES` must equal the frame count at the top of your generated file
  (otherwise the animation reads past the end or stops short).
- `COLOR_DEPTH 3` and `Y_RES 20` already match the generator output.

### 4. Rebuild & flash
**Project -> Clean -> Build -> Run.** You should now see real 8-level shading.

## Flash budget (IMPORTANT)

Greyscale costs **3x** the old 1-bit data: each frame = `COLOR_DEPTH * Y_RES * 4` =
**240 bytes**. With ~13 KB of code/HAL:

| Flash | Data budget | Max greyscale frames |
|-------|-------------|----------------------|
| 64 KB  | ~52 KB  | ~200 |
| 128 KB | ~118 KB | ~450 |

The chip is an STM32F103**C8** (rated 64 KB), but the firmware is built with
`-DSTM32F103xB` and most C8 dies physically contain 128 KB. The linker script
`STM32F103C8TX_FLASH.ld` has been set to `FLASH ... LENGTH = 128K` to use it.

- If a build/flash **succeeds and runs**, the extra flash is really there.
- If it **fails to flash past 64 KB or hangs**, this die only has 64 KB: set the
  linker `LENGTH` back to `64K` and keep the animation under ~200 frames.

Cap the export with `MAX_FRAMES` at the top of `video_convertion_v2.py`
(default 400, safe for 128 KB).

## Playback speed

Playback rate is set by `#define ANIM_FPS` in `main.c`. **Set it to the FPS of the
source video** so the animation runs at the original speed. TIM2 (`MX_TIM2_Init`) is
configured from it: 40 kHz tick, `Period = 40000/ANIM_FPS - 1`, advancing one frame per
interrupt. (Don't reintroduce the old "first 150 frames slower" divider in the TIM2
callback -- that was specific to the original strato animation.)

Find a video's FPS with:
`python -c "import cv2;c=cv2.VideoCapture('clip.mp4');print(c.get(cv2.CAP_PROP_FPS))"`

## Tuning

- **Shading too dark / washed out:** edit `GAMMA` at the top of `video_convertion_v2.py`
  (2.2 = default; lower = brighter mid-tones) and regenerate.
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
