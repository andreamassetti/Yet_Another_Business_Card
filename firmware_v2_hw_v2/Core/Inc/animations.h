/*
 * animations.h
 *
 *  Created on: Nov 21, 2025
 *      Author: andre
 */

#ifndef INC_ANIMATIONS_H_
#define INC_ANIMATIONS_H_

#include <stdint.h>

// Dimensions
#define ANIMATION_FRAMES  515  // Must match the count in the generated data .c file
#define COLOR_DEPTH       4    // Greyscale bit planes -> 2^4 = 16 brightness levels (0..15)
#define Y_RES             20
#define X_RES             19

// Frames are stored COMPRESSED (temporal delta + RLE) in `anim_rle`, and decoded one
// frame at a time into a RAM buffer by the firmware. See encode_delta_rle.py / GREYSCALE.md.
// Each frame is COLOR_DEPTH*Y_RES = 60 uint32 words (plane-major: idx = plane*Y_RES + row),
// where each word is a greyscale bit-plane column mask (Binary Code Modulation).
extern const uint8_t  anim_rle[];     // compressed byte stream
extern const uint32_t anim_rle_len;   // length of anim_rle in bytes

#endif /* INC_ANIMATIONS_H_ */
