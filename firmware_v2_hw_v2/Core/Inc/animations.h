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
#define ANIMATION_FRAMES  400  // Must match the count in the generated data .c file
#define COLOR_DEPTH       3    // Greyscale bit planes -> 2^3 = 8 brightness levels (0..7)
#define Y_RES             20
#define X_RES             19

// Greyscale data is stored as BIT PLANES (Binary Code Modulation).
//   animation_data[frame][plane][row]  == column mask of pixels lit during that plane.
// A pixel's 3-bit brightness B is spread across the planes: plane p is set when bit p of B is 1.
// Plane 0 is the LSB (shortest on-time), plane COLOR_DEPTH-1 is the MSB (longest on-time).
// "extern" means: "This variable exists in another .c file, don't create it here, just link to it."
extern const uint32_t animation_data[ANIMATION_FRAMES][COLOR_DEPTH][Y_RES];

#endif /* INC_ANIMATIONS_H_ */
