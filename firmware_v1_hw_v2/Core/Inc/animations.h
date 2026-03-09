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
#define ANIMATION_FRAMES  181  // Example
#define Y_RES             20
#define X_RES             19

// "extern" means: "This variable exists in another .c file, don't create it here, just link to it."
extern const uint32_t animation_data[ANIMATION_FRAMES][Y_RES];

#endif /* INC_ANIMATIONS_H_ */
