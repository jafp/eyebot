
#ifndef _COMMON_H_
#define _COMMON_H_

#define IMAGE_WIDTH 		320
#define IMAGE_HEIGHT		240
#define IMAGE_PIXELS		(IMAGE_WIDTH * IMAGE_HEIGHT)

// Macro to allocate memory for a single image frame buffer
#define ALLOC_IMAGE() ((unsigned char *) malloc(IMAGE_PIXELS))

#endif

