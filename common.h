
#ifndef _COMMON_H_
#define _COMMON_H_

#define CONFIG_FILE			"eyebot.conf"

#define IMAGE_WIDTH 		320
#define IMAGE_HEIGHT		240
#define IMAGE_PIXELS		(IMAGE_WIDTH * IMAGE_HEIGHT)

#define PI 						3.14159265


#define INDEX(y)				( y * WIDTH )
#define INDEX2(x,y)				( y * WIDTH + x )


/**
 * Image size
 */
#define WIDTH 					320
#define HEIGHT 					240
#define IMG_SIZE 				(WIDTH * HEIGHT)

/** 
 * Image processing
 */
#define FLOOR					255
#define LINE					0

#endif

