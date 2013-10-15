
#ifndef _COMMON_H_
#define _COMMON_H_

#define INITIAL_SPEED		28

#define I2C_MAX_RETRIES		3

// I2C address for the motor controller
#define MOTOR_CTRL_ADDR		0x23

#define MOTOR_CTRL_CMD_GET_SPEED	0x10
#define MOTOR_CTRL_CMD_SET_SPEED	0x20

#define IMAGE_WIDTH 		320
#define IMAGE_HEIGHT		240
#define IMAGE_PIXELS		(IMAGE_WIDTH * IMAGE_HEIGHT)

// Macro to allocate memory for a single image frame buffer
#define ALLOC_IMAGE() ((unsigned char *) malloc(IMAGE_PIXELS))

#endif

