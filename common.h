
#ifndef _COMMON_H_
#define _COMMON_H_

#define CONFIG_FILE		"eyebot.conf"

/**
 * Default name of the I2C bus on Raspberry Pi Rev. 2
 */	
#define I2C_BUS 		"/dev/i2c-1"

/**
 * Max retries for I2C calls
 */
#define I2C_MAX_RETRIES		10

/**
 * I2C address for the motor controller
 */
#define MOTOR_CTRL_ADDR		0x23

#define MOTOR_LEFT 0x10
#define MOTOR_RIGHT 0x20

/**
 * "Set speed" command
 */
#define MOTOR_CTRL_CMD_GET_SPEED	0x10

/**
 * "Get tachometer speed"
 */
#define MOTOR_CTRL_CMD_SET_SPEED	0x20


#define MOTOR_CTRL_CMD_BRAKE		0x30

#define MOTOR_CTRL_CMD_SET_STATE	0x50

#define MOTOR_CTRL_CMD_GOTO_POS		0x60


#define IMAGE_WIDTH 		320
#define IMAGE_HEIGHT		240
#define IMAGE_PIXELS		(IMAGE_WIDTH * IMAGE_HEIGHT)

// Macro to allocate memory for a single image frame buffer
#define ALLOC_IMAGE() ((unsigned char *) malloc(IMAGE_PIXELS))

#endif

