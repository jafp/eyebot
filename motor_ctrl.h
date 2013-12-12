
#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include <stdint.h>

#define MOTOR_CTRL_ADDR				0x23

#define MOTOR_LEFT 					0x10
#define MOTOR_RIGHT 				0x20

#define MOTOR_CTRL_CMD_GET_SPEED	0x10
#define MOTOR_CTRL_CMD_SET_SPEED	0x20
#define MOTOR_CTRL_CMD_BRAKE		0x30
#define MOTOR_CTRL_CMD_SET_STATE	0x50
#define MOTOR_CTRL_CMD_GOTO_POS		0x60
#define MOTOR_CTRL_CMD_DIST_READ	0x80
#define MOTOR_CTRL_CMD_DIST_EN		0x90
#define MOTOR_CTRL_CMD_IS_STABLE	0xA0

#define STATE_POSITION 				0x01
#define STATE_STRAIGHT				0x02
#define STATE_SPEED					0x03

#define DIR_LEFT_BRAKE				0x00
#define DIR_LEFT_REVERSE			0x01
#define DIR_LEFT_FORWARD 			0x02

#define DIR_RIGHT_BRAKE				0x00
#define DIR_RIGHT_REVERSE			0x10
#define DIR_RIGHT_FORWARD 			0x20

#define DIST_SENSOR_NONE			0x00
#define DIST_SENSOR_FRONT			0x01
#define DIST_SENSOR_SIDE			0x02

typedef struct {
	uint8_t front_val, side_1_val, side_2_val;
	float front, side_1, side_2;
} dist_readings_t;

int motor_ctrl_init();
int motor_ctrl_brake();
int motor_ctrl_forward();
int motor_ctrl_set_dir(unsigned char dir_mask);
int motor_ctrl_get_speed(unsigned char * left, unsigned char * right);
int motor_ctrl_set_speed(unsigned char left, unsigned char right);
int motor_ctrl_set_state(unsigned char state);
int motor_ctrl_goto_position(unsigned int pos_l, unsigned int pos_r);
int motor_ctrl_wait(int additional_delay_ms);

float get_dist_to_cm(unsigned char dist);
dist_readings_t dist_read_all();
int dist_read(unsigned char * front, unsigned char * side, 
	unsigned char * side2);
int dist_enable(unsigned char mask);

#endif