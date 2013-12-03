
#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#define STATE_SPEED		0x00
#define STATE_POSITION 	0xFF

int motor_ctrl_brake();
int motor_ctrl_forward();
int motor_ctrl_get_speed(unsigned char * left, unsigned char * right);
int motor_ctrl_set_speed(unsigned char left, unsigned char right);
int motor_ctrl_set_state(unsigned char state);
int motor_ctrl_goto_position(unsigned int pos_l, unsigned int pos_r);

#endif