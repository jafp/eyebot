
#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

int motor_ctrl_brake();
int motor_ctrl_forward();
int motor_ctrl_get_speed(unsigned char * left, unsigned char * right);
int motor_ctrl_set_speed(unsigned char left, unsigned char right);
int motor_goto_position(unsigned char motor, unsigned int position);

#endif