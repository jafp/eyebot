
// motor_ctrl.c stub

#include <stdio.h>

#include "common.h"
#include "i2c.h"

int motor_ctrl_forward()
{
	unsigned char forward = 0x22;
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_BRAKE, &forward, 1);
}

int motor_ctrl_brake()
{
	unsigned char brake_all = 0x00;	
	// ????
	i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_BRAKE, &brake_all, 1);
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_BRAKE, &brake_all, 1);
}
	
int motor_ctrl_get_speed(unsigned char * left, unsigned char * right)
{	
	unsigned char speeds[2];
	if (i2c_cmd_read(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_GET_SPEED, speeds, 2) == 2)
	{
		(*left) = speeds[0];
		(*right) = speeds[1];
		return 2;
	}
	return -1;
}

int motor_ctrl_set_speed(unsigned char left, unsigned char right)
{
	unsigned char speeds[2] = { left, right };
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_SET_SPEED, speeds, 2);
}

int motor_ctrl_set_state(unsigned char state)
{
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_SET_STATE, &state, 1);
}

int motor_ctrl_goto_position(unsigned int pos_l, unsigned int pos_r)
{
	unsigned char * pl = (unsigned char*) &pos_l;
	unsigned char * pr = (unsigned char*) &pos_r;
	unsigned char data[4] = { pl[0], pl[1], pr[0], pr[1] };
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_GOTO_POS, data, 4);
}


