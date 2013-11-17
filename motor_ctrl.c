
// motor_ctrl.c stub

#include <stdio.h>

#include "common.h"
#include "i2c.h"

int motor_ctrl_forward()
{
	unsigned char forward = 0x30;
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_BRAKE, &forward, 1);
}

int motor_ctrl_brake()
{
	unsigned char brake_all = 0x10;	
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

int motor_goto_position(unsigned char motor, unsigned int position)
{
	unsigned char * pos = (unsigned char*) &position;
	unsigned char data[3] = { motor, pos[0], pos[1] };
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_GOTO_POS, data, 3);
}


