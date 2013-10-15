
// motor_ctrl.c stub

#include <stdio.h>

#include "common.h"
#include "i2c.h"

int motor_ctrl_get_speed(unsigned char * left, unsigned char * right)
{	
	unsigned char speeds[2];
	int retries = 0;
	do 
	{
		if (i2c_cmd_read(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_GET_SPEED, speeds, 2) == 2)
		{
			(*left) = speeds[0];
			(*right) = speeds[1];
			return 2;
		}

		retries++;
	}
	while (retries < I2C_MAX_RETRIES);
	return -1;
}

int motor_ctrl_set_speed(unsigned char left, unsigned char right)
{
	unsigned char speeds[2] = { left, right };
	int retries = 0;
	do 
	{
		if (i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_SET_SPEED, speeds, 2) == 2)
		{
			return 2;
		}

		retries++;
	}
	while (retries < I2C_MAX_RETRIES);
	return -1;
}




