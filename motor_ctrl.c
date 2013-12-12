

#include <math.h>
#include <stdio.h>

#include "motor_ctrl.h"
#include "common.h"
#include "i2c.h"

int motor_ctrl_init()
{
	motor_ctrl_brake();
}

int motor_ctrl_forward()
{
	return motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
}

int motor_ctrl_brake()
{
	return i2c_cmd_write(MOTOR_CTRL_ADDR, 0x70, NULL, 0);
}

int motor_ctrl_set_dir(unsigned char dir_mask)
{
	return i2c_cmd_write(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_BRAKE, &dir_mask, 1);	
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

int motor_ctrl_wait(int additional_delay_ms)
{
	int counter = 0;
	int delay_ms = 150;
	int max_settling_time_ms = 3000; 
	int max_rounds = max_settling_time_ms / delay_ms; 
	unsigned char res;
	
	while (1)
	{
		i2c_cmd_read(MOTOR_CTRL_ADDR, MOTOR_CTRL_CMD_IS_STABLE, &res, 1);
		
		if (res == 1)
		{
			break;
		}
		if (counter > max_rounds)
		{
			printf("Position timeout...\n");
			break;
		}

		// Delay 5 ms
		usleep(delay_ms * 1000);
		counter++;
	}

	usleep(additional_delay_ms * 1000);
	return 0;
}

int dist_enable(unsigned char mask)
{
	return i2c_cmd_write(MOTOR_CTRL_ADDR, 0x90, &mask, 1);
}

int dist_read(unsigned char * front, unsigned char * side, unsigned char * side2)
{
	unsigned char buffer[3] = {0};
	i2c_cmd_read(MOTOR_CTRL_ADDR, 0x80, buffer, 3);
	if (front != NULL)
	{
		*front = buffer[0];
	}
	if (side != NULL)
	{
		*side = buffer[1];
	}
	if (side2 != NULL)
	{
		*side2 = buffer[2];
	}
	return 0;
}

dist_readings_t dist_read_all()
{
	dist_readings_t readings;
		
	// Read raw values from sensors
	dist_read(&readings.front_val, &readings.side_1_val, 
		&readings.side_2_val);

	// Convert the numbers to a distance in centimeters
	readings.front = get_dist_to_cm(readings.front_val);
	readings.side_1 = get_dist_to_cm(readings.side_1_val);
	readings.side_2 = get_dist_to_cm(readings.side_2_val);

	return readings;
}


/**
 * Convert number from distance sensor, to a distance in centimeters.
 * Approximation calculated using Excel and a bunch of measurements.
 *
 * \param dist
 */
float get_dist_to_cm(unsigned char dist)
{
	return 1051.43 * powf(dist, -0.944);
}
