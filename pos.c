
#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "i2c.h"
#include "motor_ctrl.h"

#define PULSES_PR_DEG	((float) 512/90)

int main(int argc, char ** argv)
{
	i2c_bus_open();
	motor_ctrl_init();

	int pos_left = 0, pos_right = 0;

	if (argc == 2)
	{
		pos_left = atoi(argv[1]) * PULSES_PR_DEG;
	}
	else if (argc == 3)
	{
		pos_left = atoi(argv[1]) * PULSES_PR_DEG;
		pos_right = atoi(argv[2]) * PULSES_PR_DEG;
	}

	printf("Left: %d, right: %d\n", pos_left, pos_right);

	motor_ctrl_set_state(STATE_POSITION);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_goto_position(pos_left, pos_right);
	motor_ctrl_wait(200);

	printf("Done.\n");

	return 0;
}