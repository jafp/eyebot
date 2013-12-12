
#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "i2c.h"
#include "motor_ctrl.h"

// Global run flag
static int run_flag = 1;

void sigint_handler()
{
	run_flag = 0;
}

int main(int argc, char ** argv)
{
	unsigned char speed = 25;
	printf("1\n");
	i2c_bus_open();
	motor_ctrl_init();
	printf("2\n");
	signal(SIGINT, sigint_handler);
	printf("3\n");

	if (argc == 2)
	{
		speed = (unsigned char) atoi(argv[1]);
	}

	printf("Running (speed %d)\n", (int) speed);
	
	motor_ctrl_set_state(STATE_STRAIGHT);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_set_speed(speed, speed);
	
	// Run until a sigint is received
	while (run_flag);

	motor_ctrl_brake();
	printf("Done.\n");

	return 0;
}