
#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "i2c.h"
#include "motor_ctrl.h"

static int run = 1;

void sigint_handler()
{
	run = 0;
}


int main()
{
	i2c_bus_open();
	motor_ctrl_init();
	signal(SIGINT, sigint_handler);

	//motor_ctrl_set_state(STATE_POSITION);
	//motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	//motor_ctrl_goto_position(512, 0);

	//sleep(4);

	//dist_enable(0xFF);

	motor_ctrl_set_speed(60, 60);
	motor_ctrl_set_state(STATE_STRAIGHT);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	

	//while (run);
	sleep(6);
	motor_ctrl_brake();

	dist_enable(0);
	
	/*
	printf("Goto!\n");

	motor_ctrl_set_state(STATE_POSITION);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_goto_position(512, 0);
	//motor_ctrl_wait(500);


	sleep(3);
	printf("Done!\n");

	motor_ctrl_set_state(STATE_POSITION);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_goto_position(512, 0);
	sleep(3);
	//motor_ctrl_wait(500);

	printf("Done 2\n");	
	*/

	return 0;
}