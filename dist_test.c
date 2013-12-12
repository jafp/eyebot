
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "i2c.h"
#include "motor_ctrl.h"
#include "avg_num.h"

static float dist_to_cm(unsigned char dist)
{
	return 1051.43 * powf(dist, -0.944);
}

int main(int argc, char ** argv)
{
	i2c_bus_open();
	//dist_enable(DIST_SENSOR_SIDE);

	avg_num_t n;
	avg_num_create(&n, 20);

	while (1)
	{
		unsigned char front = 0, side = 0, side2 = 0;
		float cm, cm2, cm0;
		
		dist_read(&front, &side, &side2);
		//avg_num_add(&n, side);

		cm0 = dist_to_cm(front);
		cm = dist_to_cm(side);
		cm2 = dist_to_cm(side2);

		printf("> %5.2f %5.2f %5.2f\n", cm0, cm, cm2);

		usleep(20000);
	}

	return 0;
}
