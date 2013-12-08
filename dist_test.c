
#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"
#include "motor_ctrl.h"

int main(int argc, char ** argv)
{
	i2c_bus_open();
	dist_enable(DIST_SENSOR_FRONT | DIST_SENSOR_SIDE);

	while (1)
	{
		unsigned char front, side;
		
		dist_read(&front, &side);
		printf("> %5d %5d\n", front, side);

		usleep(20000);
	}

	return 0;
}
