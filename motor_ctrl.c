
// motor_ctrl.c stub

#define GET_SPEED 		0x10 // ???

int motor_ctrl_get_speed(int * speeds[])
{	
	struct i2c_device * dev; // = ???
	
	// Write command byte, read two speed bytes
	i2c_dev_cmd_read(dev, GET_SPEED, speeds, 2);
}

