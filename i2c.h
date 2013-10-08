
#ifndef _I2C_H_
#define _I2C_H_

/**
 * Forward declarations
 */
struct i2c_device;


struct i2c_bus {
	int bus;
	//int latest_slave_addr;

	// Pointer to first entry in linked list of devices
	struct i2c_device * devices;
};

struct i2c_device {
	int addr;
	struct i2c_bus * bus;
	
	// ?

	// Pointer to next entry in linked list of devices
	struct i2c_device * next;
};


int i2c_bus_init(struct i2c_bus * bus, int number);
int i2c_dev_open(struct i2c_device * dev, struct i2c_bus * bus, int addr);
int i2c_dev_close(struct i2c_device * dev);

#endif
