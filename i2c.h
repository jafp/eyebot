
#ifndef _I2C_H_
#define _I2C_H_

#define I2C_BUS 			"/dev/i2c-1"
#define I2C_MAX_RETRIES		10

int i2c_bus_open();
int i2c_bus_close();
int i2c_cmd_read(int addr, unsigned char cmd, unsigned char buffer[], int length);
int i2c_cmd_write(int addr, unsigned char cmd, unsigned char buffer[], int length);

#endif
