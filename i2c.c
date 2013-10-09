
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h> 
#include <linux/i2c-dev.h>

#include "i2c.h"

/**
 * Device special file descriptor to the I2C bus
 */
static int dev_fd;

/**
 * Open the I2C bus device driver.
 */
int i2c_bus_open()
{
	if ((dev_fd = open(I2C_BUS, O_RDWR)) < 0) 
	{
		perror("Failed opening I2C bus");
		return -1;		
	}
}

/**
 * Close the I2C device driver
 */
int i2c_bus_close()
{
	// Noop
}

int i2c_cmd_read(int addr, char cmd, char buffer[], int length)
{
    int ret_val;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // Prepare command message
    messages[0].addr = addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(cmd);
    messages[0].buf = &cmd;
    
    if (length > 0)
    {
        // Receive message 
        messages[1].addr = addr;
        messages[1].flags = I2C_M_RD;
        messages[1].len = length;
        messages[1].buf = buffer;       
    }

    packets.msgs = messages;
    packets.nmsgs = 2;

    ret_val = ioctl(dev_fd, I2C_RDWR, &packets);
    if (ret_val < 0)
    {
        perror("ioctl I2C_RDWR");
    }

    return ret_val < 0 ? ret_val : length;
}

int i2c_cmd_write(int addr, char cmd, char buffer[], int length)
{
    int ret_val;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    char * buf = malloc(length + 1);
    buf[0] = cmd;

    memcpy(buf + 1, buffer, length);

    // Prepare command message
    messages[0].addr = addr;
    messages[0].flags = 0;
    messages[0].len = length + 1;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    ret_val = ioctl(dev_fd, I2C_RDWR, &packets);
    if (ret_val < 0)
    {
        perror("ioctl I2C_RDWR");
    }

    free(buf);

    return ret_val < 0 ? ret_val : length;
}

