
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h> 
#include <linux/i2c-dev.h>

#include "common.h"
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
    return 0;
}

/**
 * Close the I2C device driver
 */
int i2c_bus_close()
{
	if (close(dev_fd) < 0)
    {
        perror("Failed closing the I2C bus");
        return -1;
    }
    return 0;
}

/**
 * Execute a read command on the slave with the address given in `addr`.
 * 
 * The following steps are done in the process of reading from the slave:
 *  - The slave is aquired with the R/W bit set (write - master to slave)
 *  - A single byte is transmitted to the slave, indicating which command to execute
 *  - The restart is sent on the bus, and the slave is aquired with the R/W
 *      bit held low (read - slave to master). 
 *  - `length` number of bytes are read from the slave into `buffer`.
 *
 * \param addr      Slave address
 * \param cmd       Command identifier
 * \param buffer    Buffer where to place read bytes
 * \param length    Number of bytes to read from the slave
 */
int i2c_cmd_read(int addr, unsigned char cmd, unsigned char buffer[], int length)
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
        perror("ioctl I2C_RDWR (i2c_cmd_read)");
    }

    return ret_val;
}

int i2c_cmd_write(int addr, unsigned char cmd, unsigned char buffer[], int length)
{
    int ret_val;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    unsigned char * buf = malloc(length + 1);
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
        perror("ioctl I2C_RDWR (i2c_cmd_write)");
    }

    free(buf);

    return ret_val < 0 ? ret_val : length;
}

