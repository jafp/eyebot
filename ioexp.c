
#include <stdio.h>
#include "ioexp.h"

/**
 * Flag indicating if the IO Expander is initialized
 * and correctly configured.
 */
static bool initialized = false;

/**
 * Initialize the MCP23016 io-expander by turning off the buzzer
 * and LEDs, and setting all pins as outputs (in the mentioned order). 
 */
void ioexp_init()
{
	uint8_t tmp[2];

	// Turn off buzzer (low) and LEDs (high).
	// This is done before setting the ports as outputs,
	// so we don't see any flicker and short beeps.
	tmp[0] = 0xFF;
	tmp[1] = 0x00;
	i2c_cmd_write(IOEXP_ADDR, IOEXP_REG_GP0, tmp, 2);
	
	// Set all pins as output
	tmp[0] = 0x00; 
	tmp[1] = 0x00;
	i2c_cmd_write(IOEXP_ADDR, IOEXP_REG_IODIR1, tmp, 2);
	
	// Set initialized flag
	initialized = true;
}

/**
 * Turn on the buzzer in `ms` milliseconds.
 *
 * \param ms Number of milliseconds the buzzer should beep
 */
void ioexp_buzzer_beep(int ms)
{
	uint8_t tmp;
	
	if (!initialized)
	{
		printf("Error! IO Expander not initialized\n");
		return;
	}

	tmp = 0x01;	
	i2c_cmd_write(IOEXP_ADDR, IOEXP_REG_GP1, &tmp, 1);

	usleep(ms * 1000);

	tmp = 0x00;
	i2c_cmd_write(IOEXP_ADDR, IOEXP_REG_GP1, &tmp, 1);
}

/**
 * Turn on the LEDs masked by `mask`. The IOEXP_LEDx constants
 * can be used to mask the individual LEDs.
 *
 * \param mask Mask of turned on LEDs (1 is on, 0 is off)
 */
void ioexp_led_set(uint8_t mask)
{
	mask = ~mask;
	i2c_cmd_write(IOEXP_ADDR, IOEXP_REG_GP0, &mask, 1);
}

