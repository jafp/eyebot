
#include <stdio.h>

#include "i2c.h"
#include "ioexp.h"

int main(int argc, char ** argv)
{
	i2c_bus_open();
	ioexp_init();

	beep_start_seq();

	sleep(1);

	beep_state_change();

	//ioexp_led_set(IOEXP_LED0 | IOEXP_LED2 | IOEXP_LED6 | IOEXP_LED7);
	//sleep(2);
	
	//ioexp_led_set(0x00);
	//while (1);


	while (1)
	{
		uint8_t mask = 0x00;
		int i;
		for (i = 1; i < 8; i++)
		{
			mask = (1 << i);
			ioexp_led_set(mask);
			usleep(100000);
		}

		ioexp_buzzer_beep(20);

		for (i = 6; i >= 0; i--)
		{
			mask = (1 << i);
			ioexp_led_set(mask);
			usleep(100000);
		}

		ioexp_buzzer_beep(20);
	}


	return 0;
}