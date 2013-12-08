

#ifndef _IOEXP_H_
#define _IOEXP_H_

#include <stdint.h>
#include <stdbool.h>

#define IOEXP_ADDR			0x20

#define IOEXP_REG_GP0 		0x00
#define IOEXP_REG_GP1		0x01
#define IOEXP_REG_IODIR1	0x06
#define IOEXP_REG_IODIR2	0x07

#define IOEXP_LED0			0x01
#define IOEXP_LED1			0x02
#define IOEXP_LED2			0x04
#define IOEXP_LED3			0x08
#define IOEXP_LED4			0x10
#define IOEXP_LED5			0x20
#define IOEXP_LED6			0x40
#define IOEXP_LED7			0x80

#define beep_short()		(ioexp_buzzer_beep(60))
#define beep_medium()		(ioexp_buzzer_beep(120))
#define beep_long()			(ioexp_buzzer_beep(180))
#define beep()				(beep_medium())

#define beep_start_seq() 	\
	beep_medium(); 			\
	usleep(200000); 		\
	beep_medium();			\
	usleep(200000);			\
	beep_medium();			\

#define beep_state_change() \
	beep_short();			\
	usleep(100000);			\
	beep_short();			\

void ioexp_init();
void ioexp_buzzer_beep(int ms);
void ioexp_led_set(uint8_t mask);

#endif
