/*
 * greenlogger.h
 *
 * Created: 12/27/2011 1:37:14 PM
 *  Author: rshory
 */ 


#ifndef GREENLOGGER_H_
#define GREENLOGGER_H_
#include "interrupt.h"
#include "mega_uart_interrupt.h"

#define commandBufferLen 30

typedef struct	{ // always assumes century is 20; year 2000 to 2099
	volatile uint8_t year; // 0 to 99
	volatile uint8_t month; // 1 to 12
	volatile uint8_t day; // 1 to 31
	volatile int8_t houroffset; // timezone difference from Universal Time (GMT) -12 to +12
	volatile uint8_t hour; // 0 to 23
	volatile uint8_t minute; // 0 to 59
	volatile uint8_t second; // 0 to 59
} DateTime ;

void outputStringToUART (char* St);
void checkForCommands (void);

#endif /* GREENLOGGER_H_ */