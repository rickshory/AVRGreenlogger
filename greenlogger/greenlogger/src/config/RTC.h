/*
 * RTC.h
 *
 * Created: 12/28/2011 12:23:56 PM
 *  Author: rshory
 */ 

#include "compiler.h"

#ifndef RTC_H_
#define RTC_H_

struct	DateTime { // always assumes century is 20; year 2000 to 2099
	volatile uint8_t year; // 0 to 99
	volatile uint8_t month; // 1 to 12
	volatile uint8_t day; // 1 to 31
	volatile int8_t houroffset; // timezone difference from Universal Time (GMT) -12 to +12
	volatile uint8_t hour; // 0 to 23
	volatile uint8_t minute; // 0 to 59
	volatile uint8_t second; // 0 to 59
} ;

struct DateTime RTC_dt;


extern void rtc_init(void);
//extern char* rtc_getstring(char* dtstr, DateTime dt);
extern void datetime_getstring(char* dtstr, struct DateTime *dtp);
extern void rtc_add1sec(void);


#endif /* RTC_H_ */