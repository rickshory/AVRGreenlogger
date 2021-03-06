/*
 * DS1342.h
 *
 * Created: 2/19/2012 10:07:30 PM
 *  Author: rshory
 */ 

#ifndef DS1342_H_
#define DS1342_H_

#define DS1342_ADDR_WRITE 0xd0
#define DS1342_ADDR_READ 0xd1

#define DS1342_TIME_SECONDS  0x00
#define DS1342_TIME_MINUTES  0x01
#define DS1342_TIME_HOURS  0x02
#define DS1342_TIME_DAY  0x03
#define DS1342_TIME_DATE  0x04
#define DS1342_TIME_MONTH_CENTURY  0x05
#define DS1342_TIME_YEAR  0x06
#define DS1342_ALARM1_SECONDS  0x07
#define DS1342_ALARM1_MINUTES  0x08
#define DS1342_ALARM1_HOURS  0x09
#define DS1342_ALARM1_DAY_DATE  0x0a
#define DS1342_ALARM2_MINUTES  0x0b
#define DS1342_ALARM2_HOURS  0x0c
#define DS1342_ALARM2_DAY_DATE  0x0d
#define DS1342_CONTROL 0x0e
#define DS1342_CONTROL_STATUS 0x0f


typedef volatile struct { // always assumes century is 20; year 2000 to 2099
	uint8_t year; // 0 to 99
	uint8_t month; // 1 to 12
	uint8_t day; // 1 to 31
	int8_t houroffset; // timezone difference from Universal Time (GMT) -12 to +12
	uint8_t hour; // 0 to 23
	uint8_t minute; // 0 to 59
	uint8_t second; // 0 to 59
} dateTime ;

// functions

void rtc_init(void);
//extern char* rtc_getstring(char* dtstr, DateTime dt);
void datetime_copy(dateTime *to, dateTime *from);
uint32_t datetime_truesecs (dateTime *t);
uint32_t datetime_compareval_secs (dateTime *t);
void datetime_check_secs (uint32_t secsTot, dateTime *t);
int8_t datetime_compare(dateTime *t1, dateTime *t2);
void datetime_addSeconds(dateTime *t, uint8_t s);
void datetime_advanceIntervalShort(dateTime *t);
void datetime_advanceIntervalLong(dateTime *t);
void datetime_advanceInterval(dateTime *t);
void datetime_advanceDatestring1Day(char* s);
void datetime_advanceDatestring1stOfNextMonth(char* s);
uint8_t datetime_nextDateWithData(char* s, uint8_t forceAhead);
void datetime_normalize(dateTime *t);
void datetime_getstring(char* dtstr, dateTime *dtp);
void datetime_getFromUnixString(dateTime *dtp, char* dtstr, bool useGlobalTimeZone);
void datetime_getDefault(dateTime *t);

void rtc_add1sec(void);

uint8_t rtc_setTime (dateTime *t);
uint8_t rtc_readTime (dateTime *t);
uint8_t rtc_setAlarm1 (dateTime *t);
uint8_t rtc_readAlarm1 (dateTime *t);
uint8_t rtc_enableAlarm1 (void);
uint8_t rtc_setupNextAlarm(dateTime *pDt);
uint8_t rtc_enableSqWave (void);

uint8_t isValidDateTime(char* p);
uint8_t isValidDate(char* p);
uint8_t isValidTime(char* p);
uint8_t isValidTimezone(char* p);

#endif /* DS1342_H_ */