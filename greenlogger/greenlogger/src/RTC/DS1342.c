/*
 * DS1342.c
 *
 * Created: 2/19/2012 10:08:06 PM
 *  Author: rshory
 */ 
#include <inttypes.h>
#include "compiler.h"
#include "DS1342.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

extern volatile dateTime RTC_dt;

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];

bool rtc_setTime (dateTime *t) {
	uint8_t r, ct;
	//Manipulating the Address Counter for Reads:
	// A dummy write cycle can be used to force the 
	// address counter to a particular value.
	//	steps:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address where we intend to read (dummy read, sets pointer)
	// do repeated START
	// write DS1342_ADDR_READ
	// read byte(s) with ACK or NACK as applicable
	// do STOP

	outputStringToUART("\n\r entered time routine \n\r");
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
//			    r = I2C_Write(DS1342_CONTROL_STATUS); // for testing, look at this register
			    r = I2C_Write(DS1342_TIME_SECONDS); // for testing, look at this register
			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL_STATUS): 0x%x\n\r", r);
			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
				    outputStringToUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
					    outputStringToUART(str);
						if (r == TW_MR_SLA_ACK) {
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(0); // do NACK, since this is the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
						}							
					}							
				}	
			}
		}		
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");

	return 1;
}


bool rtc_readTime (dateTime *t) {
	uint8_t r, ct;
	//Manipulating the Address Counter for Reads:
	// A dummy write cycle can be used to force the 
	// address counter to a particular value.
	//	steps:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address where we intend to read (dummy read, sets pointer)
	// do repeated START
	// write DS1342_ADDR_READ
	// read byte(s) with ACK or NACK as applicable
	// do STOP

	outputStringToUART("\n\r entered readTime routine \n\r");
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
//			    r = I2C_Write(DS1342_CONTROL_STATUS); // for testing, look at this register
			    r = I2C_Write(DS1342_TIME_SECONDS); // for testing, look at this register
			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL_STATUS): 0x%x\n\r", r);
			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
				    outputStringToUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
					    outputStringToUART(str);
						if (r == TW_MR_SLA_ACK) {
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(0); // do NACK, since this is the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
						}							
					}							
				}	
			}
		}		
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");

	return 1;
}

/**
 * \brief Set default RTC date/time
 *
 * This function sets the default data/time of the RTC
 *  to winter solstice 2011
 *  2011-12-22 05:30:00 UTC
 */
void rtc_setdefault(void)
{
	RTC_dt.year = 11;
	RTC_dt.month = 12;
	RTC_dt.day = 22;
	RTC_dt.houroffset = 0;
	RTC_dt.hour = 5;
	RTC_dt.minute = 30;
	RTC_dt.second = 0;
}

/**
 * \brief adds 1 second to Real Time Clock
 *
 * This function adds 1 second to the Real Time Clock data/time struct and
 *  accounts for any rollover
 *  
 */
void rtc_add1sec(void)
{
    (RTC_dt.second)++;
	if (RTC_dt.second >= 60) {
		(RTC_dt.minute)++;
		RTC_dt.second -= 60;
		if (RTC_dt.minute >= 60) {
			(RTC_dt.hour)++;
			RTC_dt.minute -= 60;
			if (RTC_dt.hour >= 24) {
				(RTC_dt.day)++;
				RTC_dt.hour -= 24;
				if (RTC_dt.day >= 28) {
					switch (RTC_dt.month) {
					case 1: case 3: case 5: case 7: case 8: case 10: case 12:
						if (RTC_dt.day > 31) {
							(RTC_dt.month)++;
							RTC_dt.day = 1;
							break;
						}
					case 4: case 6: case 9: case 11:
						if (RTC_dt.day > 30) {
							(RTC_dt.month)++;
							RTC_dt.day = 1;
							break;
						}							
					case 2:	
						if ((RTC_dt.year % 4) || (RTC_dt.day > 29)) {
							(RTC_dt.month)++;
							RTC_dt.day = 1;
							break;
						}
					}
					if (RTC_dt.month > 12) {
						(RTC_dt.year)++;
						RTC_dt.month = 1;
					}
				}
			}				
		}
	}
/*	RTC_dt.year = 11;
	RTC_dt.month = 12;
	RTC_dt.day = 22;
	RTC_dt.houroffset = 0;
	RTC_dt.hour = 5;
	RTC_dt.minute = 30;
*/	
}


/**
 * \brief gets a date/time as a string
 *
 * This function converts a data/time struct to
 *  a string and returns a pointer to that string
 *  
 */
void datetime_getstring(char* dtstr, dateTime *dtp)
{
	strcpy(dtstr, "2000-00-00 00:00:00 +00");
	dtstr[2] = '0'+ ((dtp->year) / 10);
	dtstr[3] = '0'+	((dtp->year) % 10);
	dtstr[5] = '0'+	((dtp->month) / 10);
	dtstr[6] = '0'+	((dtp->month) % 10);
	dtstr[8] = '0'+	((dtp->day) / 10);
	dtstr[9] = '0'+	((dtp->day) % 10);
	dtstr[11] = '0'+ ((dtp->hour) / 10);
	dtstr[12] = '0'+ ((dtp->hour) % 10);
	dtstr[14] = '0'+ ((dtp->minute) / 10);
	dtstr[15] = '0'+ ((dtp->minute) % 10);
	dtstr[17] = '0'+ ((dtp->second) / 10);
	dtstr[18] = '0'+ ((dtp->second) % 10);
	dtstr[20] = (((dtp->houroffset) < 0) ? '-' : '+');
	dtstr[21] = '0'+ ((dtp->houroffset) / 10);
	dtstr[22] = '0'+ ((dtp->houroffset) % 10);
}

/**
 * \brief Initialize the RTC
 *
 * This function will initialize the Real Time Clock
 * 
 * 
 */
void rtc_init(void)
{
/*
steps to initialize RTC:

put 32.786 kHz crystal between pins 28 (TOSC1) and 29 (TOSC2)
xtal should be 12.5pF

enable power to Timer/Counter 2
 clear Bit 6 – PRTIM2: Power Reduction Timer/Counter2
 in PRR0 – Power Reduction Register 0
 this is default 0, so may be redundant unless set for some reason
 
Asynchronous Status Register (ASSR).
Timer/Counter (TCNT2)
Output Compare Register (OCR2A and OCR2B)
Timer Interrupt Flag Register (TIFR2)
Timer Interrupt Mask Register (TIMSK2)
*/
}