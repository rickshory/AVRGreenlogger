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

extern volatile dateTime dt_RTC;

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];

/*
bool rtc_setTime (dateTime *t) {
	uint8_t r;
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

//	outputStringToUART("\n\r entered time routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
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
*/

/**
 * \brief Read date/time from RTC
 *
 * This function reads the data/time from the external RTC chip via I2C (TWI) bus
 *  pass it a pointer to a 'dateTime' struct
 *  returns with fields filled in that struct
 */
uint8_t rtc_readTime (dateTime *t) {
	uint8_t r;
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

//	outputStringToUART("\n\r entered readTime routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
//			    r = I2C_Write(DS1342_CONTROL_STATUS); // for testing, look at this register
			    r = I2C_Write(DS1342_TIME_SECONDS); // for testing, look at this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_TIME_SECONDS): 0x%x\n\r", r);
//			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
//				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//				    outputStringToUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
//					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
//					    outputStringToUART(str);
						if (r == TW_MR_SLA_ACK) {
							// seconds
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->second = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x sec\n\r", r);
//							outputStringToUART(str);
							// minutes
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->minute  = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x min\n\r", r);
//							outputStringToUART(str);
							// hour
							r = I2C_Read(1); // do ACK, since this is not the last byte
							if (r & 0b01000000) { // 12-hour format
								t->hour  = (10 * ((r & 0x10)>>4) + (r & 0x0f));
								if (r & 0b00100000) // PM
									t->hour += 12;
							} else { // 24-hour format
								t->hour  = (10 * ((r & 0x30)>>4) + (r & 0x0f));
							}
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x hr\n\r", r);
//							outputStringToUART(str);
							// day of week
							r = I2C_Read(1); // do ACK, since this is not the last byte
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x wk day\n\r", r);
//							outputStringToUART(str);
							// day of month
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->day  = (10 * ((r & 0x30)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x day of month\n\r", r);
//							outputStringToUART(str);
							// month (bit7 = century)
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->month = (10 * ((r & 0x10)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x month\n\r", r);
//							outputStringToUART(str);
							// year, 00 to 99
							r = I2C_Read(0); // do NACK, since this is the last byte
							t->year = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x year\n\r", r);
//							outputStringToUART(str);
						}							
					}
//					outputStringToUART("\n\r exit from repeat start\n\r");
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
//				outputStringToUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToUART("\n\r I2C_Stop completed \n\r");
		} else { // could not START
			return errNoI2CStart;
		}			
	return I2C_OK;
}

/**
 * \brief Set RTC date/time
 *
 * This function sets the data/time in the external RTC chip via I2C (TWI) bus
 *  pass it a pointer to a 'dateTime' struct
 *  sets the date/time values from the fields in that struct
 */
uint8_t rtc_setTime (dateTime *t) {
	uint8_t r, d, wholeUnits;
	//Steps to set the date/time:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address (usually DS1342_TIME_SECONDS), where to start writing
	// write the data to that address; address increments to next (e.g. MINUTES)
	// continue writing values to set entire time and date
	// do STOP

	outputStringToUART("\n\r entered setTime routine \n\r");
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_TIME_SECONDS); // point to this register
			    len = sprintf(str, "\n\r I2C_Write(DS1342_TIME_SECONDS): 0x%x\n\r", r);
			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					// convert Seconds to binary coded decimal
					wholeUnits = (uint8_t)(t->second/10);
					d = (wholeUnits<<4) | ((t->second) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write seconds to RTC
					// convert Minutes to BCD
					wholeUnits = (uint8_t)(t->minute/10);
					d = (wholeUnits<<4) | ((t->minute) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write minutes to RTC
					// convert Hours to BCD
					// bit 6 clear = 24-hour format
					wholeUnits = (uint8_t)(t->hour/10);
					d = (wholeUnits<<4) | ((t->hour) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write hours to RTC
					// we don't use Day of Week
					r = I2C_Write(0); // write dummy value of zero to advance addr pointer
					// convert Day of Month
					wholeUnits = (uint8_t)(t->day/10);
					d = (wholeUnits<<4) | ((t->day) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write day to RTC
					// convert Month number
					wholeUnits = (uint8_t)(t->month/10);
					d = (wholeUnits<<4) | ((t->month) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write month to RTC
					// convert Year
					wholeUnits = (uint8_t)(t->year/10);
					d = (wholeUnits<<4) | ((t->year) - (10 * (wholeUnits)));
					r = I2C_Write(d); // write year to RTC
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
//				outputStringToUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToUART("\n\r I2C_Stop completed \n\r");
		} else { // could not START
			return errNoI2CStart;
		}			
	return I2C_OK;
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
	dt_RTC.year = 11;
	dt_RTC.month = 12;
	dt_RTC.day = 22;
	dt_RTC.houroffset = 0;
	dt_RTC.hour = 5;
	dt_RTC.minute = 30;
	dt_RTC.second = 0;
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
    (dt_RTC.second)++;
	if (dt_RTC.second >= 60) {
		(dt_RTC.minute)++;
		dt_RTC.second -= 60;
		if (dt_RTC.minute >= 60) {
			(dt_RTC.hour)++;
			dt_RTC.minute -= 60;
			if (dt_RTC.hour >= 24) {
				(dt_RTC.day)++;
				dt_RTC.hour -= 24;
				if (dt_RTC.day >= 28) {
					switch (dt_RTC.month) {
					case 1: case 3: case 5: case 7: case 8: case 10: case 12:
						if (dt_RTC.day > 31) {
							(dt_RTC.month)++;
							dt_RTC.day = 1;
							break;
						}
					case 4: case 6: case 9: case 11:
						if (dt_RTC.day > 30) {
							(dt_RTC.month)++;
							dt_RTC.day = 1;
							break;
						}							
					case 2:	
						if ((dt_RTC.year % 4) || (dt_RTC.day > 29)) {
							(dt_RTC.month)++;
							dt_RTC.day = 1;
							break;
						}
					}
					if (dt_RTC.month > 12) {
						(dt_RTC.year)++;
						dt_RTC.month = 1;
					}
				}
			}				
		}
	}
/*	dt_RTC.year = 11;
	dt_RTC.month = 12;
	dt_RTC.day = 22;
	dt_RTC.houroffset = 0;
	dt_RTC.hour = 5;
	dt_RTC.minute = 30;
*/	
}


/**
 * \brief gets a date/time as a string
 *
 * This function converts a data/time struct to
 *  a string and returns a pointer to that string
 *  
 * Fields contain scalars rather than binary coded decimal, so as to more easily do math
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