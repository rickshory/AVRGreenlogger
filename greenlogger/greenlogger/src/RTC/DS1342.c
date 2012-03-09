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
} // end of rtc_readTime

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

//	outputStringToUART("\n\r entered setTime routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_TIME_SECONDS); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_TIME_SECONDS): 0x%x\n\r", r);
//			    outputStringToUART(str);
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
} // end of rtc_setTime

/**
 * \brief Read date/time of Alarm1 from RTC
 *
 * This function reads the data/time that is currently set
 *  for Alarm1 from the external RTC chip via I2C (TWI) bus
 *  pass it a pointer to a 'dateTime' struct
 *  returns with fields filled in that struct
 *  Alarm1 only has values of Second, Minute, Hour, and Day-of-Week or Day-of-Month.
 *  (We use Day-of-Month.)
 *  The RTC chip has no provision for alarm Month and Year, so
 *  these values in the passed struct are returned unchanged.
 */
uint8_t rtc_readAlarm1 (dateTime *t) {
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
			    r = I2C_Write(DS1342_ALARM1_SECONDS); // look at this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_ALARM1_SECONDS): 0x%x\n\r", r);
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
							t->second = (10 * ((r & 0x70)>>4) + (r & 0x0f)); // ignore bit 7, alarm mask enable
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x sec\n\r", r);
//							outputStringToUART(str);
							// minutes
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->minute  = (10 * ((r & 0x70)>>4) + (r & 0x0f));
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
							// day of week; unused but read to advance pointer
							r = I2C_Read(0); // do NACK, since this is the last byte
							if (!(r &0b01000000)) { // bit 6 low = this byte holds day of month
								t->day  = (10 * ((r & 0x30)>>4) + (r & 0x0f));
//								len = sprintf(str, "\n\r I2C_Read(0): 0x%x day of month\n\r", r);
//								outputStringToUART(str);
							}
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
} // end of rtc_readAlarm1

/**
 * \brief Set RTC Alarm1
 *
 * This function sets the data/time for the next
 *  alarm (1) in the external RTC chip via I2C (TWI) bus.
 *  Pass it a pointer to a 'dateTime' struct.
 *  Sets the date/time alarm mask values from the fields in that struct.
 *  Alarm1 only takes values of Second, Minute, Hour, and Day-of-Week or Day-of-Month.
 *  (We use Day-of-Month.)
 *  The RTC chip has no provision for alarm Month and Year, so these values in the passed struct are ignored.
 */
uint8_t rtc_setAlarm1 (dateTime *t) {
	uint8_t r, d, wholeUnits;
	//Steps to set the date/time alarm:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address (here DS1342_ALARM1_SECONDS), where to start writing
	// write the data to that address; address increments to next (e.g. MINUTES)
	// continue writing to set the series of desired values
	// do STOP

//	outputStringToUART("\n\r entered setAlarm1 routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_ALARM1_SECONDS); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_ALARM1_SECONDS): 0x%x\n\r", r);
//			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					// convert Seconds to binary coded decimal
					wholeUnits = (uint8_t)(t->second/10);
					d = (wholeUnits<<4) | ((t->second) - (10 * (wholeUnits)));
					d &= 0b01111111; // clear mask bit = alarm only when Sec, Min, Hr, Day match
					r = I2C_Write(d); // write seconds to RTC chip
					// convert Minutes to BCD
					wholeUnits = (uint8_t)(t->minute/10);
					d = (wholeUnits<<4) | ((t->minute) - (10 * (wholeUnits)));
					d &= 0b01111111; // clear mask bit = alarm only when Sec, Min, Hr, Day match
					r = I2C_Write(d); // write minutes to RTC chip
					// convert Hours to BCD
					// bit 6 clear = 24-hour format
					wholeUnits = (uint8_t)(t->hour/10);
					d = (wholeUnits<<4) | ((t->hour) - (10 * (wholeUnits)));
					d &= 0b01111111; // clear mask bit = alarm only when Sec, Min, Hr, Day match
					r = I2C_Write(d); // write hours to RTC chip
					// convert Day of Month
					wholeUnits = (uint8_t)(t->day/10);
					d = (wholeUnits<<4) | ((t->day) - (10 * (wholeUnits)));
					d &= 0b00111111; // clear mask bit (7) = alarm only when Sec, Min, Hr, Day match
					// bit 6 clear = match on day-of-month
					r = I2C_Write(d); // write day to RTC chip
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
} // end of rtc_setAlarm1

/**
 * \brief Enable RTC Alarm1
 *
 * Via I2C (TWI) bus, this function sets up the external RTC chip so that
 *  a match on Alarm1 drives the INTB pin (7).
 */
uint8_t rtc_enableAlarm1 (void) {
	uint8_t r, d, wholeUnits;
	//Steps to enable Alarm1 to drive INTB:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address (DS1342_CONTROL), where to set/clear bits
	// write the data to that address
	// address automatically increments to next (DS1342_CONTROL_STATUS)
	// write the data to that address
	// do STOP
	//
	// salient bits
	//	A2IE = 0, Alarm 2 Interrupt disabled
	//	A1IE = 1, Alarm 1 Interrupt enabled
	//	INTCN = 1, interrupt (rather than sq wave) output on INTB pin
	//	ECLK = 1; route both interrupts to INTB pin, but Alarm2 is disabled

//	outputStringToUART("\n\r entered enableAlarm1 routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_CONTROL); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL): 0x%x\n\r", r);
//			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
#ifdef RTC_CHIP_IS_DS1337
					// DS1337_CONTROL, 0x0e
					// Bit 7: Enable Oscillator (EOSC) = 0, enabled
					// Bit 6: No Function
					// Bit 5: No Function
					// Bits 4 and 3: Rate Select (RS[2:1]) = 00, 1Hz, don't care, not using square wave output
					// Bit 2: Interrupt Control (INTCN) = 1, Alarm 1 interrupt output on INTA
					// Bit 1: Alarm 2 Interrupt Enable (A2IE) = 0, disabled
					// Bit 0: Alarm 1 Interrupt Enable (A1IE) = 1, enabled
					r = I2C_Write(0b00000101); // write DS1337_CONTROL					
#endif
#ifdef RTC_CHIP_IS_DS1342
					// DS1342_CONTROL, 0x0e
					// Bit 7: Enable Oscillator (EOSC) = 0, enabled
					// Bit 6: No Function
					// Bit 5: Enable Glitch Filter (EGFIL) = 0, disabled, saves power
					// Bits 4 and 3: Rate Select (RS[2:1]) = 00, 1Hz, don't care, not using square wave output
					// Bit 2: Interrupt Control (INTCN) = 1, interrupt (rather than sq wave) on INTB
					// Bit 1: Alarm 2 Interrupt Enable (A2IE) = 0, disabled
					// Bit 0: Alarm 1 Interrupt Enable (A1IE) = 1, enabled
					r = I2C_Write(0b00000101); // write DS1342_CONTROL
#endif
					
#ifdef RTC_CHIP_IS_DS1337
					// DS1337_CONTROL_STATUS, 0x0f
					// Bit 7: Oscillator Stop Flag (OSF) = 0, clear, don't care
					// Bits 6 to 2: No Function
					// Bit 1: Alarm 2 Flag (A2F) = 0, clear, A2IE is disabled
					// Bit 0: Alarm 1 Flag (A1F) = 0, clear, A1IE is enabled, Alarm 1 will set
					r = I2C_Write(0b00000000); // write DS1337_CONTROL_STATUS
#endif					
#ifdef RTC_CHIP_IS_DS1342
					// DS1342_CONTROL_STATUS, 0x0f
					// Bit 7: Oscillator Stop Flag (OSF) = 0, clear, don't care
					// Bit 6: Disable Oscillator Stop Flag (DOSF) = 1, disable OSF, save power
					// Bit 5: Loss of Signal (LOS) = 0, clear, don't care
					// Bits 4 and 3: Select Clock Source (CLKSEL[2:1]) = 00, 1Hz don't care, not using external clock input
					// Bit 2: Enable External Clock Input (ECLK) = 1, route Alarm1 to INTB, input on INTA tied low & ignored
					// Bit 1: Alarm 2 Flag (A2F) = 0, clear, A2IE is disabled
					// Bit 0: Alarm 1 Flag (A1F) = 0, clear, A1IE is enabled, Alarm 1 will set
					r = I2C_Write(0b01000100); // write DS1342_CONTROL_STATUS
#endif
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
} // end of rtc_enableAlarm1


/**
 * \brief Set default RTC date/time
 *
 * This function sets the default 
 *  data/time of the global dateTime struct dt_RTC
 *  to winter solstice 2011
 *  2011-12-21 21:30:00 PST, offset 8 hours west from
 *  2011-12-22 05:30:00 UTC
 */
void rtc_setdefault(void)
{
	dt_RTC.year = 11;
	dt_RTC.month = 12;
	dt_RTC.day = 21;
	dt_RTC.houroffset = -8;
	dt_RTC.hour = 21;
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
 * \brief Copies one dateTime struct to another
 *
 * This function takes pointers to 2 dateTime structs,
 *  and copies the first to the second
 */
void datetime_copy(dateTime *from, dateTime *to) {
	to->year = from->year;
	to->month = from->month;
	to->day = from->day;
	to->houroffset = from->houroffset;
	to->hour = from->hour;
	to->minute = from->minute;
	to->second = from->second;
}


/**
 * \brief sets date/time forward by the number of seconds
 *
 * This function takes a pointer to a dateTime struct,
 *  and advances the time by the number of seconds
 */
void datetime_addSeconds(dateTime *t, uint8_t s) {
	t->second += s; // add number of seconds
	datetime_normalize(t);
}


/**
 * \brief sets date/time forward by 10 sec
 *
 * This function takes a pointer to a dateTime struct,
 *  and advances the time by the short interval, currently
 *  10 seconds. It advances to even 10-second boundaries
 *  (10, 20, 30, ...) so it may actually advance by
 *  less than ten seconds total.
 */
void datetime_advanceIntervalShort(dateTime *t) {
	t->second = (t->second/10) * 10; // adjust to even tens-of-seconds
	t->second += 10; // advance by 10 seconds
	datetime_normalize(t);
}


/**
 * \brief sets date/time forward by 1 hr
 *
 * This function takes a pointer to a dateTime struct,
 *  and advances the time by the long interval, currently
 *  1 hour. It advances to even hour boundaries boundaries
 *  (1:00, 2:00, 3:00, ...) so it may actually advance by
 *  less than an hour total.
 */
void datetime_advanceIntervalLong(dateTime *t) {
	t->second = 0;
	t->minute = 0;
	t->hour++;
	datetime_normalize(t);
}

/**
 * \brief assures dateTime is valid
 *
 * This function takes a pointer to a dateTime struct,
 *  and assures the fields are in normal format 
 *  (seconds 0 to 59, hours 0 to 23, etc.). 
 *  It serves as the general purpose adjuster
 *  after adding an arbitrary value to seconds,
 *  minutes, or hours.
 */
void datetime_normalize(dateTime *t) {
	uint8_t m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	while (t->second > 59) {
		t->second -= 60;
		t->minute++;
	}
	while (t->minute > 59) {
		t->minute -= 60;
		t->hour++;
	}
	while (t->hour > 23) {
		t->hour -= 24;
		t->day++;
	}
	if ((t->year % 4) == 0)
		m[1] = 29;
//	else
//		m[1] = 28;
	while (t->day > m[(t->month)-1]) {
		t->day -= m[(t->month)-1];
		t->month++;
		if (t->month > 12) {
			t->month = 1;
			t->year++;
		}
	}
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
	uint8_t iTmp;
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
	if ((dtp->houroffset) < 0) {
		iTmp  = -1 * (dtp->houroffset);
		dtstr[20] = '-';
	} else {
		iTmp  = (dtp->houroffset);
		dtstr[20] = '+';
	}
	dtstr[21] = '0'+ (iTmp / 10);
	dtstr[22] = '0'+ (iTmp % 10);
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