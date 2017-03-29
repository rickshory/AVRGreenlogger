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
#include "../SDcard/ff.h"
#include "../SDcard/diskio.h"
#include "BattMonitor/ADconvert.h"
#include <util/twi.h>

extern volatile dateTime dt_RTC;
extern volatile int8_t timeZoneOffset;
extern volatile uint8_t rtcStatus;

extern volatile sFlags1 stateFlags1;
extern volatile tFlags timeFlags;
extern volatile rFlags irradFlags;
extern int len;
extern char datetime_string[25];
extern char str[128];
volatile extern adcData cellVoltageReading;


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

//	outputStringToWiredUART("\n\r entered readTime routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
//			    r = I2C_Write(DS1342_CONTROL_STATUS); // for testing, look at this register
			    r = I2C_Write(DS1342_TIME_SECONDS); // for testing, look at this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_TIME_SECONDS): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
//				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//				    outputStringToWiredUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
//					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
//					    outputStringToWiredUART(str);
						if (r == TW_MR_SLA_ACK) {
							// seconds
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->second = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x sec\n\r", r);
//							outputStringToWiredUART(str);
							// minutes
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->minute  = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x min\n\r", r);
//							outputStringToWiredUART(str);
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
//							outputStringToWiredUART(str);
							// day of week
							r = I2C_Read(1); // do ACK, since this is not the last byte
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x wk day\n\r", r);
//							outputStringToWiredUART(str);
							// day of month
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->day  = (10 * ((r & 0x30)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x day of month\n\r", r);
//							outputStringToWiredUART(str);
							// month (bit7 = century)
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->month = (10 * ((r & 0x10)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x month\n\r", r);
//							outputStringToWiredUART(str);
							// year, 00 to 99
							r = I2C_Read(0); // do NACK, since this is the last byte
							t->year = (10 * ((r & 0xf0)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x year\n\r", r);
//							outputStringToWiredUART(str);
							t->houroffset = timeZoneOffset; // get from global var
						}							
					}
//					outputStringToWiredUART("\n\r exit from repeat start\n\r");
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
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

//	outputStringToWiredUART("\n\r entered setTime routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_TIME_SECONDS); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_TIME_SECONDS): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
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
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
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

//	outputStringToWiredUART("\n\r entered readTime routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_ALARM1_SECONDS); // look at this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_ALARM1_SECONDS): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
//				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//				    outputStringToWiredUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
//					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
//					    outputStringToWiredUART(str);
						if (r == TW_MR_SLA_ACK) {
							// seconds
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->second = (10 * ((r & 0x70)>>4) + (r & 0x0f)); // ignore bit 7, alarm mask enable
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x sec\n\r", r);
//							outputStringToWiredUART(str);
							// minutes
							r = I2C_Read(1); // do ACK, since this is not the last byte
							t->minute  = (10 * ((r & 0x70)>>4) + (r & 0x0f));
//							len = sprintf(str, "\n\r I2C_Read(0): 0x%x min\n\r", r);
//							outputStringToWiredUART(str);
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
//							outputStringToWiredUART(str);
							// day of week; unused but read to advance pointer
							r = I2C_Read(0); // do NACK, since this is the last byte
							if (!(r &0b01000000)) { // bit 6 low = this byte holds day of month
								t->day  = (10 * ((r & 0x30)>>4) + (r & 0x0f));
//								len = sprintf(str, "\n\r I2C_Read(0): 0x%x day of month\n\r", r);
//								outputStringToWiredUART(str);
							}
							t->houroffset = timeZoneOffset; // get from global var
						}							
					}
//					outputStringToWiredUART("\n\r exit from repeat start\n\r");
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
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

//	outputStringToWiredUART("\n\r entered setAlarm1 routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_ALARM1_SECONDS); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_ALARM1_SECONDS): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
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
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
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

//	outputStringToWiredUART("\n\r entered enableAlarm1 routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_CONTROL); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
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
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
		} else { // could not START
			return errNoI2CStart;
		}			
	return I2C_OK;
} // end of rtc_enableAlarm1

/**
 * \brief Set up and enable next Alarm
 *
 * This function reads the current RTC time, 
 *  calculates the next alarm time based on the
 *  interval determined by flags,
 *  sets that alarm time, and enables the alarm
 */
uint8_t rtc_setupNextAlarm(dateTime *pDt) {
	uint8_t n;
	dateTime dt;
//	outputStringToWiredUART("\n\r entered setupNextAlarm fn \n\r\n\r");
	n = rtc_readTime (&dt); // get current time
//	datetime_getstring(str, &dt);
//	outputStringToWiredUART("\n\r time read from RTC: ");
//	outputStringToWiredUART(str);
//	outputStringToWiredUART("\n\r\n\r");
	
	if (n) return n;
	else { // advance to next alarm time
//		outputStringToWiredUART("\n\r about to call datetime_advanceInterval fn \n\r\n\r");
		datetime_advanceInterval(&dt);
//		outputStringToWiredUART("\n\r retd from datetime_advanceInterval fn \n\r\n\r");
//		outputStringToWiredUART("\n\r calcd alarm time: ");
//		datetime_getstring(str, &dt);
//		outputStringToWiredUART(str);
//		outputStringToWiredUART("\n\r\n\r");
		
//		outputStringToWiredUART("\n\r about to call rtc_setAlarm1 fn \n\r\n\r");
		n = rtc_setAlarm1(&dt);
//		outputStringToWiredUART("\n\r retd from rtc_setAlarm1 fn \n\r\n\r");
		if (n) return n;
		else {
			n = rtc_enableAlarm1();
			if (n) return n;
			else {
				datetime_copy(pDt, &dt);
				pDt->houroffset = timeZoneOffset;
				enableRTCInterrupt();
				return 0;
			}
		}
	}
}

/**
 * \brief Enable RTC square wave output
 *
 * Via I2C (TWI) bus, this function sets up the external RTC chip so a
 *  square wave output drives the SQW/INTB pin (7).
 */
uint8_t rtc_enableSqWave (void) {
	uint8_t r, d, wholeUnits;
	//Steps to enable Square Wave to drive SQW/INTB:
	// do START
	// write DS1342_ADDR_WRITE
	// write the memory address (DS1342_CONTROL), where to set/clear bits
	// write the data to that address
	// address automatically increments to next (DS1342_CONTROL_STATUS)
	// write the data to that address
	// do STOP
	//
	// salient bits
	//	RS2 = 1 and RS1 = 1, together specify 32.768kHz square wave
	//	A2IE = 0, Alarm 2 Interrupt disabled
	//	A1IE = 0, Alarm 1 Interrupt disabled
	//	INTCN = 0, square wave (rather than interrupt) output on SQW/INTB pin
	//	(ECLK ignored)

//	outputStringToWiredUART("\n\r entered enableAlarm1 routine \n\r");
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
//		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
//		    outputStringToWiredUART(str);
			if (r == TW_MT_SLA_ACK) {
			    r = I2C_Write(DS1342_CONTROL); // point to this register
//			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL): 0x%x\n\r", r);
//			    outputStringToWiredUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
#ifdef RTC_CHIP_IS_DS1337	// may not be relevant in this function
					// DS1337_CONTROL, 0x0e
					// Bit 7: Enable Oscillator (EOSC) = 0, enabled
					// Bit 6: No Function
					// Bit 5: No Function
					// Bits 4 and 3: Rate Select (RS[2:1]) = 00, 1Hz, don't care, not using square wave output
					// Bit 2: Interrupt Control (INTCN) = 1, Alarm 1 interrupt output on INTA
					// Bit 1: Alarm 2 Interrupt Enable (A2IE) = 0, disabled
					// Bit 0: Alarm 1 Interrupt Enable (A1IE) = 1, enabled
					r = I2C_Write(0b00000101); // write DS1337_CONTROL
					
					// DS1337_CONTROL_STATUS, 0x0f
					// Bit 7: Oscillator Stop Flag (OSF) = 0, clear, don't care
					// Bits 6 to 2: No Function
					// Bit 1: Alarm 2 Flag (A2F) = 0, clear, A2IE is disabled
					// Bit 0: Alarm 1 Flag (A1F) = 0, clear, A1IE is enabled, Alarm 1 will set
					r = I2C_Write(0b00000000); // write DS1337_CONTROL_STATUS
#endif
#ifdef RTC_CHIP_IS_DS1342
					// DS1342_CONTROL, 0x0e
					// Bit 7: Enable Oscillator (EOSC) = 0, enabled
					// Bit 6: No Function
					// Bit 5: Enable Glitch Filter (EGFIL) = 0, disabled, saves power
					// Bits 4 and 3: Rate Select (RS[2:1]) = 11, 32.768kHz square wave output
					// Bit 2: Interrupt Control (INTCN) = 0, output sq wave (rather than interrupt) on SQW/INTB
					// Bit 1: Alarm 2 Interrupt Enable (A2IE) = 0, disabled but overridden anyway by INTCN = 0
					// Bit 0: Alarm 1 Interrupt Enable (A1IE) = 0, enabled but overridden anyway by INTCN = 0
					r = I2C_Write(0b00011000); // write DS1342_CONTROL

					// DS1342_CONTROL_STATUS, 0x0f
					// Bit 7: Oscillator Stop Flag (OSF) = 0, clear, don't care
					// Bit 6: Disable Oscillator Stop Flag (DOSF) = 1, disable OSF, save power
					// Bit 5: Loss of Signal (LOS) = 0, clear, don't care
					// Bits 4 and 3: Select Clock Source (CLKSEL[2:1]) = 00, 1Hz don't care, not using external clock input
					// Bit 2: Enable External Clock Input (ECLK) = 1, input on INTA tied low & ignored
					// Bit 1: Alarm 2 Flag (A2F) = 0, clear, A2IE is disabled
					// Bit 0: Alarm 1 Flag (A1F) = 0, clear, A1IE is disabled
					r = I2C_Write(0b01000100); // write DS1342_CONTROL_STATUS
#endif
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
//				outputStringToWiredUART("\n\r exit from address device\n\r");
			} else { // could not address device
				I2C_Stop();
				return errNoI2CAddressAck;
			}
			I2C_Stop();
//		    outputStringToWiredUART("\n\r I2C_Stop completed \n\r");
		} else { // could not START
			return errNoI2CStart;
		}			
	return I2C_OK;
} // end of rtc_enableSqWave

/**
 * \brief Set default date/time
 *
 * This function sets the passed dateTime to
 *  a default value that can be used to 
 *  determine if the system time has been correctly
 *  set, and if not how long since system initialization
 * Default is winter solstice 2011
 *  date/time of the global dateTime struct dt_RTC
 *  to 
 *  2011-12-21 21:30:00 PST, offset 8 hours west from
 *  2011-12-22 05:30:00 UTC
 */
void datetime_getDefault(dateTime *t) {
	t->year = 11;
	t->month = 12;
	t->day = 21;
	t->houroffset = 0;
	t->hour = 5;
	t->minute = 30;
//	t->minute = 59; // for testing dead battery re-charging by solar cells,
	// when sufficient charge achieved, initialization loop will run one minute later on hour rollover
	t->second = 0;
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
}

/**
 * \brief Copies one dateTime struct to another
 *
 * This function takes pointers to 2 dateTime structs,
 *  and copies the second to the first
 *  This order like "equals" where the item being assigned is first
 * and the one being assigned from comes after
 */
void datetime_copy(dateTime *to, dateTime *from) {
	to->year = from->year;
	to->month = from->month;
	to->day = from->day;
	to->houroffset = from->houroffset;
	to->hour = from->hour;
	to->minute = from->minute;
	to->second = from->second;
}

/**
 * \brief gets the total seconds in dateTime
 *
 * This function takes a pointer to a dateTime struct,
 * It returns the total count of seconds, from Jan 1, 2000
 * Since this is for a data logger, there would never be earlier dates
 * It only works in the 100 years till Jan 1 2100
 * Return value is a 32 bit number, sufficient
 *  to hold the max possible: 3155760000
 */
uint32_t datetime_truesecs (dateTime *t) {
	// t->year is the number of completed years, e.g. 1 (meaning 2001) says 
	// 1 year of the century has elapsed.
	uint32_t hrs = t->year * 8766; // pre multiply 365.25 * 24 to use integers
	uint8_t m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint8_t i;
	if (((t->year) % 4) == 0) m[1] = 29; // deal with leap years
	for (i=0; i<(t->month); i++) {
		hrs += (m[i] * 24); // add on hours for the elapsed months
	}
	hrs -= (m[i] * 24); // take back the hours for the current month
	hrs += ((t->day) * 24); // add hours for the days
	hrs -= 24; // take back the hours for the current day
	hrs -= (t->houroffset); // adjust for time zone
	return (hrs * 60 * 60) + ((t->minute) * 60) + (t->second);
}


/**
 * \brief gets a compare value of seconds in dateTime
 *
 * This function takes a pointer to a dateTime struct,
 * It returns the seconds, from Jan 1, 2000
 * Since this is for a data logger, there would never be earlier dates
 * It only works in the 100 years till Jan 1 2100
 * Return value is a 32 bit number, sufficient
 *  to hold the max possible: 3155760000
 * Fn is actually inaccurate as written
 * Counts 1 more month and 1 more day than correct, but
 *  still gives consistent comparisons
 * Fn is left this way so "null" date, 2000-00-00 00:00:00
 *  will return 0 and compare as less than any real date
 */
uint32_t datetime_compareval_secs (dateTime *t) {
	// t->year is the number of completed years, e.g. 1 (meaning 2001) says 
	// 1 year of the century has elapsed. Doesn't really matter as long as 
	// it's consistent for comparisons
	uint32_t hrs = t->year * 8766; // pre multiply 365.25 * 24 to use integers
	uint8_t m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	if (((t->year) % 4) == 0) m[1] = 29; // deal with leap years
	for (uint8_t i=0; i<(t->month); i++) {
		hrs += (m[i] * 24); // add on hours for the elapsed months
	}
	hrs += ((t->day) * 24); // add hours for the days
	hrs -= (t->houroffset); // adjust for time zone
	return (hrs * 60 * 60) + ((t->minute) * 60) + (t->second);
}

/**
 * \brief compares two dateTime structs
 *
 * This function takes pointers to 2 dateTime structs,
 *  and compares the first to the second
 * If the first is earlier, as in normal time sequence, returns something >0
 * If the first is later, opposite to normal time sequence, returns <0
 * If they are equal, returns 0
 */
int8_t datetime_compare(dateTime *t1, dateTime *t2) {
	if (datetime_compareval_secs(t1) < datetime_compareval_secs(t2)) return 1;
	if (datetime_compareval_secs(t1) > datetime_compareval_secs(t2)) return -1;
	return 0;
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
//	outputStringToWiredUART("\n\r entered datetime_advanceIntervalShort fn \n\r\n\r");
//		outputStringToWiredUART("\n\r in datetime_advanceIntervalShort fn \n\r\n\r");
//		outputStringToWiredUART("\n\r passed time: ");
//		datetime_getstring(str, t);
//		outputStringToWiredUART(str);
//		outputStringToWiredUART("\n\r\n\r");
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
 * \brief sets date/time forward by the appropriate interval
 *
 * This function takes a pointer to a dateTime struct,
 *  and advances the time by the interval, long or short,
 *  specified by flags. 
 *
 * \note See the functions datetime_advanceIntervalLong
 *  and datetime_advanceIntervalShort
 */
void datetime_advanceInterval(dateTime *t) {
	uint8_t startingDay;
	startingDay = t->day;
	if (irradFlags.isDark)
		datetime_advanceIntervalLong(t);
	else
		datetime_advanceIntervalShort(t);
	if (t->day !=startingDay) { // day has rolled over
		stateFlags1.writeDataHeaders = 1; // flag to log column headers on next SD card write
	}
}

/**
 * \brief sets a date string forward by 1 day
 *
 * This function updates the passed date string (in the
 * format "2012-05-03") to the next valid date.
 *
 * \note 
 * 
 */
void datetime_advanceDatestring1Day(char* s) {
	char stDateTmp[27];
	dateTime t;
	strcpy(stDateTmp, s); // make a copy of the date string
	strcat(stDateTmp, " 01:01:01 +01"); // fill out full date/time with dummy vals
	datetime_getFromUnixString(&t, stDateTmp, 1);
	(t.day)++;
	datetime_normalize(&t);
	datetime_getstring(stDateTmp, &t);
	strncpy(s, stDateTmp, 10);
}

/**
 * \brief sets a date string forward to 1st of following month
 *
 * This function updates the passed date string (in the
 * format "2012-05-03") to the first of the following
 * month (e.g. 2012-06-01).
 *
 * \note 
 * 
 */
void datetime_advanceDatestring1stOfNextMonth(char* s) {
	char stDateTmp[27];
	dateTime t;
	strcpy(stDateTmp, s); // make a copy of the date string
	strcat(stDateTmp, " 01:01:01 +01"); // fill out full date/time with dummy vals
	datetime_getFromUnixString(&t, stDateTmp, 1);
	t.day = 1;
	(t.month)++;
	datetime_normalize(&t);
	datetime_getstring(stDateTmp, &t);
	strncpy(s, stDateTmp, 10);
}

/**
 * \brief finds the next date that has data
 *
 * This function updates the passed date string (in the
 * format "2012-05-03") to the next date
 * that has data on the system SD card.
 * Searches from the passed date up to the
 * present date.
 *
 * If forceAhead=0, can return passed date if it has data
 * If forceAhead=1, starts looking at date following the one passed
 *
 * Returns 0 if OK, 1 if no data found.
 * Will return 1 if system date/time not set.
 *
 * \note 
 * 
 */
uint8_t datetime_nextDateWithData(char* s, uint8_t forceAhead) {
	uint8_t fileFoundForDate = 0, fileErr = 0, fsRtn;
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	FILINFO fno;        // [OUT] FILINFO
	
//	int lenLocal;
//	char strLocal[128];
	char stDateEnd[27], stDateTry[12], stFullPath[14];
	if (rtcStatus <= rtcTimeSetToDefault) {
		outputStringToWiredUART("\n\r system time not valid\n\r");
		return 1;
	}
	
	strncpy(stDateEnd, datetime_string, 10);
	strcpy(stDateTry, s);
	if (forceAhead) {
//		outputStringToWiredUART("\n\r (forceAhead set)\n\r");
		datetime_advanceDatestring1Day(stDateTry);
	}
	while ((strcmp(stDateTry, stDateEnd) <= 0) && (!(fileFoundForDate)) && (!(fileErr))) {
		dateToFullFilepath(stDateTry, stFullPath);
//		outputStringToWiredUART("\r\n Date: ");
//		outputStringToWiredUART(stDateTry);
//		outputStringToWiredUART("\r\n\ FullPath: ");
//		outputStringToWiredUART(stFullPath);
//		outputStringToWiredUART("\r\n");

		if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
			outputStringToWiredUART("\r\n (power too low)\r\n");
			fsRtn = sdPowerTooLowForSDCard; // cell voltage is below threshold to safely access card
			break;
		}
		
		if(f_mount(0, &FileSystemObject)!=FR_OK) {
			outputStringToWiredUART("\r\n (could not mount SD drive)\r\n");
			fsRtn = sdMountFail;
			break;
		}

		res = f_stat(stFullPath, &fno);
//		lenLocal = sprintf(strLocal, "\n\r File stat return code: %d\n\r", res);
//		outputStringToWiredUART(strLocal);
		switch (res) {

			case FR_OK: {
				fileFoundForDate = 1;
				break;
			}
			
			case FR_NO_PATH: {
				datetime_advanceDatestring1stOfNextMonth(stDateTry);
				break;
			}
			
			case FR_NO_FILE: {
				datetime_advanceDatestring1Day(stDateTry);
				break;
			}
			
			default: {
				fileErr = 1;
				break;
			}
		} // end of switch (res)
	} // end of while stDateTry < stDateEnd
	unmountVolume:
	f_mount(0,0);
	
	if (fileErr) {
		return 1;
	}
	
	if (fileFoundForDate) {
		strcpy(s, stDateTry);
		return 0;
	}
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
//	outputStringToWiredUART("\n\r entered datetime_normalize fn \n\r\n\r");
	uint8_t m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	while ((t->second) > 59) {
		t->second -= 60;
		(t->minute)++;
	}
	while ((t->minute) > 59) {
		t->minute -= 60;
		(t->hour)++;
	}
	while ((t->hour) > 23) {
		t->hour -= 24;
		(t->day)++;
	}
	if (((t->year) % 4) == 0)
		m[1] = 29;
//	else
//		m[1] = 28;

	while ((t->day) > m[(t->month)-1]) {
		(t->day) -= m[(t->month)-1];
		(t->month)++;
		if ((t->month) > 12) {
			(t->month) = 1;
			(t->year)++;
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
//	if ((timeZoneOffset) < 0) {
		iTmp  = -1 * (dtp->houroffset);
//		iTmp  = -1 * (timeZoneOffset);
		dtstr[20] = '-';
	} else {
		iTmp  = (dtp->houroffset);
//		iTmp  = (timeZoneOffset);
		dtstr[20] = '+';
	}
	dtstr[21] = '0'+ (iTmp / 10);
	dtstr[22] = '0'+ (iTmp % 10);
}

/**
 * \brief from a string, creates date/time
 *
 * This function fills a data/time struct from
 *  a string. String must be in strict format
 *  20YY-MM-DD HH:MM:SS (century = 2000)
 *  example: "2010-12-21 10:47:13"
 *  Should previously test using fn 'isValidDateTime'
 *  If useGlobalTimeZone = true,
 *  gets timezone offset from the global 'timeZoneOffset'
 *  otherwise expects it in the 4 chars following date/time
 *  in the format ' +NN', e.g. ' +03' or ' -08'
 *  and sets global 'timeZoneOffset' from this
 */
void datetime_getFromUnixString(dateTime *dtp, char* dtstr, bool useGlobalTimeZone)
{

	dtp->year = (uint8_t)(10 * (dtstr[2] - '0')) + (dtstr[3] - '0');
	dtp->month = (10 * (dtstr[5] & 0xf)) + (dtstr[6] & 0xf);
	dtp->day = (10 * (dtstr[8] - '0')) + (dtstr[9] - '0');
	dtp->hour = (10 * (dtstr[11] - '0')) + (dtstr[12] - '0');
	dtp->minute = (10 * (dtstr[14] - '0')) + (dtstr[15] - '0');
	dtp->second = (10 * (dtstr[17] - '0')) + (dtstr[18] - '0');
	if (useGlobalTimeZone) {
		dtp->houroffset = timeZoneOffset;
	} else {
		dtp->houroffset = (10 * (dtstr[21] - '0')) + (dtstr[22] - '0');
		if (dtstr[20] == '-') {
			dtp->houroffset *= -1;
		}
		timeZoneOffset = dtp->houroffset;
	}
}

/**
 * \brief  Is the string a valid date/time?
 *
 *  This function verifies that the string in the passed
 * text buffer is a valid date/time.
 *  Input: char pointer
 *  Output: uint8_t, used as boolean; 1 = true, 0 = false
 *  Precondition: date/time starting at pointer, in strict format:
 *  20YY-MM-DD HH:MM:SS (century must be 2000)
 *  use this fn before trying to set the time
 *  This fn does not check for anything after the date/time
 * so control can pass other parameters, such as GPS
 * coordinates, in that following space
 */

uint8_t isValidDateTime(char* p)
{
    // example: "2010-12-21 10:47:13"
	if (isValidDate(p) && (*(p+10) == ' ') && isValidTime(p+11)) return 1;
	return 0;
} // end of isValidDateTime

/**
 * \brief  Is the string a valid date?
 *
 *  This function verifies that the string in the passed
 * text buffer is a valid date.
 *  Input: char pointer
 *  Output: uint8_t, used as boolean; 1 = true, 0 = false
 *  Precondition: date starting at pointer, in strict format:
 *  20YY-MM-DD (century must be 2000)
 */

uint8_t isValidDate(char* p)
{
	uint8_t i, y, m, d;
    // example: "2010-12-21"
    // year
    if (*p++ != '2') // must always start with '20'
        return 0;
    if (*p++ != '0')
        return 0;
    i = *p++;
    if ((i < '0') || (i > '9')) // tens of year
        return 0;
	y = 10 * (i & 0xf);
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of year
        return 0;
	y += (i & 0xf);
    if (*p++ != '-') // correct delimiter
        return 0;
    // month
    i = *p++;
    if ((i < '0') || (i > '1')) // tens of month
        return 0;
    m = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
//    m = 10 * (i - 48); // convert from ASCII to number
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of month
        return 0;
    m += (i & 0xf); // strip all but low nybble to convert to BCD
    if (m > 12) 
        return 0;
    if (*p++ != '-') // correct delimiter
        return 0;
    // day
    i = *p++;
    if ((i < '0') || (i > '3')) // tens of day
        return 0;
    d = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of day
        return 0;
    d += (i & 0xf); // strip all but low nybble to convert to BCD
    if (d > 31) // takes care of January, March, May, July, August, October, and December
        return 0;
    if ((m == 2)) { // February
		if (d > 29) return 0;
		if (d > 28) { // leap year?
			if (y % 4) return 0; 
			// ignore century test, device will be obsolete by year 2100
		}
	}		
    if ((m == 4) && (d > 30)) // April
        return 0;
    if ((m == 6) && (d > 30)) // June
        return 0;
    if ((m == 9) && (d > 30)) // September
        return 0;
    if ((m == 11) && (d > 30)) // November
        return 0;
    // passed all tests
    return 1;
} // end of isValidDate

/**
 * \brief  Is the string a valid time?
 *
 *  This function verifies that the string in the passed
 * text buffer is a valid time.
 *  Input: char pointer
 *  Output: uint8_t, used as boolean; 1 = true, 0 = false
 *  Precondition: time starting at pointer, in strict format:
 *  HH:MM:SS
 */

uint8_t isValidTime(char* p)
{
	uint8_t i, v;
    // example: "21:47:13"
    // hour
    i = *p++;
    if ((i < '0') || (i > '2')) // tens of hour
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of hour
        return 0;
    v += (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 23) 
        return 0;
	// kludge, Blueterm replaces input colon with semicolon
	if (*p == ';') *p = ':'; // fix it in the original string
    if (*p++ != ':') // correct delimiter
        return 0;
    // minute
    i = *p++;
    if ((i < '0') || (i > '5')) // tens of minute
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of minute
        return 0;
    v += (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 59) 
        return 0;
    if (*p == ';') *p = ':';
    if (*p++ != ':') // correct delimiter
        return 0;
    // second
    i = *p++;
    if ((i < '0') || (i > '5')) // tens of second
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of second
        return 0;
    v += (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 59) 
        return 0;
    // passed all tests
    return 1;
} // end of isValidTime


/**
 * \brief  Is the string a valid timezone offset?
 *
 *  This function verifies that the string in the passed
 * text buffer is a valid timezone offset.
 *  Input: char pointer
 *  Output: int, used as boolean; 1 = true, 0 = false
 *  Precondition: hour offset starting at pointer, in strict format:
 *  +00 ('+' means plus or minus character, '00' is zero to twelve with leading zero if <10)
 *  use this fn before trying to set the time, if hour offset is needed.
 */

uint8_t isValidTimezone(char* p)
{
    uint8_t i, h;
    // examples: "-10", "+03"
    // sign
    i = *p++;
    if (!((i == '-') || (i == '+'))) // correct sign
        return 0;

    i = *p++;
    if ((i < '0') || (i > '1')) // tens of hour
        return 0;
    h = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of hour
        return 0;
    h = h + (i & 0xf); // strip all but low nybble to convert to BCD
    if (h > 12) 
        return 0;
    // passed all tests
    return 1;
} // end of isValidTimezone


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