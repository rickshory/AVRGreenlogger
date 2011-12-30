/**
 * \file RTC.c
 *
 * \brief Real Time Clock code for the ATmega1284P
 *
 * Copyright (C) 2011 Rick Shory. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
/**
 * \mainpage
 * \section Real Time Clock
 * \section intro Introduction
 * Functions to use the Real Time Clock
 *
 * \section files Files:
 * - RTC.c: ATmega1284P Real Time Clock
 *
 * \section exampledescription Brief description of the example application
 * This section has functions to initialized the Real Time Clock,
 * set alarms and keep time
 *
 * \note 
 * 
 *
 * \section compinfo Compilation Info
 * This software was written for the <A href="http://gcc.gnu.org/">GNU GCC</A>
 * for AVR. \n
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.rickshory.com/">rickshory.com</A>.\n
 * Support and FAQ: http://support.rickshory.com/
 */
/**



*/
#include "RTC.h"
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
void datetime_getstring(char* dtstr, struct DateTime *dtp)
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