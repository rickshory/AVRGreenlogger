/**
 * \file
 *
 * \brief code for NDVI (greeness) logger, base on AVR ATmega1284P
 *
 * Copyright (C) 2011 Rick Shory, based in part on source code that is:
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
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
 *
 * greenlogger.c
 *
 * Created: 12/21/2011 10:04:15 AM
 *  Author: Rick Shory
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include "greenlogger.h"
#include "config/RTC.h"
#include <inttypes.h>
#include "SDcard/ff.h"
#include "SDcard/diskio.h"
#include "diagnostics/diagnostics.h"
#include <util/twi.h>
#include "I2C/I2C.h"
#include "Accelerometer/ADXL345.h"
#include "LtSensor/TSL2561.h"
#include "TemperatureSensor/TCN75A.h"
#include "BattMonitor/ADconvert.h"

volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker

volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep

volatile
uint8_t Timer1, Timer2;	/* 100Hz decrement timer */


int len, err = 0;
char str[128]; // generic space for strings to be output
char strJSON[128]; // string for JSON data

// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
//char* dt_stp = datetime_string;
//DateTime RTC_DT;
char commandBuffer[commandBufferLen];
char *commandBufferPtr;

volatile uint8_t stateFlags1 = 0;

extern struct DateTime RTC_dt;
//struct 
extern irrData irrReadings[4];

extern adcData cellVoltageReading;


/**
 * \brief The main application
 *
 * This application logs visible and infrared irradiance levels to an SD card
 * 
 *
 * \note 
 * 
 */
int main(void)
{
//	uint8_t data;
	uint8_t cnt;
	uint16_t cntout = 0;
//	stateFlags1 &= ~((1<<timeHasBeenSet) | (1<<timerHasBeenSynchronized));
	cli();
	setupDiagnostics();
	uart_init();
	commandBuffer[0] = '\0';
	commandBufferPtr = commandBuffer; // "empty" the command buffer
	sei();
    outputStringToUART("\r\n  enter I2C_Init\r\n");
	I2C_Init(); // enable I2C 
	outputStringToUART("\r\n  I2C_Init completed\r\n");

	// Send the test string
	for (cnt = 0; cnt < strlen(test_string); cnt++) {
		uart_putchar(test_string[cnt]);
	}
	rtc_setdefault();
    // output a counter
	while (1) {
		if (stateFlags1 & (1<<isRoused)) {
			outputStringToUART("\r\n  roused\r\n");
		} else { // allow to be roused again
			enableAccelInterrupt();
		}			
		itoa(cntout, num_string, 10);
/*
		for (cnt = 0; cnt < strlen(num_string); cnt++) {
			uart_putchar(num_string[cnt]);
		}
*/

		
		outputStringToUART(num_string);
//		uart_putchar('\t');
		outputStringToUART("\t");
		datetime_getstring(datetime_string, &RTC_dt);
/*		
		for (cnt = 0; cnt < strlen(datetime_string); cnt++) {
			uart_putchar(datetime_string[cnt]);
		}		
*/
		outputStringToUART(datetime_string);
		outputStringToUART("\r\n");
//		uart_putchar('\n');
		delay_ms(1000);
		cntout++;
		if (cntout == 65500) 
			cntout = 0;
		rtc_add1sec();
		checkForCommands();
	}
/*
	// Check if we have received the string we sent
	cnt = 0;
	do {
		// Wait for next character
		while (!uart_char_waiting_in());
		data = uart_getchar();
		// Compare to what we sent
//		Assert (data == test_string[cnt++]);
	} while (cnt < strlen(test_string));
*/	

	while (1);
}

/**
 * \brief Send a string out the uart
 *
 * Copy characters from the passed null-terminated
 * string to the UART output buffer. Inline fn "uart_putchar"
 * flags to start transmitting, if necessary.
 *
 * \
 *  
 */

void outputStringToUART (char* St) {
	uint8_t cnt;
	for (cnt = 0; cnt < strlen(St); cnt++) {
		uart_putchar(St[cnt]);
	}
	while (uart_char_queued_out())
		;
} // end of outputStringToUART

/**
 * \brief Check the uart for commands
 *
 * Check UART receive buffer for commands
 *
 * \
 *  
 */

void checkForCommands (void) {
    char c;
    while (1) {
		if (!uart_char_waiting_in()) 
			return;
		
		c = uart_getchar();
        if (c == 0x0d) // if carriage-return
            c = 0x0a; // substitute linefeed
        if (c == 0x0a) { // if linefeed, attempt to parse the command
            *commandBufferPtr++ = '\0'; // null terminate
            switch (commandBuffer[0]) { // command is 1st char in buffer

                case 'C': case 'c':
					{ // sd card diagnostics
						outputStringToUART("\r\n SD card diagnostics\r\n");
					//	len = sprintf(str, "\n\r PINB: 0x%x\n\r", (PINB));
						// send_cmd(CMD0, 0)
//					len = sprintf(str, "\n\r CMD0: 0x%x\n\r", (send_cmd(CMD0, 0)));
						
						len = sprintf(str, "\n\r disk_initialize: 0x%x\n\r", (disk_initialize(0)));
						outputStringToUART(str);
{
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code

if(f_mount(0, &FileSystemObject)!=FR_OK) {
	//  flag error
	len = sprintf(str, "\n\r f_mount failed: 0x%x\n\r", 0);
	outputStringToUART(str);
}

DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT
	) {
//	flag error.
	len = sprintf(str, "\n\r disk_initialize failed; driveStatus: 0x%x\n\r", driveStatus);
	outputStringToUART(str);
}

/* Sometimes you may want to format the disk.
if(f_mkfs(0,0,0)!=FR_OK) {
//error
}
*/

if (f_mkdir("0000")) {
	len = sprintf(str, "\n\r f_mkdir failed: 0x%x\n\r", 0);
	outputStringToUART(str);	
}

FIL logFile;
//works
if(f_open(&logFile, "0000/GpsLog.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK) {
	len = sprintf(str, "\n\r f_open failed: 0x%x\n\r", 0);
	outputStringToUART(str);
//flag error
}
//	len = sprintf(str, "\n\r f_size : 0x%x\n\r", f_size(&logFile));
	len = sprintf(str, "\n\r f_size : 0x%x\n\r", (uint16_t)f_size(&logFile));
	outputStringToUART(str);


	// Move to end of the file to append data
	res = f_lseek(&logFile, f_size(&logFile));

unsigned int bytesWritten;
f_write(&logFile, "New log opened!\n", 16, &bytesWritten);
	len = sprintf(str, "\n\r test file written: 0x%x\n\r", 0);
	outputStringToUART(str);
//Flush the write buffer with f_sync(&logFile);

//Close and unmount.
f_close(&logFile);
f_mount(0,0);
}

						break;						
					}

                case 'P': case 'p': 
				{ // force SD card power off
                    outputStringToUART("\r\n turning SD card power off\r\n");
					turnSDCardPowerOff();
					outputStringToUART("\r\n SD card power turned off\r\n");
                    break;
                }

                case '^':
				{ // force B3 high
                    outputStringToUART("\r\n forcing B3 high\r\n");
					DDRB |= 0b00001000;
					PORTB |= 0b00001000;
					outputStringToUART("\r\n B3 forced high \r\n");
                    break;
                }

                case 'v':
				{ // force B3 low
                    outputStringToUART("\r\n forcing B3 low\r\n");
					DDRB |= 0b00001000;
					PORTB &= 0b11110111;
					outputStringToUART("\r\n B3 forced low \r\n");
                    break;
                }
				case 'A': case 'a':
					{
						findADXL345();
						break;
					}

                case 'F': case 'f':
					{ // experimenting with temperature functions
						if (!initOneShotTemperatureReading()) {
							// temperature conversion time, typically 30ms
							for (Timer2 = 4; Timer2; );	// Wait for 40ms to be sure
							if (!getTemperatureReading(&temperatureReading)) {
								len = sprintf(str, "\n\r Temperature: %d degrees C\n\r", (temperatureReading.tmprHiByte));
								outputStringToUART(str);

							}
						}

						break;
					}						

                case 'I': case 'i':
					{ // experimenting with irradiance functions
						if (getIrrReading(TSL2561_UpLooking, TSL2561_CHANNEL_BROADBAND, &irrReadings[0]))
							;
						break;
					}						

                case 'B': case 'b':
					{ // experimenting with reading the battery voltage using the Analog to Digital converter
						len = sprintf(str, "\n\r Hi byte: %d \n\r", readCellVoltage(&cellVoltageReading));
						outputStringToUART(str);
						len = sprintf(str, "\n\r 16bit value: %d \n\r", cellVoltageReading.adcWholeWord);
						outputStringToUART(str);
						

						break;
					}						

                case 'T': case 't':
					{ // experimenting with time functions
						if (rtc_setTime(RTC_dt))
							;
						break;
					}						

/*
                case 'T': case 't':
					{ // set time
                    if (!isValidTimestamp(commandBuffer + 1)) {
                         outputStringToUART("\r\n Invalid timestamp\r\n");
                         break;
                    }    
                    if (!isValidTimezone(commandBuffer + 20)) {
                         outputStringToUART("\r\n Invalid hour offset\r\n");
                         break;
                    }    
                    outputStringToUART("\r\n Time changed from ");
                    strcpy(strJSON, "\r\n{\"timechange\":{\"from\":\"");
                    createTimestamp();
                    strcat(strJSON, timeStampBuffer + 2);
                    outputStringToUART(timeStampBuffer + 2);
                    strcat(strJSON, timeZoneBuffer);
                    outputStringToUART(timeZoneBuffer);
                    setTimeFromCharBuffer(commandBuffer + 3);
                    strncpy(timeZoneBuffer, commandBuffer + 20, 3);
                    strcat(strJSON, "\",\"to\":\"");
                    outputStringToUART(" to ");
                    createTimestamp();
                    strcat(strJSON, timeStampBuffer + 2);
                    outputStringToUART(timeStampBuffer + 2);
                    strcat(strJSON, timeZoneBuffer);
                    outputStringToUART(timeZoneBuffer);
                    strcat(strJSON, "\",\"by\":\"hand\"}}\r\n");
                    outputStringToUART("\r\n");
                    flags1.writeDataHeaders = 1; // log data column headers on next SD card write
                    stateFlags.timeHasBeenSet = 1; // presumably is now the correct time
                    stateFlags.timerHasBeenSynchronized = 0; // but timer not guaranteed to happen on 0 of seconds
                    startTimer1ToRunThisManySeconds(30); // keep system Roused
                    break;
                }
*/

                case 'D': case 'd': 
				{ // output file 'D'ata (or 'D'ump)
                    outputStringToUART("\r\n output file data\r\n");
                    break;
                }
/*
                case 'O': { // toggle to use SD card, or ignore it
                    if (flags1.useSDcard) {
                        flags1.useSDcard = 0;
                        outputStringToUART("\r\n Will now ignore SD card\r\n");
                    } else {
                        flags1.useSDcard = 1;
                        outputStringToUART("\r\n Will now write to SD card\r\n");
                    }
                    break;
                }
 */
/*
                case 'P': { // SD card power control
                    if (stateFlags_2.turnSDCardOffBetweenDataWrites) {
                        stateFlags_2.turnSDCardOffBetweenDataWrites = 0;
                        outputStringToUART("\r\n No SD card power control \r\n");
                    } else { 
                        stateFlags_2.turnSDCardOffBetweenDataWrites = 1;
                        SD_POWER_TRIS = 0; // assure the pin is an output, source of the SD card power under software control
                    }
                    setSDCardPowerControl(); // based on flag
                    break;
                } 
*/
/*
                case 'L': { // toggle Leveling diagnostics
                    if (stateFlags.isLeveling) {
                        stateFlags.isLeveling = 0;;
                        outputStringToUART("\r\n Ignore accelerometer output\r\n");
                    } else {
                        stateFlags.isLeveling = 1;
                        outputStringToUART("\r\n Accelerometer output enabled\r\n");
                    }
                    break;
                } 
*/
/*
                case 'S': { // toggle to sleep between readings, or not
                    if (flags1.sleepBetweenReadings) {
                        flags1.sleepBetweenReadings = 0;
                        outputStringToUART("\r\n Will stay awake continually\r\n");
                    } else {
                        flags1.sleepBetweenReadings = 1;
                        outputStringToUART("\r\n Will now sleep between readings\r\n");
                    }
                    break;
                } 
*/
/*

                case 'R': { // test the 'Rouse' bit
                    if (stateFlags.isRoused) {
                        stateFlags.isRoused = 0;
                        outputStringToUART("\r\n Rouse bit cleared\r\n");
                    } else {
                        stateFlags.isRoused = 1;
                        outputStringToUART("\r\n Rouse bit set\r\n");
                    }
                    break;
                } 
*/

/*
                case 'G': { // toggle irradiance gain
                    if (stateFlags_2.irrSensorGain) {
                        stateFlags_2.irrSensorGain = 0;
                        outputStringToUART("\r\n Low gain\r\n");
                    } else {
                        stateFlags_2.irrSensorGain = 1;
                        outputStringToUART("\r\n High gain\r\n");
                    }
                    break;
                } 
*/
                // put other commands here
                default: 
				{ // if no valid command, echo back the input
                    outputStringToUART("\r\n> ");
                    outputStringToUART(commandBuffer);
                    outputStringToUART("\r\n");
//                    startTimer1ToRunThisManySeconds(30); // keep system Roused another two minutes
					break;
                }
            } // switch (commandBuffer[0])
            commandBuffer[0] = '\0';
            commandBufferPtr = commandBuffer; // "empty" the command buffer
         } else { // some other character
             // ignore repeated linefeed (or linefeed following carriage return) or carriage return
             if (!((c == 0x0a) || (c == 0x0a))) { 
                 if (commandBufferPtr < (commandBuffer + commandBufferLen - 1)) // if there is room
                     *commandBufferPtr++ = c; // append char to the command buffer
             }
         } // done parsing character
    } // while (1)
} // end of checkForCommands

/**
 * \brief internal system heartbeat
 *
 * This function is called every 10ms by the Timer3 interrupt
 *
 * \note 
 */

void heartBeat (void)
{
//	static BYTE pv;
//	BYTE n, s;
	BYTE n;
	
	if (--ToggleCountdown <= 0) 
	{
//		PORTA ^= 0xFF;
		PORTA ^= 0x01;
		ToggleCountdown = TOGGLE_INTERVAL;
	}
	
	if (--rouseCountdown <= 0)
	{
		stateFlags1 &= ~(1<<isRoused);
	}

	n = Timer1;						/* 100Hz decrement timer */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;
/*
	n = pv;
	pv = SOCKPORT & (SOCKWP | SOCKINS);	// Sample socket switch

	if (n == pv) {					// Have contacts stabilized?
		s = Stat;

		if (pv & SOCKWP)			// WP is H (write protected)
			s |= STA_PROTECT;
		else						// WP is L (write enabled)
			s &= ~STA_PROTECT;

		if (pv & SOCKINS)			// INS = H (Socket empty)
			s |= (STA_NODISK | STA_NOINIT);
		else						// INS = L (Card inserted)
			s &= ~STA_NODISK;

		Stat = s;
	}
*/
}

