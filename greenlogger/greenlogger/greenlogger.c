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
/*
 * greenlogger.c
 *
 * Created: 12/21/2011 10:04:15 AM
 *  Author: Rick Shory
 */ 

#include <avr/io.h>
#include "greenlogger.h"
#include "SDcard/ff.h"
#include "SDcard/diskio.h"

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

extern RTC_dt;

/**
 * \brief The main application
 *
 * This application will initialize the UART, send a character and then receive
 * it and check if it is the same character as was sent.
 *
 * \note The RX and TX pins should be externally connected in order to pass the
 * test.
 */
int main(void)
{
	uint8_t data;
	uint8_t cnt;
	uint16_t cntout = 0;
	cli();
	setupDiagnostics();
	uart_init();
	commandBuffer[0] = '\0';
	commandBufferPtr = commandBuffer; // "empty" the command buffer
	sei();

	// Send the test string
	for (cnt = 0; cnt < strlen(test_string); cnt++) {
		uart_putchar(test_string[cnt]);
	}
	rtc_setdefault();
    // output a counter
	do {
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
		rtc_add1sec();
		checkForCommands();
	} while (cntout < 65500);
/*
	// Check if we have received the string we sent
	cnt = 0;
	do {
		// Wait for next character
		while (!uart_char_waiting());
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
	
/*
    char *tmp;
    if (!stateFlags.isRoused) // if system is asleep, or about to go to sleep
        return; // don't bother with any output till wakened
    if (_U1MD) { // UART module 1 was disabled to save power during sleep
        _U1MD = 0; // remove disable from UART module 1
        initUSART();
        __asm__ ("nop"); // allow time for wakeup
        __asm__ ("nop");
    }
    _U1TXIE = 1; // enable Tx interrupt so all bytes in buffer will be sent
    for (; *St != '\0'; St++) { // make sure there is room in USART1_outputbuffer
        __asm__ volatile("disi #0x3FFF"); // disable interrupts while we check & update the buffer
        tmp = (char*)(USART1_outputbuffer_head + 1);
        if (tmp >= (&USART1_outputbuffer[0] + USART1_Buffer_Length))
            tmp = (char*)(&USART1_outputbuffer[0]); // rollover to beginning if necessary
        // head=tail is the test for a full buffer
        if (tmp != USART1_outputbuffer_tail) { // there is room in USART1_outputbuffer
            *USART1_outputbuffer_head = *St;
            USART1_outputbuffer_head = (char*)tmp;
        }
        __asm__ volatile("disi #0x0000"); // enable interrupts
        _U1TXIE = 1; // enable Tx interrupt; interrupt will then occur as soon as Transmit 
        //  buffer has moved contents into Transmit Shift Register
    }
*/
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
		if (!uart_char_waiting()) 
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
	// Move to end of the file to append data
//	res = f_lseek(&logFile, (&logFile->fsize));

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

