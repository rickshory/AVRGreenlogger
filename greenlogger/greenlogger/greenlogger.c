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
#include <inttypes.h>
#include "SDcard/ff.h"
#include "SDcard/diskio.h"
#include "diagnostics/diagnostics.h"
#include <util/twi.h>
#include "I2C/I2C.h"
#include "RTC/DS1342.h"
#include "Accelerometer/ADXL345.h"
#include "LtSensor/TSL2561.h"
#include "TemperatureSensor/TCN75A.h"
#include "BattMonitor/ADconvert.h"

volatile uint8_t machineState;
volatile uint8_t iTmp;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep

volatile
uint8_t Timer1, Timer2, intTmp1;	/* 100Hz decrement timer */

int len, err = 0;
char str[128]; // generic space for strings to be output
char strJSON[128]; // string for JSON data
char strHdr[64] = "\n\rTimestamp\tBBDn\tIRDn\tBBUp\tIRUp\tT(C)\tVbatt(mV)\n\r";
char strLog[64];

// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
char commandBuffer[commandBufferLen];
char *commandBufferPtr;

volatile uint8_t stateFlags1 = 0, stateFlags2 = 0;
volatile uint8_t rtcStatus = rtcTimeNotSet;
volatile dateTime dt_RTC, dt_CurAlarm, dt_tmp, dt_LatestGPS; //, dt_NextAlarm
volatile uint8_t timeZoneOffset = 0; // globally available
volatile accelAxisData accelData;
extern irrData irrReadings[4];
volatile extern adcData cellVoltageReading;

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
	uint8_t ct, swDnUp, swBbIr;
	uint8_t errSD, cnt;
	uint16_t cntout = 0;
//	stateFlags1 &= ~((1<<timeHasBeenSet) | (1<<timerHasBeenSynchronized));
	// PortD, bit 4 controls power to the Bluetooth module
	// high = enabled
	DDRD |= (1<<4); // make output
	PORTD |= (1<<4); // set high; for testing make always-on
//	PORTD &= ~(1<<4); // set low; for testing make always-off
	// PortD, bit controls the BAUD rate of the Bluetooth module
	// high = 9600
	// low = 115k or firmware setting
	// for testing, hold at 9600
	DDRD |= (1<<7); // make output
	PORTD |= (1<<7); // set high
	
	cli();
	setupDiagnostics();
	uart_init();
	outputStringToUART("\r\n  UART Initialized\r\n");
	commandBuffer[0] = '\0';
	commandBufferPtr = commandBuffer; // "empty" the command buffer
	stateFlags1 |= (1<<writeDataHeaders); // write column headers at least once on startup
	sei();
	intTmp1 = readCellVoltage(&cellVoltageReading);
    outputStringToUART("\r\n  enter I2C_Init\r\n");
	I2C_Init(); // enable I2C 
	outputStringToUART("\r\n  I2C_Init completed\r\n");
	
	initializeADXL345();
	outputStringToUART("\r\n ADXL345 initialized\r\n");

//	// Send the test string
//	for (cnt = 0; cnt < strlen(test_string); cnt++) {
//		uart_putchar(test_string[cnt]);
//	}
	intTmp1 = rtc_readTime(&dt_RTC);
	strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
	datetime_getstring(datetime_string, &dt_RTC);
	strcat(strJSON, datetime_string);
	strcat(strJSON, "\",\"to\":\"");
	
	if (dt_RTC.year) { // 0 on power up, otherwise must have been set
		rtcStatus = rtcTimeManuallySet;
		strcat(strJSON, datetime_string);
		strcat(strJSON, "\",\"by\":\"retained\"}}\r\n");
	} else {
		rtc_setdefault();
		if (!rtc_setTime(&dt_RTC)) {
			rtcStatus = rtcTimeSetToDefault;
			outputStringToUART("\n\r time set to default ");
			datetime_getstring(datetime_string, &dt_RTC);
			outputStringToUART(datetime_string);
			outputStringToUART("\n\r\n\r");
			strcat(strJSON, datetime_string);
			strcat(strJSON, "\",\"by\":\"default\"}}\r\n");
		} else {
			outputStringToUART("\n\r could not set Real Time Clock \n\r");
			strcat(strJSON, datetime_string);
			strcat(strJSON, "\",\"by\":\"failure\"}}\r\n");
		}
	}
	stateFlags1 |= (1<<writeJSONMsg); // log JSON message on next SD card write
	
	stateFlags2 &= ~(1<<nextAlarmSet); // alarm not set yet
//	enableRTCInterrupt();
//	machineState = Idle;
	stayRoused(20);
 
	while (1) { // main program loop
		if (!(stateFlags2 & (1<<nextAlarmSet)))
		{
//			outputStringToUART("\n\r about to call setupNextAlarm \n\r\n\r");
			intTmp1 = rtc_setupNextAlarm(&dt_CurAlarm);
			stateFlags2 |= (1<<nextAlarmSet);
		}		
		

		machineState = Idle; // beginning, or done with everything; return to Idle state

		while (machineState == Idle) { // RTC interrupt will break out of this
			;
			checkForCommands();

		} // end of (machState == Idle)
		// when (machState != Idle) execution passes on from this point
//		outputStringToUART("\n\r RTC alarm occurred \n\r\n\r");
		// when RTCC occurs, changes machineState to GettingTimestamp
		stateFlags2 &= ~(1<<nextAlarmSet);
//		datetime_copy(&dt_NextAlarm, &dt_CurAlarm);
//		datetime_advanceInterval(&dt_NextAlarm);
//		if (!rtc_setAlarm1(&dt_NextAlarm)) {
//			;
//			outputStringToUART("\n\r Alarm1 set \n\r\n\r");
//			datetime_getstring(str, &dt_NextAlarm);
//			outputStringToUART(str);
//			outputStringToUART("\n\r\n\r");
//		}
//		if (!rtc_enableAlarm1()) {
//			;
//			outputStringToUART("\n\r Alarm1 enabled \n\r\n\r");
//		}
//		enableRTCInterrupt();
		stayRoused(3);

		while (1) { // various tests may break early
			;
//		setSDCardPowerControl();
			// monitor cell voltage, to decide whether there is enough power to proceed

			intTmp1 = readCellVoltage(&cellVoltageReading);
			//if (stateFlags1 & (1<<isRoused)) {
			//	outputStringToUART("\r\n  roused\r\n");
			//	// timer diagnostics
			//	len = sprintf(str, "\r\n countdown: %u seconds\r\n", (rouseCountdown/100));
			//	outputStringToUART(str);
			//}
			
			datetime_getstring(datetime_string, &dt_CurAlarm);
			outputStringToUART(datetime_string);
			if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_READ_DATA) {
				len = sprintf(str, "\t power too low to read sensors, %lumV\r\n", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
				outputStringToUART(str);
				break;
			}
			for (ct = 0; ct < 4; ct++) {
				switch (ct)
				{
				case 0:
					swDnUp = TSL2561_DnLooking;
					swBbIr = TSL2561_CHANNEL_BROADBAND;
					break;
				case 1:
					swDnUp = TSL2561_DnLooking;
					swBbIr = TSL2561_CHANNEL_INFRARED;
					break;
				case 2:
					swDnUp = TSL2561_UpLooking;
					swBbIr = TSL2561_CHANNEL_BROADBAND;
					break;
				case 3:
					swDnUp = TSL2561_UpLooking;
					swBbIr = TSL2561_CHANNEL_INFRARED;
					break;
				}
				intTmp1 = getIrrReading(swDnUp, swBbIr, &irrReadings[ct]);
				if (!intTmp1) {
					len = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier));
				} else if (intTmp1 == errNoI2CAddressAck) { // not present or not responding
					len = sprintf(str, "\t-");
				} else {
					len = sprintf(str, "\n\r Could not get reading, err code: %x \n\r", intTmp1);
				}						
				outputStringToUART(str);
			}
			// get temperature
//			if (!temperature_InitOneShotReading()) {
//				// temperature conversion time, typically 30ms
//				for (Timer2 = 4; Timer2; );	// Wait for 40ms to be sure
			if (!temperature_GetReading(&temperatureReading)) {
					len = sprintf(str, "\t%d", (temperatureReading.tmprHiByte));
					outputStringToUART(str);
			} else
				outputStringToUART("\t-");
//			} else
//				outputStringToUART("\t-");
			
			// calc cell voltage from ADC reading earlier
			// formula from datasheet: V(measured) = adcResult * (1024 / Vref)
			// using internal reference, Vref = 2.56V = 2560mV
			// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
			len = sprintf(str, "\t%lu", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
			outputStringToUART(str);
			outputStringToUART("\r\n");
			
//			outputStringToUART("\n\r data output completed \n\r\n\r");
			
//        if (stateFlags.isRoused) { 
//            // if roused, and Bluetooth is on, flag to keep BT on awhile after normal rouse timeout
//            // createTimestamp sets secsSince1Jan2000
//            timeToTurnOffBT = secsSince1Jan2000 + SECS_BT_HOLDS_POWER_AFTER_SLEEP;
//        }
//			if (dt_CurAlarm.second == 0) {
			if (!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark
				stateFlags1 |= (1<<timeToLogData);
			//	outputStringToUART("\n\r Time to log data \n\r");
			} else {
				stateFlags1 &= ~(1<<timeToLogData);
			//	outputStringToUART("\n\r Not time to log data \n\r");
			}			

			if (stateFlags1 & (1<<timeToLogData)) {
//				outputStringToUART("\n\r Entered log data routine \n\r");
				// log irradiance
				strcpy(strLog, "\n\r");
//				len = sprintf(str, "\n\r");
//				intTmp1 = writeCharsToSDCard(str, len);
				strcat(strLog, datetime_string);
//				len = sprintf(str, datetime_string);
//				intTmp1 = writeCharsToSDCard(str, len);
				for (ct = 0; ct < 4; ct++) { // write irradiance readings
					if (!(irrReadings[ct].validation)) {
						len = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier));
					} else { // no valid data for this reading
						len = sprintf(str, "\t");
					}
					strcat(strLog, str);
//					intTmp1 = writeCharsToSDCard(str, len);
				}
				// log temperature
				if (!temperatureReading.verification)
					len = sprintf(str, "\t%d", (temperatureReading.tmprHiByte));
				else
					len = sprintf(str, "\t");
				strcat(strLog, str);
//				intTmp1 = writeCharsToSDCard(str, len);
				// log cell voltage
				len = sprintf(str, "\t%lu\n\r", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
				strcat(strLog, str);
//				intTmp1 = writeCharsToSDCard(str, len);
//				len = sprintf(str, "\n\r");
//				intTmp1 = writeCharsToSDCard(str, len);
				
				len = strlen(strLog);
				errSD = writeCharsToSDCard(strLog, len);
					if (errSD) {
						tellFileWriteError (errSD);
//                stateFlags_2.isDataWriteTime = 0; // prevent trying to write anything later
					} else {
						outputStringToUART(" Data written to SD card \n\r\n\r");
					}
			}
			// let main loop restore Idle state, after assuring timer interrupts are re-established
			break; // if did everything, break here
		} // end of getting data readings
		
/*
		if (stateFlags1 & (1<<isRoused)) {
			outputStringToUART("\r\n  roused\r\n");
		} else { // allow to be roused again
			enableAccelInterrupt();
			enableRTCInterrupt();
		}			
		itoa(cntout, num_string, 10);
		
		outputStringToUART(num_string);
		outputStringToUART("\t");
		datetime_getstring(datetime_string, &dt_RTC);
		outputStringToUART(datetime_string);
		outputStringToUART("\r\n");
		delay_ms(1000);
		cntout++;
		if (cntout == 65500) 
			cntout = 0;
		rtc_add1sec();
		checkForCommands();
*/		
/*
        while (machState == Idle) { // RTCC interrupt will break out of this
            setSDCardPowerControl();
            if (stateFlags.reRoused) {
                stateFlags.reRoused = 0; // clear this flag
                stateFlags.isRoused = 1;
                startTimer1ToRunThisManySeconds(30);
            }
            if (stateFlags.isRoused) { // timer1 timeout will turn off
                // clean up after any external interrupt
                clearAnyADXL345TapInterrupt(); // otherwise will keep repeating interrupt
                _INT1IF = 0; // clear INT1 interrupt flag 
                _INT1IE = 1; // Enable INT1 external interrupt; was disabled within ISR, to keep from repeating
                flags1.isDark = 0; // wake up, if asleep due to darkness
            }


            }


        checkForCommands();

        if (1) { // currently, don't look at any previous state
            if ((flags1.isDark) && (!BT_PWR_CTL)) 
            // if it has changed to Dark, set long intervals
            // except don't do this until Bluetooth is off, or could hang with BT on
            //   wasting power for hours; BT booster control is shutdown low
                setupAlarm(0b1101011111111111);
            else // Dark has ended, change to short intervals
                setupAlarm(0b1100101111111111);
    
   // update flag
            if (flags1.isDark)
                flags1.wasDark = 1;
            else 
                flags1.wasDark = 0;
        } // end if (isDark != wasDark)

//    if (flags1.sleepBetweenReadings) { // go to sleep

        if (!stateFlags.isRoused) { // go to sleep
            outputStringToUSART("\r\n   system timout, going to sleep\r\n");
            assureSDCardIsOff();
            while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // allow any output to finish
            // if following cause lockup, fix
            _U1MD = 1; // disable UART module 1
            if (secsSince1Jan2000 > timeToTurnOffBT)
                BT_PWR_CTL = 0; // turn off power to Bluetooth module, low disables booster module
            //
            //_I2C2MD = 1; // disable I2C module 2
           Sleep();
           __asm__ ("nop");
        }
  
     } // end of (machState == Idle)
 // when (machState != Idle) execution passes on from this point

 // when RTCC occurs, changes machState to GettingTimestamp
    while (1) { // various tests may break early, 
        setSDCardPowerControl();
        // monitor cell voltage, to decide whether there is enough power to proceed
        cellVoltage = getCellVoltage();
        if (stateFlags.isRoused) { 
//            outputStringToUSART("\r\n system roused\r\n");
            // timer diagnostics

            len = sprintf(str, "\r\n timer 1: %u seconds\r\n", (int)(TMR1/128));
            outputStringToUSART(str);
        }
        createTimestamp();
        outputStringToUSART(timeStampBuffer);
        if (cellVoltage < CELL_VOLTAGE_THRESHOLD_READ_DATA) { 
            len = sprintf(str, " power too low\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
        if (stateFlags.isRoused) { 
            // if roused, and Bluetooth is on, flag to keep BT on awhile after normal rouse timeout
            // createTimestamp sets secsSince1Jan2000
            timeToTurnOffBT = secsSince1Jan2000 + SECS_BT_HOLDS_POWER_AFTER_SLEEP;
        }
//        len = sprintf(str, " char 17=%u, 17&0x01=%u, char 19=%u \n\r", timeStampBuffer[17], ((char)timeStampBuffer[17] & 0x01), timeStampBuffer[19]);
//        outputStringToUSART(str);
        if (((!((char)timeStampBuffer[17] & 0x01)) && ((char)timeStampBuffer[19] == '0')) || (flags1.isDark)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark
            stateFlags_2.isDataWriteTime = 1;
        } else {
            stateFlags_2.isDataWriteTime = 0;
        }
        if (stateFlags_2.isDataWriteTime) {
            logAnyJSON();
            assureDataHeadersLogged(); // writes headers on reset, time change, or midnight rollover
            err = writeCharsToFile (timeStampBuffer, 21);
            if (err) {
                tellFileWriteError (err);
                stateFlags_2.isDataWriteTime = 0; // prevent trying to write anything later
            }
        }
        machState = ReadingSensors;
//        getDataReadings(); // may make part or all of this a separate function later, but for now do straight-through
        // read broadband and infrared from down- and up-pointing sensors via I2C

//    I2C2CONbits.I2CEN = 0; // disable I2C module
// diagnostic for testing
//len = sprintf(str, "\r\n %u,%u\r\n ", irrReadings[0].irrWholeWord, irrReadings[0].irrMultiplier);
//outputStringToUSART(str);

    // prepare data string
    len = sprintf(str, ",%lu,%lu,%lu,%lu,%u",
          (unsigned long)((unsigned long)irrReadings[0].irrWholeWord * (unsigned long)irrReadings[0].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[1].irrWholeWord * (unsigned long)irrReadings[1].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[2].irrWholeWord * (unsigned long)irrReadings[2].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[3].irrWholeWord * (unsigned long)irrReadings[3].irrMultiplier),
           cellVoltage);
    outputStringToUSART(str);
    // if broadband reflected is less than threshold, flag that it is dark enough
    //   to go to sleep, to save power overnight
    if ((unsigned long)((unsigned long)irrReadings[0].irrWholeWord * (unsigned long)irrReadings[0].irrMultiplier) < 
             bbIrradThresholdDark)
        flags1.isDark = 1;
    else
        flags1.isDark = 0;
    if (stateFlags_2.isDataWriteTime) {
        // log data to SD card
        err = writeCharsToFile (str, len);
        if (err)
            tellFileWriteError (err);
        outputStringToUSART("\r\n SD card procedure done\r\n");
    }
//    if (stateFlags.isLeveling) { // show Leveling diagnostics
//        readAccelerometer();
//        outputStringToUSART("\r\n");
//        outputStringToUSART(str);
//    } 
    machState = Idle; // done with everything, return to Idle state
        break; // if did everything, break here
    } // end of getting data readings
    ;
 } // main program loop
} // end of Main


*/

		}	


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
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_UART) return;
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

                case 'T': case 't': { // set time
					// get info from commandBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					char tmpStr[commandBufferLen];
					strcpy(tmpStr, commandBuffer + 1);
//					if (!isValidTimestamp(commandBuffer + 1)) {
					if (!isValidTimestamp(tmpStr)) {
						outputStringToUART("\r\n Invalid timestamp\r\n");
//						outputStringToUART("\r\n> ");
//						outputStringToUART(commandBuffer);
//						outputStringToUART("\r\n");
						break;
					}
//					if (!isValidTimezone(commandBuffer + 21)) {
					if (!isValidTimezone(tmpStr + 20)) {
						outputStringToUART("\r\n Invalid hour offset\r\n");
//						outputStringToUART("\r\n> ");
//						outputStringToUART(commandBuffer);
//						outputStringToUART("\r\n");
						break;
					}
					outputStringToUART("\r\n Time changed from ");
					strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(datetime_string, &dt_RTC);
					strcat(strJSON, datetime_string);
					outputStringToUART(datetime_string);
//					datetime_getFromUnixString(&dt_tmp, commandBuffer + 1, 0);
					datetime_getFromUnixString(&dt_tmp, tmpStr, 0);
					rtc_setTime(&dt_tmp);
					strcat(strJSON, "\",\"to\":\"");
					outputStringToUART(" to ");
					datetime_getstring(datetime_string, &dt_tmp);
					strcat(strJSON, datetime_string);
					outputStringToUART(datetime_string);
					strcat(strJSON, "\",\"by\":\"hand\"}}\r\n");
					outputStringToUART("\r\n");
					stateFlags1 |= (1<<writeJSONMsg); // log JSON message on next SD card write
					stateFlags1 |= (1<<writeDataHeaders); // log data column headers on next SD card write
					rtcStatus = rtcTimeManuallySet;
					outputStringToUART(strHdr);
					intTmp1 = rtc_setupNextAlarm(&dt_CurAlarm);

					// 
/*
                   startTimer1ToRunThisManySeconds(30); // keep system Roused
*/
                    break;
                }
				
                case 'L': case 'l': 
				{ // experimenting with the accelerometer Leveling functions
					uint8_t rs;
//					outputStringToUART("\r\n about to initialize ADXL345\r\n");
//					initializeADXL345();
//					outputStringToUART("\r\n ADXL345 initialized\r\n");
					// bring out of low power mode
					// use 100Hz for now ; bit 4 set = reduced power, higher noise
					rs = setADXL345Register(ADXL345_REG_BW_RATE, 0x0a);
					if (rs) {
						len = sprintf(str, "\n\r could not set ADXL345_REG_BW_RATE: %d\n\r", rs);
						outputStringToUART(str);
						break;
					}
					outputStringToUART("\r\n set ADXL345_REG_BW_RATE, 0x0a \r\n");
					if (readADXL345Axes (&accelData)) {
						outputStringToUART("\r\n could not get ADXL345 data\r\n");
						break;
					}
					// set low power bit (4) and 25Hz sampling rate, for 40uA current
					if (setADXL345Register(ADXL345_REG_BW_RATE, 0x18)) {
						outputStringToUART("\r\n could not set ADXL345_REG_BW_RATE\r\n");
						break;
					}
//				len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", (unsigned int)((int)x1 << 8 | (int)x0),
//						  (unsigned int)((int)y1 << 8 | (int)y0),  (unsigned int)((int)z1 << 8 | (int)z0));
					len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", accelData.xWholeWord,
						accelData.yWholeWord,  accelData.zWholeWord);
						outputStringToUART(str);
                    break;
                }

                case 'C': case 'c':
					{
						len = sprintf(str, "\n\r test write to SD card 0x%x\n\r", (disk_initialize(0)));
						intTmp1 = writeCharsToSDCard(str, len);
						 // sd card diagnostics
						outputStringToUART("\r\n test write to SD card\r\n");
					//	len = sprintf(str, "\n\r PINB: 0x%x\n\r", (PINB));
						// send_cmd(CMD0, 0)
//					len = sprintf(str, "\n\r CMD0: 0x%x\n\r", (send_cmd(CMD0, 0)));
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
						if (!temperature_InitOneShotReading()) {
							// temperature conversion time, typically 30ms
							for (Timer2 = 4; Timer2; );	// Wait for 40ms to be sure
							if (!temperature_GetReading(&temperatureReading)) {
								len = sprintf(str, "\n\r Temperature: %d degrees C\n\r", (temperatureReading.tmprHiByte));
								outputStringToUART(str);

							}
						}

						break;
					}						

                //case 'I': case 'i':
					//{ // experimenting with irradiance functions
						//uint8_t result;
						//result = getIrrReading(TSL2561_UpLooking, TSL2561_CHANNEL_BROADBAND, &irrReadings[0]);
						//if (!result) {
							//len = sprintf(str, "\n\r Val: %lu; Mult: %lu; Irr: %lu \n\r", 
							     //(unsigned long)irrReadings[0].irrWholeWord, (unsigned long)irrReadings[0].irrMultiplier, 
								 //(unsigned long)((unsigned long)irrReadings[0].irrWholeWord * (unsigned long)irrReadings[0].irrMultiplier));
						//} else {
							//len = sprintf(str, "\n\r Could not get irradiance, err code: %d", result);
						//}						
						//outputStringToUART(str);
						//break;
					//}						
//
                //case 'B': case 'b':
					//{ // experimenting with reading the battery voltage using the Analog to Digital converter
						//len = sprintf(str, "\n\r Hi byte: %d \n\r", readCellVoltage(&cellVoltageReading));
						//outputStringToUART(str);
						//len = sprintf(str, "\n\r 16bit value: %d \n\r", cellVoltageReading.adcWholeWord);
						//outputStringToUART(str);
						//
//
						//break;
					//}						
//
                //case 'T': case 't':
					//{ // experimenting with time functions
						//outputStringToUART("\n\r about to set time \n\r");
						//if (!rtc_setTime(&dt_RTC)) {
							//outputStringToUART("\n\r time set \n\r");
						//}
						//outputStringToUART("\n\r about to read time \n\r");
						//if (!rtc_readTime(&dt_tmp)) {
////							dt_tmp.second = 22;
////							len = sprintf(str, "\n\r Seconds: %d \n\r", dt_tmp.second);
////							outputStringToUART(str);
////							dt_tmp.year = 55;
							//datetime_getstring(str, &dt_tmp);
							//outputStringToUART(str);
						//} else {
							//outputStringToUART("Error reading time");
						//}
						//break;
					//}						
//
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
//		PORTA ^= 0x01;
		PORTA ^= 0b00000100; // toggle bit 2, pilot light blinkey
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

