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

// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
//char* dt_stp = datetime_string;
char commandBuffer[commandBufferLen];
char *commandBufferPtr;

volatile uint8_t stateFlags1 = 0;
volatile uint8_t rtcStatus = rtcTimeNotSet;
volatile dateTime dt_RTC, dt_CurAlarm, dt_NextAlarm, dt_tmp, dt_LatestGPS;
volatile uint8_t timeZoneOffset = 0; // globally available
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
	uint8_t ct, swDnUp, swBbIr;
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

//	// Send the test string
//	for (cnt = 0; cnt < strlen(test_string); cnt++) {
//		uart_putchar(test_string[cnt]);
//	}
	rtc_setdefault();
	if (!rtc_setTime(&dt_RTC)) {
		rtcStatus = rtcTimeSetToDefault;
		datetime_copy(&dt_RTC, &dt_CurAlarm);
		datetime_copy(&dt_RTC, &dt_NextAlarm);
		datetime_advanceIntervalShort(&dt_NextAlarm);
		if (!rtc_setAlarm1(&dt_NextAlarm)) {
			outputStringToUART("\n\r Alarm1 set \n\r\n\r");
			datetime_getstring(str, &dt_NextAlarm);
			outputStringToUART(str);
			outputStringToUART("\n\r\n\r");
		}
		if (!rtc_enableAlarm1()) {
			outputStringToUART("\n\r Alarm1 enabled \n\r\n\r");
		}
		outputStringToUART("\n\r time set to default ");
		datetime_getstring(str, &dt_RTC);
		outputStringToUART(str);
		outputStringToUART("\n\r\n\r");
		
	} else {
		outputStringToUART("\n\r could not set Real Time Clock \n\r");
	}
	
	enableRTCInterrupt();
	machineState = Idle;
	stayRoused(20);
 
	while (1) { // main program loop
		while (machineState == Idle) { // RTC interrupt will break out of this
			;
			checkForCommands();

		} // end of (machState == Idle)
		// when (machState != Idle) execution passes on from this point

		// when RTCC occurs, changes machineState to GettingTimestamp
		datetime_copy(&dt_NextAlarm, &dt_CurAlarm);
		datetime_advanceIntervalShort(&dt_NextAlarm);
		if (!rtc_setAlarm1(&dt_NextAlarm)) {
			;
//			outputStringToUART("\n\r Alarm1 set \n\r\n\r");
//			datetime_getstring(str, &dt_NextAlarm);
//			outputStringToUART(str);
//			outputStringToUART("\n\r\n\r");
		}
		if (!rtc_enableAlarm1()) {
			;
//			outputStringToUART("\n\r Alarm1 enabled \n\r\n\r");
		}
		enableRTCInterrupt();
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
//			outputStringToUART("\t");
			//if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_READ_IRRADIANCE) { 
			//	outputStringToUART(" power too low\n\r");
			//	machineState = Idle;
			//	break;
			//}
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
			// formula from datasheet: V(measured) = adcResult * (1024 / Vref)
			// using internal reference, Vref = 2.56V = 2560mV
			// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
			len = sprintf(str, "\t%lu", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
			outputStringToUART(str);

			outputStringToUART("\r\n");
//        if (stateFlags.isRoused) { 
//            // if roused, and Bluetooth is on, flag to keep BT on awhile after normal rouse timeout
//            // createTimestamp sets secsSince1Jan2000
//            timeToTurnOffBT = secsSince1Jan2000 + SECS_BT_HOLDS_POWER_AFTER_SLEEP;
//        }
			stateFlags1 &= ~(1<<timeToLogData); // default clear
//			if (!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0)) {
			if (dt_CurAlarm.second == 0) {
			//  if (((!((char)timeStampBuffer[17] & 0x01)) && ((char)timeStampBuffer[19] == '0')) || (flags1.isDark)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark

				stateFlags1 |= (1<<timeToLogData);
			//	outputStringToUART("\n\r Time to log data \n\r");
			} else {
			//	outputStringToUART("\n\r Not time to log data \n\r");
			}			

			if (stateFlags1 & (1<<timeToLogData)) {
//				outputStringToUART("\n\r Entered log data routine \n\r");
				len = sprintf(str, "\n\r");
				intTmp1 = writeCharsToSDCard(str, len);
				len = sprintf(str, datetime_string);
				intTmp1 = writeCharsToSDCard(str, len);
				for (ct = 0; ct < 4; ct++) { // write irradiance readings
					if (!(irrReadings[ct].validation)) {
						len = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier));
					} else { // no valid data for this reading
						len = sprintf(str, "\t");
					}
					intTmp1 = writeCharsToSDCard(str, len);
				}
				len = sprintf(str, "\t%lu", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
				intTmp1 = writeCharsToSDCard(str, len);
				len = sprintf(str, "\n\r");
				intTmp1 = writeCharsToSDCard(str, len);	
				
				outputStringToUART("\n\r Data written to SD card \n\r");
			}
			machineState = Idle; // done with everything, return to Idle state
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
        if (cellVoltage < CELL_VOLTAGE_THRESHOLD_READ_IRRADIANCE) { 
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
       //  __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
        // explicitly initialize data to defaults
        for (irrReadingNumber = 0; irrReadingNumber < 4; irrReadingNumber++) {
            irrReadings[irrReadingNumber].irrWholeWord = 0; // default value
            irrReadings[irrReadingNumber].irrMultiplier = 1; // default gain
        }
//        len = sprintf(str, " diagnostics\n\r");
//        outputStringToUSART(str);
        len = sprintf(str, " cell voltage %u\n\r", cellVoltage);
        outputStringToUSART(str);

        I2C_Init(); // enable I2C module
        // first few Bus Collision tests should be sufficient to determine if I2C is working
        if (I2C2STATbits.BCL)
        {
            len = sprintf(str, " I2C bus collision during Init\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
        I2C_Stop(); // create a Stop condition on the I2C bus
        if (I2C2STATbits.BCL)
        {
            len = sprintf(str, " I2C bus collision during Stop\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
        for (irrSensorNumber = 0; irrSensorNumber < 2; irrSensorNumber++) {
        // 0 = down-looking sensor, 1 = up-looking sensor
             if (irrSensorNumber == 0) {
                 irrSensorWriteAddr = 0x52;
                 irrSensorReadAddr = 0x53;
             } else {
                 irrSensorWriteAddr = 0x92;
                 irrSensorReadAddr = 0x93;
             }
            I2C_Start(); // create a Start condition on the I2C bus
            if (I2C2STATbits.BCL)
            {
                len = sprintf(str, " I2C bus collision during Start\n\r");
                outputStringToUSART(str);
                machState = Idle;
                break;
            }
            r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//            d = irrSensorWriteAddr; // address the device, say we are going to write
//            r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
            if (!r) { // device did not acknowledge
                I2C_Stop();
                if (irrSensorWriteAddr == 0x52)
                    len = sprintf(str, " Down-pointing sensor not present, or not responding\n\r");
                else
                    len = sprintf(str, " Up-pointing sensor not present, or not responding\n\r");
                outputStringToUSART(str);
                continue;
            }
            d = 0x8A; // write; a byte command, setting the register to "ID" (Part number, Rev ID) = 0x0a
            r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
            r = I2C_Write(irrSensorReadAddr);
            I2C_ReStart(); // restart, preparatory to reading
            r = I2C_Read(0); // do NACK, since this is the last and only byte read
            I2C_Stop();
// len = sprintf(str, " result of reading part ID: 0x%x\n\r", r);
// outputStringToUSART(str);
            //if ((r & 0xF0) == 0x50) { // part number for TSL2561 (datasheet says 0x1n, but actually reads 0x5n)
// len = sprintf(str, " matched correct part number\n\r");
// outputStringToUSART(str);
                for (irrChannelNumber = 0; irrChannelNumber < 2; irrChannelNumber++) {
                    // 0 = broadband channel, 1 = infrared channel
// len = sprintf(str, " entered FOR loop for channels\n\r");
// outputStringToUSART(str);
                    if (irrChannelNumber == 0)
                        irrChannel = 0xAC;
                    else
                        irrChannel = 0xAE;
                    irrReadingNumber = irrChannelNumber + (irrSensorNumber + irrSensorNumber);
                    irrReadings[irrReadingNumber].irrWholeWord = 0; // default value
                    irrReadings[irrReadingNumber].irrMultiplier = 1; // default gain

                    // start with high gain and high integration time
                    stateFlags_2.irrSensorGain = 1;
                    stateFlags_2.irrSensorIntegrationPeriodHi = 1;
                    stateFlags_2.irrSensorIntegrationPeriodLo = 0;
                    while (1) { // read sensor, adjusting gain and sensitivity till OK or topped out
                        // power up device
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x80; // write; a byte command, setting the register to CONTROL = 0x00
                        r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        d = 0x03; // write to CONTROL, power-up code
                        r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        I2C_Stop();
                        // set up gain and integration time, prep to read sensor
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x81; // write; a byte command, setting the register to TIMING = 0x01
                        r = I2C_Write(d);
                        d = 0x00;
                        if (stateFlags_2.irrSensorGain)
                            d |= 0x10; // set high gain bit
                        if (stateFlags_2.irrSensorIntegrationPeriodHi)
                            d |= 0x02; // irrSensorIntegrationPeriod code 11 is disallowed so this is unambiguous
                        else {
                            if (stateFlags_2.irrSensorIntegrationPeriodLo)
                                d |= 0x01; // irrSensorIntegrationPeriod code 01, medium
                        }
// len = sprintf(str, "\n\r TIMING code, d = 0x%x\n\r", d);
// outputStringToUSART(str);

//
//                        // set sensor gain 0=low, 1=high
//                        if (stateFlags_2.irrSensorGain)
//                            d = 0x12; // write to TIMING, high gain, long integration time
//                        else
//                            d = 0x10; 
                        r = I2C_Write(d); // write to TIMING, the gain and integration time
                        I2C_Stop();
                        // get reading
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr);
// d = irrSensorWriteAddr;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        r = I2C_Write(irrChannel);
// d = irrChannel;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        for (intTmp = 1; intTmp < 100; intTmp++) {  // poll device up to 100 times
                            I2C_ReStart(); // restart, preparatory to reading
                            r = I2C_Write(irrSensorReadAddr); // address the device, say we are going to read
// d = irrSensorReadAddr;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                            irrReadings[irrReadingNumber].irrLoByte = I2C_Read(1); // do ACK, because not last byte
                            irrReadings[irrReadingNumber].irrHiByte = I2C_Read(0); // do NACK, since this is the last byte
                            I2C_Stop();
// len = sprintf(str, "\n\r after %i tries, reading[%u] = %u\n\r", intTmp, irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord);
// outputStringToUSART(str);
                            if ((irrReadings[irrReadingNumber].irrWholeWord > 0))
                                break;
                            for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
                            while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // in case something is printing
                        } // end of polling FOR loop, either zero or value
                        I2C_Stop();
 //len = sprintf(str, "\n\r exited FOR loop, reading[%u] = %u, Gain = %c, IntegrCode = %c%c\n\r", 
 //         irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord, (stateFlags_2.irrSensorGain ? '1' : '0'),
 //         (stateFlags_2.irrSensorIntegrationPeriodHi ? '1' : '0'), 
 //         (stateFlags_2.irrSensorIntegrationPeriodLo ? '1' : '0'));
 //outputStringToUSART(str);
                        // turn off device
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x80; // write; a byte command, setting the register to CONTROL = 0x00
                        r = I2C_Write(d);
                        d = 0x00; // write to CONTROL, power-down code
                        r = I2C_Write(d);
                        I2C_Stop();
                       // exit loop here, one way or another
                        if (irrReadings[irrReadingNumber].irrWholeWord < 0xFFFF)
                            break; // if zero (dark) or valid reading less than topped out at 2^16-1
                        if ((!stateFlags_2.irrSensorGain) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodHi) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodLo))
                            break; // also if topped out but we have done everything possible,
                                   // low gain and minimum integration period
                        // if we didn't exit, adjust bits, try not to top out
                        if (!stateFlags_2.irrSensorGain) { // if gain is already low
                            if (stateFlags_2.irrSensorIntegrationPeriodHi) { // integ period high, code 10
                                stateFlags_2.irrSensorIntegrationPeriodHi = 0; // set to medium code 01
                                stateFlags_2.irrSensorIntegrationPeriodLo = 1;
                            } else { // integ period is medium or low, 01 or 00
                                if (stateFlags_2.irrSensorIntegrationPeriodLo) // if medium, code 01
                                    stateFlags_2.irrSensorIntegrationPeriodLo = 0; // set to low, code 00
                            }
                        }
                        if (stateFlags_2.irrSensorGain) // if gain is high
                            stateFlags_2.irrSensorGain = 0; // set gain low
                    } // end of attempts at reading device, either OK or topped out, or zero
                    // calculate multiplier
                    // if high integration time, code 10, leave as default for now
                    if (!stateFlags_2.irrSensorIntegrationPeriodHi) {
                        if (stateFlags_2.irrSensorIntegrationPeriodLo) // medium, code 01
                            irrReadings[irrReadingNumber].irrMultiplier = 4; // (322/81), 0.62% error
                        else // low, code 00
                            irrReadings[irrReadingNumber].irrMultiplier = 29; // (322/11), 0.93% error
                    }
                    if (!stateFlags_2.irrSensorGain) // if gain is low
                        irrReadings[irrReadingNumber].irrMultiplier *= 16; // multiply by 16

                } // end of bb or ir channels
             
            //} // end of if correct device part number
            // else {} // determine if TSL2581 and work with that

        } // end of up- or down-looking device

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
					{
						len = sprintf(str, "\n\r test write to SD card 0x%x\n\r", (disk_initialize(0)));
						intTmp1 = writeCharsToSDCard(str, len);
						 // sd card diagnostics
						outputStringToUART("\r\n test write to SD card\r\n");
					//	len = sprintf(str, "\n\r PINB: 0x%x\n\r", (PINB));
						// send_cmd(CMD0, 0)
//					len = sprintf(str, "\n\r CMD0: 0x%x\n\r", (send_cmd(CMD0, 0)));
						
/*

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

// Sometimes you may want to format the disk.
//	if(f_mkfs(0,0,0)!=FR_OK) {
//		error
//	}
//

res = f_mkdir("0000");
if (!((res == FR_OK) || (res == FR_EXIST)))
{
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
*/

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
						uint8_t result;
						result = getIrrReading(TSL2561_UpLooking, TSL2561_CHANNEL_BROADBAND, &irrReadings[0]);
						if (!result) {
							len = sprintf(str, "\n\r Val: %lu; Mult: %lu; Irr: %lu \n\r", 
							     (unsigned long)irrReadings[0].irrWholeWord, (unsigned long)irrReadings[0].irrMultiplier, 
								 (unsigned long)((unsigned long)irrReadings[0].irrWholeWord * (unsigned long)irrReadings[0].irrMultiplier));
						} else {
							len = sprintf(str, "\n\r Could not get irradiance, err code: %d", result);
						}						
						outputStringToUART(str);
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
						outputStringToUART("\n\r about to set time \n\r");
						if (!rtc_setTime(&dt_RTC)) {
							outputStringToUART("\n\r time set \n\r");
						}
						outputStringToUART("\n\r about to read time \n\r");
						if (!rtc_readTime(&dt_tmp)) {
//							dt_tmp.second = 22;
//							len = sprintf(str, "\n\r Seconds: %d \n\r", dt_tmp.second);
//							outputStringToUART(str);
//							dt_tmp.year = 55;
							datetime_getstring(str, &dt_tmp);
							outputStringToUART(str);
						} else {
							outputStringToUART("Error reading time");
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

