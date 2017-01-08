/**
 * \file
 *
 * \brief code for NDVI (greenness) logger, based on AVR ATmega1284P
 *
 * Copyright (C) 2011 - 2013 Rick Shory, based in part on source code that is:
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
 *
 **
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
#include "GPS/GPStime.h"

volatile uint8_t machineState;
volatile uint8_t iTmp;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep
volatile uint16_t btCountdown = 0; // timer for trying Bluetooth connection
volatile uint16_t timer3val;

volatile
uint8_t Timer1, Timer2, intTmp1;	/* 100Hz decrement timer */

int len, err = 0;
char str[128]; // generic space for strings to be output
char strJSON[256]; // string for JSON data
char strHdr[64] = "\n\rTimestamp\tBBDn\tIRDn\tBBUp\tIRUp\tT(C)\tVbatt(mV)\n\r";
char strLog[64];

// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
char commandBuffer[commandBufferLen];
char *commandBufferPtr;

volatile sFlags1 stateFlags1 = {0};
volatile iFlags initFlags = {0};
volatile bFlags btFlags = {0};
volatile tFlags timeFlags = {0};
volatile rFlags irradFlags = {0};
volatile mFlags motionFlags = {0};
volatile uint8_t rtcStatus = rtcTimeNotSet;
volatile dateTime dt_RTC, dt_CurAlarm, dt_tmp, dt_LatestGPS, dt_CkGPS; //, dt_NextAlarm
volatile int8_t timeZoneOffset = 0; // globally available
volatile accelAxisData accelData;
extern irrData irrReadings[4];
volatile extern adcData cellVoltageReading;
uint16_t previousADCCellVoltageReading = 0;
uint16_t refDarkVoltage = 0;
// for heuristics on when to try getting time from GPS
uint16_t maxCellVoltageToday = 0; // track maximum cell voltage of the current day
uint16_t dayPtMaxChargeToday = 0; // track minute-of-day when cell has highest charge, in the current day
uint16_t maxAvgCellVoltage = 0; // moving average of maximum cell voltage through the day
uint16_t dayPtMaxAvgCellCharge = 0; // moving modulo average, minute-of-day since "midnight" when cell has the most power
chargeInfo cellReadings[DAYS_FOR_MOVING_AVERAGE]; // array to hold multiple days' max cell charge info, for getting average
chargeInfo *cellReadingsPtr;

unsigned long darkCutoffIR = (unsigned long)DEFAULT_IRRADIANCE_THRESHOLD_DARK_IR;
unsigned long darkCutOffBB = (unsigned long)DEFAULT_IRRADIANCE_THRESHOLD_DARK_BB;
unsigned long lngTmp1, lngTmp2;

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
	uint8_t errSD, cnt, r;
	uint16_t cntout = 0;
	strJSON[0] = '\0'; // "erase" the string
	DDRD &= ~(1<<5); // make the Bluetooth connection monitor pin an input
	PORTD &= ~(1<<5); // disable internal pull-up resistor
	DDRD |= (1<<4); // make Bluetooth power control an output
	DDRD |= (1<<7); // make Bluetooth baud rate control an output
	DDRB |= (1<<GPS_SUBSYSTEM_CTRL); // make GPS subsystem control an output
	
	GPS_idle; // default

	BT_power_off();
//	BT_baud_9600();
	BT_baud_115k();
	
//	cli();
//	setupDiagnostics(); // need heartbeat timer to correctly turn SD power off (may work around this)
//	// also may help with power control tracking while testing dead battery re-charge
//	sei();
//	turnSDCardPowerOff();

	// force SD card power off, and pins in lowest power modes

	if (!(PRR0 & (1<<PRSPI))) // is SPI power on (Pwr Save bit clear)?
	{
		if (SPCR & (1<<SPE)) // is SPI enabled?
		{
			SPCR &= ~(1<<SPE); // disable SPI
		}
		PRR0 |= (1<<PRSPI); // turn off power to SPI module, stop its clock
	}		
	DESELECT();

    DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_SCK)); // change SPI output lines MOSI and SCK into inputs
	// pins might source current momentarily
	 // set port bits to 0, disable any internal pull-ups; tri-state the pins
	SPI_PORT &= ~((1<<SPI_MOSI_BIT)|(1<<SPI_SCK_BIT)|(1<<SPI_MISO_BIT)); // MISO was already an input
	
	SD_CS_DD &= ~(1<<SD_CS_BIT); // change SS to an input, momentarily sources current through internal pull-up
	SD_CS_PORT &= ~(1<<SD_CS_BIT); // set port bit to zero, tri-state the input

	SD_PWR_DD |= (1<<SD_PWR_BIT);          // Turns on PWR pin as output 
	SD_PWR_PORT |= (1<<SD_PWR_BIT);   // Drive PWR pin high; this will turn FET off
	SD_PWR_DD &= ~(1<<SD_PWR_BIT);          // change PWR pin to an input
	// internal pull-up momentarily pulls high, along with external pull-up
	SD_PWR_PORT &= ~(1<<SD_PWR_BIT);   // tri-state the pin; external pull-up keeps FET off
	
//	Stat |= STA_NOINIT;      // Set STA_NOINIT	
	
	commandBuffer[0] = '\0'; // "empty" the command buffer
	commandBufferPtr = commandBuffer;
	stateFlags1.writeDataHeaders = 1; // write column headers at least once on startup

	intTmp1 = readCellVoltage(&cellVoltageReading);
	
	I2C_Init(); // enable I2C
	initFlags.initI2C = 1;
		
	intTmp1 = rtc_readTime(&dt_RTC);
	strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
	datetime_getstring(datetime_string, &dt_RTC);
	strcat(strJSON, datetime_string);
	strcat(strJSON, "\",\"to\":\"");
	
	if (dt_RTC.year) { // 0 on power up, otherwise must already have been set
		rtcStatus = rtcTimeRetained;
		strcat(strJSON, datetime_string);
		strcat(strJSON, "\",\"by\":\"retained\"}}\r\n");
	} else { // RTC year = 0 on power up, clock needs to be set
		rtc_setdefault();
		if (!rtc_setTime(&dt_RTC)) {
			rtcStatus = rtcTimeSetToDefault;
			datetime_getstring(datetime_string, &dt_RTC);
			strcat(strJSON, datetime_string);
			strcat(strJSON, "\",\"by\":\"default\"}}\r\n");
		} else {
			rtcStatus = rtcTimeSetFailed;
			strcat(strJSON, datetime_string);
			strcat(strJSON, "\",\"by\":\"failure\"}}\r\n");
		}
	}
	stateFlags1.writeJSONMsg = 1; // log JSON message on next SD card write	
	timeFlags.nextAlarmSet = 0; // alarm not set yet
	irradFlags.isDark = 1; // set the Dark flag, default till full-power initializations passed

	// tune uC osc down to 7.3728 MHz, implement 115200 baud
	tuneMainOsc();

	// try allowing the following on first power-up, even if cell is barely charged
	
	cli();
	setupDiagnostics();
	uart0_init();
	initFlags.initUART0 = 1;
	uart1_init();
	initFlags.initUART1 = 1;
	sei();
				
	if (initFlags.initI2C) 
		outputStringToUART0("\r\n  I2C_Init completed\r\n");
				
	if (initFlags.initUART0) 
		outputStringToUART0("\r\n  UART0 Initialized\r\n");
					
	if (initFlags.initUART1)
		outputStringToUART1("\r\n  UART1 Initialized\r\n");
				
	r = initializeADXL345();
	if (r) {
		len = sprintf(str, "\n\r ADXL345 initialize failed: %d\n\r\n\r", r);
		outputStringToUART0(str);
	} else {
		initFlags.initAccelerometer = 1;
		outputStringToUART0("\r\n ADXL345 initialized\r\n");
	}
				
	switch (rtcStatus) {
					
		case rtcTimeRetained:
			outputStringToBothUARTs("\n\r time retained through uC reset\n\r");
			break;
						
		case rtcTimeSetToDefault:
			outputStringToBothUARTs("\n\r time set to default, now elapsed to ");
			datetime_getstring(datetime_string, &dt_RTC);
			outputStringToBothUARTs(datetime_string);
			outputStringToBothUARTs("\n\r\n\r");
			break;	
					
		case rtcTimeSetFailed:
			outputStringToBothUARTs("\n\r could not set Real Time Clock \n\r");
			break;
					
	}
				
	// attempt to read/write the time zone; will retry later if e.g. power too low
	syncTimeZone();

	
	stateFlags1.isRoused = 1; // force on for testing, enable UART output
//	for (Timer1 = 3; Timer1; );	// Wait for 30ms
	outputStringToUART0("\r\n  test\r\n");
//	for (Timer1 = 3; Timer1; );	// Wait for 30ms
	outputStringToUART0("\r\n  test\r\n");
//	for (Timer1 = 3; Timer1; );	// Wait for 30ms
	outputStringToUART0("\r\n  test\r\n");
//	for (Timer1 = 3; Timer1; );	// Wait for 30ms

	outputStringToBothUARTs("\n\r Power good \n\r\n\r");
	
	// try to adjust the uC clock frequency
	// first step, measure the uC clock, relative to the RTC, which latter should be very accurate
	// eventually put this in a more reasonable place, but for now right here before main loop
	
	// go into uC clock adjust mode
	outputStringToUART0("\r\n going into uC adjust mode\r\n");
	timeFlags.nextAlarmSet = 0; // clear flag
	disableRTCInterrupt();
	intTmp1 = rtc_enableSqWave();
	// PRTIM1 make sure power reduction register bit if off so timers run
	
	
	// go back into normal timekeeping mode
	outputStringToUART0("\r\n returning to timekeeping mode\r\n");
	if (!(timeFlags.nextAlarmSet)) {
		if (!rtc_setupNextAlarm(&dt_CurAlarm))
			timeFlags.nextAlarmSet = 1;
	}
	
	cellReadingsPtr = cellReadings; // set up to track daily maximum cell voltage
	

	while (1) { // main program loop
		// code that will only run once when/if cell voltage first goes above threshold,
		// sufficient to run initializations and modules that take more power
		if (!(stateFlags1.reachedFullPower)) { 
			if (cellVoltageReading.adcWholeWord > CELL_VOLTAGE_GOOD_FOR_STARTUP) {
				stateFlags1.reachedFullPower = 1; // flag, so this loop does not happen again till next reset
				if (cellVoltageReading.adcWholeWord > CELL_VOLTAGE_GOOD_FOR_ALL_FUNCTIONS) {
					// probably, somebody has just popped a fresh battery in, and wants to set up this device
					stayRoused(18000); // keep system roused for 3 minutes (180 sec) for diagnostic output
				} else {
					// probably, battery has slowly charged from dead, and nobody is here watching
					// long diagnostics would just waste power for no good reason
					stayRoused(300); // only briefly, 3 seconds, then go into low power mode
				}
				if (cellVoltageReading.adcWholeWord > CELL_VOLTAGE_GOOD_FOR_ALL_FUNCTIONS) {
					// it's likely someone is setting up this device with a fresh battery
					tuneMainOsc(); // re-tune, in case clock was slow on first low-power startup
					keepBluetoothPowered(180); // start with Bluetooth power on for 3 minutes
				}
			} // end test CELL_VOLTAGE_GOOD_FOR_STARTUP
		} // end test reachedFullPower flag
		
		// end of segment that runs only once, when Full Power first achieved
		
		// beginning of loop that runs repeatedly
		// tests of normal operation
		checkCriticalPower();
		if (stateFlags1.reachedFullPower) { // only run this after cell has charged to full power and modules initialized
			if (motionFlags.tapDetected) {
				outputStringToUART0("\n\r Tap detected \n\r\n\r");
				if (stateFlags1.isRoused) { // if tap detected while already roused
					stayRoused(12000); // 2 minutes (120 seconds)
					tuneMainOsc(); // re-tune, in case clock was slow on first low-power startup
					keepBluetoothPowered(120); // try for two minutes to get a Bluetooth connection
				}
				motionFlags.tapDetected = 0;
			}
			if (stateFlags1.isRoused) { // if roused
				irradFlags.isDark = 0; // clear the Dark flag
				timeFlags.nextAlarmSet = 0; // flag that the next alarm might not be correctly set
	//			if (irradFlagsisDark)
	//				timeFlags.nextAlarmSet = 0; // flag that the next alarm might not be correctly set
			}
		
			if (BT_connected()) {
				// keep resetting this, so BT power will stay on for 2 minutes after connection lost
				// to allow easy reconnection
				keepBluetoothPowered(120);
			} else { // not connected
				if (btFlags.btWasConnected) { // connection lost
					; // action(s) when connection lost
				}
				btFlags.btWasConnected = 0; // clear the flag
			}		
		} // end of this full power segment
		
		machineState = Idle; // beginning, or done with everything; return to Idle state

		while (machineState == Idle) { // RTC interrupt will break out of this
			checkCriticalPower();
		
//			if (stateFlags1.reachedFullPower) { // another full-power-only segment
			intTmp1 =  clearAnyADXL345TapInterrupt();
			if (intTmp1) {
				len = sprintf(str, "\r\n could not clear ADXL345 Tap Interrupt: %d\r\n", intTmp1);
				outputStringToUART0(str);
			}
			enableAccelInterrupt();
			checkForBTCommands();
			checkForCommands();
//			} // end of this full-power segment
			
			if (!(timeFlags.nextAlarmSet)) {
	//			outputStringToUART0("\n\r about to call setupNextAlarm \n\r\n\r");
				if (!rtc_setupNextAlarm(&dt_CurAlarm))
					timeFlags.nextAlarmSet = 1;
			}

			if (!(stateFlags1.isRoused)) { // may add other conditions later
				// go to sleep
				PORTA &= ~(0b00000100); // turn off bit 2, pilot light blinkey
				// SE bit in SMCR must be written to logic one and a SLEEP
				//  instruction must be executed.

				// When the SM2..0 bits are written to 010, the SLEEP instruction makes the MCU enter
				// Power-down mode
	
				// SM2 = bit 3
				// SM1 = bit 2
				// SM0 = bit 1
				// SE = bit 0
				// don't set SE yet
				SMCR = 0b00000100;
				// set SE (sleep enable)
				SMCR |= (1<<SE);
				// go intoPower-down mode SLEEP
				asm("sleep");
			}

		} // end of (machState == Idle)
			// when (machState != Idle) execution passes on from this point
			// when RTCC alarm or Accelerometer tap occurs, changes machineState to WakedFromSleep
			
		checkCriticalPower();
		
		// Tap interrupt will not be active until first time initialization, so
		//  following flag should not be settable till then anyway
		//  but put internal check in case code rearranged
		if (motionFlags.tapDetected) { // if it was a tap, go into Roused state
			if (stateFlags1.reachedFullPower) { // only if had achieved full power and initialized
				stayRoused(3000); // 30 seconds
			} else {
				stayRoused(300); // 3 seconds
			}
			motionFlags.tapDetected = 0; // clear the flag
		}
			
		timeFlags.nextAlarmSet = 0; // flag that current alarm is no longer valid

		while (timeFlags.alarmDetected) { // interrupt that woke from sleep was RTC alarm
			// use 'while' loop to allow various tests to break out
			timeFlags.alarmDetected = 0; // clear flag so this will only happen once in any case
			// monitor cell voltage, to decide whether there is enough power to proceed
			// remember previous voltage; very first read on intialize, so should be meaningful
			previousADCCellVoltageReading = cellVoltageReading.adcWholeWord;
			intTmp1 = readCellVoltage(&cellVoltageReading);
			if (cellVoltageReading.adcWholeWord > cellReadingsPtr->level) {
				cellReadingsPtr->level = cellVoltageReading.adcWholeWord;
				datetime_copy(&dt_CurAlarm, &(cellReadingsPtr->timeStamp));
//				maxCellVoltageToday = cellVoltageReading.adcWholeWord;
//				dayPtMaxChargeToday = (60 * dt_CurAlarm.hour) + (dt_CurAlarm.minute);
			}
			if (!(stateFlags1.reachedFullPower)) { // if not achieved full power and initialized, skip this data acquisition loop
				// for testing, set to 10-second interval, so don't have to wait an hour to see if battery charging worked
				irradFlags.isDark = 0; // remove this line when done testing dead battery re-charging
				break; // will test reachedFullPower at top of main program loop
			}
			
			if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_UART) {
				// power too low for any output, no need to even read sensors
				// remove comment-out of following line when done testing dead battery re-charge
//				irradFlags.isDark = 1; // act as if dark, to save power
				break;
			}
			
			if ((timeFlags.checkGpsToday) & (datetime_compare(&dt_CkGPS, &dt_CurAlarm) > 1)) {
				// alarm has passed GPS check time
				GPS_initTimeRequest(); // send a low-going reset pulse, to start subsystem uC
			}
			
			// see if it's time to log data
			if ((!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0)) || (irradFlags.isDark)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark
				timeFlags.timeToLogData = 1;
				if (irradFlags.isDark) { // store the voltage reading at this point
					refDarkVoltage = cellVoltageReading.adcWholeWord; 
				}
				
			//	outputStringToUART0("\n\r Time to log data \n\r");
			} else {
				timeFlags.timeToLogData = 0;
			//	outputStringToUART0("\n\r Not time to log data \n\r");
			}
			
			// if not time to log data, and not roused
			if ((!(timeFlags.timeToLogData)) && (!(stateFlags1.isRoused))) {
				// won't do anything with results anyway, don't bother reading sensors, save power
				stayRoused(5); // rouse for 0.05 second to flash the pilot light
				break; 
			}

			datetime_getstring(datetime_string, &dt_CurAlarm);
			outputStringToBothUARTs(datetime_string);
			
			if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_READ_DATA) {
				len = sprintf(str, "\t power too low to read sensors, %lumV\r\n", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
				outputStringToBothUARTs(str);
				break;
			}
			
			// attempt to assure time zone is synchronized
			syncTimeZone(); // internally tests if work is already done
			
			// read sensors
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
				outputStringToBothUARTs(str);
			}
			if (!temperature_GetReading(&temperatureReading)) {
					len = sprintf(str, "\t%d", (int8_t)(temperatureReading.tmprHiByte));
					outputStringToBothUARTs(str);
			} else
				outputStringToBothUARTs("\t-");
			
			// calc cell voltage from ADC reading earlier
			// formula from datasheet: V(measured) = adcResult * (1024 / Vref)
			// using internal reference, Vref = 2.56V = 2560mV
			// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
			len = sprintf(str, "\t%lu", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
			outputStringToBothUARTs(str);
			outputStringToBothUARTs("\r\n");
			
			// begin to build log string while testing, even if we end up not logging data
			strcpy(strLog, "\n\r");
			strcat(strLog, datetime_string);
			irradFlags.isDarkBBDn = 0;
			irradFlags.isDarkIRDn = 0;
			irradFlags.isDarkBBUp = 0;
			irradFlags.isDarkIRUp = 0; // default clear
			for (ct = 0; ct < 4; ct++) { // generate irradiance readings
				if (!(irrReadings[ct].validation)) {
					lngTmp1 = (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier);
//						len = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier));
					len = sprintf(str, "\t%lu", lngTmp1);
					if ((ct == 0) || (ct == 3)) { // broadband
						lngTmp2 = darkCutOffBB;
					} else { // infrared
						lngTmp2 = darkCutoffIR;
					}
					if (lngTmp1 < lngTmp2) {
						switch (ct) {
							case 0:
								irradFlags.isDarkBBDn = 1;
								break;
							case 1:
								irradFlags.isDarkIRDn = 1;
								break;
							case 2:
								irradFlags.isDarkBBUp = 1;
								break;
							case 3:
								irradFlags.isDarkIRUp = 1;
								break;
						}
					}
				} else { // no valid data for this reading
					len = sprintf(str, "\t");
					// treat invalid readings as if dark
					switch (ct) {
						case 0:
							irradFlags.isDarkBBDn = 1;
							break;
						case 1:
							irradFlags.isDarkIRDn = 1;
							break;
						case 2:
							irradFlags.isDarkBBUp = 1;
							break;
						case 3:
							irradFlags.isDarkIRUp = 1;
							break;
					}
				}
				strcat(strLog, str);
			} // end of irradiance sensor validity testing

			if (timeFlags.timeToLogData) {
//				outputStringToUART0("\n\r Entered log data routine \n\r");
				// (previously built irradiance part of log string)
				
				// log temperature
				if (!temperatureReading.verification)
					len = sprintf(str, "\t%d", (int8_t)(temperatureReading.tmprHiByte));
				else
					len = sprintf(str, "\t");
				strcat(strLog, str);
				// log cell voltage
				len = sprintf(str, "\t%lu\n\r", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
				strcat(strLog, str);
				
				len = strlen(strLog);
				errSD = writeCharsToSDCard(strLog, len);
				if (errSD) {
					tellFileError (errSD);
				} else {
					outputStringToBothUARTs(" Data written to SD card \n\r\n\r");
				}
			} // end of test if time to log data
			
			// test if dark or not dark
			// if all sensors are less than thresholds, or missing; and system not in Roused state
			if ((irradFlags.isDarkBBDn) && 
				    (irradFlags.isDarkIRDn) && 
					(irradFlags.isDarkBBUp) && 
					(irradFlags.isDarkIRUp) && 
					(!(stateFlags1.isRoused))) {
				// flag that it is dark
				irradFlags.isDark = 1;
			} else { // or not
				// try new algorithm:
				if (cellVoltageReading.adcWholeWord > CELL_VOLTAGE_THRESHOLD_SD_CARD) { // if cell is high
					irradFlags.isDark = 0; // always leave the Dark state
				} else { // if cell voltage is low, only leave the Dark state
					// if cell has charged somewhat since last reading
					// this should eliminate early morning drain, when data will not be good anyway
					// require a small increase in cell voltage to ignore random jitter
					if (cellVoltageReading.adcWholeWord > (refDarkVoltage + 3)) {
						irradFlags.isDark = 0;
					}						
				}
			} // end of testing for dark or not dark		


			// let main loop restore Idle state, after assuring timer interrupts are re-established
			break; // if did everything, break here
		} // end of data acquisition segment
				
		turnSDCardPowerOff();
		
		if (stateFlags1.reachedFullPower) { // another full-power-only segment
			if (!BT_connected()) { // timeout diagnostics if no Bluetooth connection
				if (stateFlags1.isRoused) {
					len = sprintf(str, "\r\n sleep in %u seconds\r\n", (rouseCountdown/100));
					outputStringToUART0(str);
				}
				if (BT_powered()) {
					len = sprintf(str, "\r\n BT off in %u seconds\r\n", (btCountdown/100));
					outputStringToUART0(str);
				}	
			}
		} // end of this full-power-only segment
	} // end of main program loop
} // end of fn main


/**
 * \brief tune system oscillator to 7.3728 MHz, implement 115200 baud
 *
 * Adjust system oscillator from ~8MHz to 7.3728 MHz
 * Need this frequency to generate 115200 baud with
 *  usable accuracy
 * Talks to RTC chip, so needs I2C working
 * \
 *  
 */
void tuneMainOsc(void)	{
	uint16_t cyCt, cyCtNxtUp;
	OSCCAL = 0x7F; // set OSCCAL (oscillator calibration byte) to high end of lower range
	cyCt = cyPerRTCSqWave();
	do  { 
		cyCtNxtUp = cyCt;
		OSCCAL--;
		cyCt = cyPerRTCSqWave();
	} while ((unsigned long)cyCt > RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ);
	// we are just below the ideal number; if the next higher count was closer ...					
	if ((unsigned long)(RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ - (unsigned long)cyCt) > (unsigned long)((unsigned long)cyCtNxtUp - RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ)) {
		OSCCAL++; // ... tweak OSCCAL up one
	}
}


/**
 * \brief Check if power is critically low
 *
 * Shut down all possible modules if cell voltage is too low
 * Expects a reading of the cell voltage in global
 *  cellVoltageReading.adcWholeWord
 *
 * \
 *  
 */

void checkCriticalPower(void){
//	return; // for testing, disable this fn
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_CRITICALLY_LOW) { // power too low, shut everything down
		if (stateFlags1.reachedFullPower) { // skip till initialized, or can prevent climbing out of reset
			shutDownBluetooth();
			endRouse();
			motionFlags.tapDetected = 0; // ignore any Tap interrupt
			irradFlags.isDark = 1; // behave as if in the Dark
			timeFlags.nextAlarmSet = 0; // flag that the next alarm might not be correctly set
			refDarkVoltage = CELL_VOLTAGE_CRITICALLY_LOW; // allow testing that cell is recharging
		}
	}
}

/**
 * \brief Send a string out UART0
 *
 * Copy characters from the passed null-terminated
 * string to the UART0 output buffer. Inline fn "uart0_putchar"
 * flags to start transmitting, if necessary.
 *
 * \
 *  
 */

void outputStringToUART0 (char* St) {
	uint8_t cnt;
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_UART) return;
	if (stateFlags1.isRoused) { // if system not roused, no output
		for (cnt = 0; cnt < strlen(St); cnt++) {
			uart0_putchar(St[cnt]);
		}	
	}
	while (uart0_char_queued_out())
		;
} // end of outputStringToUART0

/**
 * \brief Send a string out UART1
 *
 * Copy characters from the passed null-terminated
 * string to the UART1 output buffer. Inline fn "uart1_putchar"
 * flags to start transmitting, if necessary.
 *
 * \
 *  
 */

void outputStringToUART1 (char* St) {
	uint8_t cnt;
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_UART) return;
	if (btFlags.btSerialBeingInput) return; // suppress output if user in typing
	if (BT_connected()) {
		for (cnt = 0; cnt < strlen(St); cnt++) {
			uart1_putchar(St[cnt]);
		}
	}
	while (uart1_char_queued_out())
		;
} // end of outputStringToUART1

/**
 * \brief Send the same string out both UART0 and UART1
 *
 * Call the fn to send a string out each UART
 *
 * \
 *  
 */
void outputStringToBothUARTs (char* St) {
	outputStringToUART0 (St);
	outputStringToUART1 (St);
}	

/**
 * \brief Check the UART0 for commands
 *
 * Check UART0 receive buffer for commands
 *
 * \
 *  
 */

void checkForCommands (void) {
    char c;
    while (uart0_char_waiting_in()) {
		c = uart0_getchar();
		if (c == 0x08) { // backspace
			if (commandBufferPtr > commandBuffer) { // if there is anything to backspace
				commandBufferPtr--;
			}
		}
        if (c == 0x0d) // if carriage-return
            c = 0x0a; // substitute linefeed
        if (c == 0x0a) { // if linefeed, attempt to parse the command
			stayRoused(120); // keep system roused
            *commandBufferPtr++ = '\0'; // null terminate
            switch (commandBuffer[0]) { // command is 1st char in buffer

				 case 'G': case 'g': { // get time from GPS
					 // for testing, manually initiate a get-time request from GPS
					 GPS_initTimeRequest();
					 break;
				 }					 
				
				 case 'V': case 'v': { // show firmware version
					 outputStringToBothUARTs(versionString);
					 break;
				 }					 

                case 'T': case 't': { // set time
					// get info from commandBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					char tmpStr[commandBufferLen];
					strcpy(tmpStr, commandBuffer + 1);
					if (!isValidDateTime(tmpStr)) {
						outputStringToUART0("\r\n Invalid timestamp\r\n");
						break;
					}
					if (!isValidTimezone(tmpStr + 20)) {
						outputStringToUART0("\r\n Invalid hour offset\r\n");
						break;
					}
					outputStringToUART0("\r\n Time changed from ");
					strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(datetime_string, &dt_RTC);
					strcat(strJSON, datetime_string);
					outputStringToUART0(datetime_string);
					datetime_getFromUnixString(&dt_tmp, tmpStr, 0);
					rtc_setTime(&dt_tmp);
					strcat(strJSON, "\",\"to\":\"");
					outputStringToUART0(" to ");
					datetime_getstring(datetime_string, &dt_tmp);
					strcat(strJSON, datetime_string);
					outputStringToUART0(datetime_string);
					strcat(strJSON, "\",\"by\":\"hand\"}}\r\n");
					outputStringToUART0("\r\n");
					stateFlags1.writeJSONMsg = 1; // log JSON message on next SD card write
					stateFlags1.writeDataHeaders = 1; // log data column headers on next SD card write
					rtcStatus = rtcTimeManuallySet;
					outputStringToUART0(strHdr);
					if (!rtc_setupNextAlarm(&dt_CurAlarm)) timeFlags.nextAlarmSet = 1;
					timeZoneOffset = dt_tmp.houroffset;
					timeFlags.timeZoneWritten = 0; // flag that time zone needs to be written
					syncTimeZone(); // attempt to write the time zone; will retry later if e.g. power too low

					// 
/*
                   startTimer1ToRunThisManySeconds(30); // keep system Roused
*/
                    break;
                }
				
                case 'L': case 'l': 
				{ // experimenting with the accelerometer Leveling functions
					uint8_t rs, val;
//					outputStringToUART0("\r\n about to initialize ADXL345\r\n");
//					rs = initializeADXL345();
//					if (rs) {
//						len = sprintf(str, "\n\r initialize failed: %d\n\r", rs);
//						outputStringToUART0(str);
//						break;
//					}
//					outputStringToUART0("\r\n ADXL345 initialized\r\n");

					// bring out of low power mode
					// use 100Hz for now ; bit 4 set = reduced power, higher noise
					rs = setADXL345Register(ADXL345_REG_BW_RATE, 0x0a);
					if (rs) {
						len = sprintf(str, "\n\r could not set ADXL345_REG_BW_RATE: %d\n\r", rs);
						outputStringToUART0(str);
						break;
					}
//					for (iTmp = 1; iTmp < 6; iTmp++) { // try reading bit 7, INT_SOURCE.DATA_READY
//						rs = readADXL345Register(ADXL345_REG_INT_SOURCE, &val);
//						if (rs) {
//							len = sprintf(str, "\n\r could not read ADXL345_REG_INT_SOURCE: %d\n\r", rs);
//							outputStringToUART0(str);
//							break;
//						}
//						if (val & (1 << 7)) {
//							len = sprintf(str, "\n\r INT_SOURCE.DATA_READY set: 0x%x\n\r", val);
//						} else {
//							len = sprintf(str, "\n\r INT_SOURCE.DATA_READY clear: 0x%x\n\r", val);
//						}							
//						outputStringToUART0(str);
//					}


//					outputStringToUART0("\r\n set ADXL345_REG_BW_RATE, 0x0a \r\n");
					if (readADXL345Axes (&accelData)) {
						outputStringToUART0("\r\n could not get ADXL345 data\r\n");
						break;
					}
					// set low power bit (4) and 25Hz sampling rate, for 40uA current
					rs = setADXL345Register(ADXL345_REG_BW_RATE, 0x18);
					if (rs) {
						len = sprintf(str, "\n\r could not set ADXL345_REG_BW_RATE: %d\n\r", rs);
						outputStringToUART0(str);
						break;
					}
//				len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", (unsigned int)((int)x1 << 8 | (int)x0),
//						  (unsigned int)((int)y1 << 8 | (int)y0),  (unsigned int)((int)z1 << 8 | (int)z0));
					len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", accelData.xWholeWord,
						accelData.yWholeWord,  accelData.zWholeWord);
						outputStringToUART0(str);
                    break;
                }

                case 'C': case 'c':
					{
						len = sprintf(str, "\n\r test write to SD card 0x%x\n\r", (disk_initialize(0)));
						intTmp1 = writeCharsToSDCard(str, len);
						 // sd card diagnostics
						outputStringToUART0("\r\n test write to SD card\r\n");
					//	len = sprintf(str, "\n\r PINB: 0x%x\n\r", (PINB));
						// send_cmd(CMD0, 0)
//					len = sprintf(str, "\n\r CMD0: 0x%x\n\r", (send_cmd(CMD0, 0)));
						break;						
					}

                case 'P': case 'p': 
				{ // force SD card power off
                    outputStringToUART0("\r\n turning SD card power off\r\n");
					turnSDCardPowerOff();
					outputStringToUART0("\r\n SD card power turned off\r\n");
                    break;
                }
/*
                case '^':
				{ // force B3 high
                    outputStringToUART0("\r\n forcing B3 high\r\n");
					DDRB |= 0b00001000;
					PORTB |= 0b00001000;
					outputStringToUART0("\r\n B3 forced high \r\n");
                    break;
                }

                case 'v':
				{ // force B3 low
                    outputStringToUART0("\r\n forcing B3 low\r\n");
					DDRB |= 0b00001000;
					PORTB &= 0b11110111;
					outputStringToUART0("\r\n B3 forced low \r\n");
                    break;
                }
*/
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
								outputStringToUART0(str);

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
						//outputStringToUART0(str);
						//break;
					//}						
//
                //case 'B': case 'b':
					//{ // experimenting with reading the battery voltage using the Analog to Digital converter
						//len = sprintf(str, "\n\r Hi byte: %d \n\r", readCellVoltage(&cellVoltageReading));
						//outputStringToUART0(str);
						//len = sprintf(str, "\n\r 16bit value: %d \n\r", cellVoltageReading.adcWholeWord);
						//outputStringToUART0(str);
						//
//
						//break;
					//}						
//				
//
                case 'D': case 'd': 
				{ // output file 'D'ata (or 'D'ump)
                    outputStringToUART0("\r\n (data dump only works from Bluetooth)\r\n");
                    break;
                }

                // put other commands here
                default: 
				{ // if no valid command, echo back the input
                    outputStringToUART0("\r\n> ");
                    outputStringToUART0(commandBuffer);
                    outputStringToUART0("\r\n");
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
	int16_t t;
	
	if (--ToggleCountdown <= 0) {
//		PORTA ^= 0xFF;
//		PORTA ^= 0x01;
		PORTA ^= 0b00000100; // toggle bit 2, pilot light blinkey
		ToggleCountdown = TOGGLE_INTERVAL;
	}
	
	if (btCountdown > rouseCountdown)
		rouseCountdown = btCountdown; // stay roused at least as long as trying to get a BT connection

	t = btCountdown;
	if (t) btCountdown = --t;
	if (!btCountdown)
	{
		BT_power_off();
	}
	
	t = rouseCountdown;
	if (t) rouseCountdown = --t;
	
//	if (--rouseCountdown <= 0)
	if (!rouseCountdown)
	{
		stateFlags1.isRoused = 0;
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

