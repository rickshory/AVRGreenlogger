/**
 * \file
 *
 * \brief code for NDVI (greenness) logger, based on AVR ATmega1284P
 *
 * Copyright (C) 2011 - 2017 Rick Shory, based in part on source code that is:
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
#include "Bluetooth/RN42.h"
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
volatile uint16_t gpsTimeReqCountdown = 0; // timeout for GPS time request
volatile uint16_t levelingCountdown = 0; // timeout for showing the diagnostics for XYZ leveling of the instrument
volatile uint16_t timer3val;

volatile
uint8_t Timer1, Timer2, intTmp1;	/* 100Hz decrement timer */

int len, err = 0;
char str[128]; // generic space for strings to be output
char stCellReading[128]; // space for cell reading string
char strJSON[512]; // string for generalized JSON data
char strJSONtc[256]; // string for time change information, as JSON
char strJSONloc[256]; // string for location information, as JSON
char strHdr[64] = "\n\rTimestamp\tBBDn\tIRDn\tBBUp\tIRUp\tT(C)\tVbatt(mV)\n\r";
char strLog[64];

// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
char commandBuffer[COMMAND_BUFFER_LENGTH];
char *commandBufferPtr;

volatile sFlags1 stateFlags1 = {0};
volatile iFlags initFlags = {0};
volatile bFlags btFlags = {0};
volatile tFlags timeFlags = {0};
volatile gFlags gpsFlags = {0};
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
volatile gpsLocation curLocation, prevLocation;
int32_t gpsSecsElapsed; // seconds elapsed since latest GPS reading
// target, seconds since latest GPS check, to schedule GPS time again
int32_t secsCtToCkGpsTime = DAYS_FOR_MOVING_AVERAGE * 86400ul; 
// for heuristics on when to try getting time from GPS
uint16_t maxCellVoltageToday = 0; // track maximum cell voltage of the current day
uint16_t dayPtMaxChargeToday = 0; // track minute-of-day when cell has highest charge, in the current day
uint16_t maxAvgCellVoltage = 0; // moving average of maximum cell voltage through the day
uint16_t dayPtMaxAvgCellCharge = 0; // moving modulo average, minute-of-day since "midnight" when cell has the most power
chargeInfo cellReadings[DAYS_FOR_MOVING_AVERAGE]; // array to hold multiple days' max cell charge info, for 
	// getting average. Initialization to zero flags that they are not valid items yet.
chargeInfo *cellReadingsPtr = cellReadings; // set up to track daily maximum cell voltage

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
	uint8_t errSD, r;
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
	strcat(strJSONtc, "\r\n{\"timechange\":{\"from\":\"");
	datetime_getstring(datetime_string, &dt_RTC);
	strcat(strJSONtc, datetime_string);
	strcat(strJSONtc, "\",\"to\":\"");
	
	if (dt_RTC.year) { // 0 on power up, otherwise must already have been set
		rtcStatus = rtcTimeRetained;
		strcat(strJSONtc, datetime_string);
		strcat(strJSONtc, "\",\"by\":\"retained\"}}\r\n");
//		initFlags.gpsTimeAutoInit = 1; // no need to try to auto-initialize
	} else { // RTC year = 0 on power up, clock needs to be set
		datetime_getDefault(&dt_RTC);
		if (!rtc_setTime(&dt_RTC)) {
			rtcStatus = rtcTimeSetToDefault;
			datetime_getstring(datetime_string, &dt_RTC);
			strcat(strJSONtc, datetime_string);
			strcat(strJSONtc, "\",\"by\":\"default\"}}\r\n");
		} else {
			rtcStatus = rtcTimeSetFailed;
			strcat(strJSONtc, datetime_string);
			strcat(strJSONtc, "\",\"by\":\"failure\"}}\r\n");
		}
	}
	stateFlags1.writeTimeChangeMsg = 1; // log JSON Time Change message on next SD card write	
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
		outputStringToWiredUART("\r\n  I2C_Init completed\r\n");
				
	if (initFlags.initUART0) 
		outputStringToWiredUART("\r\n  UART0 Initialized\r\n");
					
	if (initFlags.initUART1)
		outputStringToBluetoothUART("\r\n  UART1 Initialized\r\n");
				
	r = initializeADXL345();
	if (r) {
		len = sprintf(str, "\n\r ADXL345 initialize failed: %d\n\r\n\r", r);
		outputStringToWiredUART(str);
	} else {
		initFlags.initAccelerometer = 1;
		outputStringToWiredUART("\r\n ADXL345 initialized\r\n");
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
	
	//initialize latest GPS-read time to null, so will compare as
	// less than any real time, and will trigger a request for GPS time
	dt_LatestGPS.year = 0;
	dt_LatestGPS.month = 0;
	dt_LatestGPS.day = 0;
	dt_LatestGPS.houroffset = 0;
	dt_LatestGPS.hour = 0;
	dt_LatestGPS.minute = 0;
	dt_LatestGPS.second = 0;
	
	// initialize GPS locations
	datetime_copy(&(curLocation.timeStamp), &dt_LatestGPS);
	curLocation.latVal = 0.0; // defaults
	curLocation.lonVal = 0.0;
	curLocation.latStr[0] = '\0'; // these flag that the location is undefined
	curLocation.lonStr[0] = '\0';
	datetime_copy(&(prevLocation.timeStamp), &dt_LatestGPS);
	prevLocation.latVal = 0.0;
	prevLocation.lonVal = 0.0;
	prevLocation.latStr[0] = '\0';
	prevLocation.lonStr[0] = '\0';
		
	// initialize array that will be used for tracking moving average
	for (uint8_t i=0; i<DAYS_FOR_MOVING_AVERAGE; i++) {
		cellReadings[i].level = 0;
		cellReadings[i].timeStamp.year = 0;
		cellReadings[i].timeStamp.month = 0; // this flags this instance as still empty
		cellReadings[i].timeStamp.day = 0; // this flags this instance as still empty
		cellReadings[i].timeStamp.houroffset = 0;
		cellReadings[i].timeStamp.hour = 0;
		cellReadings[i].timeStamp.minute = 0;
		cellReadings[i].timeStamp.second = 0;
	}

	stateFlags1.isRoused = 1; // force on for testing, enable UART output
	
	// for diagnostics, show the set of cell readings, even if still empty
	showCellReadings();
	
	outputStringToBothUARTs("\n\r Power good \n\r\n\r");
	
	if (!(timeFlags.nextAlarmSet)) {
		if (!rtc_setupNextAlarm(&dt_CurAlarm))
			timeFlags.nextAlarmSet = 1;
	}

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
				outputStringToWiredUART("\n\r Tap detected \n\r\n\r");
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
			}
		
			if (BT_connected()) {
				// keep resetting this, so BT power will stay on for 2 minutes after connection lost
				// to allow easy reconnection
				keepBluetoothPowered(120);
			} else { // not connected
				if (btFlags.btWasConnected) { // connection lost
					// action(s) when connection lost
					btFlags.btSerialBeingInput = 0; // prevent hang on half-finished commands
				}
				btFlags.btWasConnected = 0; // clear the flag
			}		
		} // end of this full power segment
		
		machineState = Idle; // beginning, or done with everything; return to Idle state

		while (machineState == Idle) { // RTC interrupt will break out of this
			checkCriticalPower();
			intTmp1 =  clearAnyADXL345TapInterrupt();
			if (intTmp1) {
				len = sprintf(str, "\r\n could not clear ADXL345 Tap Interrupt: %d\r\n", intTmp1);
				outputStringToWiredUART(str);
			}
			enableAccelInterrupt();
			checkForBTCommands();
			checkForCommands();
			
			if (!(timeFlags.nextAlarmSet)) {
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
			// we do not yet use the 'cellIsCharging' flag
			if ((int16_t)(cellVoltageReading.adcWholeWord) - (int16_t)(previousADCCellVoltageReading) > 2) {
				stateFlags1.cellIsCharging = 1;
			} else {
				stateFlags1.cellIsCharging = 0;
			}
			
			// try to get GPS time soon after startup, but don't waste battery if not available
			// - Try every 2 minutes for first 10 minutes
			// - Try again every 20 minutes for first hour
			// - Try every four hours first day
			// - after that drop through to normal checking interval
			dateTime dtCk;
			datetime_getDefault(&dtCk);
			uint16_t minsCt = (uint16_t)((datetime_totalsecs(&dt_CurAlarm) - datetime_totalsecs(&dtCk))/60) ;

			if ((minsCt <= (60 * 24)) && (initFlags.gpsTimeAutoInit == 0)) {
#ifdef VERBOSE_DIAGNOSTICS
				{ // temporary diagnostics
					int l;
					char s[64];
					l = sprintf(s, "%d minutes elapsed\n\r", minsCt);
					outputStringToBothUARTs(s);
					(void)l; // avoid compiler warning
				}
#endif
				switch (minsCt) {
					case (2): // 2 minutes
					case (4): // 4 minutes
					case (6): // 6 minutes
					case (8): // 8 minutes
					case (10): // 10 minutes
					case (20): // 20 minutes
					case (40): // 40 minutes
					case (60): // 1 hour
					case (60 * 4): // 4 hours
					case (60 * 8): // 8 hours
					case (60 * 12): // 12 hours
					case (60 * 16): // 16 hours
					case (60 * 20): // 20 hours
					case (60 * 24): // 24 hours
#ifdef VERBOSE_DIAGNOSTICS					
						{ // temporary diagnostics
							int l;
							char s[64];
							l = sprintf(s, "About to call 'GPS_initTimeRequest' on minute %d\n\r", minsCt);
							outputStringToBothUARTs(s);
							(void)l; // avoid compiler warning
						}
#endif
						// following requests should not collide or stack because
						// fn below disallows until 3-minute timeout
						GPS_initTimeRequest();
					/*
					
						if (gpsFlags.checkGpsToday) { // by present code structure, this would not be
							// pending, but try to make fail-safe in case code is rearranged
							
							GPS_initTimeRequest();
						}
						*/
						break;
					
					default:
						// we have tried long enough to auto-initialize
						if (minsCt > (60 * 24)) initFlags.gpsTimeAutoInit = 1; // not presently reachable
						break;
				}
			} // end of if (initFlags.gpsTimeAutoInit == 0)
			
#ifdef VERBOSE_DIAGNOSTICS
			// temporary diagnostics
			{
				int l;
				char s[64], t[32];
				l = sprintf(s, " target seconds: %.0f\n\r",
					(double)secsCtToCkGpsTime);
				outputStringToBothUARTs(s);
				l = sprintf(s, "elapsed seconds: %.0f\n\r",
					(double)((int32_t)((datetime_totalsecs(&dt_CurAlarm)
					- (datetime_totalsecs(&dt_LatestGPS))))));
				outputStringToBothUARTs(s);
				outputStringToBothUARTs("latest GPS time: ");
				datetime_getstring(t, &dt_LatestGPS);
				outputStringToBothUARTs(t);
				outputStringToBothUARTs("\n\r");
				(void)l; // avoid compiler warning
			}
			
			// more diagnostics
			if (gpsFlags.checkGpsToday) {
				char t[32];
				outputStringToBothUARTs("'checkGpsToday' flagged, and pending at: ");
				datetime_getstring(t, &dt_CkGPS);
				outputStringToBothUARTs(t);
				outputStringToBothUARTs("\n\r");
			}
#endif
			
			// test whether to request time from the GPS
			gpsSecsElapsed = (int32_t)((datetime_totalsecs(&dt_CurAlarm) - (datetime_totalsecs(&dt_LatestGPS))));
			if (gpsSecsElapsed > secsCtToCkGpsTime) {
				if (gpsFlags.checkGpsToday) { // don't flag another till this one serviced
#ifdef VERBOSE_DIAGNOSTICS
					int l;
					char s[64];
					outputStringToBothUARTs("'checkGpsToday' flagged, and time passed\n\r");
					outputStringToBothUARTs("pending at: ");
					datetime_getstring(datetime_string, &dt_CkGPS);
					outputStringToBothUARTs(datetime_string);
					outputStringToBothUARTs("\n\r");
					l = sprintf(s, "%.0f seconds overdue\n\r", 
						(double)(gpsSecsElapsed - secsCtToCkGpsTime));
					outputStringToBothUARTs(s);
					(void)l; // avoid compiler warning
/* don't do this; would run on every single alarm
					// diagnostics, e.g.
					// {"GPStime":{"pending":"2017-01-17 22:10:24 -00","now":"2017-01-18 22:45:03 +00"}}
					strcat(strJSON, "\r\n{\"GPStime\":{\"pending\":\"");
					datetime_getstring(datetime_string, &dt_CkGPS);
					strcat(strJSON, datetime_string);
					strcat(strJSON, "\",\"now\":\"");
					datetime_getstring(datetime_string, &dt_CurAlarm);
					strcat(strJSON, datetime_string);
					strcat(strJSON, "\"}}\r\n");
*/
#endif
				} else { // gpsFlags.checkGpsToday not yet set, set up a GPS-time request
					if (initFlags.gpsTimeAutoInit) { // wait till we have tried to auto-initialize
						// get most fields (year, month, etc.) of timestamp for checking GPS from the current alarm time
						datetime_copy(&dt_CkGPS, &dt_CurAlarm);
						// get hour and minute from average
						uint16_t avgMinuteOfDayWhenMaxCharge = getAverageMinute(cellReadings);
						dt_CkGPS.hour = (uint8_t)(avgMinuteOfDayWhenMaxCharge / 60);
						dt_CkGPS.minute = (uint8_t)(avgMinuteOfDayWhenMaxCharge % 60);
						dt_CkGPS.second = 0; // don't really matter much
						dt_CkGPS.houroffset = 0; // average is derived as if Universal Time
						if (datetime_compare(&dt_CkGPS, &dt_CurAlarm) >= 0) {
							// timezone adjust has made the GPS check time earlier than the current alarm
							(dt_CkGPS.day)++; // move one day ahead
							datetime_normalize(&dt_CkGPS);
						}
						gpsFlags.checkGpsToday = 1;
						// e.g. {"GPStime":{"setupfor":"2017-01-19 22:10:24 -00","now":"2017-01-18 22:45:03 +00"}}
						strcat(strJSON, "\r\n{\"GPStime\":{\"setupfor\":\"");
						datetime_getstring(datetime_string, &dt_CkGPS);
						strcat(strJSON, datetime_string);
						strcat(strJSON, "\",\"now\":\"");
						datetime_getstring(datetime_string, &dt_CurAlarm);
						strcat(strJSON, datetime_string);
						strcat(strJSON, "\"}}\r\n");
						stateFlags1.writeJSONMsg = 1;
					} else { // gpsTimeAutoInit == 0
						// if it's time to log data
						if ((!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0)) || (irradFlags.isDark)) {
							int l;
							char n[16];
							strcat(strJSON, "\r\n{\"GPStime\":{\"dueButNotSetup\":\"");
							strcat(strJSON, "gpsTimeAutoInit == 0\n\r");
							strcat(strJSON, "\",\"secsSinceGPSReading\":\"");
							l = sprintf(n, "%.0f\n\r", (double)(gpsSecsElapsed));
							strcat(strJSON, n);
							strcat(strJSON, "\",\"secsPastDue\":\"");
							l = sprintf(n, "%.0f\n\r", (double)(gpsSecsElapsed - secsCtToCkGpsTime));
							strcat(strJSON, n);
							strcat(strJSON, "\"}}\r\n");
							stateFlags1.writeJSONMsg = 1;
							(void)l; // avoid compiler warning
						}
					}
				} // end test whether to access GPS
			} else { // not time to set up a request yet
/* don't do this; would run on every single alarm
				// e.g {"GPStime":{"notsetup":"2017-01-19 22:45:03 +00","now":"2017-01-18 22:45:03 +00"}}
				strcat(strJSON, "\r\n{\"GPStime\":{\"notsetup\":\"");
				datetime_getstring(datetime_string, &dt_LatestGPS);
				strcat(strJSON, datetime_string);
				strcat(strJSON, "\",\"now\":\"");
				datetime_getstring(datetime_string, &dt_CurAlarm);
				strcat(strJSON, datetime_string);
				strcat(strJSON, "\"}}\r\n");
*/
			}
			
			// test if date has changed
			if ((cellReadingsPtr->timeStamp.day != dt_CurAlarm.day) 
					|| (cellReadingsPtr->timeStamp.month != dt_CurAlarm.month)
					|| (cellReadingsPtr->timeStamp.year != dt_CurAlarm.year)) {
				// date is different, because of day rollover or time change
				// temporary diagnostics e.g.
				// {"movAvgItem":{"was":"0\t1.335\t2017-01-17 22:10:24 -00","now":"1\t1.231\t2017-01-18 22:45:03 +00"}}
				strcat(strJSON, "\r\n{\"movAvgItem\":{\"was\":\"");
				len = sprintf(str, "%i\t", (cellReadingsPtr - cellReadings));
				strcat(strJSON, str);
				chargeInfo_getString(str, cellReadingsPtr);
				strcat(strJSON, str);
				if ((dt_CurAlarm.year - cellReadingsPtr->timeStamp.year) > 2) {
					// if previous year is > 2 years different, either was previously 0 (initial
					//  null value) or from default date (see fn 'datetime_getDefault'); in either case,
					//  don't track a new date but overwrite the current one
					cellReadingsPtr->level = cellVoltageReading.adcWholeWord;
					datetime_copy(&(cellReadingsPtr->timeStamp), &dt_CurAlarm);
				} else { // only a regular new date
					// point to the next position to fill in the readings array
					cellReadingsPtr++;
					if ((cellReadingsPtr - cellReadings) >= DAYS_FOR_MOVING_AVERAGE)
								cellReadingsPtr = cellReadings;
					datetime_copy(&(cellReadingsPtr->timeStamp), &dt_CurAlarm);
					cellReadingsPtr->level = 0; // initialize
				}
				strcat(strJSON, "\",\"now\":\"");
				len = sprintf(str, "%i\t", (cellReadingsPtr - cellReadings));
				strcat(strJSON, str);
				chargeInfo_getString(str, cellReadingsPtr);
				strcat(strJSON, str);
				strcat(strJSON, "\"}}\r\n");
				stateFlags1.writeJSONMsg = 1;
			}
			
			// track the maximum cell voltage for this date
			if (cellVoltageReading.adcWholeWord > cellReadingsPtr->level) {
				cellReadingsPtr->level = cellVoltageReading.adcWholeWord;
				datetime_copy(&(cellReadingsPtr->timeStamp), &dt_CurAlarm);
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

			
			// see if it's time to log data
			if ((!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0)) || (irradFlags.isDark)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark
				timeFlags.timeToLogData = 1;
				if (irradFlags.isDark) { // store the voltage reading at this point
					refDarkVoltage = cellVoltageReading.adcWholeWord; 
				}
				
			//	outputStringToWiredUART("\n\r Time to log data \n\r");
			} else {
				timeFlags.timeToLogData = 0;
			//	outputStringToWiredUART("\n\r Not time to log data \n\r");
			}
			
			// if not time to log data, and not roused
			if ((!(timeFlags.timeToLogData)) && (!(stateFlags1.isRoused))) {
				// won't do anything with results anyway, don't bother reading sensors, save power
				stayRoused(5); // rouse for 0.05 second to flash the pilot light
				break; 
			}
			
			if ((gpsFlags.checkGpsToday) // flag to check but has not been serviced
					&& (datetime_compare(&dt_CkGPS, &dt_CurAlarm) > 1)) { // alarm has passed GPS check time
				GPS_initTimeRequest(); // send a low-going reset pulse, to start subsystem uC
				// e.g. {"GPStime":{"requested":"2017-01-18 22:45:03 +00"}}
				strcat(strJSON, "\r\n{\"GPStime\":{\"requested\":\"");
				datetime_getstring(datetime_string, &dt_CurAlarm);
				strcat(strJSON, datetime_string);
				strcat(strJSON, "\"}}\r\n");
				stateFlags1.writeJSONMsg = 1;
			}
			
			stateFlags1.logSilently = 0; // show diagnostics while gathering data
			if (makeLogString()) break;
			if (timeFlags.timeToLogData) {
//				len = strlen(strLog); // 'makeLogString' internally creates log string
//				errSD = writeCharsToSDCard(strLog, len);
				errSD = writeLogStringToSDCard();
				if (errSD) {
					tellFileError (errSD);
				} else {
					outputStringToBothUARTs(" Data written to SD card \n\r\n\r");
				}
			} // end of test if time to log data
			
			// test if dark or not dark, 'makeLogString' internally sets these flags
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
		} // end of data acquisition segment, while AlarmDetected
				
		turnSDCardPowerOff();
		
		if (stateFlags1.reachedFullPower) { // another full-power-only segment
			if (!BT_connected()) { // timeout diagnostics if no Bluetooth connection
				if (stateFlags1.isRoused) {
					len = sprintf(str, "\r\n sleep in %d seconds\r\n", (rouseCountdown/100));
					outputStringToWiredUART(str);
				}
				if (BT_powered()) {
					len = sprintf(str, "\r\n BT off in %d seconds\r\n", (btCountdown/100));
					outputStringToWiredUART(str);
				}	
			}
		} // end of this full-power-only segment
	} // end of main program loop
} // end of fn main

/**
 * \brief make a log string
 *
 * Create a string that is the series of data values
 *  to log. That string will be in the buffer 'strLog'.
 * If the logSilently flag is set, does
 *  not output any diagnostics to the UARTs
 *  while creating the string; otherwise echoes
 *  everything to both UARTs.
 * Uses many global variables:
 * Expects the timestamp to be in dt_CurAlarm
 * Expects the cell voltage in cellVoltageReading
 * Uses buffers str, strLog
 * Uses irrReadings
 * Sets flags isDarkBBDn, isDarkIRDn, isDarkBBUp, isDarkIRUp
 * The calling routine decides whether to write 
 *  the created string to the SD card.
 * Returns an error code, 0= no error.
 * \
 *  
 */

uint8_t makeLogString(void) {
	int strLn;
	uint8_t iTmp0;
	char ts[25];
	strcpy(strLog, "\n\r");
	if (!(stateFlags1.logSilently)) outputStringToBothUARTs("\r\n");
	datetime_getstring(ts, &dt_CurAlarm);
	strcat(strLog, ts);
	if (!(stateFlags1.logSilently)) outputStringToBothUARTs(ts);
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_READ_DATA) {
		if (!(stateFlags1.logSilently)) {
			strLn = sprintf(str, "\t power too low to read sensors, %lumV\r\n", 
					(unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
			outputStringToBothUARTs(str);
		}
		return 1; // check this
	} // end of testing if power too low
	
	// attempt to assure time zone is synchronized
	syncTimeZone(); // internally tests if work is already done
	
	// attempt to read irradiance sensors
	for (uint8_t i=0; i<4; i++) {
		uint8_t swDnUp, swBbIr;
		switch (i) {
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
		iTmp0 = getIrrReading(swDnUp, swBbIr, &irrReadings[i]);
		
		if (iTmp0) { // some error
			if (iTmp0 == errNoI2CAddressAck) { // not present or not responding, valid blank
				if (!(stateFlags1.logSilently)) outputStringToBothUARTs("\t-");
			} else { // unrecoverable error
				if (!(stateFlags1.logSilently)) {
					strLn = sprintf(str, "\n\r Could not get reading, err code: %x \n\r", iTmp0);
					outputStringToBothUARTs(str);
				}
				return 2; // check this, return iTmp0?
			}
		} else { // no error getting this reading
			if (!(stateFlags1.logSilently)) {
				strLn = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[i].irrWholeWord 
						* (unsigned long)irrReadings[i].irrMultiplier));
				outputStringToBothUARTs(str);
			}
		}
	} // end of for loop that reads irradiance sensors
	// got all 4 irradiance readings OK, append them to the log string
	irradFlags.isDarkBBDn = 0;
	irradFlags.isDarkIRDn = 0;
	irradFlags.isDarkBBUp = 0;
	irradFlags.isDarkIRUp = 0; // default clear
	for (uint8_t i=0; i<4; i++) { 
		unsigned long irrVal, irrDarkCutoff;
		if (irrReadings[i].validation) { // only error that could have got to this point is a valid blank
			strcat(strLog, "\t");
			switch (i) { // treat invalid readings as if dark
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
		} else { // a valid reading
			irrVal = (unsigned long)((unsigned long)irrReadings[i].irrWholeWord 
				* (unsigned long)irrReadings[i].irrMultiplier);
			strLn = sprintf(str, "\t%lu", irrVal);
			strcat(strLog, str);
			if ((i == 0) || (i == 2)) { // broadband
				irrDarkCutoff = darkCutOffBB;
			} else { // infrared
				irrDarkCutoff = darkCutoffIR;
			}
			if (irrVal < irrDarkCutoff) {
				switch (i) {
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
			} // end of if a reading is below dark threshold
		} // end of if valid or blank reading
	} // end of for loop that appends irradiance readings
	
	// log temperature
	// insert spacer, even if can not get temperature reading
	strcat(strLog, "\t");
	if (!(stateFlags1.logSilently)) outputStringToBothUARTs("\t");
	if (!(temperature_GetReading(&temperatureReading))) { // got temperature
		strLn = sprintf(str, "\t%d", (int8_t)(temperatureReading.tmprHiByte));
		strcat(strLog, str);
		if (!(stateFlags1.logSilently)) outputStringToBothUARTs(str);
	}
	
	// log cell voltage
	// calc cell voltage from ADC reading earlier
	// formula from datasheet: V(measured) = adcResult * (1024 / Vref)
	// using internal reference, Vref = 2.56V = 2560mV
	// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
	strLn = sprintf(str, "\t%lu\n\r", (unsigned long)(2.5 * (unsigned long)(cellVoltageReading.adcWholeWord)));
	strcat(strLog, str);
	if (!(stateFlags1.logSilently)) outputStringToBothUARTs(str);
	
	return 0; // got log string OK
}

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
 * \brief Send a string out the wired UARTUART0
 *
 * Copy characters from the passed null-terminated
 * string to the UART0 output buffer. 
 * UART0 is the hardwired port.
 * Inline fn "uart0_putchar" flags to start transmitting, if necessary.
 *
 * \
 *  
 */

void outputStringToWiredUART (char* St) {
	uint8_t cnt;
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_UART) return;
	if (stateFlags1.isRoused) { // if system not roused, no output
		for (cnt = 0; cnt < strlen(St); cnt++) {
			uart0_putchar(St[cnt]);
		}	
	}
	while (uart0_char_queued_out())
		;
} // end of outputStringToWiredUART

/**
 * \brief Send a string out the Bluetooth UART
 *
 * Copy characters from the passed null-terminated
 * string to the UART1 output buffer.
 * UART1 is the Bluetooth port.
 * Inline fn "uart1_putchar" flags to start transmitting, if necessary.
 *
 * \
 *  
 */

void outputStringToBluetoothUART (char* St) {
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
} // end of outputStringToBluetoothUART

/**
 * \brief Send the same string out both UART0 and UART1
 *
 * Call the fn to send a string out each UART
 *
 * \
 *  
 */
void outputStringToBothUARTs (char* St) {
	outputStringToWiredUART (St);
	outputStringToBluetoothUART (St);
}	


/**
 * \brief Show the cell readings
 *
 * Show the series of cell readings
 * that are used to track the moving
 * average of maximum daily cell charge
 * \
 *  
 */
void showCellReadings(void) {
	outputStringToBothUARTs("\r\nCell readings\r\n");
	for (uint8_t i=0; i<DAYS_FOR_MOVING_AVERAGE; i++) {
		// give diagnostics on the readings being stored for the moving average
		chargeInfo_getString(stCellReading, &(cellReadings[i]));
		outputStringToBothUARTs(stCellReading);
	}
	outputStringToBothUARTs("\r\nDone with cell readings\r\n");
}

/**
 * \brief Get the cell readings into strJSON
 *
 * Get the series of cell readings
 * that are used to track the moving
 * average of maximum daily cell charge
 * into strJSON.
 * Intended use for temporary diagnostics, with
 * strJSON starting empty, and the calling routing
 * erasing strJSON again after using it
 * \
 *  
 */
void getCellReadingsIntoStrJSON(void) {
	char s[128];
	strcat(strJSON,"{\"cellreadings\":{\n\r\n\r");
	for (uint8_t i=0; i<DAYS_FOR_MOVING_AVERAGE; i++) {
		// get diagnostics on the readings being stored for the moving average
		chargeInfo_getString(s, &(cellReadings[i]));
		strcat(strJSON, s);
		strcat(strJSON, "\n\r");
	}
	// not proper JSON, but good enough for diagnostics
	strcat(strJSON,"\"}}\n\r\n\r");
}

/**
 * \brief Get latest GPS time into strJSON
 *
 * Get dt_LatestGPS
 * into strJSON.
 * Intended use for temporary diagnostics, with
 * strJSON starting empty, and the calling routing
 * erasing strJSON again after using it
 * \
 *  
 */
void getLatestGpsTimeIntoStrJSON(void) {
	char s[64];
	int32_t d;
	int l;
	strcat(strJSON,"{\"latestGPStime\":{\n\r");
	datetime_getstring(s, &dt_LatestGPS);
	strcat(strJSON, s);
	strcat(strJSON, "\n\r");
	d = (int32_t)(datetime_totalsecs(&dt_CurAlarm) - (datetime_totalsecs(&dt_LatestGPS)));
	l = sprintf(s, "%.0f elapsed\n\r", (double)d);
	strcat(strJSON, s);
	l = sprintf(s, "%.0f target\n\r", (double)secsCtToCkGpsTime);
	strcat(strJSON, s);
	// not proper JSON, but good enough for diagnostics
	strcat(strJSON,"\"}}\n\r");
	(void)l; // avoid compiler warning
}

/**
 * \brief Get XYZ level data into strJSON
 *
 * Get the XYZ axis info from the accelerometer
 * into strJSON.
 * Intended to log in each new file, to verify
 * instrument was level.
 * Ideally, strJSON should have been written and
 * cleared before calling this.
 * \
 *  
 */
void getAxesIntoStrJSON(void) {
	accelAxisData d;
	char s[64];
	int l;
	// e.g. {"axisInfo":[{"X":"-1", "Y":"2", "Z":"-160"}]}
	strcat(strJSON,"\n\r{\"axisInfo\":{");
	uint8_t rs = getAvAccelReadings(&d);
	if (rs) {
		l = sprintf(s, "\"errCode\":\"%i\"", rs);
	} else {
		l = sprintf(s, "\"X\":\"%i\", \"Y\":\"%i\", \"Z\":\"%i\"", d.xWholeWord, d.yWholeWord,  d.zWholeWord);
	}
	strcat(strJSON, s);
	strcat(strJSON, "}]}\n\r");
	(void)l; // avoid compiler warning
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
					 gpsFlags.gpsReqTest = 1; // this is a manually initiated test, not from the system
					 gpsFlags.gpsTimeRequested = 0; // allow manual request to override
					 GPS_initTimeRequest();
					 break;
				 }					 
				
				 case 'V': case 'v': { // show firmware version
					 outputStringToBothUARTs(VERSION_STRING);
					 break;
				 }					 

                case 'T': case 't': { // set time
					// get info from commandBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					// Presently, a set-time command from the GPS could not come in by Bluetooth, but 
					// a request to the GPS could be sent by Bluetooth. In that case, user is probably
					// monitoring Bluetooth modem, UART1, and so diagnostics should go there
					char tmpStr[COMMAND_BUFFER_LENGTH];
					char jts[25];
					strcpy(tmpStr, commandBuffer + 1);
					if (!isValidDateTime(tmpStr)) {
						if (gpsFlags.gpsTimeRequestByBluetooth) {
							outputStringToBluetoothUART("\r\n Invalid timestamp from GPS\r\n");
						} else {
							outputStringToWiredUART("\r\n Invalid timestamp\r\n");
						}
						break;
					}
					if (!isValidTimezone(tmpStr + 20)) {
						if (gpsFlags.gpsTimeRequestByBluetooth) {
							outputStringToBluetoothUART("\r\n Invalid hour offset from GPS\r\n");
						} else {
							outputStringToWiredUART("\r\n Invalid hour offset\r\n");
						}
						break;
					}
					strcpy(strJSONtc, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(jts, &dt_RTC);
					strcat(strJSONtc, jts);

					if (gpsFlags.gpsTimeRequestByBluetooth) {
						outputStringToBluetoothUART("\r\n Time changed, by GPS, from ");
						outputStringToBluetoothUART(jts);
					} else {
						outputStringToWiredUART("\r\n Time changed from ");
						outputStringToWiredUART(jts);
					}
					datetime_getFromUnixString(&dt_tmp, tmpStr, 0);
					rtc_setTime(&dt_tmp);
					strcat(strJSONtc, "\",\"to\":\"");
					datetime_getstring(jts, &dt_tmp);
					strcat(strJSONtc, jts);
					if (gpsFlags.gpsTimeRequestByBluetooth) {
						outputStringToBluetoothUART(" to ");
						outputStringToBluetoothUART(jts);
						outputStringToBluetoothUART("\r\n");					
					} else {
						outputStringToWiredUART(" to ");
						outputStringToWiredUART(jts);
						outputStringToWiredUART("\r\n");
					}
					if (gpsFlags.gpsTimeRequested) { // set-time signal was requested from GPS
						// for now, assume that's where this came from
						strcat(strJSONtc, "\",\"by\":\"GPS\"}}\r\n");
						rtcStatus = rtcHasGPSTime;
						
						datetime_copy(&dt_LatestGPS, &dt_tmp);
						if (gpsFlags.gpsReqTest) { // this was a manually initiated test request, not from the system
							showCellReadings();
							gpsFlags.gpsReqTest = 0; // test is over
						}
						if (strlen(tmpStr) >= 25) {
							// there may be extra data, such as location
							saveGPSLocation(tmpStr + 24); // attempt to extract lat/lon
							if (gpsFlags.gpsNewLocation) {
								outputStringToBluetoothUART(strJSONloc);
							}
						}
						gpsFlags.gpsTimeRequested = 0; // request has been serviced
						gpsFlags.gpsTimeRequestByBluetooth = 0; // end of request by BT
						gpsFlags.checkGpsToday = 0; // time is set by GPS, clear any pending request
						initFlags.gpsTimeAutoInit = 1; // no longer try to auto-initialize
					} else { // time was set manually
						strcat(strJSONtc, "\",\"by\":\"hand\"}}\r\n");
						outputStringToWiredUART("\r\n");
						rtcStatus = rtcTimeManuallySet;
					}
					stateFlags1.writeTimeChangeMsg = 1; // log JSON Time Change message on next SD card write
					stateFlags1.writeDataHeaders = 1; // log data column headers on next SD card write
					gpsFlags.checkGpsToday = 0; // un-flag this on any time change, force to re-test
					
					outputStringToWiredUART(strHdr);
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
					char ls[32];			
					if (getAvAccelReadings(&accelData)) break;
					outputStringToBluetoothUART("\r\n test point 2\r\n");
					len = sprintf(ls, "X = %i, Y = %i, Z = %i\n\r", accelData.xWholeWord,
						accelData.yWholeWord,  accelData.zWholeWord);
						outputStringToBothUARTs(str);
					outputStringToBluetoothUART("\r\n test point 3\r\n");
                    break;
                }

                case 'C': case 'c':
					{
						len = sprintf(str, "\n\r test write to SD card 0x%x\n\r", (disk_initialize(0)));
						intTmp1 = writeCharsToSDCard(str, len);
						 // sd card diagnostics
						outputStringToWiredUART("\r\n test write to SD card\r\n");
					//	len = sprintf(str, "\n\r PINB: 0x%x\n\r", (PINB));
						// send_cmd(CMD0, 0)
//					len = sprintf(str, "\n\r CMD0: 0x%x\n\r", (send_cmd(CMD0, 0)));
						break;						
					}

                case 'P': case 'p': 
				{ // force SD card power off
                    outputStringToWiredUART("\r\n turning SD card power off\r\n");
					turnSDCardPowerOff();
					outputStringToWiredUART("\r\n SD card power turned off\r\n");
                    break;
                }
/*
                case '^':
				{ // force B3 high
                    outputStringToWiredUART("\r\n forcing B3 high\r\n");
					DDRB |= 0b00001000;
					PORTB |= 0b00001000;
					outputStringToWiredUART("\r\n B3 forced high \r\n");
                    break;
                }

                case 'v':
				{ // force B3 low
                    outputStringToWiredUART("\r\n forcing B3 low\r\n");
					DDRB |= 0b00001000;
					PORTB &= 0b11110111;
					outputStringToWiredUART("\r\n B3 forced low \r\n");
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
								outputStringToWiredUART(str);

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
						//outputStringToWiredUART(str);
						//break;
					//}						
//
                //case 'B': case 'b':
					//{ // experimenting with reading the battery voltage using the Analog to Digital converter
						//len = sprintf(str, "\n\r Hi byte: %d \n\r", readCellVoltage(&cellVoltageReading));
						//outputStringToWiredUART(str);
						//len = sprintf(str, "\n\r 16bit value: %d \n\r", cellVoltageReading.adcWholeWord);
						//outputStringToWiredUART(str);
						//
//
						//break;
					//}						
//				
//
                case 'D': case 'd': 
				{ // output file 'D'ata (or 'D'ump)
                    outputStringToWiredUART("\r\n (data dump only works from Bluetooth)\r\n");
                    break;
                }

                // put other commands here
                default: 
				{ // if no valid command, echo back the input
                    outputStringToWiredUART("\r\n> ");
                    outputStringToWiredUART(commandBuffer);
                    outputStringToWiredUART("\r\n");
//                    startTimer1ToRunThisManySeconds(30); // keep system Roused another two minutes
					break;
                }
            } // switch (commandBuffer[0])
            commandBuffer[0] = '\0';
            commandBufferPtr = commandBuffer; // "empty" the command buffer
         } else { // some other character
             // ignore repeated linefeed (or linefeed following carriage return) or carriage return
             if (!((c == 0x0a) || (c == 0x0a))) { 
                 if (commandBufferPtr < (commandBuffer + COMMAND_BUFFER_LENGTH - 1)) // if there is room
                     *commandBufferPtr++ = c; // append char to the command buffer
             }
         } // done parsing character
    } // while (1)
} // end of checkForCommands

void outputLevelingDiagnostics(void) {
	BYTE errSD;
	int16_t xPrev, yPrev, zPrev;
	showLeveling(12000); // show leveling diagnostics for 2 minutes (120 sec); continue as long as changing
//	showLeveling(300); // show leveling diagnostics for 30 sec; continue as long as changing
	while (motionFlags.isLeveling) { // cleared by timeout
		while (timeFlags.alarmDetected) { // continue logging during leveling diagnostics
			// use 'while' loop to allow various tests to break out
			timeFlags.alarmDetected = 0; // clear flag so 'while' loop will only happen once in any case
			timeFlags.nextAlarmSet = 0; // flag that current alarm is no longer valid
			// will use to trigger setting next alarm

			// test if time to log; even number of minutes, and zero seconds
			if (!(!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0))) {
				break;
			}
				
			// check cell voltage is high enough
			int intTmp1 = readCellVoltage(&cellVoltageReading);
			if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
				break;
			}

			stateFlags1.logSilently = 1; // don't show any diagnostics while gathering data
			if (makeLogString()) break; // exit on error
			errSD = writeLogStringToSDCard();
			if (errSD) {
				tellFileError (errSD);
			} // end of silent logging, this latest data
				
		} // end of alarm detected
			
		// set next alarm
		if (!(timeFlags.nextAlarmSet)) {
			if (!rtc_setupNextAlarm(&dt_CurAlarm))
			timeFlags.nextAlarmSet = 1;
		}
		// end of continue logging during leveling diagnostics
		
		// try displaying leveling diagnostics as fast as possible, no particular pacing
		{
			accelAxisData d;
			char ls[32];
			int len;
			uint8_t rs = getAvAccelReadings(&d);
			if (rs) {
				len = sprintf(ls, "\n\r error code %i\n\r", rs);
				outputStringToBothUARTs(ls);
			} else {
				len = sprintf(ls, " X = %i, Y = %i, Z = %i\n\r", d.xWholeWord, d.yWholeWord,  d.zWholeWord);
				outputStringToBothUARTs(ls);
				// first time, test below will always be true; but after that will really test for change
				if ((abs((int)(xPrev - (d.xWholeWord))) > 1) || 
						(abs((int)(yPrev - (d.yWholeWord))) > 1) || 
						(abs((int)(zPrev - (d.zWholeWord))) > 1)) { // still moving, sustain the timeout
//					showLeveling(6000); // show leveling diagnostics another minute (60 secs)
					showLeveling(100); // show leveling diagnostics another 10 secs
				}
				xPrev = (d.xWholeWord);
				yPrev = (d.yWholeWord);
				zPrev = (d.zWholeWord);
			}
		}		
	} // timeout will clear the isLeveling flag, and allow to pass beyond here
	stateFlags1.logSilently = 0; // go back to displaying logging
	return;
}

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

	if (gpsTimeReqCountdown > rouseCountdown)
		rouseCountdown = gpsTimeReqCountdown; // stay roused at least as long as GPS time request is active		
	
	if (levelingCountdown > rouseCountdown)
		rouseCountdown = levelingCountdown; // stay roused at least as long as showing leveling diagnostics
		
	t = levelingCountdown;
	if (t) levelingCountdown = --t;
	
	if (!levelingCountdown) {
		motionFlags.isLeveling = 0;
		stateFlags1.logSilently = 0;
	}
		
	t = gpsTimeReqCountdown;
	if (t) gpsTimeReqCountdown = --t;
	
	if (!gpsTimeReqCountdown) {
		gpsFlags.gpsTimeRequested = 0; // GPS time request terminate
		gpsFlags.gpsTimeRequestByBluetooth = 0; // BT request terminate
	} 
	
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
		gpsFlags.gpsTimeRequested = 0; // set-time request from GPS times out if system goes out of Roused mode
		gpsFlags.gpsReqTest = 0; // any test now ends
		gpsFlags.gpsTimeRequestByBluetooth = 0; // BT request terminate
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

