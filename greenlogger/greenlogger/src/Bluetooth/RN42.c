/*
 * RN41.c
 *
 * Created: 4/25/2012 10:50:13 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "../Bluetooth/RN42.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"
#include "../Accelerometer/ADXL345.h"
#include "../TemperatureSensor/TCN75A.h"
#include "../SDcard/diskio.h"
#include "../BattMonitor/ADconvert.h"
#include "../LtSensor/TSL2561.h"
#include "../GPS/GPStime.h"
#include <util/twi.h>

char btCmdBuffer[COMMAND_BUFFER_LENGTH];
char *btCmdBufferPtr;
uint8_t errSD;

extern volatile uint8_t machineState;
extern volatile uint16_t timer3val;
extern char commandBuffer[COMMAND_BUFFER_LENGTH];
extern char *commandBufferPtr;
extern char str[128]; // generic space for strings to be output
extern char strLog[64];
extern char strJSONtc[256]; // string for JSON data
extern volatile uint8_t Timer1, Timer2, intTmp1;
extern volatile dateTime dt_RTC, dt_CurAlarm, dt_tmp, dt_LatestGPS; //, dt_NextAlarm
extern volatile sFlags1 stateFlags1;
extern volatile bFlags btFlags;
extern volatile tFlags timeFlags;
extern volatile gFlags gpsFlags;
extern volatile rFlags irradFlags;
extern volatile mFlags motionFlags ;
extern volatile uint8_t rtcStatus;
extern char strHdr[64];
extern int len, err;
extern volatile accelAxisData accelData;
extern volatile int8_t timeZoneOffset;
extern unsigned long darkCutoffIR, darkCutOffBB;
extern volatile adcData cellVoltageReading;
extern irrData irrReadings[4];
extern chargeInfo cellReadings[DAYS_FOR_MOVING_AVERAGE];


/**
 * \brief measure RTC square wave
 *
 * Count how many cycles of the main oscillator occur for one half of the RTC square wave output
 * Used for calibrating the main oscillator
 * \
 *  
 */
uint16_t cyPerRTCSqWave(void) {
	uint8_t i, sreg;
	twoByteData cyPerSec;
	// go into uC clock adjust mode
//	outputStringToBluetoothUART("\r\n going into uC adjust mode\r\n");
	timeFlags.nextAlarmSet = 0; // clear flag
	disableRTCInterrupt();
	intTmp1 = rtc_enableSqWave();
	// PRTIM1 make sure power reduction register bit if off so timers run
					
	machineState = Idle; // flag to wait
	enableRTCInterrupt();
	while (machineState == Idle) { // RTC Interrupt will break out of this
		;
	}
	setupTimer3_1shot(); // zeroes Timer3
	for (i=0; i<128; i++) { // count osc cycles for 64 full RTC square wave cycles
		machineState = Idle; // flag to wait
		// RTC interrupt disables itself
		enableRTCInterrupt();
		while (machineState == Idle) { // RTC Interrupt will break out of this
			;
		}
	}
	// read Timer3
	// Save global interrupt flag
	sreg = SREG;
	// Disable interrupts
	cli();
	// Read TCNTn into cyPerSec
	cyPerSec.loByte = TCNT3L;
	cyPerSec.hiByte = TCNT3H;
	// Restore global interrupt flag
	SREG = sreg;	
	// go back into normal timekeeping mode
	setupTimer3_10ms();
	disableRTCInterrupt();
//	outputStringToBluetoothUART("\r\n returning to timekeeping mode\r\n");
	if (!(timeFlags.nextAlarmSet)) {
		if (!rtc_setupNextAlarm(&dt_CurAlarm))
			timeFlags.nextAlarmSet = 1;
	}
	return (uint16_t)cyPerSec.wholeWord;
}


/**
 * \brief Check UART1 (Bluetooth) for commands
 *
 * Check the UART1 receive buffer for commands.
 * UART1 communicates through the RN-42 Bluetooth module.
 * \
 *  
 */

void checkForBTCommands (void) {
	char c;
	char stTmp[COMMAND_BUFFER_LENGTH];
	if (!BT_connected()) return;

	if (!(btFlags.btWasConnected)) { // new connection
		// initialize and clear buffer, ignore anything there before
		uart1_init_input_buffer();
		btCmdBuffer[0] = '\0'; // "empty" the command buffer
		btCmdBufferPtr = btCmdBuffer;
		btFlags.btWasConnected = 1; // set flag
		return; // bail now, pick it up on next pass
	}
	
	while (uart1_char_waiting_in()) {
		motionFlags.isLeveling = 0; // go out of Leveling mode on any input
		c = uart1_getchar();
		if (c == 0x08) { // backspace
			if (btCmdBufferPtr > btCmdBuffer) { // if there is anything to backspace
				btCmdBufferPtr--;
			}
		}
		if (c == 0x0d) // if carriage-return
			c = 0x0a; // substitute linefeed
		if (c == 0x0a) { // if linefeed, attempt to parse the command
			*btCmdBufferPtr++ = '\0'; // null terminate
			btFlags.btSerialBeingInput = 0; // flag that input is complete; allow string output
			switch (btCmdBuffer[0]) { // command is 1st char in buffer

				 case 'G': case 'g': { // get time from GPS
					 // for testing, manually initiate a get-time request from GPS
					 gpsFlags.gpsReqTest = 1; // this is a manually initiated test, not from the system
					 gpsFlags.gpsTimeRequestByBluetooth = 1; // request came in by the BT modem
					 
					 // for testing, put diagnostics here too, redundant
					 showCellReadings();
					
					 GPS_initTimeRequest();
					 break;
				 }					 
				
				 case 'V': case 'v': { // show firmware version
					 outputStringToBothUARTs(VERSION_STRING);
					 break;
				 }

				case 'T': case 't': { // set time
					// get info from btCmdBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					strcpy(stTmp, btCmdBuffer + 1);
					if (!isValidDateTime(stTmp)) {
						outputStringToBluetoothUART("\r\n Invalid timestamp\r\n");
						break;
					}
					if (!isValidTimezone(stTmp + 20)) {
						outputStringToBluetoothUART("\r\n Invalid hour offset\r\n");
						break;
					}
					char ts[25];
					outputStringToBluetoothUART("\r\n Time changed from ");
					strcat(strJSONtc, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(ts, &dt_RTC);
					strcat(strJSONtc, ts);
					outputStringToBluetoothUART(ts);
					datetime_getFromUnixString(&dt_tmp, stTmp, 0);
					rtc_setTime(&dt_tmp);
					strcat(strJSONtc, "\",\"to\":\"");
					outputStringToBluetoothUART(" to ");
					datetime_getstring(ts, &dt_tmp);
					strcat(strJSONtc, ts);
					outputStringToBluetoothUART(ts);
/*	presently, a set-time command from the GPS could not come in by Bluetooth, though a
     request to the GPS to send the command could go out by Bluetooth
*/
					strcat(strJSONtc, "\",\"by\":\"hand\"}}\r\n");
					outputStringToBluetoothUART("\r\n");
					stateFlags1.writeTimeChangeMsg = 1; // log the Time Change JSON message on next SD card write
					stateFlags1.writeDataHeaders = 1; // log data column headers on next SD card write
					rtcStatus = rtcTimeManuallySet;
					outputStringToBluetoothUART(strHdr);
					if (!rtc_setupNextAlarm(&dt_CurAlarm))
						timeFlags.nextAlarmSet = 1;
					// cache timezone offset in persistent storage on SD card
					timeZoneOffset = dt_tmp.houroffset;
					timeFlags.timeZoneWritten = 0; // flag that time zone needs to be written
					syncTimeZone(); // attempt to write the time zone; will retry later if e.g. power too low
					gpsFlags.checkGpsToday = 0; // un-flag this on any time change, force to re-test
					break;
				}
				
				case 'L': case 'l': 
				{
					showLeveling(12000); // show leveling diagnostics for 2 minutes (120 sec)
					//	showLeveling(300); // show leveling diagnostics for 30 sec
//					outputLevelingDiagnostics(); // hang here till timeout
/*
					// experimenting with the accelerometer Leveling functions
					char ls[32];
					uint8_t rs = getAvAccelReadings(&accelData);
					if (rs) {
						len = sprintf(ls, "\n\r error code %i\n\r", rs);
						outputStringToBothUARTs(ls);
						break;
					} 
					len = sprintf(ls, "\n\r X = %i, Y = %i, Z = %i\n\r", accelData.xWholeWord,
					accelData.yWholeWord,  accelData.zWholeWord);
					outputStringToBothUARTs(ls);
*/
					break;
				}

				case 'D': case 'd': 
				{ // output file 'D'ata (or 'D'ump)
					strcpy(stTmp, btCmdBuffer); // make a copy of the command
					BT_dataDump(stTmp); // let the general fn take it from there
					break;
				}
				// put other commands here
				default: 
				{ // if no valid command, echo back the input
					outputStringToBluetoothUART("\r\n> ");
					outputStringToBluetoothUART(btCmdBuffer);
					outputStringToBluetoothUART("\r\n");
//					startTimer1ToRunThisManySeconds(30); // keep system Roused another two minutes
					break;
				}
			} // switch (btCmdBuffer[0])
			btCmdBuffer[0] = '\0'; // "empty" the command buffer
			btCmdBufferPtr = btCmdBuffer;
		} else { // some other character
			if (btCmdBufferPtr < (btCmdBuffer + COMMAND_BUFFER_LENGTH - 1)) { // if there is room
				if (btCmdBufferPtr == btCmdBuffer) { // if first input
					outputStringToBluetoothUART("\r\n\r\n> "); // show the user a blank line & prompt
				}
				btFlags.btSerialBeingInput = 1; // flag that the user is inputting characters; blocks outputStringToBluetoothUART fn
				*btCmdBufferPtr++ = c; // append char to the command buffer
				uart1_putchar(c); // echo the character
			}					
		} // done parsing character
	} // while characters in buffer
} // end of checkForBTCommands

void BT_dataDump(char* stOpt) {
	uint8_t errBTDump;
//	char stBeginTryDate[12] = "2011-12-21"; // system default starting date, winter solstice 2011.
	char stBeginTryDate[12] = "2012-01-01"; //
	// There would be no data before this unless user mistakenly set system time earlier.
	char outputOpt = 'c'; // c=output contents, f=output filedate only
	char stTryDate[12], stEndTryDate[25];
	datetime_getstring(stEndTryDate, &dt_CurAlarm); // get current timestamp as string
	stEndTryDate[10] = '\0'; // truncate to only the date
//	outputStringToBluetoothUART("\n\r current date: \"");
//	outputStringToBluetoothUART(stEndTryDate);
//	outputStringToBluetoothUART("\"\n\r");
	// find first date that has data
	errBTDump = datetime_nextDateWithData(stBeginTryDate, 0);
	if (errBTDump == sdPowerTooLowForSDCard) {
		outputStringToBluetoothUART("\n\r Power too low for SD card.\n\r\n\r");
		return;
	}
	switch (stOpt[1]) {
		case '\0': { // "D" alone, most common option. Dump any days' data collected since previous dump.
			errBTDump = readLastDumpDateFromSDCard(stBeginTryDate); // attempt to fetch the previous dump date
			if (errBTDump) {
				outputStringToBluetoothUART("\n\r Failed to find previous dump date. Dumping all data.\n\r\n\r");
			}
			
			outputStringToBluetoothUART("\n\r lastDump date read from SD card: ");
			outputStringToBluetoothUART(stBeginTryDate);
			outputStringToBluetoothUART("\n\r");
		
			break;
		}		
		
		case 'A': case 'a': {  // all data, defaults are already set up
			break;
		}		
		
		case 'D': case 'd': { // output the list of dates that have data
			outputOpt = 'f'; // output dates rather than content
			break;
		}		
		
		default: {
			// see if it's a valid date
			if (isValidDate(stOpt + 1)) { // if so, dump data for this day only
				strncpy(stBeginTryDate, stOpt + 1, 10);
				stBeginTryDate[11] = '\0'; // assure terminated
				errBTDump = fileExistsForDate(stBeginTryDate);
				if (errBTDump == sdFileOpenFail) { // file does not exist
					outputStringToBluetoothUART("\n\r no data for ");
					outputStringToBluetoothUART(stBeginTryDate);
					outputStringToBluetoothUART(". Use \"dd\" to list dates having data.\n\r\n\r");
					return;
				}					
				strcpy(stEndTryDate, stBeginTryDate); // output one day only
			} else {
				outputStringToBluetoothUART("\n\r Invalid option.\n\r\n\r");
				return;
			}
			break;
		}

	} // end switch (stOpt[1])

	if (outputOpt == 'c')
		outputStringToBluetoothUART("\n\r{\"datadump\":\"begin\"}\n\r");
	else
		outputStringToBluetoothUART("\n\r{\"datesHavingData\":\"begin\"}\n\r");
	strcpy(stTryDate, stBeginTryDate);
	do { // loop like this so dump will output at least one day
		while (timeFlags.alarmDetected) { // continue logging during data dump
			uint8_t ct, swDnUp, swBbIr;
			// use 'while' loop to allow various tests to break out
			timeFlags.alarmDetected = 0; // clear flag so 'while' loop will only happen once in any case
			timeFlags.nextAlarmSet = 0; // flag that current alarm is no longer valid
			// will use to trigger setting next alarm

            // test if time to log; even number of minutes, and zero seconds
			if (!(!((dt_CurAlarm.minute) & 0x01) && (dt_CurAlarm.second == 0))) {
				break;
			}
			
			// check cell voltage is high enough
			intTmp1 = readCellVoltage(&cellVoltageReading);
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
		// end of continue logging during data dump
		
//			outputStringToBluetoothUART("\n\r");
		errBTDump = fileExistsForDate(stTryDate);
		// somewhat redundant to test first because output fns test internally, but avoids output for nonexistent files
		if (!errBTDump) {
			if (outputOpt == 'c') { // contents
				keepBluetoothPowered(120); // or eventually will time out
				outputStringToBluetoothUART("\n\r{\"datafor\":\"");
				outputStringToBluetoothUART(stTryDate);
				outputStringToBluetoothUART("\"}\n\r");	
				errBTDump = outputContentsOfFileForDate(stTryDate);
					if (errBTDump) {
						tellFileError (errBTDump);
						if (errBTDump == sdPowerTooLowForSDCard) {
							return;
						}
					}
				// most likely fail point is during day's output
				// mark success of each day, so as not to repeat on recovery
				errBTDump = writeLastDumpDateToSDCard(stTryDate);
				if (errBTDump) {
					tellFileError (errBTDump);
				}
			} else { // list of dates only
//				outputStringToBluetoothUART("\n\r{\"datafor\":\"");
				outputStringToBluetoothUART(stTryDate);
				outputStringToBluetoothUART("\n\r");
			}


		} else { // tested if file exists, and some error
			if (errBTDump != sdFileOpenFail) // file does not exist, ignore
				tellFileError (errBTDump);
		}
		strcpy(stBeginTryDate, stTryDate); // remember the date we just tried
		datetime_advanceDatestring1Day(stTryDate);
	} while (strcmp(stTryDate, stEndTryDate) <= 0);

//		outputStringToBluetoothUART("\n\r");
	if (outputOpt == 'c') { // data file contents 
		outputStringToBluetoothUART("\n\r{\"datadump\":\"end\"}\n\r\n\r");
		outputStringToBluetoothUART("\n\r");
		errBTDump = writeLastDumpDateToSDCard(stBeginTryDate);
		if (errBTDump) {
			tellFileError (errBTDump);
		}
	} else { // list of dates with data, only
		outputStringToBluetoothUART("\n\r{\"datesHavingData\":\"end\"}\n\r");
	}
		
	return;
}


