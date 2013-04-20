/*
 * RN41.c
 *
 * Created: 4/25/2012 10:50:13 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "RN42.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"
#include "../Accelerometer/ADXL345.h"
#include "../TemperatureSensor/TCN75A.h"
#include "../SDcard/diskio.h"
#include <util/twi.h>

char btCmdBuffer[commandBufferLen];
char *btCmdBufferPtr;
uint8_t errSD;

extern volatile uint8_t machineState;
extern volatile uint16_t timer3val;
extern char commandBuffer[commandBufferLen];
extern char *commandBufferPtr;
extern char str[128]; // generic space for strings to be output
extern char strJSON[256]; // string for JSON data
extern volatile uint8_t Timer1, Timer2, intTmp1;
extern volatile dateTime dt_RTC, dt_CurAlarm, dt_tmp, dt_LatestGPS; //, dt_NextAlarm
extern char datetime_string[25];
extern volatile uint8_t stateFlags1, stateFlags2, timeFlags, irradFlags, motionFlags, btFlags;
extern volatile uint8_t rtcStatus;
extern char strHdr[64];
extern int len, err;
extern volatile accelAxisData accelData;
extern volatile int8_t timeZoneOffset;
extern unsigned long darkCutoffIR, darkCutOffBB;

/**
 * \brief turns on power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
inline void BT_power_on(void)
{
	PORTD |= (1<<4); // set high; power on
}

/**
 * \brief turns off power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
inline void BT_power_off(void)
{
	PORTD &= ~(1<<4); // set low; turn off
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 9600
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
inline void BT_baud_9600(void)
{
	PORTD |= (1<<7); // set high, baud 9600
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 115k
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
inline void BT_baud_115k(void)
{
	PORTD &= ~(1<<7); // set low, baud 115k
}

/**
 * \brief check if the RN-42 Bluetooth module has a live connection
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
inline bool BT_connected(void)
{
	return (PIND & (1<<5)); // read pin
}

/**
 * \brief check if the RN-42 Bluetooth module is powered
 *
 * PortD, bit 4 is an output that enables the power supply to the 
 * RN-42 Bluetooth module: 
 * HIGH when enabled, LOW when shut down.
 * This functions reads that state of that pin.
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
inline bool BT_powered(void)
{
	return (PIND & (1<<4)); // read pin
}

/**
 * \brief measure RTC square wave
 *
 * Count how many cycles of the main oscillator occur for one half of the RTC square wave output
 * Used for calibrating the main oscillator
 * \
 *  
 */
uint16_t cyPerRTCSqWave(void) {
	uint8_t sreg;
	twoByteData cyPerSec;
	// go into uC clock adjust mode
//	outputStringToUART1("\r\n going into uC adjust mode\r\n");
	timeFlags &= ~(1<<nextAlarmSet); // clear flag
	disableRTCInterrupt();
	intTmp1 = rtc_enableSqWave();
	// PRTIM1 make sure power reduction register bit if off so timers run
					
	machineState = Idle; // flag to wait
	enableRTCInterrupt();
	while (machineState == Idle) { // RTC Interrupt will break out of this
		;
	}
	setupTimer3_1shot(); // zeroes Timer3
	machineState = Idle; // flag to wait
	// RTC interrupt disables itself
	enableRTCInterrupt();
	while (machineState == Idle) { // RTC Interrupt will break out of this
		;
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
	cyPerSec.wholeWord += 15; // assume 15 cycles of latency
//	len = sprintf(str, "Cycle count from RTC: %d\r\n", (uint16_t)cyPerSec.wholeWord);
//	outputStringToUART1(str);
//	len = sprintf(str, "calibration byte: %d\r\n", OSCCAL);
//	outputStringToUART1(str);				
	
	// go back into normal timekeeping mode
	setupTimer3_10ms();
	disableRTCInterrupt();
//	outputStringToUART1("\r\n returning to timekeeping mode\r\n");
	if (!(timeFlags & (1<<nextAlarmSet))) {
		intTmp1 = rtc_setupNextAlarm(&dt_CurAlarm);
		timeFlags |= (1<<nextAlarmSet);
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
	char stTmp[commandBufferLen];
	if (!BT_connected()) return;

	if (!(btFlags & (1<<btWasConnected))) { // new connection
		// initialize and clear buffer, ignore anything there before
		uart1_init_input_buffer();
		btCmdBuffer[0] = '\0'; // "empty" the command buffer
		btCmdBufferPtr = btCmdBuffer;
		btFlags |= (1<<btWasConnected); // set flag
		return; // bail now, pick it up on next pass
	}
	
	while (uart1_char_waiting_in()) {
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
			btFlags &= ~(1<<btSerialBeingInput); // flag that input is complete; allow string output
			switch (btCmdBuffer[0]) { // command is 1st char in buffer

                case 'O': case 'o': { // experiment with oscillator control
					// go into uC clock adjust mode
					outputStringToUART1("\r\n going into uC adjust mode\r\n");
					len = sprintf(str, "Cycle count from RTC: %d\r\n", cyPerRTCSqWave());
					outputStringToUART1(str);
					len = sprintf(str, "calibration byte: %d\r\n", OSCCAL);
					outputStringToUART1(str);
					outputStringToUART1("\r\n returning to timekeeping mode\r\n");
                    break;
                }

				case 'T': case 't': { // set time
					// get info from btCmdBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					strcpy(stTmp, btCmdBuffer + 1);
					if (!isValidDateTime(stTmp)) {
						outputStringToUART1("\r\n Invalid timestamp\r\n");
						break;
					}
					if (!isValidTimezone(stTmp + 20)) {
						outputStringToUART1("\r\n Invalid hour offset\r\n");
						break;
					}
					outputStringToUART1("\r\n Time changed from ");
					strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(datetime_string, &dt_RTC);
					strcat(strJSON, datetime_string);
					outputStringToUART1(datetime_string);
					datetime_getFromUnixString(&dt_tmp, stTmp, 0);
					rtc_setTime(&dt_tmp);
					strcat(strJSON, "\",\"to\":\"");
					outputStringToUART1(" to ");
					datetime_getstring(datetime_string, &dt_tmp);
					strcat(strJSON, datetime_string);
					outputStringToUART1(datetime_string);
					strcat(strJSON, "\",\"by\":\"hand\"}}\r\n");
					outputStringToUART1("\r\n");
					stateFlags1 |= (1<<writeJSONMsg); // log JSON message on next SD card write
					stateFlags1 |= (1<<writeDataHeaders); // log data column headers on next SD card write
					rtcStatus = rtcTimeManuallySet;
					outputStringToUART1(strHdr);
					intTmp1 = rtc_setupNextAlarm(&dt_CurAlarm);
					// cache timezone offset in persistent storage on SD card
					timeZoneOffset = dt_tmp.houroffset;
					timeFlags &= ~(1<<timeZoneWritten); // flag that time zone needs to be written
					syncTimeZone(); // attempt to write the time zone; will retry later if e.g. power too low
					break;
				}
				
				case 'L': case 'l': 
				{ // experimenting with the accelerometer Leveling functions
					uint8_t rs, val;
//					outputStringToUART1("\r\n about to initialize ADXL345\r\n");
//					rs = initializeADXL345();
//					if (rs) {
//						len = sprintf(str, "\n\r initialize failed: %d\n\r", rs);
//						outputStringToUART1(str);
//						break;
//					}
//					outputStringToUART1("\r\n ADXL345 initialized\r\n");

					// bring out of low power mode
					// use 100Hz for now ; bit 4 set = reduced power, higher noise
					rs = setADXL345Register(ADXL345_REG_BW_RATE, 0x0a);
					if (rs) {
						len = sprintf(str, "\n\r could not set ADXL345_REG_BW_RATE: %d\n\r", rs);
						outputStringToUART1(str);
						break;
					}
//					for (iTmp = 1; iTmp < 6; iTmp++) { // try reading bit 7, INT_SOURCE.DATA_READY
//						rs = readADXL345Register(ADXL345_REG_INT_SOURCE, &val);
//						if (rs) {
//							len = sprintf(str, "\n\r could not read ADXL345_REG_INT_SOURCE: %d\n\r", rs);
//							outputStringToUART1(str);
//							break;
//						}
//						if (val & (1 << 7)) {
//							len = sprintf(str, "\n\r INT_SOURCE.DATA_READY set: 0x%x\n\r", val);
//						} else {
//							len = sprintf(str, "\n\r INT_SOURCE.DATA_READY clear: 0x%x\n\r", val);
//						}							
//						outputStringToUART1(str);
//					}


//					outputStringToUART1("\r\n set ADXL345_REG_BW_RATE, 0x0a \r\n");
					if (readADXL345Axes (&accelData)) {
						outputStringToUART1("\r\n could not get ADXL345 data\r\n");
						break;
					}
					// set low power bit (4) and 25Hz sampling rate, for 40uA current
					rs = setADXL345Register(ADXL345_REG_BW_RATE, 0x18);
					if (rs) {
						len = sprintf(str, "\n\r could not set ADXL345_REG_BW_RATE: %d\n\r", rs);
						outputStringToUART1(str);
						break;
					}
//				len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", (unsigned int)((int)x1 << 8 | (int)x0),
//						  (unsigned int)((int)y1 << 8 | (int)y0),  (unsigned int)((int)z1 << 8 | (int)z0));
					len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", accelData.xWholeWord,
						accelData.yWholeWord,  accelData.zWholeWord);
						outputStringToUART1(str);
					break;
				}

				case 'D': case 'd': 
				{ // output file 'D'ata (or 'D'ump)
					strcpy(stTmp, btCmdBuffer); // make a copy of the command
					BT_dataDump(stTmp); // let the general fn take it from there
//					outputStringToUART1("\r\n output file data\r\n");
//					 errSD = outputContentsOfFileForDate("2012-05-03"); // testing
//					 if (errSD) {
//						tellFileError (errSD);
//					}
					break;
				}
				// put other commands here
				default: 
				{ // if no valid command, echo back the input
					outputStringToUART1("\r\n> ");
					outputStringToUART1(btCmdBuffer);
					outputStringToUART1("\r\n");
//					startTimer1ToRunThisManySeconds(30); // keep system Roused another two minutes
					break;
				}
			} // switch (btCmdBuffer[0])
			btCmdBuffer[0] = '\0'; // "empty" the command buffer
			btCmdBufferPtr = btCmdBuffer;
		} else { // some other character
			if (btCmdBufferPtr < (btCmdBuffer + commandBufferLen - 1)) { // if there is room
				if (btCmdBufferPtr == btCmdBuffer) { // if first input
					outputStringToUART1("\r\n\r\n> "); // show the user a blank line & prompt
				}
				btFlags |= (1<<btSerialBeingInput); // flag that the user is inputting characters; blocks outputStringToUART1 fn
				*btCmdBufferPtr++ = c; // append char to the command buffer
				uart1_putchar(c); // echo the character
			}					
		} // done parsing character
	} // while characters in buffer
} // end of checkForBTCommands

void BT_dataDump(char* stOpt) {
//	char stBeginTryDate[12] = "2011-12-21"; // system default starting date, winter solstice 2011.
	char stBeginTryDate[12] = "2012-01-01"; //
	char outputOpt = 'c'; // c=output contents, f=output filedate only
	// There would be no data before this unless user mistakenly set system time earlier.
	char stTryDate[12], stEndTryDate[25];
	datetime_getstring(stEndTryDate, &dt_CurAlarm); // get current timestamp as string
	stEndTryDate[10] = '\0'; // truncate to only the date
//	outputStringToBothUARTs("\n\r current date: \"");
//	outputStringToBothUARTs(stEndTryDate);
//	outputStringToBothUARTs("\"\n\r");
	switch (stOpt[1]) {
		case '\0': { // "D" alone, most common option. Dump any days' data collected since previous dump.
			errSD = readLastDumpDateFromSDCard(stBeginTryDate); // attempt to fetch the previous dump date
			if (errSD) {
				outputStringToBothUARTs("\n\r Failed to find previous dump date. Dumping all data.\n\r\n\r");
			break;
		}
		
		case 'A': case 'a': { // all data, defaults are already set up
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
				errSD = fileExistsForDate(stBeginTryDate);
				if (errSD != sdFileOpenFail) { // file does not exist
					outputStringToBothUARTs("\n\r no data for ");
					outputStringToBothUARTs(stBeginTryDate);
					outputStringToBothUARTs(". Use \"dd\" to list dates having data.\n\r\n\r");
					return;
				}					
				strcpy(stEndTryDate, stBeginTryDate); // output one day only
			} else {
				outputStringToBothUARTs("\n\r Invalid option.\n\r\n\r");
				return;
			}
			break;
		}
		
		
	}
	
//	if (stOpt[1] == '\0') { // "D" alone, most common option. Dump any days' data collected since previous dump.
//		errSD = readLastDumpDateFromSDCard(stBeginTryDate); // attempt to fetch the previous dump date
//		if (errSD) {
//			outputStringToBothUARTs("\n\r Failed to find previous dump date. Dumping all data.\n\r\n\r");
//		} else if ((stOpt[1] == 'A') || (stOpt[1] == 'a')) {
//			; // valid, all data; but this is already the default so don't have to do anything
//		} else {
//			if (isValidDate(stOpt + 1)) {
//				strncpy(stBeginTryDate,stOpt + 1, 10);
//			} else {
//				outputStringToBothUARTs("\n\r Invalid option.\n\r\n\r");
//				return;
//			}
//		}
		
//	}
	
//	if ((stOpt[1] == '\0') || (stOpt[1] == 'A') || (stOpt[1] == 'a')) { // valid options: "A" (all), or blank
//		if (stOpt[1] == '\0') { // basic command with no parameters; default, output everything since last time
//			errSD = readLastDumpDateFromSDCard(stBeginTryDate); // attempt to fetch any later date
//		}
	if (outputOpt == 'c')
		outputStringToBothUARTs("\n\r{\"datadump\":\"begin\"}\n\r");
	else
		outputStringToBothUARTs("\n\r{\"datesHavingData\":\"begin\"}\n\r");
	strcpy(stTryDate, stBeginTryDate);
	do { // loop like this so dump will output at least one day
//			outputStringToBothUARTs("\n\r");
		errSD = fileExistsForDate(stTryDate);
		// somewhat redundant to test first because output fns test internally, but avoids output for nonexistent files
		if (!errSD) {
			if (outputOpt == 'c') { // contents
				outputStringToBothUARTs("\n\r{\"datafor\":\"");
				outputStringToBothUARTs(stTryDate);
				outputStringToBothUARTs("\"}\n\r");	
				errSD = outputContentsOfFileForDate(stTryDate);
					if (errSD) {
						tellFileError (errSD);
					}
				// most likely fail point is during day's output
				// mark success of each day, so as not to repeat on recovery
				errSD = writeLastDumpDateToSDCard(stTryDate);
				if (errSD) {
					tellFileError (errSD);
				}
			} else { // list of dates only
//				outputStringToBothUARTs("\n\r{\"datafor\":\"");
				outputStringToBothUARTs(stTryDate);
				outputStringToBothUARTs("\n\r");
			}


		} else { // tested if file exists, and some error
 //				outputStringToBothUARTs(stTryDate);
 //				outputStringToBothUARTs("  ");
			if (errSD != sdFileOpenFail) // file does not exist, ignore
				tellFileError (errSD);
		}
		strcpy(stBeginTryDate, stTryDate); // remember the date we just tried
		datetime_advanceDatestring1Day(stTryDate);
	} while (strcmp(stTryDate, stEndTryDate) <= 0);

//		outputStringToBothUARTs("\n\r");
	if (outputOpt == 'c') { // data file contents 
		outputStringToBothUARTs("\n\r{\"datadump\":\"end\"}\n\r\n\r");
		errSD = writeLastDumpDateToSDCard(stBeginTryDate);
		if (errSD) {
			tellFileError (errSD);
		}
	} else { // list of dates with data, only
		outputStringToBothUARTs("\n\r{\"datesHavingData\":\"end\"}\n\r");
	}
		
	return;
}

//	if ((stOpt[1] == 'F') || (stOpt[1] == 'f')) { // show existing files
//		strcpy(stTryDate, stBeginTryDate);
//		outputStringToBothUARTs("\n\rFiles exist for:\n\r");
//		do {
//			strcpy(stBeginTryDate, stTryDate);
//			errSD = fileExistsForDate(stTryDate);
//			// somewhat redundant to test first because output fns test internally, but avoids output for nonexistent files
//			if (!errSD) {
//				outputStringToBothUARTs(stTryDate);
//				outputStringToBothUARTs("\n\r");
//			} else { // tested if file exists, and some error
// //				outputStringToBothUARTs(stTryDate);
// //				outputStringToBothUARTs("  ");
//				if (errSD != sdFileOpenFail) // file does not exist, ignore
//					tellFileError (errSD);
//			}
//			datetime_advanceDatestring1Day(stTryDate);
//		} while (strcmp(stTryDate, stEndTryDate) <= 0);
//		outputStringToBothUARTs("\n\r{\"datadump\":\"endfilelist\"}\n\r");
//		errSD = writeLastDumpDateToSDCard(stBeginTryDate);
//		if (errSD) {
//			tellFileError (errSD);
//		}
		
//		return;
//	}

/*
	// try command as a date
	if (!isValidDate(stOpt + 1)) {
		outputStringToBothUARTs(" Not a valid date");
		return;
	}
	if (!fileExistsForDate(outputStringToBothUARTs)) {
		outputStringToBothUARTs(" No file for this date");
		return;
	}
	outputStringToBothUARTs("\n\r{\"datadump\":\"begin\"}\n\r");
	outputStringToBothUARTs("\n\r{\"datafor\":\"");
	outputStringToBothUARTs(stOpt + 1);
	outputStringToBothUARTs("\"}\n\r");
	errSD = outputContentsOfFileForDate(stOpt + 1);
	outputStringToBothUARTs("\n\r{\"datadump\":\"end\"}\n\r");
	if (errSD) {
		tellFileError (errSD);
	} //else { // if it was a valid date, and we dumped that data
	errSD = writeLastDumpDateToSDCard(stOpt + 1); // record this date
	if (errSD) {
		tellFileError (errSD);
	}			
//	}
*/
}