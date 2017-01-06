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
#include "../BattMonitor/ADconvert.h"
#include "../LtSensor/TSL2561.h"
#include "../GPS/GPStime.h"
#include <util/twi.h>

char btCmdBuffer[commandBufferLen];
char *btCmdBufferPtr;
uint8_t errSD;

extern volatile uint8_t machineState;
extern volatile uint16_t timer3val;
extern char commandBuffer[commandBufferLen];
extern char *commandBufferPtr;
extern char str[128]; // generic space for strings to be output
extern char strLog[64];
extern char strJSON[256]; // string for JSON data
extern volatile uint8_t Timer1, Timer2, intTmp1;
extern volatile dateTime dt_RTC, dt_CurAlarm, dt_tmp, dt_LatestGPS; //, dt_NextAlarm
extern char datetime_string[25];
extern volatile sFlags1 stateFlags1;
extern volatile uint8_t stateFlags2, timeFlags, irradFlags, motionFlags, btFlags;
extern volatile uint8_t rtcStatus;
extern char strHdr[64];
extern int len, err;
extern volatile accelAxisData accelData;
extern volatile int8_t timeZoneOffset;
extern unsigned long darkCutoffIR, darkCutOffBB;
extern volatile adcData cellVoltageReading;
extern irrData irrReadings[4];

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
	uint8_t i, sreg;
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

				 case 'G': case 'g': { // get time from GPS
					 // for testing, manually initiate a get-time request from GPS
					 GPS_initTimeRequest();
					 break;
				 }					 
				
				 case 'V': case 'v': { // show firmware version
					 outputStringToBothUARTs(versionString);
					 break;
				 }

/*
                case 'O': case 'o': { // experiment with oscillator control
					uint16_t cyCt, cyCtNxtUp;
//					uint16_t ct0a, ct0b, ct0c, cta[100], ctb[100], ctc[100];
//					uint8_t i, os0, os[100];
					// go into uC clock adjust mode

					outputStringToUART1("\r\n going into uC adjust mode\r\n");
					len = sprintf(str, "baud register UBBR1: %d\r\n", UBRR1);
					outputStringToUART1(str);
					
					ct0a = cyPerRTCSqWave(); // get starting cycle count, 3 samples
					ct0b = cyPerRTCSqWave(); 
					ct0c = cyPerRTCSqWave();
					os0 = OSCCAL; // remember OSCCAL
					
				//	OSCCAL = 0x7F; // set OSCCAL to high end of lower range
					for (i=0; i<100; i++) {
						OSCCAL-=1; // adjust down
						os[i] = OSCCAL;
						cta[i] = cyPerRTCSqWave(); // take three readings
						ctb[i] = cyPerRTCSqWave();
						ctc[i] = cyPerRTCSqWave();
					}
					
					OSCCAL = os0; // restore
					
					len = sprintf(str, "original OSCCAL\t%d\t cycle counts\t%lu\t%lu\t%lu\r\n", os0, (unsigned long)ct0a, (unsigned long)ct0b, (unsigned long)ct0c);
					outputStringToUART1(str);
					
					for (i=0; i<100; i++) {
						len = sprintf(str, "OSCCAL set to\t%d\t cycle counts\t%lu\t%lu\t%lu\r\n", os[i], (unsigned long)cta[i], (unsigned long)ctb[i], (unsigned long)ctc[i]);
						outputStringToUART1(str);
					}

					// try tuning uC osc down to 7.3728 MHz
					OSCCAL = 0x7F; // set OSCCAL (oscillator calibration byte) to high end of lower range
					cyCt = cyPerRTCSqWave();
					do  { 
						cyCtNxtUp = cyCt;
						OSCCAL--;
						cyCt = cyPerRTCSqWave();
					} while ((unsigned long)cyCt > RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ);
					// we are just below the ideal number, if the next higher count was closer ...					
					if ((unsigned long)(RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ - (unsigned long)cyCt) > (unsigned long)((unsigned long)cyCtNxtUp - RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ)) {
						OSCCAL++; // ... tweak OSCCAL up one
					}
//					UBRR1 = 47; // sets 9600 1x baud when osc=7.3728 MHz
					UBRR1 = 95; // sets 9600 2x baud when osc=7.3728 MHz
					UBRR0 = 95; // sets 9600 2x baud when osc=7.3728 MHz

//					UBRR1 = 3; // sets 115200 baud when osc=7.3728 MHz					
					
					// see if we still get any sense out of the uart
					len = sprintf(str, "OSCCAL\t%d\tCT_below\t%lu\tCT_above\t%lu\r\n", OSCCAL, (unsigned long)cyCt, (unsigned long)cyCtNxtUp);
					outputStringToUART1(str);

					outputStringToUART1("\r\n returning to timekeeping mode\r\n");
                    break;
                }
*/

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
					stateFlags1.writeJSONMsg = 1; // log JSON message on next SD card write
					stateFlags1.writeDataHeaders = 1; // log data column headers on next SD card write
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
	uint8_t errBTDump;
//	char stBeginTryDate[12] = "2011-12-21"; // system default starting date, winter solstice 2011.
	char stBeginTryDate[12] = "2012-01-01"; //
	// There would be no data before this unless user mistakenly set system time earlier.
	char outputOpt = 'c'; // c=output contents, f=output filedate only
	char stTryDate[12], stEndTryDate[25];
	datetime_getstring(stEndTryDate, &dt_CurAlarm); // get current timestamp as string
	stEndTryDate[10] = '\0'; // truncate to only the date
//	outputStringToUART1("\n\r current date: \"");
//	outputStringToUART1(stEndTryDate);
//	outputStringToUART1("\"\n\r");
	// find first date that has data
	errBTDump = datetime_nextDateWithData(stBeginTryDate, 0);
	if (errBTDump == sdPowerTooLowForSDCard) {
		outputStringToUART1("\n\r Power too low for SD card.\n\r\n\r");
		return;
	}
	switch (stOpt[1]) {
		case '\0': { // "D" alone, most common option. Dump any days' data collected since previous dump.
			errBTDump = readLastDumpDateFromSDCard(stBeginTryDate); // attempt to fetch the previous dump date
			if (errBTDump) {
				outputStringToUART1("\n\r Failed to find previous dump date. Dumping all data.\n\r\n\r");
			}
			
			outputStringToUART1("\n\r lastDump date read from SD card: ");
			outputStringToUART1(stBeginTryDate);
			outputStringToUART1("\n\r");
		
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
					outputStringToUART1("\n\r no data for ");
					outputStringToUART1(stBeginTryDate);
					outputStringToUART1(". Use \"dd\" to list dates having data.\n\r\n\r");
					return;
				}					
				strcpy(stEndTryDate, stBeginTryDate); // output one day only
			} else {
				outputStringToUART1("\n\r Invalid option.\n\r\n\r");
				return;
			}
			break;
		}

	} // end switch (stOpt[1])

	if (outputOpt == 'c')
		outputStringToUART1("\n\r{\"datadump\":\"begin\"}\n\r");
	else
		outputStringToUART1("\n\r{\"datesHavingData\":\"begin\"}\n\r");
	strcpy(stTryDate, stBeginTryDate);
	do { // loop like this so dump will output at least one day
		while (timeFlags & (1<<alarmDetected)) { // continue logging during data dump
			uint8_t ct, swDnUp, swBbIr;
			// use 'while' loop to allow various tests to break out
			timeFlags &= ~(1<<alarmDetected); // clear flag so 'while' loop will only happen once in any case
			timeFlags &= ~(1<<nextAlarmSet); // flag that current alarm is no longer valid
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
			
			// attempt to assure time zone is synchronized
			syncTimeZone(); // internally tests if work is already done
			
			datetime_getstring(datetime_string, &dt_CurAlarm);

			// read irradiance sensors
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

			} // end of irradiance sensor 'for' loop
			intTmp1 = temperature_GetReading(&temperatureReading);
			// build log string
			strcpy(strLog, "\n\r");
			strcat(strLog, datetime_string);
			irradFlags &= ~((1<<isDarkBBDn) | (1<<isDarkIRDn) | (1<<isDarkBBUp) | (1<<isDarkIRUp)); // default clear
			for (ct = 0; ct < 4; ct++) { // generate irradiance readings
				if (!(irrReadings[ct].validation)) {
					len = sprintf(str, "\t%lu", (unsigned long)((unsigned long)irrReadings[ct].irrWholeWord * (unsigned long)irrReadings[ct].irrMultiplier));
				} else { // no valid data for this reading
					len = sprintf(str, "\t");
				}
				strcat(strLog, str);
			} // end of prearing irradiance section

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
			} 
		
		} // end of data acquisition loop
		
		// set next alarm
		if (!(timeFlags & (1<<nextAlarmSet))) {
			intTmp1 = rtc_setupNextAlarm(&dt_CurAlarm);
			timeFlags |= (1<<nextAlarmSet);
		}
		
//			outputStringToUART1("\n\r");
		errBTDump = fileExistsForDate(stTryDate);
		// somewhat redundant to test first because output fns test internally, but avoids output for nonexistent files
		if (!errBTDump) {
			if (outputOpt == 'c') { // contents
				keepBluetoothPowered(120); // or eventually will time out
				outputStringToUART1("\n\r{\"datafor\":\"");
				outputStringToUART1(stTryDate);
				outputStringToUART1("\"}\n\r");	
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
//				outputStringToUART1("\n\r{\"datafor\":\"");
				outputStringToUART1(stTryDate);
				outputStringToUART1("\n\r");
			}


		} else { // tested if file exists, and some error
			if (errBTDump != sdFileOpenFail) // file does not exist, ignore
				tellFileError (errBTDump);
		}
		strcpy(stBeginTryDate, stTryDate); // remember the date we just tried
		datetime_advanceDatestring1Day(stTryDate);
	} while (strcmp(stTryDate, stEndTryDate) <= 0);

//		outputStringToUART1("\n\r");
	if (outputOpt == 'c') { // data file contents 
		outputStringToUART1("\n\r{\"datadump\":\"end\"}\n\r\n\r");
		outputStringToUART1("\n\r");
		errBTDump = writeLastDumpDateToSDCard(stBeginTryDate);
		if (errBTDump) {
			tellFileError (errBTDump);
		}
	} else { // list of dates with data, only
		outputStringToUART1("\n\r{\"datesHavingData\":\"end\"}\n\r");
	}
		
	return;
}


