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
#include <util/twi.h>

char btCmdBuffer[commandBufferLen];
char *btCmdBufferPtr;

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
 * \brief Check UART1 (Bluetooth) for commands
 *
 * Check the UART1 receive buffer for commands.
 * UART1 communicates through the RN-42 Bluetooth module.
 * \
 *  
 */

void checkForBTCommands (void) {
	char c;
	if (!BT_connected()) return;

	if (!(btFlags & (1<<btWasConnected))) { // new connection
		// initialize and clear buffer, ignore anything there before
		uart1_init_input_buffer();
		btCmdBuffer[0] = '\0'; // "empty" the command buffer
		btCmdBufferPtr = btCmdBuffer;
		btFlags |= (1<<btWasConnected); // set flag
		return; // bail now, pick it up on next pass
	}
	
//	if (btFlags & (1<<btCmdServiced)) { // just serviced a command
//		// initialize and clear buffer, ignore anything there before
//		uart1_init_input_buffer();
//		btFlags &= ~(1<<btCmdServiced); // clear flag
//		return; // bail now, pick it up on next pass
//	}
	
//	if (!uart1_char_waiting_in()) return;
//	while (uart1_char_waiting_in()) {
//		cli();
//		c = uart1_getchar();
//		sei();
	while (1) {
		cli();
		if (!uart1_char_waiting_in()) {
			sei();
			return;
		}
		c = uart1_getchar();
		sei();

		if (c == 0x0d) // if carriage-return
			c = 0x0a; // substitute linefeed
		if (c == 0x0a) { // if linefeed, attempt to parse the command
			*btCmdBufferPtr++ = '\0'; // null terminate
			btFlags &= ~(1<<btSerialBeingInput); // flag that input is complete; allow string output
			switch (btCmdBuffer[0]) { // command is 1st char in buffer

				case 'T': case 't': { // set time
					// get info from btCmdBuffer before any UART output, 
					// because in some configurations any Tx feeds back to Rx
					char tmpStr[commandBufferLen];
					strcpy(tmpStr, btCmdBuffer + 1);
					if (!isValidDateTime(tmpStr)) {
						outputStringToUART1("\r\n Invalid timestamp\r\n");
						break;
					}
					if (!isValidTimezone(tmpStr + 20)) {
						outputStringToUART1("\r\n Invalid hour offset\r\n");
						break;
					}
					outputStringToUART1("\r\n Time changed from ");
					strcat(strJSON, "\r\n{\"timechange\":{\"from\":\"");
					intTmp1 = rtc_readTime(&dt_RTC);
					datetime_getstring(datetime_string, &dt_RTC);
					strcat(strJSON, datetime_string);
					outputStringToUART1(datetime_string);
					datetime_getFromUnixString(&dt_tmp, tmpStr, 0);
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
					outputStringToUART1("\r\n output file data\r\n");
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
			// for testing, explicitly empty the command buffer
			{
				uint8_t cnt;
				for (cnt = 0; cnt < commandBufferLen; cnt++) {
					btCmdBuffer[cnt] = '\0';
				}
			}
			uart1_init_input_buffer();
//			btFlags |= (1<<btCmdServiced); // set flag that command was serviced
//			return; //
		} else { // some other character
			// ignore repeated linefeed (or linefeed following carriage return) or carriage return
//			if (!((c == 0x0a) || (c == 0x0a))) {
				if (btCmdBufferPtr < (btCmdBuffer + commandBufferLen - 1)) { // if there is room
					if (btCmdBufferPtr == btCmdBuffer) { // if first input
						outputStringToUART1("\r\n\r\n> "); // show the user a blank line & prompt
					}
					btFlags |= (1<<btSerialBeingInput); // flag that the user is inputting characters; blocks outputStringToUART1 fn
					*btCmdBufferPtr++ = c; // append char to the command buffer
					uart1_putchar(c); // echo the character
//					// for testing, echo the whole command buffer
//					{
//						uint8_t cnt;
//						for (cnt = 0; cnt < commandBufferLen; cnt++) {
//							uart1_putchar(btCmdBuffer[cnt]);
//						}
//						while (uart1_char_queued_out())
//							;						
//					}
				}					
//			}
		} // done parsing character
	} // while characters in buffer
} // end of checkForBTCommands

