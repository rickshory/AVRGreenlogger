/*
 * greenlogger.h
 *
 * Created: 12/27/2011 1:37:14 PM
 *  Author: rshory
 */ 


#ifndef GREENLOGGER_H_
#define GREENLOGGER_H_

#define RTC_CHIP_IS_DS1337
//#define RTC_CHIP_IS_DS1342

#include "interrupt.h"
#include "mega_uart_interrupt.h"

#define commandBufferLen 30

#define CELL_VOLTAGE_THRESHOLD_SD_CARD 456 // corresponds to 1140mV, where the NiMH cell voltage just starts to droop
#define CELL_VOLTAGE_THRESHOLD_READ_DATA 427 // corresponds to 1067mV where cell voltage is just about to plummet
#define CELL_VOLTAGE_THRESHOLD_UART 404 // corresponds to 1010mV where cell voltage is just above cutoff

// temporarily set artificially high, for testing
#define IRRADIANCE_THRESHOLD_DARK_IR 500 // infrared readings below this are considered "darkness"
#define IRRADIANCE_THRESHOLD_DARK_BB 1000 // broadband readings below this are considered "darkness"

//#define IRRADIANCE_THRESHOLD_DARK_IR 50 // infrared readings below this are considered "darkness"
//#define IRRADIANCE_THRESHOLD_DARK_BB 100 // broadband readings below this are considered "darkness"


enum machStates
{
 Asleep = 0, 
 Idle,  // done with work but not yet allowed to go to sleep
 GettingTimestamp, // first step towards acquiring data
 ReadingSensors, // in the process of acquiring sensor data
 WritingData, // in the process of writing acquired data
 ServicingCommand, // servicing an input command
 TransferringData, // in the process of sending logged data
 WritingFile // in the process of saving data to SD card
};


// if an I2C attempt fails, is virtually always does so on one of these early steps
//  if these pass, the rest of the transmission is very reliable

enum errI2C
{
	I2C_OK = 0,
	errNoI2CStart, // could not initiate Start state on I2C bus
	errNoI2CAddressAck, // no acknowledgment by a device at the tested address, to WRITE
	errNoI2CDataAck, // no acknowledgment of a data write
	errNoI2CRepStart, // could not do a Repeat Start
	errNoI2CAddrAckRead, // could not address device to READ
	errBadParameters // invalid parameters passed
};

enum stateRTC
{
	rtcTimeNotSet = 0, // RTC time has not been set in any way, running from power-up default
	rtcTimeSetToDefault, // RTC date/time has been set to the default, winter solstice 2011
	rtcTimeManuallySet, // RTC date/time was manually set by command line; change noted in log
	rtcHasGPSTime // RTC has a valid date/time acquired from the GPS; the global "dt_LatestGPS"
	// stores when this was last done, to allow tracking its "freshness"
};

enum stateFlags1Bits
{
	isReadingSensors, // has been awakened by RTCC interrupt and is reading data
	isRoused, // triggered by external interrupt, and re-triggered by any activity while awake
	reRoused, // re-triggered while awake, used to reset timeout
	writeJSONMsg, // there is a JSON message to log
	writeDataHeaders, // flag to write column headers to SD card
	//  done on init, reset, time change, and midnight rollover
	tryBluetooth, // turn on the Bluetooth module and try to get a connection
	BT_was_connected, // track transitions between connected to not connected
	sfBit7 // unused
 };

enum timeFlagsBits
{
	nextAlarmSet, // next alarm has been correctly set
	timeToLogData, // flag that it is time to log data to SD card
	tfBit2, // unused
	tfBit3, // unused
	tfBit4, // unused
	tfBit5, // unused
	tfBit6, // unused
	tfBit7 // unused
};

enum irradFlagsBits 
{
	testDark, // temporary flag
	isDarkBBDn, // broadband down-looking tests as dark
	isDarkIRDn, // infrared down-looking tests as dark
	isDarkBBUp, // broadband up-looking tests as dark
	isDarkIRUp, // infrared up-looking tests as dark
	isDark, // combined tests show light level(s) below threshold, use long sampling intervals
	ifBit6, // unused
	ifBit7 // unused
};


enum motionFlagsBits 
{
	// 
	tapDetected, // accelerometer tap detected, used for diagnostics
	isLeveling, // we are getting accelerometer axis readings, used for leveling the system
	accelerometerIsThere, // set if system finds ADXL345 on I2C bus
	mfBit3, // unused
	mfBit4, // unused
	mfBit5, // unused
	mfBit6, // unused
	mfBit7 // unused
};

void outputStringToUART0 (char* St);
void outputStringToUART1 (char* St);
void checkForCommands (void);
void enableAccelInterrupt (void);
void disableAccelInterrupt(void);
void enableRTCInterrupt(void);
void disableRTCInterrupt(void);

#endif /* GREENLOGGER_H_ */