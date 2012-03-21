/*
 * greenlogger.h
 *
 * Created: 12/27/2011 1:37:14 PM
 *  Author: rshory
 */ 


#ifndef GREENLOGGER_H_
#define GREENLOGGER_H_

#define RTC_CHIP_IS_DS1337
// #define RTC_CHIP_IS_DS1342

#include "interrupt.h"
#include "mega_uart_interrupt.h"

#define commandBufferLen 30


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
	errNoI2CAddressAck, // no acknowledgment by a device at the tested address
	errNoI2CDataAck, // no acknowledgment of a data write
	errNoI2CRepStart, // could not do a Repeat Start
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
 timeToLogData, // flag that it is time to log data to SD card
 writeDataHeaders, // flag to write column headers to SD card
	//  done on init, reset, time change, and midnight rollover
 keepBT_on, // keep power on the the Bluetooth module, even after rouse timeout
 isLeveling, // diagnostics show accelerometer output rather than light sensors, used for leveling the system
 accelerometerIsThere // set if system finds ADXL345 on I2C bus
};

enum stateFlags2Bits 
{
	nextAlarmSet, // next alarm has been correctly set
	sFbit1, // not used yet
	sFbit2, // not used yet
	sFbit3, // not used yet
	sFbit4, // not used yet
	sFbit5, // not used yet
	sFbit6, // not used yet
	sFbit7 // not used yet
};


void outputStringToUART (char* St);
void checkForCommands (void);
void enableAccelInterrupt (void);
void disableAccelInterrupt(void);
void enableRTCInterrupt(void);
void disableRTCInterrupt(void);

#endif /* GREENLOGGER_H_ */