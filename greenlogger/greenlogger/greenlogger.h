/*
 * greenlogger.h
 *
 * Created: 12/27/2011 1:37:14 PM
 *  Author: rshory
 */ 


#ifndef GREENLOGGER_H_
#define GREENLOGGER_H_

//#define RTC_CHIP_IS_DS1337
#define RTC_CHIP_IS_DS1342

#include "interrupt.h"
#include "mega_uart_interrupt.h"

#define commandBufferLen 30

#define versionString "\r\n Ver 3.00\t 2013-04-22\r\n "

// ADC reading * 2.5 = mV
#define CELL_VOLTAGE_GOOD_FOR_ALL_FUNCTIONS 520 // corresponds to 1300mV, sufficient for high drain functions like GPS
#define CELL_VOLTAGE_GOOD_FOR_STARTUP 480 // corresponds to 1200mV, sufficient for startup sequence using Bluetooth
#define CELL_VOLTAGE_THRESHOLD_SD_CARD 456 // corresponds to 1140mV, where the NiMH cell voltage just starts to droop
#define CELL_VOLTAGE_THRESHOLD_READ_DATA 427 // corresponds to 1067mV where cell voltage is just about to plummet
#define CELL_VOLTAGE_THRESHOLD_UART 404 // corresponds to 1010mV where cell voltage is just above cutoff
#define CELL_VOLTAGE_CRITICALLY_LOW 400 // corresponds to 1000mV, safely above the 800mV to 920mV hysteresis range where
// the boost regulator goes on and off

// temporarily set artificially high, for testing
//#define DEFAULT_IRRADIANCE_THRESHOLD_DARK_IR 500 // infrared readings below this are considered "darkness"
//#define DEFAULT_IRRADIANCE_THRESHOLD_DARK_BB 1000 // broadband readings below this are considered "darkness"

#define DEFAULT_IRRADIANCE_THRESHOLD_DARK_IR 50 // infrared readings below this are considered "darkness"
#define DEFAULT_IRRADIANCE_THRESHOLD_DARK_BB 100 // broadband readings below this are considered "darkness"

#define RTC_64CYCLES_FOR_MAIN_OSC_7372800HZ 28800 // target count when summing 64 full cycles of the RTC square wave
// for tuning main oscillator to 7.3728 MHz

enum machStates
{
 Asleep = 0, 
 Idle,  // done with work but not yet allowed to go to sleep
 WakedFromSleep, // first test on wake from sleep
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
	rtcTimeSetFailed, // failure to set RTC
	rtcTimeRetained, // RTC found to have valid time on uC reset
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
	reachedFullPower, // cell charging achieved high enough voltage to allow high-power modules, initializations
	sfBit6, // unused
	sfBit7 // unused
 };

enum initFlagsBits
{
	initI2C, // I2C has been initialized
	initUART0, // UART0 has been initialized
	initUART1, // UART1 has been initialized
	initAccelerometer, // Accelerometer has been initialized
	sf2Bit4, // unused
	sf2Bit5, // unused
	sf2Bit6, // unused
	sf2Bit7 // unused
 };


enum btFlagsBits
{
	btWasConnected, // track transitions between connected to not connected
	btSerialBeingInput, // characters being input to the Bluetooth serial port, may verify as a command
	btSerialFirstInput, // first input after buffer clear
	btCmdServiced, // Bluetooth command has been serviced on this read cycle
	btBit3, // unused
	btBit4, // unused
	btBit5, // unused
	btBit6, // unused
	btBit7 // unused
	};

enum timeFlagsBits
{
	nextAlarmSet, // next alarm has been correctly set
	timeToLogData, // flag that it is time to log data to SD card
	alarmDetected, // the RTC Alarm has caused an interrupt
	timeZoneWritten, // time zone has been written to the SD card
	timeZoneRead, // time zone has been read from the SD card
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

void tuneMainOsc(void);
void checkCriticalPower(void);
void outputStringToUART0 (char* St);
void outputStringToUART1 (char* St);
void outputStringToBothUARTs (char* St);
void checkForCommands (void);
void enableAccelInterrupt (void);
void disableAccelInterrupt(void);
void enableRTCInterrupt(void);
void disableRTCInterrupt(void);

#endif /* GREENLOGGER_H_ */