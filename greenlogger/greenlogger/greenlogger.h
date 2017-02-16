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

#define COMMAND_BUFFER_LENGTH 64

#define VERSION_STRING "\r\n Ver 4.03\t 2017-01-18\r\n "

// ADC reading * 2.5 = mV
#define CELL_VOLTAGE_GOOD_FOR_ALL_FUNCTIONS 520 // corresponds to 1300mV, sufficient for high drain functions like GPS
#define CELL_VOLTAGE_OK_FOR_GPS 490 // corresponds to 1225mV, determined empirically as OK to complete time request
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

#define GPS_TIME_REQUEST PB1 // the pin to pulse low, to initiate a GPS time request

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
	rtcTimeSetToDefault, // RTC date/time has been set to the default, winter solstice 2011
	rtcTimeRetained, // RTC found to have valid time on uC reset
	rtcTimeManuallySet, // RTC date/time was manually set by command line; change noted in log
	rtcHasGPSTime // RTC has a valid date/time acquired from the GPS; the global "dt_LatestGPS"
	// stores when this was last done, to allow tracking its "freshness"
};

typedef volatile union sBF { // status bit flags
	unsigned char sF1Val;
	struct
	{
		unsigned char isReadingSensors:1; // has been awakened by RTCC interrupt and is reading data
		unsigned char isRoused:1; // triggered by external interrupt, and re-triggered by any activity while awake
		unsigned char reRoused:1; // re-triggered while awake, used to reset timeout
		unsigned char writeJSONMsg:1; // there is a JSON message to log
		unsigned char writeDataHeaders:1; // flag to write column headers to SD card
			// done on initialization, reset, time change, and midnight rollover
		unsigned char reachedFullPower:1; // cell charging achieved high enough voltage to allow 
			// high-power modules, initializations
		unsigned char logSilently:1; // do not give any diagnostics while logging; used during processes
			// such as data dump and instrument leveling
		unsigned char sfBit7:1; // unused
	};
} sFlags1;

typedef volatile union iBF { // initialization bit flags
	unsigned char iFVal;
	struct
	{
		unsigned char initI2C:1; // I2C has been initialized
		unsigned char initUART0:1; // UART0 has been initialized
		unsigned char initUART1:1; // UART1 has been initialized
		unsigned char initAccelerometer:1; // Accelerometer has been initialized
		unsigned char gpsTimePassedAutoInit:1; // GPS-time is no longer in auto-initialization; either
			// successfully auto-set time from GPS
			// user set time from GPS
			// time retained through reset
		unsigned char sf2Bit5:1; // unused
		unsigned char sf2Bit6:1; // unused
		unsigned char sf2Bit7:1; // unused
	};
} iFlags;

typedef volatile union bBF { // Bluetooth bit flags
	unsigned char bFVal;
	struct
	{
		unsigned char btWasConnected:1; // track transitions between connected to not connected
		unsigned char btSerialBeingInput:1; // characters being input to the Bluetooth serial port, may verify as a command
		unsigned char btSerialFirstInput:1; // first input after buffer clear
		unsigned char btCmdServiced:1; // Bluetooth command has been serviced on this read cycle
		unsigned char btBit3:1; // unused
		unsigned char btBit4:1; // unused
		unsigned char btBit5:1; // unused
		unsigned char btBit6:1; // unused
		unsigned char btBit7:1; // unused
	};
} bFlags;

typedef volatile union tBF { // time bit flags
	unsigned char tFVal;
	struct
	{
		unsigned char nextAlarmSet:1; // next alarm has been correctly set
		unsigned char timeToLogData:1; // flag that it is time to log data to SD card
		unsigned char alarmDetected:1; // the RTC Alarm has caused an interrupt
		unsigned char timeZoneWritten:1; // time zone has been written to the SD card
		unsigned char timeZoneRead:1; // time zone has been read from the SD card
		unsigned char tfBit5:1; // unused
		unsigned char tfBit6:1; // unused
		unsigned char tfBit7:1; // unused
	};
} tFlags;

typedef volatile union gBF { // GPS bit flags
	unsigned char gFVal;
	struct
	{
		unsigned char checkGpsToday:1; // used with GPS alarm time to initiate a set-time request from the GPS
		unsigned char gpsTimeRequested:1; // flag that a request is out to the GPS for a set-time signal
		unsigned char gpsReqTest:1; // this is a manual test request, to distinguish from a system generated one
		unsigned char gpsTimeRequestByBluetooth:1; // request came in by the Bluetooth modem
		unsigned char gpsGotLocation:1; // there is a valid location from the GPS
		unsigned char gpsNewLocation:1; // the location is acquired but not written to SD card yet
		unsigned char gfBit6:1; // unused
		unsigned char gfBit7:1; // unused
	};
} gFlags;

typedef volatile union rBF { // irradiance bit flags
	unsigned char rFVal;
	struct
	{
		unsigned char isDarkBBDn:1; // broadband down-looking tests as dark
		unsigned char isDarkIRDn:1; // infrared down-looking tests as dark
		unsigned char isDarkBBUp:1; // broadband up-looking tests as dark
		unsigned char isDarkIRUp:1; // infrared up-looking tests as dark
		unsigned char isDark:1; // combined tests show light level(s) below threshold, use long sampling intervals
		unsigned char testDark:1; // temporary flag
		unsigned char ifBit6:1; // unused
		unsigned char ifBit7:1; // unused
	};
} rFlags;

typedef volatile union mBF { // motion bit flags
	unsigned char mFVal;
	struct
	{
		unsigned char tapDetected:1; // accelerometer tap detected, used for diagnostics
		unsigned char isLeveling:1; // we are getting accelerometer axis readings, used for leveling the system
		unsigned char accelerometerIsThere:1; // set if system finds ADXL345 on I2C bus
		unsigned char mfBit3:1; // unused
		unsigned char mfBit4:1; // unused
		unsigned char mfBit5:1; // unused
		unsigned char mfBit6:1; // unused
		unsigned char mfBit7:1; // unused
	};
} mFlags;


void tuneMainOsc(void);
void checkCriticalPower(void);
void outputStringToWiredUART (char* St);
void outputStringToBluetoothUART (char* St);
void outputStringToBothUARTs (char* St);
void checkForCommands (void);
void enableAccelInterrupt (void);
void disableAccelInterrupt(void);
void enableRTCInterrupt(void);
void disableRTCInterrupt(void);
void showCellReadings(void);
uint8_t makeLogString(void);

#endif /* GREENLOGGER_H_ */