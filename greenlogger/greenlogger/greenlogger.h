/*
 * greenlogger.h
 *
 * Created: 12/27/2011 1:37:14 PM
 *  Author: rshory
 */ 


#ifndef GREENLOGGER_H_
#define GREENLOGGER_H_
#include "interrupt.h"
#include "mega_uart_interrupt.h"

#define commandBufferLen 30

/*
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
*/

enum stateRTC
{
	rtcTimeNotSet = 0, // RTC time has not been set in any way, running from power-up default
	rtcTimeSetToDefault, // RTC date/time has been set to the default, winter solstice 2011
	rtcHasGPSTime // RTC has a valid date/time acquired from the GPS; 
};

enum stateFlags1Bits
{
 timeHasBeenSet = 0, // false when coming out of reset, true after RTCC has been set
 timerHasBeenSynchronized, // alarm interrupt happens on 0 second boundary (00, 10, 20, ...)
 isReadingSensors, // has been awakened by RTCC interrupt and is reading data
 isRoused, // triggered by external interrupt, and re-triggered by any activity while awake
 reRoused, // re-triggered while awake, used to reset timeout
 keepBT_on, // keep power on the the Bluetooth module, even after rouse timeout
 isLeveling, // diagnostics show accelerometer output rather than light sensors, used for leveling the system
 accelerometerIsThere // set if system finds ADXL345 on I2C bus
};
/*
typedef struct	{ // always assumes century is 20; year 2000 to 2099
	volatile uint8_t year; // 0 to 99
	volatile uint8_t month; // 1 to 12
	volatile uint8_t day; // 1 to 31
	volatile int8_t houroffset; // timezone difference from Universal Time (GMT) -12 to +12
	volatile uint8_t hour; // 0 to 23
	volatile uint8_t minute; // 0 to 59
	volatile uint8_t second; // 0 to 59
} DateTime ;
*/

void outputStringToUART (char* St);
void checkForCommands (void);
void enableAccelInterrupt (void);

#endif /* GREENLOGGER_H_ */