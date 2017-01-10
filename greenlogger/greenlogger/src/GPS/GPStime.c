/*
 * GPStime.c
 *
 * Created: 4/10/2014 8:59:10 PM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "../GPS/GPStime.h"
#include "../Bluetooth/RN42.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"
#include "../Accelerometer/ADXL345.h"
#include "../TemperatureSensor/TCN75A.h"
#include "../SDcard/diskio.h"
#include "../BattMonitor/ADconvert.h"
#include "../LtSensor/TSL2561.h"
#include <util/twi.h>
#include <math.h>

// generic control of GPS subsystem

/**
 * \brief assures GPS subsystem is idle
 *
 * PortB, bit 1 controls reset to the GPS subsystem,
 * brief low causes reset, and initiates a time request sequence
 * high = no reset, idle
 */
inline void GPS_idle(void)
{
	PORTB |= (1<<GPS_SUBSYSTEM_CTRL); // set high; no reset
}


/**
 * \brief send GPS time-request signal
 *
 * Send low-going reset pulse to external uC, to
 * initiate the time-request sequence in the GPS subsystem.
 * Subsystem attempts to power cycle the GPS and
 * parse a time signal from the GPS NMEA
 * data. If successful, returns a set-time command on this uC's
 * UART. This uC should stay roused for up to 3 minutes, to 
 * receive that command.
 */
void GPS_initTimeRequest(void)
{
	outputStringToBothUARTs("\r\n sending get-time request to GPS subsystem \r\n");
	stayRoused(18000); // stay awake for up to 3 minutes to receive any reply
	PORTB &= ~(1<<GPS_SUBSYSTEM_CTRL); // set low
	// uC in GPS subsystem, at Vcc 3V, needs a 700ns low-going pulse for definite reset
	// each clock cycle of this uC, at 8MHz, is 125ns
	// so 6 clock cycles of this uC would be 750ns and ought to do it
	for ( uint8_t i=6; i; i--){ 
		// loop opcodes will make the time more than double, 
		// but still brief enough to not interfere with overall
		// program flow
		asm volatile ("nop");
    }
	PORTB |= (1<<GPS_SUBSYSTEM_CTRL); // set high
}

uint16_t getAverageMinute (dateTime *startOfArrayOfTimes) {
	dateTime *curTime;
	int16_t minutesFromTime;
	double minuteRadians, sumSine = 0, sumCosine = 0;
	for (uint8_t i=0; i++; i<DAYS_FOR_MOVING_AVERAGE) {
		curTime = startOfArrayOfTimes + i;
		if (curTime->day > 0) { // day=0 flags that this is not a filled-in item
			// get minutes
			// adjust by hour offset to always treat as if Universal Time
			// range zero to 1440, can be negative after offset by time zone
			minutesFromTime = (curTime->hour - curTime->houroffset) * 60 + curTime->minute;
			// map to 2Pi radians
			double minuteRadians = 2 * M_PI * minutesFromTime / 1440;
			sumSine += sin(minuteRadians);
			sumCosine += cos(minuteRadians);
		} // don't even need to count how many
	}
	// we have summed all the valid items and are now ready to calc the average
	minuteRadians = atan2(sumSine, sumCosine); // fn output range is -pi to +pi
	if (minuteRadians < 0) minuteRadians += (2 * M_PI); // should now range 0 to 2pi
	return (uint16_t) (1440 * minuteRadians / (2 * M_PI)); // convert back to a positive minutes count
}

void chargeInfo_getString(char* ciStr, chargeInfo *cip) {
	// get charge level into string
	// chargeInfo.level is the 32 bit reading stored direct from the ADC
	// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
	int iLen;
	iLen = sprintf(ciStr, "%lumV\t", (unsigned long)(2.5 * (unsigned long)(cip->level)));
	// if not a valid date/time, will show as "2000-00-00 00:00:00 +00"
	datetime_getstring(ciStr + iLen, &(cip->timeStamp));
	strcat(ciStr, "\n");
}