/*
 * GPStime.h
 *
 * Created: 4/10/2014 9:01:25 PM
 *  Author: rshory
 */ 


#ifndef GPStime_H_
#define GPStime_H_

#include "compiler.h"
#include "RTC/DS1342.h"

#define GPS_SUBSYSTEM_CTRL 1 // bit 1 of PortB, of this uC
// controls reset of the uC in the GPS subsystem

#define DAYS_FOR_MOVING_AVERAGE 16 // sets size of the array of chargeInfo's

typedef volatile struct { // used for tracking the cell voltage daily maximum, when
						// most power is available for tasks whose timing is 
						// flexible, such as reading the GPS
	uint16_t level; // ADC reading of the cell voltage
	dateTime timeStamp; // when this charge level was read

} chargeInfo ;

extern inline void GPS_idle(void);
extern void GPS_initTimeRequest(void);
void getAverageTime (dateTime* startOfArrayOfTimes, uint8_t startIndex, uint8_t endIndex, uint16_t avgMinutes);

#endif /* GPStime_H_ */