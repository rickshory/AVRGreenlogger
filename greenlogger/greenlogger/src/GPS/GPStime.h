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

//#define DAYS_FOR_MOVING_AVERAGE 16 // sets size of the array of chargeInfo's
#define DAYS_FOR_MOVING_AVERAGE 2 // special testing version

typedef volatile struct { // location acquired from GPS
	double latVal; // value of latitude as double
	double lonVal; // value of longitude as double
	// 6 decimal places can store about 1m accuracy at the equator
	char latStr[12]; // latitude as string, e.g. -89.132435
	char lonStr[12]; // longitude as string, e.g. -179.132465
	dateTime timeStamp; // when this location was acquired
} gpsLocation;

typedef volatile struct { // used for tracking the cell voltage daily maximum, when
						// most power is available for tasks whose timing is 
						// flexible, such as reading the GPS
	uint16_t level; // ADC reading of the cell voltage
	dateTime timeStamp; // when this charge level was read

} chargeInfo;

extern inline void GPS_idle(void);
extern void GPS_initTimeRequest(void);
extern uint16_t getAverageMinute (chargeInfo* startOfArrayOfReadings);
void chargeInfo_getString(char* ciStr, chargeInfo *cip);
void saveGPSLocation(char* locStr);

#endif /* GPStime_H_ */