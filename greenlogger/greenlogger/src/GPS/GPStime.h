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

#define DAYS_FOR_MOVING_AVERAGE 16 // sets size of the array of chargeInfo's

typedef volatile struct { // used for tracking the cell voltage daily maximum, when
						// most power is available for tasks whose timing is 
						// flexible, such as reading the GPS
	uint16_t level; // ADC reading of the cell voltage
	dateTime timeStamp; // when this charge level was read

} chargeInfo ;

extern inline void GPS_idle(void);
extern inline void GPS_initTimeRequest(void);
extern inline bool GPS_powered(void);
extern inline void GPS_power_on(void);
extern inline void GPS_power_off(void);

#endif /* GPStime_H_ */