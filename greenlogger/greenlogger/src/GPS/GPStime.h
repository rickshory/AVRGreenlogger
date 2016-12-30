/*
 * GPStime.h
 *
 * Created: 4/10/2014 9:01:25 PM
 *  Author: rshory
 */ 


#ifndef GPStime_H_
#define GPStime_H_

#include "compiler.h"

#define DAYS_FOR_MOVING_AVERAGE 16 // sets size of the array of chargePoints

typedef volatile struct { // used for tracking the cell voltage daily maximum, when
						// most power is available for tasks whose timing is 
						// flexible, such as reading the GPS
	uint16_t chargeLevel; // ADC reading of the cell voltage
	uint16_t minuteOfDay; // which minute of the day (max 1440) this charge level was read
	// may regret not making this a dateTime
} chargePoint ;

extern inline bool GPS_powered(void);
extern inline void GPS_power_on(void);
extern inline void GPS_power_off(void);

#endif /* GPStime_H_ */