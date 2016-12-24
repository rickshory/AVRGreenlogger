/*
 * GPStime.h
 *
 * Created: 4/10/2014 9:01:25 PM
 *  Author: rshory
 */ 


#ifndef GPStime_H_
#define GPStime_H_

#include "compiler.h"

extern inline bool GPS_powered(void);
extern inline void GPS_power_on(void);
extern inline void GPS_power_off(void);

#endif /* GPStime_H_ */