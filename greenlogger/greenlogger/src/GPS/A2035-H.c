/*
 * A2035_H.c
 *
 * Created: 7/23/2013 11:12:49 AM
 *  Author: rshory
 */ 
#include <inttypes.h>
#include "A2035-H.h"
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

/**
 * \brief turns on power to the A2035-H GPS module
 *
 * PortB, bit 1 controls power to the A2035-H GPS module,
 * high = enabled,
 */
inline void GPS_power_on(void)
{
	PORTB |= (1<<1); // set high; power on
}

/**
 * \brief turns off power to the A2035-H GPS module
 *
 * PortB, bit 1 controls power to the A2035-H GPS module,
 * high = enabled,
 */
inline void GPS_power_off(void)
{
	PORTB &= ~(1<<1); // set low; turn off
}

/**
 * \brief check if the A2035-H GPS module is powered
 *
 * PortB, bit 1 is an output that enables the power supply to the 
 * A2035-H GPS module: 
 * HIGH when enabled, LOW when shut down.
 * This functions reads that state of that pin.
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
inline bool GPS_powered(void)
{
	return (PINB & (1<<1)); // read pin
}


/**
 * \brief sets the on/off line to the A2035-H GPS module HIGH
 *
 * PortB, bit 0 is the A2035-H GPS module on/off line,
 * drive high for ~200ms to turn on/off
 */
inline void GPS_On_Off_High(void)
{
	PORTB |= (1<<0); // set high
}

/**
 * \brief sets the on/off line to the A2035-H GPS module LOW
 *
 * PortB, bit 0 is the A2035-H GPS module on/off line,
 * drive high for ~200ms to turn on/off
 */
inline void GPS_On_Off_Low(void)
{
	PORTB &= ~(1<<0); // set low
}
