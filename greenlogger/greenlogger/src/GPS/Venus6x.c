/*
 * CProgram1.c
 *
 * Created: 4/10/2014 8:59:10 PM
 *  Author: rshory
 */ 

#include <inttypes.h>
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
 * \brief turns on power to the Venus GPS module
 *
 * PortB, bit 1 controls power to the Venus SUP005R GPS module,
 * high = enabled,
 */
inline void GPS_power_on(void)
{
	PORTB |= (1<<1); // set high; power on
}

/**
 * \brief turns off power to the Venus GPS module
 *
 * PortB, bit 1 controls power to the Venus SUP005R GPS module,
 * high = enabled,
 */
inline void GPS_power_off(void)
{
	PORTB &= ~(1<<1); // set low; turn off
}
