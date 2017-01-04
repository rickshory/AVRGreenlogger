/*
 * GPStime.c
 *
 * Created: 4/10/2014 8:59:10 PM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "GPStime.h"
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

#define GPS_SUBSYSTEM_CTRL 1 // bit 1 of PortB, of this uC
// controls reset of the uC in the GPS subsystem

// no longer using Venus GPS module, unavailable
// rewriting this to generic "GPStime"

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


//extern inline void GPS_initTimeRequest(void);
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
inline void GPS_initTimeRequest(void)
{
	PORTB &= ~(1<<GPS_SUBSYSTEM_CTRL); // set low
	// uC in GPS subsystem, at Vcc 3V, needs a 700ns low-going pulse for definite reset
	// each clock cycle of this uC, at 8MHz, is 125ns
	// so 6 clock cycles of this uC would be 750ns and ought to do it
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	PORTB |= (1<<GPS_SUBSYSTEM_CTRL); // set high
}


 /**
 * \brief check if the Venus GPS module is powered
 *
 * PortB, bit 1 is an output that enables the power supply to the 
 * Venus SUP005R GPS module: 
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
