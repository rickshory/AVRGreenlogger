/*
 * RN41.c
 *
 * Created: 4/25/2012 10:50:13 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "RN42.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

/**
 * \brief turns on power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
inline void BT_power_on(void)
{
	PORTD |= (1<<4); // set high; power on
}

/**
 * \brief turns off power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
inline void BT_power_off(void)
{
	PORTD &= ~(1<<4); // set low; turn off
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 9600
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
inline void BT_baud_9600(void)
{
	PORTD |= (1<<7); // set high, baud 9600
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 115k
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
inline void BT_baud_115k(void)
{
	PORTD &= ~(1<<7); // set low, baud 115k
}

/**
 * \brief Sets pin that monitors RN-42 Bluetooth module to be an input
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 */
inline void BT_connection_setInput(void)
{
	DDRD &= ~(1<<5); // make input
}

/**
 * \brief check if the RN-42 Bluetooth module has a live connection
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
inline bool BT_connected(void)
{
	return (PIND & (1<<5)); // read pin
}
