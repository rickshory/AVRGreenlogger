/*
 * RN41.h
 *
 * Created: 4/25/2012 10:49:34 AM
 *  Author: rshory
 */ 


#ifndef RN41_H_
#define RN41_H_

#include "compiler.h"

extern inline void BT_power_on(void);
extern inline void BT_power_off(void);
extern inline void BT_baud_9600(void);
extern inline void BT_baud_115k(void);

/**
 * \brief Sets pin that monitors RN-42 Bluetooth module to be an input
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 */
static inline void BT_connection_setInput(void)
{
	DDRD &= ~(1<<5); // make input
}

/**
 * \brief Function for checking if the RN-42 Bluetooth module has connected
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
static inline bool BT_connected(void)
{
	return (PORTD &= (1<<5)); // read pin
}


#endif /* RN41_H_ */