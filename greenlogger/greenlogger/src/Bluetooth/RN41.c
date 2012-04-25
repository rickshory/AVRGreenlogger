/*
 * RN41.c
 *
 * Created: 4/25/2012 10:50:13 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "RN41.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

inline void BT_power_on(void)
{
	// Disable interrupts
//	cli();
	// PortD, bit 4 controls power to the RN-41 Bluetooth module
	// high = enabled
	DDRD |= (1<<4); // make output
	PORTD |= (1<<4); // set high; power on
	// Re-enable interrupts
//	sei();
}

inline void BT_power_off(void)
{
	// Disable interrupts
//	cli();
	// PortD, bit 4 controls power to the RN-41 Bluetooth module
	// high = enabled
	DDRD |= (1<<4); // make output
	PORTD &= ~(1<<4); // set low; turn off
	// Re-enable interrupts
//	sei();
}

inline void BT_baud_9600(void)
{
	// Disable interrupts
//	cli();
	// PortD, bit 7 controls the BAUD rate of the RN-41 Bluetooth module
	// high = 9600
	// low = 115k or firmware setting
	DDRD |= (1<<7); // make output
	PORTD |= (1<<7); // set high, baud 9600
	// Re-enable interrupts
//	sei();
}

inline void BT_baud_115k(void)
{
	// Disable interrupts
//	cli();
	// PortD, bit 7 controls the BAUD rate of the RN-41 Bluetooth module
	// high = 9600
	// low = 115k or firmware setting
	DDRD |= (1<<7); // make output
	PORTD &= ~(1<<7); // set low, baud 115k
	// Re-enable interrupts
//	sei();
}

