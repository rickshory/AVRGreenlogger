/*
 * ExtInterrupts.c
 *
 * Created: 2/9/2012 10:43:58 AM
 *  Author: rshory
 */ 

#include <avr/io.h>
#include "compiler.h"
#include "ExtInterrupts.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"
#include "../Bluetooth/RN42.h"

extern volatile uint8_t machineState;
extern volatile uint16_t timer3val;
extern volatile uint16_t rouseCountdown;
extern volatile uint16_t btCountdown;
extern volatile sFlags1 stateFlags1;
extern volatile bFlags btFlags;
extern volatile tFlags timeFlags;
extern volatile gFlags gpsFlags;
extern volatile mFlags motionFlags;

extern volatile dateTime dt_CurAlarm, dt_NextAlarm;

void stayRoused(uint16_t dSec)
{
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	if ((dSec) > rouseCountdown) { // never trim the rouse interval, only extend it
		rouseCountdown = dSec;
	}
	stateFlags1.isRoused = 1;
	PORTA |= (0b00000100); // set pilot light on
	sei();
}

void endRouse(void) {
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	rouseCountdown = 0;
	stateFlags1.isRoused = 0;
	gpsFlags.gpsTimeRequested = 0; // set-time request from GPS times out if system goes out of Roused mode
	gpsFlags.gpsReqTest; // any test ends
	PORTA &= ~(0b00000100); // force pilot light off
	sei();
	
}

void keepBluetoothPowered(uint8_t sec)
{
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	if ((100 * sec) > btCountdown) { // never trim the interval, only extend it
		btCountdown = 100 * sec;
	}
//	BT_power_on();
	PORTD |= (1<<4); // Bluetooth power on
	sei();
}

void shutDownBluetooth(void) {
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	btCountdown = 0;
//	BT_power_on();
	PORTD &= ~(1<<4); // set low; turn Bluetooth power off
	btFlags.btWasConnected = 0; // clear the flag, so will not re-try
	sei();
}

/**
 * \brief Enables wake up by accelerometer
 *
 * The microcontroller goes into sleep mode between readings, to save power.
 *  This function enables waking up the microcontroller
 *  by the accelerometer. The accelerometer controls a transition on a 
 *  certain pin. This function sets up the microcontroller to respond to the
 *  pin change with an interrupt, which will bring it out of sleep mode.
 *  It uses Pin-Change interrupt 10 (PCINT10), bit B2.
 *  On 44-TQFP package, this physical pin 42
 *  On 40-DIP package, this is physical pin 3
 */

void enableAccelInterrupt(void)
{
	DDRB &= ~(1<<2); // make the pin an input, clear the bit
	PORTB |= (1<<2); // enable internal pull-up, write a 1 to the data register
	// each pin change interrupt responds to a set of up to 8 pins, with the unwanted pins masked 
	PCMSK1 |= (1<<PCINT10); // unmask bit 2, PCINT10
	PCIFR |= (1<<PCIF1); // explicitly clear the interrupt flag, in case a spurious change occurred; writing a 1 clears
	// Pin Change Interrupt 1 triggers on a change on any of PCINT8 to PCINT15 pins (if unmasked)
	PCICR |= (1<<PCIE1); // enable Pin Change Interrupt 1
}

void disableAccelInterrupt(void)
{
	PCICR &= ~(1<<PCIE1); // disable Pin Change Interrupt 1, PCINT15:8		
}

//! Interrupt occurs on every pin change of PCINT10 pin, B2
// is triggered by accelerometer
ISR(PCINT1_vect)
{
	motionFlags.tapDetected = 1; // flag it
	disableAccelInterrupt();
	machineState = WakedFromSleep;
}

/**
 * \brief Enables wake up by RTC
 *
 * The microcontroller goes into sleep mode between readings, to save power.
 *  This function enables waking up the microcontroller
 *  by the external Real Time Clock chip. The RTC controls a transition on a 
 *  certain pin. This function sets up the microcontroller to respond to the
 *  pin change with an interrupt, which will bring it out of sleep mode.
 *  It uses Pin-Change interrupt 3 (PCINT3), bit A3.
 *  On 44-TQFP package, this physical pin 34
 *  On 40-DIP package, this is physical pin 37
 */

void enableRTCInterrupt(void)
{
	DDRA &= ~(1<<3); // make the pin an input, clear the bit
#ifdef RTC_CHIP_IS_DS1337
	PORTA |= (1<<3); // enable internal pull-up, write a 1 to the data register
#endif
#ifdef RTC_CHIP_IS_DS1342
	PORTA &= ~(1<<3); // RTC output is CMOS push/pull, don't need internal pull-up, write a 0 to the data register
#endif
	// each pin change interrupt responds to a set of up to 8 pins, with the unwanted pins masked 
	PCMSK0 |= (1<<PCINT3); // unmask bit 3, PCINT3
	PCIFR |= (1<<PCIF0); // explicitly clear the interrupt flag, in case a spurious change occurred; writing a 1 clears
	// Pin Change Interrupt 0 triggers on a change on any of PCINT0 to PCINT7 pins (if unmasked)
	PCICR |= (1<<PCIE0); // enable Pin Change Interrupt 0
}

void disableRTCInterrupt(void)
{
	PCICR &= ~(1<<PCIE0); // disable Pin Change Interrupt 0, PCINT7:0		
}

//! Interrupt occurs on every pin change of PCINT3 pin, A3
// is triggered by Real Time Clock
ISR(PCINT0_vect)
{
//	timer3val = TCNT3;
	timeFlags.alarmDetected = 1; // flag it
	disableRTCInterrupt();
	machineState = WakedFromSleep;
}