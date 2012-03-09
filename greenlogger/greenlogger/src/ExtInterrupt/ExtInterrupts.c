/*
 * ExtInterrupts.c
 *
 * Created: 2/9/2012 10:43:58 AM
 *  Author: rshory
 */ 

#include <avr/io.h>
#include "compiler.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"

extern volatile uint8_t machineState;
extern volatile uint16_t rouseCountdown;
extern volatile char stateFlags1;

extern volatile dateTime dt_CurAlarm, dt_NextAlarm;

void stayRoused(int8_t sec)
{
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	rouseCountdown = 100 * sec;
	stateFlags1 |= (1<<isRoused);
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
 *  It uses Pin-Change interrupt 10 (PCINT10), bit B2
 *  on 44-TQFP package, this physical pin 42
 *  on 40-DIP package, this is physical pin 3
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
	disableAccelInterrupt();
	stayRoused(5);
}

/**
 * \brief Enables wake up by RTC
 *
 * The microcontroller goes into sleep mode between readings, to save power.
 *  This function enables waking up the microcontroller
 *  by the external Real Time Clock chip. The RTC controls a transition on a 
 *  certain pin. This function sets up the microcontroller to respond to the
 *  pin change with an interrupt, which will bring it out of sleep mode.
 *  It uses Pin-Change interrupt 3 (PCINT3), bit A3
 *  on 44-TQFP package, this physical pin 34
 *  on 40-DIP package, this is physical pin 37
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
	disableRTCInterrupt();
	machineState = GettingTimestamp;
/*
	datetime_copy(&dt_NextAlarm, &dt_CurAlarm);
	datetime_advanceIntervalShort(&dt_NextAlarm);
	if (!rtc_setAlarm1(&dt_NextAlarm)) {
		;
//		outputStringToUART("\n\r Alarm1 set \n\r\n\r");
//		datetime_getstring(str, &dt_NextAlarm);
//		outputStringToUART(str);
//		outputStringToUART("\n\r\n\r");
	}
	if (!rtc_enableAlarm1()) {
		;
//		outputStringToUART("\n\r Alarm1 enabled \n\r\n\r");
	}
	enableRTCInterrupt();
	stayRoused(3);
*/
}