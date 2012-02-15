/*
 * ExtInterrupts.c
 *
 * Created: 2/9/2012 10:43:58 AM
 *  Author: rshory
 */ 

#include <avr/io.h>
#include "compiler.h"
#include "../greenlogger.h"

extern volatile uint16_t rouseCountdown;
extern volatile char stateFlags1;

void stayRoused(int8_t sec)
{
	cli(); // temporarily disable interrupts to prevent Timer3 from
		// changing the count partway through
	rouseCountdown = 100 * sec;
	stateFlags1 |= (1<<isRoused);
	sei();
}


/*
 To wake up from accelerometer interrupt,
 will use Pin-Change interrupt 10 (PCINT10), bit B2
 on 44-TQFP package, this physical pin 42
 on 40-DIP package, this is physical pin 3
*/



void enableAccelInterrupt(void)
{
	DDRB &= ~(1<<2); // make the pin an input, clear the bit
	PORTB |= (1<<2); // enable internal pull-up, write a 1 to the data register
	PCMSK1 |= (1<<PCINT10); // unmask bit 2, PCINT10
	PCIFR |= (1<<PCIF1); // explicitly clear the interrupt flag, in case a spurious change occurred; writing a 1 clears
	PCICR |= (1<<PCIE1); // enable Pin Change Interrupt 1, PCINT15:8	
}

void disableAccelInterrupt(void)
{
	PCICR &= ~(1<<PCIE1); // disable Pin Change Interrupt 1, PCINT15:8		
}

//! Interrupt occurs on every pin change of PCINT10 pin, B2
ISR(PCINT1_vect)
{
	disableAccelInterrupt();
	stayRoused(5);
}

