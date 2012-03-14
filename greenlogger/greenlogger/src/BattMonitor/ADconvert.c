/*
 * ADconvert.c
 *
 * Created: 2/25/2012 9:10:58 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "compiler.h"
#include "ADconvert.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>
#include <avr/interrupt.h>

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];
extern uint8_t Timer1, Timer2;

// global
adcData cellVoltageReading;

// steps to do an Analog-to-Digital conversion
/*

connect internal 2.56V reference voltage to the AREF pin by writing to the REFSn bits in the ADMUX Register
decouple the internal voltage reference by an external capacitor at the AREF pin to improve noise immunity
on 40-DIP = pin 32
on 44-TQFP = pin 29

Set the ADPS bits in ADCSRA to choose ADC prescaling, to provide an input clock frequency between
 50kHz and 200kHz to the successive approximation circuitry.

When ADATE or ADEN is cleared ADMUX can be safely updated.
select the analog input channel by writing to the MUX bits in ADMUX

enable ADC by setting the ADC Enable bit, ADEN in ADCSRA
clear the conversion complete interrupt flag

Select trigger source by setting the ADC Trigger Select bits, ADTS in ADCSRB (see description of the ADTS
bits for a list of the trigger sources).
Enable Auto Triggering by setting the ADC Auto Trigger Enable bit, ADATE in ADCSRA.

If Auto Triggering is enabled, single conversions can be started by writing ADSC in ADCSRA to
one. ADSC can also be used to determine if a conversion is in progress. The ADSC bit will be
read as one during a conversion, independently of how the conversion was started.

enable ADC
do not initiate conversion yet
select Single Conversion Mode
enable ADC conversion complete interrupt
enter ADC Noise Reduction mode (or Idle mode)
The ADC will start a conversion once the CPU has been halted.

the ADC interrupt will wake up the CPU and execute the ADC Conversion Complete interrupt routine.
 in ISR:
read ADCL first, then ADCH, to ensure that the content of the Data Registers belongs to the same conversion

Vin = (ADC * Vref) / 1024

before entering other sleep modes than Idle mode and ADC Noise Reduction mode,
write zero to ADEN to avoid excessive power consumption

 REFS0
*/

uint8_t readCellVoltage (adcData *cellV) {
	
	// set initial conditions; conversion-complete interrupt will fill in these values
	cellV->adcWholeWord = 0;
	cellV->adcMultiplier = 0; // currently, flags that we have no conversion yet
	
	// Set prescale by ADPS bits in ADCSRA.
	// using 8MHz CPU clock:
	//	desired ADC clock	calculated division factor
	// min	  50kHz				160
	// max	 200kHz				 40
	// so, a division factor of either 64 (125kHz) or 128 (62.5kHz) would be OK
	// using 128
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
	//
	// The prescaler starts counting from the moment the ADC is switched on by setting the ADEN bit

	// REFS1	REFS0	Voltage Reference Selection
	//  0		 0		 AREF, Internal Vref turned off
	//	0		 1		 AVCC with external capacitor at AREF pin
	//	1		 0		 Internal 1.1V Voltage Reference with external capacitor at AREF pin
	//	1		 1		 Internal 2.56V Voltage Reference with external capacitor at AREF pin
	// for testing, use AVCC as the reference voltage
//	ADMUX |= (1<<REFS0);
	//use 2.56v internal ref
	ADMUX |= (1<<REFS1) | (1<<REFS0);
	
//	ADMUX = 0b00000001 | (1<<REFS1) | (1<<REFS0); // ADC1 single-ended; use 2.56v internal ref
	// use ADC1, which is port A bit 1. On 40-dip = physical pin 39; on 44-TQFP = physical pin 36
	ADMUX |= 0b00000001;
	
//	ADMUX |= (1<<ADLAR); // for testing, Left-adjust the result, only use high 8 bits
	
	
	// DIDR0 – Digital Input Disable Register 0
	DIDR0 |= (1<<ADC1D); // disable digital input buffer on this pin to save power

	// enable ADC conversion complete interrupt
	ADCSRA |= (1<<ADIE);
	// clear interrupt flag
	ADCSRA |= (1<<ADIF); // writing a 1 clears this flag
	// global enable interrupts
	sei();
	// enable ADC
	ADCSRA |= (1<<ADEN);	
	// initiate a single conversion
	ADCSRA |= (1<<ADSC);
	// enter CPU Idle mode

// SE bit in SMCR must be written to logic one and a SLEEP
//  instruction must be executed. The SM2, SM1, and SM0

// When the SM2..0 bits are written to 001, the SLEEP instruction makes the MCU enter ADC
// Noise Reduction mode
	
	// SM2 = bit 3
	// SM1 = bit 2
	// SM0 = bit 1
	// SE = bit 0
	// don't set SE yet
	SMCR = 0b00000010;
	// set SE (sleep enable)
	SMCR |= (1<<SE);
	// go into Noise Reduction mode SLEEP
	asm("sleep");
	// after wakeup, clear SE to prevent SLEEP by accident
	SMCR &= ~(1<<SE);
	
	while (ADCSRA & (1<<ADSC))
		; // should break out of this when conversion is complete, regardless of Idle mode
		
	
	
	
/*
Auto Triggering is
enabled by setting the ADC Auto Trigger Enable bit, ADATE in ADCSRA. The trigger source is
selected by setting the ADC Trigger Select bits, ADTS in ADCSRB (see description of the ADTS
bits for a list of the trigger sources). When a positive edge occurs on the selected trigger signal,
the ADC prescaler is reset and a conversion is started. This provides a method of starting conversions
at fixed intervals. If the trigger signal still is set when the conversion completes, a new
conversion will not be started. If another positive edge occurs on the trigger signal during conversion,
the edge will be ignored. Note that an Interrupt Flag will be set even if the specific
interrupt is disabled or the global interrupt enable bit in SREG is cleared. A conversion can thus
be triggered without causing an interrupt. However, the Interrupt Flag must be cleared in order to
trigger a new conversion at the next interrupt event.
*/
	cellV->adcLoByte = ADCL;
	cellV->adcHiByte = ADCH;
	cellV->adcMultiplier = 1; // currently, flags a completed conversion
	
	// disable ADC to save power in sleep modes
	ADCSRA &= ~(1<<ADEN);
	
	return cellV->adcHiByte; // for testing
}

//! Interrupt occurs when ADC conversion is complete
ISR(ADC_vect)
{
	// disable ADC conversion complete interrupt
	ADCSRA &= ~(1<<ADIE);
}

// alternative:
// EMPTY_INTERRUPT(ADC_vect);