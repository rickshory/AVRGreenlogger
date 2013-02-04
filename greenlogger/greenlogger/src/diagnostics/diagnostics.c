/*
 * diagnostics.c
 *
 * Created: 2/1/2012 2:23:10 PM
 *  Author: rshory
 *  sets up Timer3 with an interrupt that occurs every 10ms. This calls a function "heartBeat"
  which internally toggles pin A2 every 1 second. An LED there allows verifying this is working.
  The heartBeat function counts down two variables Timer1 and Timer2
  which can be used as a delay timer by any routine by using code like this:
  
	for (Timer1 = 3; Timer1; );	// Wait for 30ms
  
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "diagnostics.h"

static volatile uint8_t Timer1, Timer2;		// 100Hz decrement timers

void setupDiagnostics (void)
{
	// may not need following lines
//	PRR0 &= ~(1<<PRTIM0); // assure timer0 module power is on (clear Power Reduction bit)
//	PRR0 &= ~(1<<PRTIM1); // assure timer1 module power is on (clear Power Reduction bit)
//	PRR0 &= ~(1<<PRTIM2); // assure timer2 module power is on (clear Power Reduction bit)
	// may not need following unless bit is explicitly set elsewhere
	PRR1 &= ~(1<<PRTIM3); // assure timer3 module power is on (clear Power Reduction bit)
	
//	PRR1
    ioinit(); //Setup IO pins and defaults
	setupTimer3();
}

void ioinit (void)
{
    //1 = output, 0 = input
//    DDRA = 0b11111111; //All outputs
//	PORTA = 0xFF; // will be toggled by timer3 interrupts
//    DDRA = 0b00000001; // A0 output
//	PORTA = 0x01; // will be toggled by timer3 interrupts
    DDRA = 0b00000100; // A2 output
	PORTA = 0b00000100; // timer3 interrupts will toggle this pilot light blinkey
}

/*! Sets up Timer3 for periodic interrupt of 10ms
//! 
*/
void setupTimer3( void )
{	
	// Enables the Timer3 interrupt when OCR3A equals TCNT3
	sei();
	// TIMSK3 – Timer/Counter3 Interrupt Mask Register
	// 7 –
	// 6 –
	// 5 ICIE3, input capture enable
	// 4 –
	// 3 –
	// 2 OCIE3B, Timer/Counter3, Output Compare B Match Interrupt Enable
	// 1 OCIE3A, Timer/Counter3, Output Compare A Match Interrupt Enable
	// 0 TOIE3, Timer/Counter3, Overflow Interrupt Enable
	TIMSK3 = 0b00000010;
	
	//Set up the timer
	// TCCR3A - Timer/Counter 3 Control Register A
	// 7 COM3A1; COM3xm (x=A,B m=0,1) all = 0, don't drive any pins
	// 6 COM3A0
	// 5 COM3B1
	// 4 COM3B0
	// 3 - 
	// 2 - 
	// 1 WGM31; WGM30m (m=1,0,3,2), Timer/Counter Mode
	// 0 WGM30
	TCCR3A = 0b00000000;
	// TCCR3B - Timer/Counter 3 Control Register B
	// 7 ICNC3 = 0; don't use noise canceler
	// 6 ICES3 = 0; input capture falling edge (input capture is disabled in this design)
	// 5 - 
	// 4 WGM33 (see above)
	// 3 WGM32 = 1; enable Clear Timer on Compare (CTC)
	// 2 CS32 
	// 1 CS31
	// 0 CS30
	TCCR3B = 0b00001011;	//bits 2:0  	101 counter uses clk/1024
							//		100 counter uses clk/256
							//		011 counter uses clk/64
							//		010 counter uses clk/8
							//		001 counter uses clk/1
	// TCCR3C – Timer/Counter 3 Control Register C
	// TCCR3C = 0b00000000; // default 0, not used in this design
	
	OCR3A = CT_10MS;
	TCNT3 = 0;
	
}

//! Interrupt occurs every time TCNT3 equals OCR3A
ISR(TIMER3_COMPA_vect)
{
//	TCNT3 = 0; // not needed if CTC enabled
//	OCR3A = CT_100MS; // not needed, retains value
	heartBeat();
}
