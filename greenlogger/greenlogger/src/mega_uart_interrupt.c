/**
 * \file
 *
 * \brief based on megaAVR STK600 UART interrupt example
 *
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
 *
 * Buffer leakage bug fixed by Rick Shory 2012-05-01
 */
/**
 * \mainpage
  * \section intro Introduction
 * This example demonstrates how to use the megaAVR UART with interrupts.
 *
 * \section files Files:
 * - mega_uart_interrupt_example.c: megaAVR UART interrupt example
 *
 * \section compinfo Compilation Info
 * This software was written for the <A href="http://gcc.gnu.org/">GNU GCC</A>
 * for AVR. \n
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 * Support and FAQ: http://support.atmel.no/
 */

 // _ASSERT_ENABLE_ is used for enabling assert, typical for debug purposes
// #define _ASSERT_ENABLE_
#include <string.h>
#include "compiler.h"
#include "mega_uart_interrupt.h"

/**
 * \def UART0_BUFFER_SIZE
 * \brief The size of the UART buffers
 */
#define UART0_BUFFER_SIZE 128
#define UART1_BUFFER_SIZE 128

// set the correct BAUD and F_CPU defines before including setbaud.h
#include "conf_clock.h"
#include "conf_uart.h"

/**
 * \name avr_libc_inc avr libc include files
 * @{
 */
#include <util/setbaud.h>
#include <avr/interrupt.h>
//! @}

#include "ring_buffer.h"

// buffers for use with the ring buffer
uint8_t uart0_out_buffer[UART0_BUFFER_SIZE];
uint8_t uart0_in_buffer[UART0_BUFFER_SIZE];
//! ring buffer to use for UART0 transmission
struct ring_buffer uart0_ring_buffer_out;
//! ring buffer to use for UART0 reception
struct ring_buffer uart0_ring_buffer_in;
uint8_t uart1_out_buffer[UART1_BUFFER_SIZE];
uint8_t uart1_in_buffer[UART1_BUFFER_SIZE];
//! ring buffer to use for UART1 transmission
struct ring_buffer uart1_ring_buffer_out;
//! ring buffer to use for UART1 reception
struct ring_buffer uart1_ring_buffer_in;

/**
 * \brief UART0 data register empty interrupt handler
 *
 * This handler is called each time UART0 data register is available for
 * sending data.
 */
ISR(UART0_DATA_EMPTY_IRQ)
{
	// if there is data in the ring buffer, fetch it and send it
	if (!ring_buffer_is_empty(&uart0_ring_buffer_out)) {
		UDR0 = ring_buffer_get(&uart0_ring_buffer_out);
	}
	else {
		// no more data to send, turn off data ready interrupt
		UCSR0B &= ~(1 << UDRIE0);
	}
}

/**
 * \brief UART1 data register empty interrupt handler
 *
 * This handler is called each time UART1 data register is available for
 * sending data.
 */
ISR(UART1_DATA_EMPTY_IRQ)
{
	// if there is data in the ring buffer, fetch it and send it
	if (!ring_buffer_is_empty(&uart1_ring_buffer_out)) {
		UDR1 = ring_buffer_get(&uart1_ring_buffer_out);
	}
	else {
		// no more data to send, turn off data ready interrupt
		UCSR1B &= ~(1 << UDRIE1);
	}
}

/**
 * \brief UART0 Data RX interrupt handler
 *
 * This is the handler for UART0 receive data
 */
ISR(UART0_RX_IRQ)
{
	ring_buffer_put(&uart0_ring_buffer_in, UDR0);
}

/**
 * \brief UART1 Data RX interrupt handler
 *
 * This is the handler for UART1 receive data
 */
ISR(UART1_RX_IRQ)
{
	ring_buffer_put(&uart1_ring_buffer_in, UDR1);
}

/**
 * \brief Initialize UART0 with correct baud rate settings
 *
 * This function will initialize the UART0 baud rate registers with the correct
 * values using the AVR libc setbaud utility. In addition sets UART0 to
 * 8-bit, 1 stop and no parity.
 */
void uart0_init(void)
{
#if defined UBRR0H
	// get the values from the setbaud tool
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

#if USE_2X
	UCSR0A |= (1 << U2X0);
#endif

	// enable RX and TX and set interrupts on rx complete
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// 8-bit, 1 stop bit, no parity, asynchronous UART
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << USBS0) |
			(0 << UPM01) | (0 << UPM00) | (0 << UMSEL01) |
			(0 << UMSEL00);

	// initialize the in and out buffer for UART0
	uart0_ring_buffer_out = ring_buffer_init(uart0_out_buffer, UART0_BUFFER_SIZE);
	uart0_ring_buffer_in = ring_buffer_init(uart0_in_buffer, UART0_BUFFER_SIZE);
}

/**
 * \brief Initialize UART1 with correct baud rate settings
 *
 * This function will initialize the UART1 baud rate registers with the correct
 * values using the AVR libc setbaud utility. In addition sets UART1 to
 * 8-bit, 1 stop and no parity.
 */
void uart1_init(void)
{
#if defined UBRR1H
	// get the values from the setbaud tool
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

#if USE_2X
	UCSR1A |= (1 << U2X1);
#endif

	// enable RX and TX and set interrupts on rx complete
	UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);

	// 8-bit, 1 stop bit, no parity, asynchronous UART
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10) | (0 << USBS1) |
			(0 << UPM11) | (0 << UPM10) | (0 << UMSEL11) |
			(0 << UMSEL10);

	// initialize the in and out buffer for UART1
	uart1_ring_buffer_out = ring_buffer_init(uart1_out_buffer, UART1_BUFFER_SIZE);
	uart1_ring_buffer_in = ring_buffer_init(uart1_in_buffer, UART1_BUFFER_SIZE);
}

/**
 * \brief Function for putting a char in the UART0 buffer
 *
 * \param data the data to add to the UART0 buffer and send
 *
 */
inline void uart0_putchar(uint8_t data)
{
	// Disable interrupts to get exclusive access to uart0_ring_buffer_out.
	cli();
	if (ring_buffer_is_empty(&uart0_ring_buffer_out)) {
		// First data in buffer, enable data ready interrupt
		UCSR0B |=  (1 << UDRIE0);
	}
	// Put data in buffer
	ring_buffer_put(&uart0_ring_buffer_out, data);

	// Re-enable interrupts
	sei();
}

/**
 * \brief Function for putting a char in the UART1 buffer
 *
 * \param data the data to add to the UART1 buffer and send
 *
 */
inline void uart1_putchar(uint8_t data)
{
	// Disable interrupts to get exclusive access to uart1_ring_buffer_out.
	cli();
	if (ring_buffer_is_empty(&uart1_ring_buffer_out)) {
		// First data in buffer, enable data ready interrupt
		UCSR1B |=  (1 << UDRIE1);
	}
	// Put data in buffer
	ring_buffer_put(&uart1_ring_buffer_out, data);

	// Re-enable interrupts
	sei();
}

/**
 * \brief Function for getting a char from the UART0 receive buffer
 *
 * \retval Next data byte in recieve buffer
 */
inline uint8_t uart0_getchar(void)
{
	return ring_buffer_get(&uart0_ring_buffer_in);
}

/**
 * \brief Function for getting a char from the UART1 receive buffer
 *
 * \retval Next data byte in recieve buffer
 */
inline uint8_t uart1_getchar(void)
{
	return ring_buffer_get(&uart1_ring_buffer_in);
}

/**
 * \brief Function to check if we have a char waiting in the UART0 receive buffer
 *
 * \retval true if data is waiting
 * \retval false if no data is waiting
 */
inline bool uart0_char_waiting_in(void)
{
	return !ring_buffer_is_empty(&uart0_ring_buffer_in);
}

/**
 * \brief Function to check if we have a char waiting in the UART1 receive buffer
 *
 * \retval true if data is waiting
 * \retval false if no data is waiting
 */
inline bool uart1_char_waiting_in(void)
{
	return !ring_buffer_is_empty(&uart1_ring_buffer_in);
}

/**
 * \brief Function to check if we have a char waiting in the UART0 send buffer
 *
 * \retval true if outgoing data is queued
 * \retval false if no outgoing data is queued
 */
inline bool uart0_char_queued_out(void)
{
	return !ring_buffer_is_empty(&uart0_ring_buffer_out);
}

/**
 * \brief Function to check if we have a char waiting in the UART1 send buffer
 *
 * \retval true if outgoing data is queued
 * \retval false if no outgoing data is queued
 */
inline bool uart1_char_queued_out(void)
{
	return !ring_buffer_is_empty(&uart1_ring_buffer_out);
}

/**
 * \brief Initialize UART1 receive buffer
 *
 */
inline void uart1_init_input_buffer(void)
{
	uart1_ring_buffer_in = ring_buffer_init(uart1_in_buffer, UART1_BUFFER_SIZE);
}	

//
/**
 * \brief General short delays
 *
 * \x number of milliseconds to wait
 */

void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}

