/**
 * \file
 *
 * \brief code for NDVI (greeness) logger, base on AVR ATmega1284P
 *
 * Copyright (C) 2011 Rick Shory, based in part on source code that is:
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
/**
/*
 * greenlogger.c
 *
 * Created: 12/21/2011 10:04:15 AM
 *  Author: Rick Shory
 */ 

#include <avr/io.h>
#include "greenlogger.h"
#include "SDcard/ff.h"


// the string we send and receive on UART
const char test_string[] = "Count \n";
char num_string[20];
char datetime_string[25];
//char* dt_stp = datetime_string;
//DateTime RTC_DT;

extern RTC_dt;

/**
 * \brief The main application
 *
 * This application will initialize the UART, send a character and then receive
 * it and check if it is the same character as was sent.
 *
 * \note The RX and TX pins should be externally connected in order to pass the
 * test.
 */
int main(void)
{
	uint8_t data;
	uint8_t cnt;
	uint16_t cntout = 0;
	cli();
	setupDiagnostics();
	uart_init();
	sei();

	// Send the test string
	for (cnt = 0; cnt < strlen(test_string); cnt++) {
		uart_putchar(test_string[cnt]);
	}
	rtc_setdefault();
    // output a counter
	do {
		itoa(cntout, num_string, 10);
		for (cnt = 0; cnt < strlen(num_string); cnt++) {
			uart_putchar(num_string[cnt]);
		}
		uart_putchar('\t');
		datetime_getstring(datetime_string, &RTC_dt);
		
		for (cnt = 0; cnt < strlen(datetime_string); cnt++) {
			uart_putchar(datetime_string[cnt]);
		}		
		uart_putchar('\n');
		delay_ms(1000);
		cntout++;
		rtc_add1sec();
	} while (cntout < 65500);
	// Check if we have received the string we sent
	cnt = 0;
	do {
		// Wait for next character
		while (!uart_char_waiting());
		data = uart_getchar();
		// Compare to what we sent
//		Assert (data == test_string[cnt++]);
	} while (cnt < strlen(test_string));

	while (1);
}
