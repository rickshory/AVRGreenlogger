/*
 * mega_uart_interrupt.h
 *
 * Created: 12/27/2011 1:24:41 PM
 *  Author: rshory
 */ 

#ifndef MEGA_UART_INTERRUPT_H_
#define MEGA_UART_INTERRUPT_H_
extern void uart0_init(void);
extern inline void uart0_putchar(uint8_t data);
extern inline uint8_t uart0_getchar(void);
extern inline bool uart0_char_waiting_in(void);
extern inline bool uart0_char_queued_out(void);
extern void uart1_init(void);
extern inline void uart1_putchar(uint8_t data);
extern inline uint8_t uart1_getchar(void);
extern inline bool uart1_char_waiting_in(void);
extern inline bool uart1_char_queued_out(void);
extern void delay_ms(uint16_t x);

#endif /* MEGA_UART_INTERRUPT_H_ */
