/******************************************************************************
*
*               Inter-Integrated Circuit (I2C) functions
*          I2C is also known as Two-wire Serial Interface (TWI)
*
******************************************************************************
* FileName:           I2C.c
* Dependencies:       (see below)
* Processor:          ATmega1284P
* Compiler:           GCC
* Company:            rickshory.com
* Version:            1.0.0
*
*****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

/**
 * \brief initializes the I2C peripheral
 *
 * sets bit rate, enables pull-ups, assures power is on
 * 
 *
 * \note ---
 * 
 */
void I2C_Init (void)
{
	// set up the Bit Rate Generator Unit
	// SCL frequency = (CPU Clock frequency)/(16 + 2(TWBR) * 4^(TWPS))
	//  TWBR = Value of the TWI Bit Rate Register.
	//  TWPS = Value of the prescaler bits in the TWI Status Register
	// TWPS (4^TWPS)
	// 0b00     1
	// 0b01     4
	// 0b10    16
	// 0b11    64
	// TWBR = (((CPU Clock frequency)/(SCL frequency)) - 16)/(2 * 4^(TWPS))
	// CPU clock = 8MHz, TWPS = 0b00, TWBR = 32, SCL freq = 100kHz
	TWBR = 32;
	TWSR = 0; // TWPS is bits 0 & 1; other bits are read-only
	
	// set the Port bits on the SCL & SDA lines, enable internal pull-up resistors
	// C0 is SCL, C1 is SDA
//	DDRC &= 0b11111100; // force to be inputs
//	PORTC |= 0b00000011; // enable pull-ups
	
	PRR0 &= ~(1<<PRTWI); // assure I2C module power is on (clear Power Reduction bit)
		
}
	// the following functions use these bits:
	// in TWCR– TWI Control Register:
	//Bit 7 – TWINT: interrupt flag
	//Bit 6 – TWEA: enable Acknowledge 
	//Bit 5 – TWSTA: START Condition Bit
	//Bit 4 – TWSTO: STOP Condition Bit
	//Bit 3 – TWWC: TWI Write Collision Flag
	//Bit 2 – TWEN: enable module
	//Bit 1 – Reserved
	//Bit 0 – TWIE: Interrupt Enable

/**
 * \brief creates a Start condition on the I2C bus
 *
 * Holds until Start condition is established
 * Used both for Start and ReStart (repeat start), these two are distinguished by the return value
 * Returns status; valid Start is TW_START (0x08), valid ReStart is TW_REP_START (0x10)
 *
 * \note Initialize I2C before calling this function
 * 
 */
uint8_t I2C_Start (void)
{
	uint8_t r;
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); // initiate Start sequence
	while (!(TWCR & (1<<TWINT))) // hardware sets TWINT bit when Start sequence done
		; // timeout or other exit here?
	r = (TWSR & TW_STATUS_MASK);
	return r;
}

/**
 * \brief creates a Stop condition on the I2C bus
 *
 * Holds until Stop condition is established
 * 
 *
 * \note Initialize I2C before calling this function
 * 
 */
void I2C_Stop (void)
{
	// clear interrupt flag, enable I2C, and request a Stop condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	// hardware clears TWSTO bit when Stop sequence done
	while (TWCR & (1<<TWSTO))
        ; // timeout or other exit here?
}

/**
 * \brief writes a single byte out on the I2C bus
 *
 * Input:     byte to be written
 * Returns:    0 if ACK, otherwise the status byte
 * 
 * \note  I2C should be initialized and Start condition on the bus before calling this function
 * 
 */

uint8_t  I2C_Write (uint8_t data)
{
    uint8_t r;
	TWDR = data; // put byte in data register
	TWCR = (1<<TWINT)|(1<<TWEN); // clear interrupt flag, enable I2C
	while (!(TWCR & (1<<TWINT))) // wait for interrupt flag to be set
		; // timeout or other exit here?
	r = (TWSR & TW_STATUS_MASK); // mask prescale bits
//	if (r == TW_MT_DATA_ACK)
//		return 0;
	return r;
}

/*************************************************************************
  Function:
    BYTE I2C_Read (BYTE ack)
  Summary:
    reads a single byte from the I2C bus
  Conditions:
    I2C should be enabled, then Write (device address, and any setup), 
         then ReStart preparatory to reading from slave device
  Input:
     set ack = 1 if to be acknowledged, 0 if NACK
  Return Values:
    contents of I2C2RCV
  Side Effects:
    None
  Description:
    Holds until byte is read, also until Acknowledge process is complete
  Remarks:
    None
  *************************************************************************/
/**
 * \brief reads a single byte from the I2C bus
 *
 * I2C should be enabled, then Write (device address, and any setup), 
 *        then ReStart preparatory to reading from slave device
 * Input: set ack = 1 if to be acknowledged, 0 if NACK
 * NACK indicates to slave to keep transmitting
 * ACK tells slave this is the last byte
 * Returns byte that was read
 *
 * \note Initialize I2C before calling this function. Unlike Start, ignores the state of signals
 * 
 */
uint8_t I2C_Read (uint8_t ack)
{
	if (ack)
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA); // set Ack enable bit
		// tell slave not to send any more
	else
		TWCR = (1<<TWINT)|(1<<TWEN); // only clear Int bit and I2C enable
		// all except last byte
	while (!(TWCR & (1<<TWINT)))
		; // timeout or other exit here?
//	TWCR |= (1<<TWEA); // restore default state of ACK
	return TWDR;
}


