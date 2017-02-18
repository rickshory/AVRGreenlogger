/*
 * RN42.h
 *
 * Created: 4/25/2012 10:49:34 AM
 *  Author: rshory
 */ 


#ifndef RN42_H_
#define RN42_H_

#include "compiler.h"

/**
 * \brief turns on power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
static inline void BT_power_on(void)
{
	PORTD |= (1<<4); // set high; power on
}

/**
 * \brief turns off power to the RN-42 Bluetooth module
 *
 * PortD, bit 4 controls power to the RN-42 Bluetooth module,
 * high = enabled,
 */
static inline void BT_power_off(void)
{
	PORTD &= ~(1<<4); // set low; turn off
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 9600
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
static inline void BT_baud_9600(void)
{
	PORTD |= (1<<7); // set high, baud 9600
}

/**
 * \brief sets RN-42 Bluetooth module baud rate to 115k
 *
 * PortD, bit 7 controls the BAUD rate of the RN-42 Bluetooth module: 
 * high = 9600, low = 115k or firmware setting
 */
static inline void BT_baud_115k(void)
{
	PORTD &= ~(1<<7); // set low, baud 115k
}

/**
 * \brief check if the RN-42 Bluetooth module has a live connection
 *
 * PortD, bit 5 reads the "CONNECTED" output of the RN-42 Bluetooth module: 
 * HIGH when connected, LOW otherwise
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
static inline bool BT_connected(void)
{
	return (PIND & (1<<5)); // read pin
}

/**
 * \brief check if the RN-42 Bluetooth module is powered
 *
 * PortD, bit 4 is an output that enables the power supply to the 
 * RN-42 Bluetooth module: 
 * HIGH when enabled, LOW when shut down.
 * This functions reads that state of that pin.
 *
 * \retval true  if connection is active
 * \retval false if no connection
 */
static inline bool BT_powered(void)
{
	return (PIND & (1<<4)); // read pin
}

void checkForBTCommands (void);
void BT_dataDump(char* stOpt);
uint16_t cyPerRTCSqWave(void);

// union allows writing in Lo and Hi bytes of ADC, and reading out whole word
typedef struct {
    union {
        struct {
            uint8_t loByte, hiByte; // this is the correct endian-ness
        };
        struct  {
            uint16_t wholeWord;
        };
    };
} twoByteData;

#endif /* RN42_H_ */