/*
 * DS1342.c
 *
 * Created: 2/19/2012 10:08:06 PM
 *  Author: rshory
 */ 
#include <inttypes.h>
#include "compiler.h"
#include "DS1342.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];

bool rtc_setTime (struct DateTime *t) {
	uint8_t r, ct;
	//Manipulating the Address Counter for Reads:
	// A dummy write cycle can be used to force the 
	// address counter to a particular value.

// do START
// write DS1342_ADDR_WRITE
// write the memory address where we intend to read (dummy read, set's pointer)
// do repeated START
// write DS1342_ADDR_READ
// read byte(s) with ACK or NACK as applicable
// do STOP

	outputStringToUART("\n\r entered time routine \n\r");
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(DS1342_ADDR_WRITE); // address the device, say we are going to write
		    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_WRITE): 0x%x\n\r", r);
		    outputStringToUART(str);
			if (r == TW_MT_SLA_ACK) {
//			    r = I2C_Write(DS1342_CONTROL_STATUS); // for testing, look at this register
			    r = I2C_Write(DS1342_TIME_SECONDS); // for testing, look at this register
			    len = sprintf(str, "\n\r I2C_Write(DS1342_CONTROL_STATUS): 0x%x\n\r", r);
			    outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
				    outputStringToUART(str);
					if (r == TW_REP_START) {
					    r = I2C_Write(DS1342_ADDR_READ); // address the device, say we are going to read
					    len = sprintf(str, "\n\r I2C_Write(DS1342_ADDR_READ): 0x%x\n\r", r);
					    outputStringToUART(str);
						if (r == TW_MR_SLA_ACK) {
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(1); // do ACK, since this is not the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
							r = I2C_Read(0); // do NACK, since this is the last byte
							len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
							outputStringToUART(str);
						}							
					}							
				}	
			}
		}		
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");

	return 1;
}
