/*
 * TSL2561.c
 *
 * Created: 2/21/2012 7:01:23 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "compiler.h"
#include "TSL2561.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];


struct irrData irrReadings[4];

uint16_t irrReadingNumber, cellVoltage;

bool getIrrReading (struct irrData *rd) {
	uint8_t  irrSensorNumber, irrSensorReadAddr, irrSensorWriteAddr, irrChannelNumber, irrChannel;
	uint8_t r, d, ct;
	irrSensorWriteAddr = TSL2561_UP_ADDR_WRITE;
	irrSensorReadAddr = TSL2561_UP_ADDR_READ;

	outputStringToUART("\n\r entered irradiance routine \n\r");
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
		if (r == TW_START) {
		    r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
		    len = sprintf(str, "\n\r I2C_Write(TSL2561_UP_ADDR_WRITE): 0x%x\n\r", r);
		    outputStringToUART(str);
			if (r != TW_MT_SLA_ACK) {
				outputStringToUART("\n\r sensor not present or not responding\n\r");
			} else {				
				d = 0x8a; // write; a byte command, setting the register to "ID" (Part number, Rev ID) = 0x0a
				r = I2C_Write(d);
				len = sprintf(str, "\n\r I2C_Write(0x8a): 0x%x\n\r", r);
				outputStringToUART(str);
				if (r == TW_MT_DATA_ACK) { // write-to-point-to-register was ack'd
					r = I2C_Start(); // restart
				    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
				    outputStringToUART(str);
					if (r == TW_REP_START) {
						r = I2C_Read(0); // do NACK, since this is the last and only byte read
						//	I2C_Stop(); // do this at the end, for now
						len = sprintf(str, "\n\r result of reading part ID: 0x%x\n\r", r);
						outputStringToUART(str);
									//if ((r & 0xF0) == 0x50) { // part number for TSL2561 (datasheet says 0x1n, but actually reads 0x5n)
						// len = sprintf(str, " matched correct part number\n\r");
						// outputStringToUSART(str);						
					}	
				}
			}
		}		
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");

	return 1;

}

