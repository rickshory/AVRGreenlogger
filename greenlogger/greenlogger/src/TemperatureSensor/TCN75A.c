/*
 * TCN75A.c
 *
 * Created: 2/24/2012 5:57:30 AM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include "compiler.h"
#include "TCN75A.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>

extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];
extern uint8_t Timer1, Timer2;

uint8_t r;

uint8_t initOneShotTemperatureReading (void) {
	// put the device in SHUTDOWN mode to save power
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT);
//				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT): 0x%x\n\r", r);
//				outputStringToUART(str);
			} else { // could not write data to device
				I2C_Stop();
				return 3;
			}
		} else { // could not address device
			
		}
	I2C_Stop();
//	outputStringToUART("\n\r STOP completed \n\r");
	} else { // could not START
		return 1;
	}
	
	// write ONE-SHOT to device, for a single temperature conversion
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT);
//				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT): 0x%x\n\r", r);
//				outputStringToUART(str);
			} else { // could not write data to device
				I2C_Stop();
				return 3;
			}
		} else { // could not address device
			I2C_Stop();
			return 2;
		}
	I2C_Stop();
//	outputStringToUART("\n\r STOP completed \n\r");
	} else { // could not START
		return 1;
	}
	return 0;
}

uint8_t getTemperatureReading (tmprData *tr) {
/*
	// put the device in SHUTDOWN mode to save power
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT);
				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT): 0x%x\n\r", r);
				outputStringToUART(str);
			}
		}
	I2C_Stop();
	outputStringToUART("\n\r STOP completed \n\r");
	}
	
	// write ONE-SHOT to device, for a single temperature conversion
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT);
				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT): 0x%x\n\r", r);
				outputStringToUART(str);
			}
		}
	I2C_Stop();
	outputStringToUART("\n\r STOP completed \n\r");
	}
	// temperature conversion time, typically 30ms
	for (Timer2 = 4; Timer2; );	// Wait for 40ms to be sure
*/
	// read ambient temperature from the device
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_TA);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Start(); // restart, prep to read
//				len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//				outputStringToUART(str);
				if (r == TW_REP_START) {
					r = I2C_Write(TCN75A_ADDR_READ); // address the device, say we are going to read
//					len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_READ): 0x%x\n\r", r);
//					outputStringToUART(str);
					if (r == TW_MR_SLA_ACK) {
						tr->tmprHiByte = I2C_Read(1); // do ACK, because not last byte
						tr->tmprLoByte = I2C_Read(0); // do NACK, since this is the last byte
//						len = sprintf(str, "\n\r I2C_READ(tr->tmprHiByte): 0x%x\n\r", (tr->tmprHiByte));
//						len = sprintf(str, "\n\r I2C_READ(tr->tmprHiByte): %d\n\r", (tr->tmprHiByte));
//						outputStringToUART(str);
					} else { // could not address device to READ
						I2C_Stop();
						return 5;
					}
				} else { // could not reSTART
					I2C_Stop();
					return 4;
				}
			} else { // could not write to device
				I2C_Stop();
				return 3;
			}
		} else { // could not address device
			I2C_Stop();
			return 2;
		}
	I2C_Stop();
//	outputStringToUART("\n\r STOP completed \n\r");
	} else { // could not START
		return 1;
	}
	return 0;
}