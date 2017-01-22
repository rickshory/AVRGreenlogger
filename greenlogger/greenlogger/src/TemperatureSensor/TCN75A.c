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

uint8_t temperature_DeviceShutdown (void) {
	// put the device in SHUTDOWN mode to save power
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToWiredUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToWiredUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT);
//				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT): 0x%x\n\r", r);
//				outputStringToWiredUART(str);
				I2C_Stop();
				return I2C_OK;
			} else { // could not write data to device
				I2C_Stop();
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
		}
	I2C_Stop();
//	outputStringToWiredUART("\n\r STOP completed \n\r");
	} else { // could not START
		return errNoI2CStart;
	}
}

uint8_t temperature_InitOneShotReading (void) {
	// put the device in SHUTDOWN mode to save power
	r = temperature_DeviceShutdown();
	if (r) return r;
	//r = I2C_Start();
////    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
////    outputStringToWiredUART(str);
	//if (r == TW_START) {
		//r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
////		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
////		outputStringToWiredUART(str);
		//if (r == TW_MT_SLA_ACK) {
			//r = I2C_Write(TCN75A_CONFIG);
////			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
////			outputStringToWiredUART(str);
			//if (r == TW_MT_DATA_ACK) {
				//r = I2C_Write(TCN75A_SHUTDOWN_BIT);
////				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT): 0x%x\n\r", r);
////				outputStringToWiredUART(str);
			//} else { // could not write data to device
				//I2C_Stop();
				//return errNoI2CDataAck;
			//}
		//} else { // could not address device
			//I2C_Stop();
			//return errNoI2CAddressAck;
		//}
	//I2C_Stop();
////	outputStringToWiredUART("\n\r STOP completed \n\r");
	//} else { // could not START
		//return errNoI2CStart;
	//}
	//
	// write ONE-SHOT to device, for a single temperature conversion
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToWiredUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_CONFIG);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToWiredUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT);
//				len = sprintf(str, "\n\r I2C_Write(TCN75A_SHUTDOWN_BIT | TCN75A_ONE_SHOT_BIT): 0x%x\n\r", r);
//				outputStringToWiredUART(str);
				I2C_Stop();
				return I2C_OK;
			} else { // could not write data to device
				I2C_Stop();
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
		}
//	outputStringToWiredUART("\n\r STOP completed \n\r");
	} else { // could not START
		return errNoI2CStart;
	}
}

uint8_t temperature_GetReading (tmprData *tr) {
	r = temperature_InitOneShotReading();
	if (r) {
		tr->verification = r;
		return r;
	}		
	// temperature conversion time, typically 30ms
	for (Timer2 = 4; Timer2; );	// Wait for 40ms to be sure
	// read ambient temperature from the device
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToWiredUART(str);
	if (r == TW_START) {
		r = I2C_Write(TCN75A_ADDR_WRITE); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToWiredUART(str);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(TCN75A_TA);
//			len = sprintf(str, "\n\r I2C_Write(TCN75A_CONFIG): 0x%x\n\r", r);
//			outputStringToWiredUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Start(); // restart, prep to read
//				len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//				outputStringToWiredUART(str);
				if (r == TW_REP_START) {
					r = I2C_Write(TCN75A_ADDR_READ); // address the device, say we are going to read
//					len = sprintf(str, "\n\r I2C_Write(TCN75A_ADDR_READ): 0x%x\n\r", r);
//					outputStringToWiredUART(str);
					if (r == TW_MR_SLA_ACK) {
						tr->tmprHiByte = I2C_Read(1); // do ACK, because not last byte
						tr->tmprLoByte = I2C_Read(0); // do NACK, since this is the last byte
//						len = sprintf(str, "\n\r I2C_READ(tr->tmprHiByte): 0x%x\n\r", (tr->tmprHiByte));
//						len = sprintf(str, "\n\r I2C_READ(tr->tmprHiByte): %d\n\r", (tr->tmprHiByte));
//						outputStringToWiredUART(str);
						I2C_Stop();
						tr->verification = I2C_OK;
						return I2C_OK;
					} else { // could not address device to READ
						I2C_Stop();
						tr->verification = errNoI2CAddrAckRead;
						return errNoI2CAddrAckRead;
					}
				} else { // could not reSTART
					I2C_Stop();
					tr->verification = errNoI2CRepStart;
					return errNoI2CRepStart;
				}
			} else { // could not write to device
				I2C_Stop();
				tr->verification = errNoI2CDataAck;
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			tr->verification = errNoI2CAddressAck;
			return errNoI2CAddressAck;
		}
//	outputStringToWiredUART("\n\r STOP completed \n\r");
	} else { // could not START
		tr->verification = errNoI2CStart;
		return errNoI2CStart;
	}
}