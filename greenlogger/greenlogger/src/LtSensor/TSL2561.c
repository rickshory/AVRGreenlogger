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



uint16_t irrReadingNumber, cellVoltage;

// steps:
// initialize data struct
// see if the sensor device is there (up- or down-looking)
// do down-looking sensor first then up-looking
// for each sensor, broadband first then infrared
// for each reading start at highest sensitivity,
//  if topped out, try lower until either not topped out or as low as possible
// gain and integration time in the TIMING register sets the sensitivity
// bits are:
// 7 to 5: reserved, write 0
// 4: GAIN: 0 = low gain (1×); 1 = high gain (16×)
// 3: Manual integration mode; we do not use, write 0
// 2: reserved, write 0
// 1 & 0: INTEG: integration time, see below
//
// So, in this order, we will try these bit settings (regTimingVal) for this register:
// 0b00010010 // 16x high gain, 402ms integration time. Multiplier 1
// 0b00000010 // 1x low gain, 402ms integration time. Multiplier 16 * 1 = 16
// 0b00000001 // 1x low gain, 101ms integration time. Multiplier 16 * (322/81) = 63.60493827, use 64 (0.62% error)
// 0b00000000 // 1x low gain, 13.7ms integration time. Multiplier 16 * (322/11) = 468.3636364, use 468 (0.08% error)
// (these bit patterns #defined in the header file)
// power up and power down each device, for each channel, for each integration/gain setting
// on each sensor/channel/sensitivity, do 10 iterations, until either nonzero reading or 10 tries


uint8_t getIrrReading (uint8_t sensPosition, uint8_t sensChannel, irrData *rd) {
	uint8_t irrSensorReadAddr, irrSensorWriteAddr, irrChannel;
	uint8_t r, d, ct, regTimingVal = TSL2561_GAIN_HI_INTEG_LONG, intTmp;
	uint16_t unsignedIntTmp;
	// set defaults
	rd->irrWholeWord = 0;
	rd->irrMultiplier = TSL2561_GAIN_HI_INTEG_LONG_MULTIPLIER;
	
	if (sensPosition == TSL2561_UpLooking) {
		irrSensorWriteAddr = TSL2561_UP_ADDR_WRITE;
		irrSensorReadAddr = TSL2561_UP_ADDR_READ;
	} else if (sensPosition == TSL2561_DnLooking) {
		irrSensorWriteAddr = TSL2561_DN_ADDR_WRITE;
		irrSensorReadAddr = TSL2561_DN_ADDR_READ;		
	} else {
		rd->validation = errBadParameters;
		return rd->validation;
	}
//	outputStringToUART0("\n\r entered irradiance routine \n\r");
	// check device ID, could use to read from various devices
	// can comment out this whole first section, if always using TSL2561

	while (1) { // read sensor, decreasing sensitivity till either not topped out, or min sensitivity
		// set up and get a reading
		// power up the device
		r = I2C_Start();
	//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
	//    outputStringToUART0(str);
		if (r == TW_START) {
			r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
	//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
	//		outputStringToUART0(str);
			if (r == TW_MT_SLA_ACK) {
				irrChannel = (TSL2561_CMD_BIT | TSL2561_WRD_BIT | sensChannel);
				// write; a byte command, setting the register to CONTROL = 0x00
				d = (TSL2561_CMD_BIT | TSL2561_CONTROL);
				r = I2C_Write(d);
	//			len = sprintf(str, "\n\r I2C_Write(TSL2561_CMD_BIT | TSL2561_CONTROL): 0x%x\n\r", r);
	//			outputStringToUART0(str);
				if (r == TW_MT_DATA_ACK) {
					d = TSL2561_PWR_ON; // write to CONTROL, power-up code
					r = I2C_Write(d);
	//				len = sprintf(str, "\n\r I2C_Write(TSL2561_PWR_ON): 0x%x\n\r", r);
	//				outputStringToUART0(str);
					I2C_Stop();
				} else { // could not write data to device
					I2C_Stop();
					rd->validation = errNoI2CDataAck;
					return rd->validation;
				}
			} else { // could not address device
//				outputStringToUART0("\n\r sensor not present or not responding\n\r");
				I2C_Stop();
				rd->validation = errNoI2CAddressAck;
				return rd->validation;
			}
		} else { // could not START
			rd->validation = errNoI2CStart;
			return rd->validation;
		}			
		// device powered up
		// set up sensitivity, prep to read sensor
		r = I2C_Start();
	//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
	//    outputStringToUART0(str);
		if (r == TW_START) {
			r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
	//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
	//		outputStringToUART0(str);
			if (r == TW_MT_SLA_ACK) {  // write; a byte command, setting the register to TIMING = 0x01
				d = (TSL2561_CMD_BIT | TSL2561_TIMING);
				r = I2C_Write(d);
				if (r == TW_MT_DATA_ACK) { // write to TIMING
					// for testing, use high gain and 402 ms integration time
					r = I2C_Write(regTimingVal); // write to TIMING, the gain and integration time
	//				len = sprintf(str, "\n\r I2C_Write(regTimingVal): 0x%x\n\r", r);
	//				outputStringToUART0(str);
					I2C_Stop();
				} else { // could not write data to device
					I2C_Stop();
					rd->validation = errNoI2CDataAck;
					return rd->validation;
				}
			} else { // could not address device
//				outputStringToUART0("\n\r sensor not present or not responding\n\r");
				I2C_Stop();
				rd->validation = errNoI2CAddressAck;
				return rd->validation;
			}
		} else { // could not START
			rd->validation = errNoI2CStart;
			return rd->validation;
		} // gain and integration time set up
		//
		// get the irradiance reading
		r = I2C_Start();
	//	len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
	//	outputStringToUART0(str);
		if (r == TW_START) {
			r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
	//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
	//		outputStringToUART0(str);
			if (r == TW_MT_SLA_ACK) { // write; a byte command, setting the register to the desired channel
				r = I2C_Write(irrChannel);
	//			len = sprintf(str, "\n\r I2C_Write(irrChannel): 0x%x\n\r", r);
	//			outputStringToUART0(str);
				for (intTmp = 1; intTmp < 10; intTmp++) {  // poll device up to 10 times
	//				len = sprintf(str, "\n\r  Loop: 0x%x\n\r", intTmp);
	//				outputStringToUART0(str);
					r = I2C_Start(); // restart, preparatory to reading
					r = I2C_Write(irrSensorReadAddr); // address the device, say we are going to read
	// d = irrSensorReadAddr;
	// r = I2C_Write(d);
	// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
	// outputStringToUSART(str);
					rd->irrLoByte =  I2C_Read(1); // do ACK, because not last byte
					rd->irrHiByte = I2C_Read(0); // do NACK, since this is the last byte
	//				I2C_Stop(); // needed?
	// len = sprintf(str, "\n\r after %i tries, reading[%u] = %u\n\r", intTmp, irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord);
	// outputStringToUSART(str);
	//				if ((uint16_t)((rd->irrHiByte) * 256 + (rd->irrLoByte)) > 0)
					if ((rd->irrWholeWord) > 0)
						break;
					for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
						 //       while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // in case something is printing
				} // end of polling FOR loop, either zero or value
				I2C_Stop();
//				len = sprintf(str, "\n\r reading: %lu\n\r", (unsigned long)(rd->irrWholeWord));
//				outputStringToUART0(str);
			
			} else { // could not address device
//				outputStringToUART0("\n\r sensor not present or not responding\n\r");
				I2C_Stop();
				rd->validation = errNoI2CAddressAck;
				return rd->validation;
			}
		} else { // could not START
			rd->validation = errNoI2CStart;
			return rd->validation;
		}
		// power down the device
		r = I2C_Start();
	//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
	//    outputStringToUART0(str);
		if (r == TW_START) {
			r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
	//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
	//		outputStringToUART0(str);
			if (r == TW_MT_SLA_ACK) {
				// write; a byte command, setting the register to CONTROL = 0x00
				d = (TSL2561_CMD_BIT | TSL2561_CONTROL);
				r = I2C_Write(d);
	//			len = sprintf(str, "\n\r I2C_Write(TSL2561_CMD_BIT | TSL2561_CONTROL): 0x%x\n\r", r);
	//			outputStringToUART0(str);
				if (r == TW_MT_DATA_ACK) {
					d = TSL2561_PWR_OFF; // write to CONTROL, power-down code
					r = I2C_Write(d);
	//				len = sprintf(str, "\n\r I2C_Write(TSL2561_PWR_OFF): 0x%x\n\r", r);
	//				outputStringToUART0(str);
					I2C_Stop();
				} else { // could not write data to device
					I2C_Stop();
					rd->validation = errNoI2CDataAck;
					return rd->validation;
				}
			} else { // could not address device
//				outputStringToUART0("\n\r sensor not present or not responding\n\r");
				I2C_Stop();
				rd->validation = errNoI2CAddressAck;
				return rd->validation;
			}
		} else { // could not START
			rd->validation = errNoI2CStart;
			return rd->validation;
		}
		// exit the while(1) loop here, one way or another
		//  break out of it either by value<max, or val=0 after 10 iterations, or minimum sensitivity
		
		if (rd->irrWholeWord < 0xffff)
			break; // if zero (dark) or valid reading less than topped out at 2^16-1
		if (regTimingVal == TSL2561_GAIN_LO_INTEG_SHORT)
			break; // also if topped out but minimum sensitivity
		// if we didn't exit, decrease sensitivity, try to not top out
		switch (regTimingVal) {
			case TSL2561_GAIN_LO_INTEG_MED: // if Medium, make Low
				regTimingVal = TSL2561_GAIN_LO_INTEG_SHORT;
				rd->irrMultiplier = TSL2561_GAIN_LO_INTEG_SHORT_MULTIPLIER;
				break;
			case TSL2561_GAIN_LO_INTEG_LONG: // if High, make Medium
				regTimingVal = TSL2561_GAIN_LO_INTEG_MED;
				rd->irrMultiplier = TSL2561_GAIN_LO_INTEG_MED_MULTIPLIER;
				break;
			case TSL2561_GAIN_HI_INTEG_LONG: // if VeryHigh, make High
				regTimingVal = TSL2561_GAIN_LO_INTEG_LONG;
				rd->irrMultiplier = TSL2561_GAIN_LO_INTEG_LONG_MULTIPLIER;
				break;
		}
	} // end of while(1) loop
	rd->validation = I2C_OK;
	return rd->validation;
}

