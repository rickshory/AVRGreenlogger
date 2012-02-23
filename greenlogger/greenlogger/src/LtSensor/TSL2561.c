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


//struct irrData irrReadings[4];

uint16_t irrReadingNumber, cellVoltage;

bool getIrrReading (uint8_t sensPosition, uint8_t sensChannel, irrData *rd) {
	uint8_t  irrSensorNumber, irrSensorReadAddr, irrSensorWriteAddr, irrChannelNumber, irrChannel;
	uint8_t r, d, ct, intTmp;
	uint16_t unsignedIntTmp;
	// set defaults
//	rd->irrLoByte = 0;
//	rd->irrHiByte = 0;
	rd->irrWholeWord = 0;
	rd->irrMultiplier = 1;

	if (sensPosition == TSL2561_UpLooking) {
		irrSensorWriteAddr = TSL2561_UP_ADDR_WRITE;
		irrSensorReadAddr = TSL2561_UP_ADDR_READ;
	} else if (sensPosition == TSL2561_DnLooking) {
		irrSensorWriteAddr = TSL2561_DN_ADDR_WRITE;
		irrSensorReadAddr = TSL2561_DN_ADDR_READ;		
	} else {
		return 0;
	}
//	outputStringToUART("\n\r entered irradiance routine \n\r");
	// check device ID, could use to read from various devices
	// can comment out this whole first section, if always using TSL2561
/*
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
		len = sprintf(str, "\n\r I2C_Write(TSL2561_UP_ADDR_WRITE): 0x%x\n\r", r);
		outputStringToUART(str);
		if (r != TW_MT_SLA_ACK) {
			outputStringToUART("\n\r sensor not present or not responding\n\r");
			return 0;
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
					// could use this to switch based on different devices
								//if ((r & 0xF0) == 0x50) { // part number for TSL2561 (datasheet says 0x1n, but actually reads 0x5n)
					// len = sprintf(str, " matched correct part number\n\r");
					// outputStringToUART(str);						
				}	
			}
		}
	}		
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");
*/
	
	// set up and get a reading
	// power up the device
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			irrChannel = (TSL2561_CMD_BIT | TSL2561_WRD_BIT | sensChannel);
/*	enable this section later
                    // start with high gain and high integration time
                    stateFlags_2.irrSensorGain = 1;
                    stateFlags_2.irrSensorIntegrationPeriodHi = 1;
                    stateFlags_2.irrSensorIntegrationPeriodLo = 0;
                    while (1) { // read sensor, adjusting gain and sensitivity till OK or topped out
*/
			// write; a byte command, setting the register to CONTROL = 0x00
			d = (TSL2561_CMD_BIT | TSL2561_CONTROL);
			r = I2C_Write(d);
//			len = sprintf(str, "\n\r I2C_Write(TSL2561_CMD_BIT | TSL2561_CONTROL): 0x%x\n\r", r);
//			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				d = TSL2561_PWR_ON; // write to CONTROL, power-up code
				r = I2C_Write(d);
//				len = sprintf(str, "\n\r I2C_Write(TSL2561_PWR_ON): 0x%x\n\r", r);
//				outputStringToUART(str);
				I2C_Stop();
			}

		} else {
			outputStringToUART("\n\r sensor not present or not responding\n\r");
			return 0;
		}
	}
	// set up gain and integration time, prep to read sensor
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {  // write; a byte command, setting the register to TIMING = 0x01
			d = (TSL2561_CMD_BIT | TSL2561_TIMING);
			r = I2C_Write(d);
			if (r == TW_MT_DATA_ACK) { // write to TIMING
				// for testing, use high gain and 402 ms integration time
				d = (TSL2561_GAIN_BIT | TSL2561_INTEG_BIT_HI);
				/* implement this section later
                        d = 0x00;
                        if (stateFlags_2.irrSensorGain)
                            d |= 0x10; // set high gain bit
                        if (stateFlags_2.irrSensorIntegrationPeriodHi)
                            d |= 0x02; // irrSensorIntegrationPeriod code 11 is disallowed so this is unambiguous
                        else {
                            if (stateFlags_2.irrSensorIntegrationPeriodLo)
                                d |= 0x01; // irrSensorIntegrationPeriod code 01, medium
                        }
// len = sprintf(str, "\n\r TIMING code, d = 0x%x\n\r", d);
// outputStringToUSART(str);

//
//                        // set sensor gain 0=low, 1=high
//                        if (stateFlags_2.irrSensorGain)
//                            d = 0x12; // write to TIMING, high gain, long integration time
//                        else
//                            d = 0x10; 
                        r = I2C_Write(d); // write to TIMING, the gain and integration time
                        I2C_Stop();

				
				*/
				r = I2C_Write(d); // write to TIMING, the gain and integration time
//				len = sprintf(str, "\n\r I2C_Write(TSL2561_GAIN_BIT | TSL2561_INTEG_BIT_HI): 0x%x\n\r", r);
//				outputStringToUART(str);
				I2C_Stop();
			}
		}
	}
	// get the irradiance reading
	r = I2C_Start();
//	len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//	outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {  // write; a byte command, setting the register to the desired channel
			r = I2C_Write(irrChannel);
//			len = sprintf(str, "\n\r I2C_Write(irrChannel): 0x%x\n\r", r);
//			outputStringToUART(str);
			for (intTmp = 1; intTmp < 100; intTmp++) {  // poll device up to 100 times
//				len = sprintf(str, "\n\r  Loop: 0x%x\n\r", intTmp);
//				outputStringToUART(str);
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
			len = sprintf(str, "\n\r reading: %d\n\r", (rd->irrWholeWord));
			outputStringToUART(str);
			
		}
	}		
			
/*
 //len = sprintf(str, "\n\r exited FOR loop, reading[%u] = %u, Gain = %c, IntegrCode = %c%c\n\r", 
 //         irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord, (stateFlags_2.irrSensorGain ? '1' : '0'),
 //         (stateFlags_2.irrSensorIntegrationPeriodHi ? '1' : '0'), 
 //         (stateFlags_2.irrSensorIntegrationPeriodLo ? '1' : '0'));
 //outputStringToUSART(str);
*/
	// power down the device
	r = I2C_Start();
//    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
//    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//		len = sprintf(str, "\n\r I2C_Write(TSL2561_??_ADDR_WRITE): 0x%x\n\r", r);
//		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			// write; a byte command, setting the register to CONTROL = 0x00
			d = (TSL2561_CMD_BIT | TSL2561_CONTROL);
			r = I2C_Write(d);
//			len = sprintf(str, "\n\r I2C_Write(TSL2561_CMD_BIT | TSL2561_CONTROL): 0x%x\n\r", r);
//			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				d = TSL2561_PWR_OFF; // write to CONTROL, power-down code
				r = I2C_Write(d);
//				len = sprintf(str, "\n\r I2C_Write(TSL2561_PWR_OFF): 0x%x\n\r", r);
//				outputStringToUART(str);
				I2C_Stop();
			}
		}
	}					
			


/*
                        // turn off device
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x80; // write; a byte command, setting the register to CONTROL = 0x00
                        r = I2C_Write(d);
                        d = 0x00; // write to CONTROL, power-down code
                        r = I2C_Write(d);
                        I2C_Stop();
                       // exit loop here, one way or another
                        if (irrReadings[irrReadingNumber].irrWholeWord < 0xFFFF)
                            break; // if zero (dark) or valid reading less than topped out at 2^16-1
                        if ((!stateFlags_2.irrSensorGain) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodHi) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodLo))
                            break; // also if topped out but we have done everything possible,
                                   // low gain and minimum integration period
                        // if we didn't exit, adjust bits, try not to top out
                        if (!stateFlags_2.irrSensorGain) { // if gain is already low
                            if (stateFlags_2.irrSensorIntegrationPeriodHi) { // integ period high, code 10
                                stateFlags_2.irrSensorIntegrationPeriodHi = 0; // set to medium code 01
                                stateFlags_2.irrSensorIntegrationPeriodLo = 1;
                            } else { // integ period is medium or low, 01 or 00
                                if (stateFlags_2.irrSensorIntegrationPeriodLo) // if medium, code 01
                                    stateFlags_2.irrSensorIntegrationPeriodLo = 0; // set to low, code 00
                            }
                        }
                        if (stateFlags_2.irrSensorGain) // if gain is high
                            stateFlags_2.irrSensorGain = 0; // set gain low
                    } // end of attempts at reading device, either OK or topped out, or zero
                    // calculate multiplier
                    // if high integration time, code 10, leave as default for now
                    if (!stateFlags_2.irrSensorIntegrationPeriodHi) {
                        if (stateFlags_2.irrSensorIntegrationPeriodLo) // medium, code 01
                            irrReadings[irrReadingNumber].irrMultiplier = 4; // (322/81), 0.62% error
                        else // low, code 00
                            irrReadings[irrReadingNumber].irrMultiplier = 29; // (322/11), 0.93% error
                    }
                    if (!stateFlags_2.irrSensorGain) // if gain is low
                        irrReadings[irrReadingNumber].irrMultiplier *= 16; // multiply by 16

                } // end of bb or ir channels
             
*/

	return 1;

}

