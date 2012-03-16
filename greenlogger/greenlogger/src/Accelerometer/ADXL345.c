/*
 * ADXL345.c
 *
 * Created: 2/16/2012 8:24:34 PM
 *  Author: rshory
 */ 
#include <inttypes.h>
#include "ADXL345.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include <util/twi.h>


extern volatile uint8_t stateFlags1;
extern int len;
extern char str[128];

/*****************************************
* try to detect whether the ADXL345 Accelerometer is there, and functioning
* returns with global "stateFlags1.accelerometerIsThere" set or cleared
*****************************************/
uint8_t findADXL345 (void) {
    uint8_t r;
    outputStringToUART("\n\r entered findADXL345 routine \n\r");
	stateFlags1 &= ~(1<<accelerometerIsThere); // flag cleared, until accel found
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
	if (r == TW_START) {
		r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
		len = sprintf(str, "\n\r I2C_Write(ADXL345_ADDR_WRITE): 0x%x\n\r", r);
		outputStringToUART(str);
		if (r == TW_MT_SLA_ACK) {
			stateFlags1 |= (1<<accelerometerIsThere); // accel found, set flag
			r = I2C_Write(ADXL345_REG_DEVID); // tell the device the register we are going to want
			len = sprintf(str, "\n\r I2C_Write(ADXL345_REG_DEVID): 0x%x\n\r", r);
			outputStringToUART(str);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Start(); // restart, preparatory to reading
				len = sprintf(str, "\n\r ReStart: 0x%x\n\r", r);
				outputStringToUART(str);
				if (r == TW_REP_START){
					r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
					len = sprintf(str, "\n\r I2C_Write(ADXL345_ADDR_READ): 0x%x\n\r", r);
					outputStringToUART(str);
					if (r == TW_MR_SLA_ACK){
						r = I2C_Read(0); // do NACK, since this is the last byte
						len = sprintf(str, "\n\r I2C_Read(0): 0x%x\n\r", r);
						outputStringToUART(str);
					}       
				} else {
					I2C_Stop();
					return errNoI2CRepStart;
				}
			} else { // could not write data to device
				I2C_Stop();
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
		}
	} else { // could not START
		return errNoI2CStart;
	}
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");
	return I2C_OK;
} // end of findAccelerometer

/*****************************************
* initialize ADXL345 Accelerometer, for system leveling and to detect taps
*****************************************/

uint8_t initializeADXL345 (void) {
    uint8_t r;
	outputStringToUART("\n\r entered initializeADXL345 routine \n\r");
//	findADXL345();
//    if (!(stateFlags1 & (1<<accelerometerIsThere)) {
//        return;
//    }
// len = sprintf(str, "\n\r accelerometer found \n\r");
// outputStringToUSART(str);
    // put device in standby to configure
	// if I2C is going to fail, does so virtually always in first few steps
	// so test the first START, ADDR_WRITE, and DATA_WRITE; after that proceed boldly
    r = I2C_Start();
	if (r == TW_START) {
		r = I2C_Write(ADXL345_ADDR_WRITE);
		if (r == TW_MT_SLA_ACK) {
			r = I2C_Write(ADXL345_REG_POWER_CTL);
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Write(0x00); // 
				I2C_Stop();
				// set data format
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_DATA_FORMAT);
				r = I2C_Write(0x0b); // Full resolution, +/-16g, 4mg/LSB.
				I2C_Stop();
				// set data rate
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_BW_RATE);
				r = I2C_Write(0x0a); // use 100Hz for now ; bit 4 set = reduced power, higher noise
				I2C_Stop();
				// set up tap detection
				// enable Tap Axes
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_TAP_AXES);
				r = I2C_Write(0x0f); // bits: 3= suppresses detection between double taps, 2=X, 1=Y, 0=Z
				I2C_Stop();
				// set tap threshold
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_THRESH_TAP);
				//    d = 0x01; // most sensitive, 62.5 mg/LSB (0xFF = +16 g)
				r = I2C_Write(0x30); // try ~3 g
				I2C_Stop();
				// set tap duration
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_DUR);
				//    d = 0x01; // most brief, 625 &#956;s/LSB.
				r = I2C_Write(0xa0); // try 0.1s
				I2C_Stop();
				// enable the Single Tap interrupt (later will change to Double Tap)
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_INT_ENABLE);
				r = I2C_Write(0x40); //  0x40 = single tap,  0x20 = double tap,  0x60 = both
				I2C_Stop();
				// initialize by clearing any interrupts
				r = clearAnyADXL345TapInterrupt();
				if (!r)
				 return r;
				//bring out of standby and set device to measure
				I2C_Start();
				r = I2C_Write(ADXL345_ADDR_WRITE);
				r = I2C_Write(ADXL345_REG_POWER_CTL);
				r = I2C_Write(0x08); //
				I2C_Stop();
				outputStringToUART("\n\r accelerometer initialized \n\r");
			} else { // could not write data to device
				I2C_Stop();
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
		}
	} else { // could not START
		return errNoI2CStart;
	}
	I2C_Stop();
	return I2C_OK;
} // end of initializeADXL345


/*****************************************
* clear any interrupt from the ADXL345 Accelerometer
*   uses I2C bus, and that should be initialized and stopped before calling this fn
*****************************************/

uint8_t clearAnyADXL345TapInterrupt (void) {
    char r;
//    if (!stateFlags.accelerometerIsThere) {
//        return;
//    }
	do {
		// clear the interrupt by reading the INT_SOURCE register
		r = I2C_Start();
		if (r == TW_START) {
			r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
			if (r == TW_MT_SLA_ACK) {
				r = I2C_Write(ADXL345_REG_INT_SOURCE); // tell the device the register we are going to want
				if (r == TW_MT_DATA_ACK) {
					r = I2C_Start(); // restart, preparatory to reading
					r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
					r = I2C_Read(0); // do NACK, since this is the last and only byte read
					I2C_Stop();
				} else { // could not write data to device
					I2C_Stop();
					return errNoI2CDataAck;
				}
			} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
			}
		} else { // could not START
			return errNoI2CStart;
		}
// len = sprintf(str, " result of clearing ADXL345 interrupt: 0x%x\n\r", r);
// outputStringToUSART(str);
    } while (r & 0x40); // tap = 0x40, double tap = 0x20
	return I2C_OK;
} // end of clearAnyADXL345TapInterrupt


/*****************************************
* read Accelerometer, for system leveling
*****************************************/

uint8_t readADXL345Axes (accelAxisData *d) {
    uint8_t r; // accelerometer readings
//    if (!stateFlags.accelerometerIsThere) {
//        // globals "len" and "str" available on return
//        len = sprintf(str, "\n\r no accelerometer");
//        return;
//    }
//    // bring out of low power mode
//	// use 100Hz for now ; bit 4 set = reduced power, higher noise
//	r = setADXL345Register(ADXL345_REG_BW_RATE, 0x0a);
//	if (!r) {
//		d->validation = r;
//		return r;
//	}
	
//			r = I2C_Write(ADXL345_REG_BW_RATE); // tell the device the register we are going to want
//			if (r == TW_MT_DATA_ACK) {
//				r = I2C_Write(0x0a); 
//				I2C_Stop();
//
//				I2C_Start();
//				r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
//				r = I2C_Write(ADXL345_REG_DATAX0); // tell the device the register we are going to want
				// read data
    r = I2C_Start();
	if (r == TW_START) {
		r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
		if (r == TW_MT_SLA_ACK) {
			// ADXL345_REG_DATA_FORMAT is contiguous just before data registers
			r = I2C_Write(ADXL345_REG_DATAX0); // tell the device the register we are going to want
//			r = I2C_Write(ADXL345_REG_DATA_FORMAT); // tell the device the register we are going to want
			if (r == TW_MT_DATA_ACK) {
				r = I2C_Start(); // restart, preparatory to reading
				r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
//				// read format info
//				d->dataFormat = I2C_Read(1); // do ACK, since not the last byte
//				len = sprintf(str, "\n\r format = 0x%x\n\r", d->dataFormat);
//				outputStringToUART(str);

//				I2C_Stop();
//				r = I2C_Write(ADXL345_ADDR_WRITE);
				
				// read data
				d->xLoByte = I2C_Read(1); // do ACK, since not the last byte
				d->xHiByte = I2C_Read(1); // do ACK, since not the last byte
				d->yLoByte = I2C_Read(1); // do ACK, since not the last byte
				d->yHiByte = I2C_Read(1); // do ACK, since not the last byte
				d->zLoByte = I2C_Read(1); // do ACK, since not the last byte
				d->zHiByte = I2C_Read(0); // do NACK, since this is the last byte
				len = sprintf(str, "\n\r X0=0x%x, X1=0x%x, Y0=0x%x, Y1=0x%x, Z0=0x%x, Z1=0x%x\n\r", 
				     d->xLoByte, d->xHiByte, d->yLoByte, d->yHiByte, d->zLoByte, d->zHiByte);
				outputStringToUART(str);
				I2C_Stop();
				d->validation = I2C_OK;
				return I2C_OK;

				
//				// globals "len" and "str" available on return
//				len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", (unsigned int)((int)x1 << 8 | (int)x0),
//						  (unsigned int)((int)y1 << 8 | (int)y0),  (unsigned int)((int)z1 << 8 | (int)z0));
				// put back in low power mode
//				I2C_Start();
//				r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
//				r = I2C_Write(ADXL345_REG_BW_RATE); // tell the device the register we are going to want
//			//                d = 0x16; // set low power bit (4) and slowest sampling rate, 6.25Hz, for 40uA current
//				r = I2C_Write(0x18); // set low power bit (4) and 25Hz sampling rate, for 40uA current		
				
			} else { // could not write data to device
				I2C_Stop();
				d->validation = errNoI2CDataAck;
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			d->validation = errNoI2CAddressAck;
			return errNoI2CAddressAck;
		}
	} else { // could not START
		d->validation = errNoI2CStart;
		return errNoI2CStart;
	}
//    I2C_Stop();
} // end of readADXL345Axes

// Generic function to set a single ADXL345 register to a value
uint8_t setADXL345Register (uint8_t reg, uint8_t val) {
	uint8_t r;
    r = I2C_Start();
	if (r == TW_START) {
//		outputStringToUART("\n\r setADXL345Register: about to write address \n\r");
		r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
		if (r == TW_MT_SLA_ACK) {
//			outputStringToUART("\n\r setADXL345Register: about to write data \n\r");
			r = I2C_Write(reg); // tell the device the register we are going to want
			if (r == TW_MT_DATA_ACK) {
//				outputStringToUART("\n\r setADXL345Register: about to write value \n\r");
				r = I2C_Write(val); // set the value
//				outputStringToUART("\n\r setADXL345Register: about to Stop \n\r");
				I2C_Stop();
//				outputStringToUART("\n\r setADXL345Register: Stop completed \n\r");
				return I2C_OK;
			} else { // could not write data to device
				I2C_Stop();
				return errNoI2CDataAck;
			}
		} else { // could not address device
			I2C_Stop();
			return errNoI2CAddressAck;
		}
	} else { // could not START
		return errNoI2CStart;
	}
};
