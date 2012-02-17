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
void findADXL345 (void) {
    uint8_t r;
//    len = sprintf(str, "\n\r entered findAccelerometer routine \n\r");
    outputStringToUART("\n\r entered findAccelerometer routine \n\r");
	stateFlags1 &= ~(1<<accelerometerIsThere); // flag cleared, until accel found
	r = I2C_Start();
    len = sprintf(str, "\n\r I2C_Start: 0x%x\n\r", r);
    outputStringToUART(str);
	// if (r == TW_START)
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    len = sprintf(str, "\n\r I2C_Write(ADXL345_ADDR_WRITE): 0x%x\n\r", r);
    outputStringToUART(str);
	if (r == TW_MT_SLA_ACK)
		stateFlags1 |= (1<<accelerometerIsThere); // accel found, set flag
	if (stateFlags1 & (1<<accelerometerIsThere)) {
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
//        
			}
		}

	}
    I2C_Stop();
    outputStringToUART("\n\r I2C_Stop completed \n\r");
/*
    stateFlags.accelerometerIsThere = 0; // until found
    I2C_Stop();
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    if (r)
        stateFlags.accelerometerIsThere = 1; // if NACK, assume device is not there
    if (stateFlags.accelerometerIsThere) {  // get the device ID, can use if needed for further test
        r = I2C_Write(ADXL345_REG_DEVID); // tell the device the register we are going to want
        I2C_ReStart(); // restart, preparatory to reading
        r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
        r = I2C_Read(0); // do NACK, since this is the last byte
        // result should be 0xE5
        I2C_Stop();
        len = sprintf(str, "\n\r got Accelerometer device ID: 0x%x\n\r", (int)r);
        outputStringToUSART(str);
    } else {
        len = sprintf(str, " Accelerometer not found: 0x%x\n\r", r);
        outputStringToUSART(str);
    }
*/
} // end of findAcceleromete
