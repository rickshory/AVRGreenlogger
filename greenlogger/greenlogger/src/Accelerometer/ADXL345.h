/*
 * ADXL345.h
 *
 * Created: 2/16/2012 6:02:46 AM
 *  Author: rshory
 */ 

// functions

void findADXL345 (void);


#ifndef ADXL345_H_
#define ADXL345_H_

// defines for the ADXL345 accellerometer

#define ADXL345_ADDR_WRITE 0x3A
#define ADXL345_ADDR_READ 0x3B

#define ADXL345_REG_DEVID  0x00 // R  11100101  Device ID. 
#define ADXL345_REG_THRESH_TAP  0x1D // R/W  00000000  Tap threshold. 
#define ADXL345_REG_OFSX  0x1E // R/W  00000000  X-axis offset. 
#define ADXL345_REG_OFSY  0x1F // R/W  00000000  Y-axis offset. 
#define ADXL345_REG_OFSZ  0x20 // R/W  00000000  Z-axis offset. 
#define ADXL345_REG_DUR  0x21 // R/W  00000000  Tap duration. 
#define ADXL345_REG_Latent  0x22 // R/W  00000000  Tap latency. 
#define ADXL345_REG_Window  0x23 // R/W  00000000  Tap window. 
#define ADXL345_REG_THRESH_ACT  0x24 // R/W  00000000  Activity threshold. 
#define ADXL345_REG_THRESH_INACT  0x25 // R/W  00000000  Inactivity threshold. 
#define ADXL345_REG_TIME_INACT  0x26 // R/W  00000000  Inactivity time. 
#define ADXL345_REG_ACT_INACT_CTL  0x27 // R/W  00000000  Axis enable control for activity and inactivity detection. 
#define ADXL345_REG_THRESH_FF  0x28 // R/W  00000000  Free-fall threshold. 
#define ADXL345_REG_TIME_FF  0x29 // R/W  00000000  Free-fall time. 
#define ADXL345_REG_TAP_AXES  0x2A // R/W  00000000  Axis control for tap/double tap. 
#define ADXL345_REG_ACT_TAP_STATUS  0x2B // R  00000000  Source of tap/double tap. 
#define ADXL345_REG_BW_RATE  0x2C // R/W  00001010  Data rate and power mode control. 
#define ADXL345_REG_POWER_CTL  0x2D // R/W  00000000  Power-saving features control. 
#define ADXL345_REG_INT_ENABLE  0x2E // R/W  00000000  Interrupt enable control. 
#define ADXL345_REG_INT_MAP  0x2F // R/W  00000000  Interrupt mapping control. 
#define ADXL345_REG_INT_SOURCE  0x30 // R  00000010  Source of interrupts. 
#define ADXL345_REG_DATA_FORMAT  0x31 // R/W  00000000  Data format control. 
#define ADXL345_REG_DATAX0  0x32 // R  00000000  X-Axis Data 0. 
#define ADXL345_REG_DATAX1  0x33 // R  00000000  X-Axis Data 1. 
#define ADXL345_REG_DATAY0  0x34 // R  00000000  Y-Axis Data 0. 
#define ADXL345_REG_DATAY1  0x35 // R  00000000  Y-Axis Data 1. 
#define ADXL345_REG_DATAZ0  0x36 // R  00000000  Z-Axis Data 0. 
#define ADXL345_REG_DATAZ1  0x37 // R  00000000  Z-Axis Data 1. 
#define ADXL345_REG_FIFO_CTL  0x38 // R/W  00000000  FIFO control.

//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

#endif /* ADXL345_H_ */