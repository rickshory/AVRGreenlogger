/*
 * TCN75A.h
 *
 * Created: 2/24/2012 5:57:06 AM
 *  Author: rshory
 */ 


#ifndef TCN75A_H_
#define TCN75A_H_

// defines for the TCN75A temperature sensor

// address can't be:
// 0x3A/0x3B, the accelerometer
// 0xd0/0xd1, the real-time clock
// 0x52/0x53 or 0x92/0x93, the irradiance sensor
//
// address select pins define bits 3:1, upper 4 are set to 0xd_
// in this design all address select pins tied high, to give an address of 0xde/0xdf
// 
#define TCN75A_ADDR_WRITE 0x9e
#define TCN75A_ADDR_READ 0x9f

#define TCN75A_TA		0b00000000 // Temperature register
#define TCN75A_CONFIG	0b00000001 // Configuration register
#define TCN75A_THYST	0b00000010 // Temperature Hysteresis register
#define TCN75A_TSET		0b00000011 // Temperature Limit-set register


// Configuration register bits
// bit 7 ONE-SHOT bit
//	1 = Enabled
//	0 = Disabled (Power-up default)
#define TCN75A_ONE_SHOT_BIT		0b10000000
// bit 6-5 - ADC RESOLUTION bits
//	00 = 9 bit or 0.5°C (Power-up default)
//	01 = 10 bit or 0.25°C
//	10 = 11 bit or 0.125°C
//	11 = 12 bit or 0.0625°C
// bit 4-3 FAULT QUEUE bits
//	00 = 1 (Power-up default)
//	01 = 2
//	10 = 4
//	11 = 6
// bit 2 ALERT POLARITY bit
//	1 = Active-high
//	0 = Active-low (Power-up default)
#define TCN75A_ALERT_HI_BIT		0b00000100
// bit 1 COMP/INT bit
//	1 = Interrupt mode
//	0 = Comparator mode (Power-up default)
// bit 0 SHUTDOWN bit
//	1 = Enable
//	0 = Disable (Power-up default)
#define TCN75A_SHUTDOWN_BIT		0b00000001

// union allows writing in Lo and Hi bytes of temperature, and reading out whole word
typedef struct {
    union {
        struct {
            uint8_t tmprLoByte, tmprHiByte; // this is the correct endian-ness
        };
        struct  {
            uint16_t tmprWholeWord;
        };
    };
    uint8_t verification;
} tmprData;

tmprData temperatureReading;

// functions
uint8_t temperature_DeviceShutdown(void);
uint8_t temperature_InitOneShotReading (void);
uint8_t temperature_GetReading (tmprData *tr);

#endif /* TCN75A_H_ */