/*
 * TSL2561.h
 *
 * Created: 2/21/2012 7:00:57 AM
 *  Author: rshory
 */ 


#ifndef TSL2561_H_
#define TSL2561_H_

// I2C address when ADDR SEL pin is tied low = down-pointing sensor
#define TSL2561_DN_ADDR_WRITE 0x52
#define TSL2561_DN_ADDR_READ 0x53
// I2C address when ADDR SEL pin is tied high = up-pointing sensor
#define TSL2561_UP_ADDR_WRITE 0x92
#define TSL2561_UP_ADDR_READ 0x93

// registers
#define	TSL2561_CONTROL	0x00	// Control of basic functions
#define	TSL2561_TIMING	0x01	// Integration time/gain control
#define	TSL2561_THRESHLOWLOW	0x02	// Low byte of low interrupt threshold
#define	TSL2561_THRESHLOWHIGH	0x03	// High byte of low interrupt threshold
#define	TSL2561_THRESHHIGHLOW	0x04	// Low byte of high interrupt threshold
#define	TSL2561_THRESHHIGHHIGH	0x05	// High byte of high interrupt threshold
#define	TSL2561_INTERRUPT	0x06	// Interrupt control
//#define	?? Reserved	0x07	
//#define	TSL2561_CRC	0x08	// Factory test — not a user register
//#define	?? Reserved	0x09	
#define	TSL2561_ID	0x0a	// Part number/ Rev ID
//#define	?? Reserved	0x0b	
#define	TSL2561_DATA0LOW	0x0c	// Low byte of ADC channel 0
#define	TSL2561_DATA0HIGH	0x0d	// High byte of ADC channel 0
#define	TSL2561_DATA1LOW	0x0e	// Low byte of ADC channel 1
#define	TSL2561_DATA1HIGH	0x0f	// High byte of ADC channel 1

// options
#define TSL2561_CHANNEL_BROADBAND	0xac	// broadband channel
#define TSL2561_CHANNEL_INFRARED	0xae	// infrared channel

enum TSL2561_Opt_UpDn
{
 TSL2561_DnLooking = 0, // ADDR SEL pin is tied low = down-pointing sensor; WRITE 0x52, READ 0x53
 TSL2561_UpLooking // ADDR SEL pin is tied high = up-pointing sensor WRITE 0x92, READ 0x93
};

// union to allow reading in Lo and Hi bytes of irradiance, and reading out whole word
//typedef 
struct irrData {
    union irrValue {
        struct irrBytes {
            uint8_t irrLoByte, irrHiByte;
        };
        struct irrWord {
            uint16_t irrWholeWord;
        };
    };
    uint16_t irrMultiplier;
};


// functions
bool getIrrReading (struct irrData *rd);


#endif /* TSL2561_H_ */