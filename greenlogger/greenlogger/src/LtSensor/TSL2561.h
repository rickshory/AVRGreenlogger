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

// controls within registers
#define	TSL2561_CMD_BIT 0b10000000 // this alone specifies none of the following, i.e. it's a byte command
#define	TSL2561_CLI_BIT 0b01000000 // write 1 to clear any interrupt
#define	TSL2561_WRD_BIT 0b00100000 // WORD protocol
#define	TSL2561_BLK_BIT 0b00010000 // BLOCK protocol
// bits 3:0 specify the register (listed above) to apply this command to

#define	TSL2561_PWR_ON 0x03 // write this to the CONTROL register to power up the device
#define	TSL2561_PWR_OFF 0x00 // write this to the CONTROL register to power down the device

// bits of the TIMING register
#define	TSL2561_GAIN_BIT 0b0001000 // 0 = low gain (1×); 1 = high gain (16×)
//INTEG FIELD VALUE	SCALE	NOMINAL INTEGRATION TIME
//	00				0.034		13.7 ms
//	01				0.252		101 ms
//	10				1			402 ms
//	11				??			N/A
#define	TSL2561_INTEG_BIT_LO 0b0000001 //
#define	TSL2561_INTEG_BIT_HI 0b0000010 //



// options
//#define TSL2561_CHANNEL_BROADBAND	0xac	// broadband channel
//#define TSL2561_CHANNEL_INFRARED	0xae	// infrared channel
#define TSL2561_CHANNEL_BROADBAND	0x0c	// broadband channel
#define TSL2561_CHANNEL_INFRARED	0x0e	// infrared channel


enum TSL2561_Opt_UpDn
{
 TSL2561_DnLooking = 0, // ADDR SEL pin is tied low = down-pointing sensor; WRITE 0x52, READ 0x53
 TSL2561_UpLooking // ADDR SEL pin is tied high = up-pointing sensor WRITE 0x92, READ 0x93
};

// union allows writing in Lo and Hi bytes of irradiance, and reading out whole word
typedef struct {
    union {
        struct {
            uint8_t irrLoByte, irrHiByte; // this is the correct endian-ness
        };
        struct  {
            uint16_t irrWholeWord;
        };
    };
    uint16_t irrMultiplier;
} irrData;

irrData irrReadings[4];
//typedef 
/*
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
*/
/*
struct irrData {
	uint8_t irrLoByte, irrHiByte;
	uint16_t irrMultiplier;
};
*/
// functions
bool getIrrReading (uint8_t sensPosition, uint8_t sensChannel, irrData *rd);


#endif /* TSL2561_H_ */