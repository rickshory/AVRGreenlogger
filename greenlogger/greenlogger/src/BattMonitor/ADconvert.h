/*
 * ADconvert.h
 *
 * Created: 2/25/2012 9:10:33 AM
 *  Author: rshory
 */ 


#ifndef ADCONVERT_H_
#define ADCONVERT_H_

// union allows writing in Lo and Hi bytes of ADC, and reading out whole word
typedef struct {
    union {
        struct {
            uint8_t adcLoByte, adcHiByte; // this is the correct endian-ness
        };
        struct  {
            uint16_t adcWholeWord;
        };
    };
    uint16_t adcMultiplier;
} adcData;


// functions
uint8_t readCellVoltage (adcData *cellV);

#endif /* ADCONVERT_H_ */