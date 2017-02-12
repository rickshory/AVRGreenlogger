/*
 * RN42.h
 *
 * Created: 4/25/2012 10:49:34 AM
 *  Author: rshory
 */ 


#ifndef RN42_H_
#define RN42_H_

#include "compiler.h"

extern inline void BT_power_on(void);
extern inline void BT_power_off(void);
extern inline void BT_baud_9600(void);
extern inline void BT_baud_115k(void);
extern inline bool BT_connected(void);
extern inline bool BT_powered(void);
void checkForBTCommands (void);
void BT_dataDump(char* stOpt);
uint16_t cyPerRTCSqWave(void);

// union allows writing in Lo and Hi bytes of ADC, and reading out whole word
typedef struct {
    union {
        struct {
            uint8_t loByte, hiByte; // this is the correct endian-ness
        };
        struct  {
            uint16_t wholeWord;
        };
    };
} twoByteData;

#endif /* RN42_H_ */