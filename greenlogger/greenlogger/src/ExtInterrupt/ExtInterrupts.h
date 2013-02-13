/*
 * ExtInterrupts.h
 *
 * Created: 4/27/2012 2:32:42 PM
 *  Author: rshory
 */ 


#ifndef EXTINTERRUPTS_H_
#define EXTINTERRUPTS_H_

void stayRoused(uint16_t sec);
void endRouse(void);
void keepBluetoothPowered(uint8_t sec);
void shutDownBluetooth(void);



#endif /* EXTINTERRUPTS_H_ */