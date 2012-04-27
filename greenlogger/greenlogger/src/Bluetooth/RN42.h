/*
 * RN41.h
 *
 * Created: 4/25/2012 10:49:34 AM
 *  Author: rshory
 */ 


#ifndef RN41_H_
#define RN41_H_

#include "compiler.h"

extern inline void BT_power_on(void);
extern inline void BT_power_off(void);
extern inline void BT_baud_9600(void);
extern inline void BT_baud_115k(void);
extern inline void BT_connection_setInput(void);
extern inline bool BT_connected(void);



#endif /* RN41_H_ */