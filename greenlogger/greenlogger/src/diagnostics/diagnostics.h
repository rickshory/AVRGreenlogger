/*
 * diagnostics.h
 *
 * Created: 2/1/2012 2:23:38 PM
 *  Author: rshory
 */ 


#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_

#define CT_10MS 1250
//#define TOGGLE_INTERVAL 100

//Define functions
//======================
void setupDiagnostics (void);
void ioinit(void);      //Initializes IO
void setupTimer3_10ms( void );
void setupTimer3_1shot( void );
//void delay_ms(uint16_t x); //General purpose delay
//======================

#endif /* DIAGNOSTICS_H_ */