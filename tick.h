/* 
 * File:   tick.h
 * Author: das_m
 *
 * Created on March 29, 2023, 11:12 PM
 */

#ifndef TICK_H
#define	TICK_H
#include <xc.h>

extern int16_t iADCValCh1;
extern int16_t iADCValCh2;
extern volatile uint8_t timer0ReloadVal;

void initTimer0(void);
void initTimer3(void);
void initExternalInterrupt(void);
void initPWM(void);
void setPWMDuty(unsigned int iVal);
void TickGenerator(void);
void initLVD(void);
void initADC(void);
unsigned int getAdcValue(unsigned char ucChannel);
unsigned int getAdcValue1(void);
void selectChannel(unsigned char ucChannel);
void adcTask(void);
unsigned int GetFreq(void);
void SerialInit(void);
#endif	/* TICK_H */

