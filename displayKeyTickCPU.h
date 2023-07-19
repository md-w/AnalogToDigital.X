#ifndef DISPLAYKEYTICKCPU_H
#define	DISPLAYKEYTICKCPU_H
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define MODULE_SIZE 4
#define MAX_DIGIT 4

// #define DISPLAY_LONG		1
#define DISPLAY_INT 1
#define DISPLAY_SIGNED_INT 1
#define DISPLAY_CHAR_INPOS 1
// #define CHECK_AND_INR_DCR_INT	1
#define CHECK_AND_INR_DCR_INT_SHIFT 1
#define CHECK_AND_INR_DCR_CHAR 1
#define CHECK_AND_INR_DCR_SIGNED_CHAR 1

#define PROKEY_MASK 0x08
#define ENTKEY_MASK 0x04
#define UPKEY_MASK 0x02
#define DNKEY_MASK 0x01

#define DISP_DATA_PORT LATD
#define DISP_MUX_PORT LATB
#define DISP_DATA_PORT_TRIS TRISD

#define DISP_MUX_PORT_DIGIT3 LATBbits.LATB4
#define DISP_MUX_PORT_DIGIT2 LATBbits.LATB5
#define DISP_MUX_PORT_DIGIT1 LATBbits.LATB6
#define DISP_MUX_PORT_DIGIT0 LATBbits.LATB7

#define DISP_MUX_PORT_TRIS_DIGIT3 TRISBbits.TRISB4
#define DISP_MUX_PORT_TRIS_DIGIT2 TRISBbits.TRISB5
#define DISP_MUX_PORT_TRIS_DIGIT1 TRISBbits.TRISB6
#define DISP_MUX_PORT_TRIS_DIGIT0 TRISBbits.TRISB7


//#define DISP_MUX_PORT_TRIS TRISB
#define DISP_KEY_COM_BIT PORTBbits.RB1
#define DISP_KEY_COM_BIT_LAT LATBbits.LATB1
#define DISP_KEY_COM_BIT_TRIS TRISBbits.TRISB1
#define DISP_KEY_COM_BIT_PULLUP_ INTCON2bits.RBPU

#define SEG_A 10
#define SEG_B 11
#define SEG_C 12
#define SEG_D 13
#define SEG_E 14
#define SEG_F 15
#define SEG_G 16
#define SEG_H 17
#define SEG_I 18
#define SEG_J 19
#define SEG_K 20
#define SEG_L 21
#define SEG_M 22
#define SEG_N 23
#define SEG_O 24
#define SEG_P 25
#define SEG_Q 26
#define SEG_R 27
#define SEG_S 28
#define SEG_T 29
#define SEG_U 30
#define SEG_V 31
#define SEG_W 32
#define SEG_X 33
#define SEG_Y 34
#define SEG_Z 35
#define SEG_SPACE 36
#define SEG_MINUS 37
#define SEG_QUESTION 38

#define KEY_PRESS_COUNT 50 // 12ms*KEY_PRESS_COUNT = 250ms

extern unsigned char *pucReadDispBuffer;
extern unsigned int uiTemp;
extern unsigned char keyDown;
extern unsigned char keyHold;
extern unsigned char selectedIndx;

//extern bool tick100mSec;
extern bool tick250mSec;
extern bool tick500mSec;
extern bool tick1000mSec;
extern bool tick1000mSec2;
extern bool tick3000mSec;
//extern bool tick3000mSec2;
//extern bool tick36000mSec1;
//extern bool tick36000mSec2;
//extern bool tick3min;
//extern bool tick6min;
//extern bool bToggleTick6min;
extern bool bToggleBitFast;
extern bool bToggleBitSlow;
    
void displayISR(void);
void initDisplay(void);
void display(void);
void clear(void);
void clearDisplay(void);
void directOrWithDisplay(unsigned char segmentValue, unsigned char digitNo);
void directAndWithDisplay(unsigned char segmentValue, unsigned char digitNo);
void directAssign(unsigned char segmentValue, unsigned char digitNo);
void digitAssign(unsigned char segmentValue, unsigned char digitNo);
#ifdef DISPLAY_LONG
void displayLong(unsigned long hexData, unsigned char dotPosition);
#endif
#ifdef DISPLAY_INT
void displayInt(unsigned int hexData, unsigned char dotPosition);
#endif
#ifdef DISPLAY_SIGNED_INT
void displaySignedInt(int hexData, unsigned char dotPosition);
#endif
#ifdef DISPLAY_CHAR_INPOS
void displayCharInPos(unsigned char hexData, unsigned char pos);
#endif
#ifdef CHECK_AND_INR_DCR_SIGNED_CHAR
unsigned char checkAndInrDcrSignedChar(char *value, char upperLimit, char lowerLimit);
#endif
#ifdef CHECK_AND_INR_DCR_CHAR
unsigned char checkAndInrDcrChar(unsigned char *value, unsigned char upperLimit, unsigned char lowerLimit);
#endif
#ifdef CHECK_AND_INR_DCR_INT
void checkAndInrDcrInt(unsigned int *value, unsigned int upperLimit, unsigned int lowerLimit);
#endif
#ifdef CHECK_AND_INR_DCR_INT_SHIFT
void checkAndInrDcrIntShift(unsigned int *value);
#endif

unsigned char minusOneWithModulo(unsigned char val, const unsigned char modulo_val);
unsigned char plusOneWithModulo(unsigned char val, const unsigned char modulo_val);

#endif	/* DISPLAYKEYTICKCPU_H */

