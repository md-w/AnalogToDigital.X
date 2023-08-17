
// refer file:///C:/Program%20Files/Microchip/xc8/v2.40/docs/chips/18f4520.html

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 2         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config CCP2MX = PORTBE  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "tick.h"
#include "displayKeyTickCPU.h"
#include "eeprom_wrapper.h"

#define PSWD                        10

#define ANALOG_FULLSCALE   1500

#define M_MAX   ((char)120)
#define M_MIN   ((char)-120)

#define C_MAX   ((char)120)
#define C_MIN   ((char)-120)

#define SET_POINT_HI_MAX  (ANALOG_FULLSCALE/10 - 1)
#define SET_POINT_HI_MIN  (1)

#define SET_POINT_LO_MAX  (ANALOG_FULLSCALE/10 - 1)
#define SET_POINT_LO_MIN  (1)

typedef struct _structControllerSP {
    unsigned char ucLoSetPoint;
    unsigned char ucHiSetPoint;
    char cMCh1;
    char cCCh1;
} STRUCT_CONTROLLER_SP;

unsigned char dispMainState = 0;
unsigned char dispSubState = 0;
unsigned char statusByte0 = 0;
unsigned char statusByte1 = 0;
unsigned char sec60Counter = 60;
unsigned int fpsCounter = 0;
unsigned int outputCounter = 2;
unsigned int fps = 0;
unsigned char ucPassword = PSWD + 5;
bool interruptFlag = 0;
bool interruptFlagDisp = 0;
bool outputLatch = 0;
bool alarmLatch = 0;
STRUCT_CONTROLLER_SP SetPoint;

#define RELAY_LAT LATBbits.LATB3
#define LED_LAT_3 LATCbits.LATC5
#define LED_LAT_2 LATCbits.LATC6
#define LED_LAT_1 LATCbits.LATC7

#define RELAY_TRIS TRISBbits.TRISB3
#define LED_TRIS_3 TRISCbits.TRISC5
#define LED_TRIS_2 TRISCbits.TRISC6
#define LED_TRIS_1 TRISCbits.TRISC7

#define _XTAL_FREQ 20000000

void DELAY_milliseconds(uint16_t milliseconds) {
    while (milliseconds--) {
        __delay_ms(1);
    }
}

void INTERRUPT_Initialize(void) {
    // Disable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    RCONbits.IPEN = 0;
}

void __interrupt() INTERRUPT_InterruptManagerLow(void) {
    // interrupt handler
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        // reload TMR0
        TMR0L = timer0ReloadVal;
        displayISR();
        // clear the TMR0 interrupt flag
        INTCONbits.TMR0IF = 0;
    } else if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        fpsCounter++;
        interruptFlag = 1;
        INTCONbits.INT0IF = 0;
    }
}


#define INTERRUPT_GlobalInterruptEnable() (INTCONbits.GIE = 1)
#define INTERRUPT_GlobalInterruptDisable() (INTCONbits.GIE = 0)
#define INTERRUPT_GlobalInterruptHighEnable() (INTCONbits.GIEH = 1)
#define INTERRUPT_GlobalInterruptHighDisable() (INTCONbits.GIEH = 0)
#define INTERRUPT_GlobalInterruptLowEnable() (INTCONbits.GIEL = 1)
#define INTERRUPT_GlobalInterruptLowDisable() (INTCONbits.GIEL = 0)
#define INTERRUPT_PeripheralInterruptEnable() (INTCONbits.PEIE = 1)
#define INTERRUPT_PeripheralInterruptDisable() (INTCONbits.PEIE = 0)

void SYSTEM_Initialize(void) {

    INTERRUPT_Initialize();
    initDisplay();
    initTimer0();
    initExternalInterrupt();
}

void initVariables(void) {
    readByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
    if ((SetPoint.ucHiSetPoint < SET_POINT_HI_MIN) || (SetPoint.ucHiSetPoint > SET_POINT_HI_MAX)) {
        SetPoint.ucHiSetPoint = 100;
    }
    if ((SetPoint.ucLoSetPoint < SET_POINT_HI_MIN) || (SetPoint.ucLoSetPoint > SET_POINT_HI_MAX)) {
        SetPoint.ucLoSetPoint = 10;
    }
    if ((SetPoint.cMCh1 < M_MIN) || (SetPoint.cMCh1 > M_MAX)) {
        SetPoint.cMCh1 = 0;
    }
    if ((SetPoint.cCCh1 < C_MIN) || (SetPoint.cCCh1 > C_MAX)) {
        SetPoint.cCCh1 = 0;
    }
}

void main(void) {
    SYSTEM_Initialize();
    LED_TRIS_3 = 0;
    LED_TRIS_2 = 0;
    LED_TRIS_1 = 0;
    RELAY_TRIS = 0;
    RELAY_LAT = 0;
    INTERRUPT_GlobalInterruptEnable();
    initVariables();
    initADC();
    while (1) {
        statusByte0 = 0x00;
        statusByte1 = 0x00;
        adcTask();
        if (tick1000mSec) {
            tick1000mSec = 0;
        }
        if (tick250mSec) {
            tick250mSec = 0;
            interruptFlagDisp = 0;
            if (interruptFlag) {
                interruptFlag = 0;
                interruptFlagDisp = 1;
            }
        }

        if (interruptFlagDisp) {
            statusByte1 |= 0x01;
        }
        if (outputLatch) {
            RELAY_LAT = 1;
            statusByte1 |= 0x02;
        } else {
            RELAY_LAT = 0;
        }
//        if (alarmLatch) {
//            
//        } else {
//            
//        }
        int iRoomTemp = (((long)iADCValCh1 + (long)SetPoint.cCCh1)*((long)ANALOG_FULLSCALE+(long)SetPoint.cMCh1))/(long)10230;
        //        int iRoomTemp = ((long)iADCValCh1*(long)ANALOG_FULLSCALE)/(long)10230;
        switch (dispMainState) {
            case 0:
                displayInt(iRoomTemp, 0);
                if (keyDown & DNKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    outputLatch = 0;
                }
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                }
                break;
            case 1: //enter PRO
                statusByte0 |= 0x01;
                digitAssign(SEG_P, 2);
                digitAssign(SEG_R, 1);
                digitAssign(SEG_O, 0);
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                }
                break;
            case 2: //PRO
                statusByte0 |= 0x01;
                switch (dispSubState) {
                    case 0:
                        statusByte0 |= 0x04;
                        checkAndInrDcrChar(&SetPoint.ucHiSetPoint, SET_POINT_HI_MAX, SET_POINT_HI_MIN);
                        displayInt(SetPoint.ucHiSetPoint, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        statusByte0 |= 0x08;
                        checkAndInrDcrChar(&SetPoint.ucLoSetPoint, SetPoint.ucHiSetPoint, SET_POINT_LO_MIN);
                        displayInt(SetPoint.ucLoSetPoint, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;
                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 3: //enter CAL
                statusByte0 |= 0x02;
                digitAssign(SEG_C, 2);
                digitAssign(SEG_A, 1);
                digitAssign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    selectedIndx = 0;
                }
                break;
            case 4: //CAL
                statusByte0 |= 0x02;
                switch (dispSubState) {
                    case 0:
                        statusByte0 |= 0x04;
                        checkAndInrDcrSignedChar(&SetPoint.cMCh1, M_MAX, M_MIN);
                        displaySignedInt(SetPoint.cMCh1, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        statusByte0 |= 0x08;
                        checkAndInrDcrSignedChar(&SetPoint.cCCh1, C_MAX, C_MIN);
                        displaySignedInt(SetPoint.cCCh1, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;
                        
                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            default:
                dispMainState = 0;
                break;
        }
//        if (statusByte0 & 0x01) {
//            LED_LAT_1 = 0;
//        } else {
//            LED_LAT_1 = 1;
//        }
        if (statusByte1 & 0x01) {
            LED_LAT_2 = 0;
        } else {
            LED_LAT_2 = 1;
        }
        if (statusByte1 & 0x02) {
            LED_LAT_3 = 0;
        } else {
            LED_LAT_3 = 1;
        }
        statusByte0 |= statusByte0 << 4;
        directAssign(statusByte0, 3);
        display();
        ClrWdt();
    }
}
