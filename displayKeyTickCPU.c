#include "displayKeyTickCPU.h"

unsigned char ucDispBuffer1[MODULE_SIZE]; // = {0x3F,0x06,0x5B,0x4F,0x66,0x6D};//,0x00,0x00,0x00};
unsigned char ucDispBuffer2[MODULE_SIZE]; // = {0x7D,0x07,0x7F,0x6F,0x77,0x7C};//,0x00,0x00,0x00};
unsigned char *pucDispBuffer; // disp buffer for writing
unsigned char *pucReadDispBuffer = &ucDispBuffer1[0];
volatile bool bUseDispBuffer1ForReading;
volatile bool bRequestBufferReleaseForWriting;
// bit					bBufferReleasedForWriting;
unsigned int uiTemp = 0;
unsigned char keyDown;
unsigned char keyHold;
unsigned char selectedIndx = 0;

bool tick250mSec = 0;
bool tick500mSec = 0;
bool tick1000mSec = 0;
bool tick1000mSec2 = 0;
bool tick3000mSec = 0;

bool bToggleBitFast = 0;
bool bToggleBitSlow = 0;

const unsigned char int2sevenSeg[] = {
    0x3F, // 0  '0'
    0x06, // 1  '1'	0000 0110
    0x5B, // 2  '2'
    0x4F, // 3  '3'
    0x66, // 4  '4'
    0x6D, // 5  '5'
    0x7D, // 6  '6'
    0x07, // 7  '7'
    0x7F, // 8  '8'
    0x6F, // 9  '9'
    0x77, // 10 'A'
    0x7C, // 11 'b'
    0x39, // 12 'C'
    0x5E, // 13 'd'
    0x79, // 14 'E'
    0x71, // 15 'F'
    0x3D, // 16 '?'	G
    0x74, // 17 'h'
    0x04, // 18 'i'	0000 0100
    0x1E, // 19 '?'	J
    0x36, // 20 '?'	k
    0x38, // 21 'L'
    0x53, // 22 '?'	M
    0X54, // 23 'n'
    0x5C, // 24 'o'
    0x73, // 25 'P'
    0x53, // 26 '?'	q
    0x50, // 27 'r'
    0x6D, // 28 '?'	S
    0x78, // 29 't'
    0x3E, // 30 'U'	0011 1110
    0x1C, // 31 'v'	0001 1100
    0x53, // 32 '?'	W
    0x53, // 33 '?'	X
    0x6E, // 34 'y'
    0x53, // 35 '?'	Z
    0x00, // 36 ' '
    0x40, // 37 '-'
    0x53, // 38 '?'
};

void displayISR(void) // 2msec time period
{
    static unsigned int uiTickCount = 0;
    static unsigned char ucDispIndx = 0;
    static unsigned char keyStatus = 0;

    static unsigned char keyStateSaved = 0;
    static unsigned char keyPressCounter = 0;

    if (uiTickCount < 500 - 7 - 1) { //	
        uiTickCount++;
    } else {
        uiTickCount = 0;
        tick1000mSec = 1;
        tick1000mSec2 = 1;
    }
    if ((uiTickCount % 250) == 0) {
        bToggleBitSlow = !bToggleBitSlow;
        tick500mSec = 1;
    }
    if ((uiTickCount % 125) == 0) {
        bToggleBitFast = !bToggleBitFast;
        tick250mSec = 1;
    }

    //    if (ucDispIndx == 0) {
    //        ucDispIndxPrevious = MODULE_SIZE - 1;
    //    } else {
    //        ucDispIndxPrevious = ucDispIndx - 1;
    //    }
    //    
    //    DISP_MUX_PORT |= (0x80 >> ucDispIndxPrevious);

    DISP_MUX_PORT_DIGIT0 = 1;
    DISP_MUX_PORT_DIGIT1 = 1;
    DISP_MUX_PORT_DIGIT2 = 1;
    DISP_MUX_PORT_DIGIT3 = 1;


    DISP_DATA_PORT = ~(*(pucReadDispBuffer + ucDispIndx));
    //    DISP_MUX_PORT &= ~(0x80 >> ucDispIndx);
    if (0 == ucDispIndx) {
        DISP_MUX_PORT_DIGIT0 = 0;
    } else if (1 == ucDispIndx) {
        DISP_MUX_PORT_DIGIT1 = 0;
    } else if (2 == ucDispIndx) {
        DISP_MUX_PORT_DIGIT2 = 0;
    } else if (3 == ucDispIndx) {
        DISP_MUX_PORT_DIGIT3 = 0;
    }
    if (!DISP_KEY_COM_BIT)
        keyStatus |= (0x01 << ucDispIndx);
    else
        keyStatus &= ~(0x01 << ucDispIndx);

    ucDispIndx = plusOneWithModulo(ucDispIndx, MODULE_SIZE);
    if (ucDispIndx == 0) {
        //        ucDispIndx = 0;
        // for a round trip operation take keyboard decision total time = MODULE_SIZE*2ms = 12ms
        // keyDown = keyStatus;
        keyDown |= (~keyStateSaved & keyStatus);
        keyStateSaved = keyStatus;
        if (keyStatus) {
            if (--keyPressCounter == 0) {
                keyPressCounter = KEY_PRESS_COUNT;
                keyHold |= keyStatus;
            }
        } else {
            keyPressCounter = KEY_PRESS_COUNT;
        }

        if (bRequestBufferReleaseForWriting) {
            if (bUseDispBuffer1ForReading) { // was reading from buffer1 now release for writing
                bUseDispBuffer1ForReading = 0;
                pucDispBuffer = (unsigned char *) &ucDispBuffer1;
                pucReadDispBuffer = (unsigned char *) &ucDispBuffer2;
            } else {
                bUseDispBuffer1ForReading = 1;
                pucDispBuffer = (unsigned char *) &ucDispBuffer2;
                pucReadDispBuffer = (unsigned char *) &ucDispBuffer1;
            }
            bRequestBufferReleaseForWriting = 0;
        }
    }

    // if(!bRequestBufferReleaseForWriting)
    // bBufferReleasedForWriting = 1;
}

void initDisplay(void) {
    DISP_DATA_PORT_TRIS = 0x00;
    DISP_MUX_PORT_TRIS_DIGIT3 = 0;
    DISP_MUX_PORT_TRIS_DIGIT2 = 0;
    DISP_MUX_PORT_TRIS_DIGIT1 = 0;
    DISP_MUX_PORT_TRIS_DIGIT0 = 0;
    //    DISP_MUX_PORT_TRIS = 0x00;
    DISP_KEY_COM_BIT_TRIS = 1;
//    DISP_KEY_COM_BIT = 1;
    DISP_KEY_COM_BIT_LAT = 1; // enable pullup
    DISP_KEY_COM_BIT_PULLUP_ = 0;
    //
    //    DISP_DATA_PORT = 0xFF;
    //    DISP_MUX_PORT = 0x00;
    bUseDispBuffer1ForReading = 1;
    pucReadDispBuffer = (unsigned char *) &ucDispBuffer1;
    pucDispBuffer = (unsigned char *) &ucDispBuffer2;
    bRequestBufferReleaseForWriting = 0;
    // bBufferReleasedForWriting = 0;
}

void display(void) {
    // while(bRequestBufferReleaseForWriting)	//sync with dispISR
    //	;
    // unsigned int i;
    bRequestBufferReleaseForWriting = 1;
    while (1) {
        if (!bRequestBufferReleaseForWriting) // sync with dispISR
            break;
    }
}

void clear(void) {
    unsigned char i;
    for (i = 0; i < MODULE_SIZE; i++) {
        *(pucDispBuffer + i) = 0x00;
    }
}

void clearDisplay(void) {
    clear();
    display();
}

void directOrWithDisplay(unsigned char segmentValue, unsigned char digitNo) {
    if (digitNo < MODULE_SIZE)
        *(pucDispBuffer + digitNo) |= segmentValue;
}

void directAndWithDisplay(unsigned char segmentValue, unsigned char digitNo) {
    if (digitNo < MODULE_SIZE)
        *(pucDispBuffer + digitNo) &= segmentValue;
}

void directAssign(unsigned char segmentValue, unsigned char digitNo) {
    if (digitNo < MODULE_SIZE)
        *(pucDispBuffer + digitNo) = segmentValue;
}

void digitAssign(unsigned char segmentValue, unsigned char digitNo) {
    if (digitNo < MODULE_SIZE)
        *(pucDispBuffer + digitNo) = int2sevenSeg[segmentValue];
}
#ifdef DISPLAY_LONG

void displayLong(unsigned long hexData, unsigned char dotPosition) {
    register unsigned char temp;
    register unsigned char i = 0;
    clear();
    // i = 0;
    do {
        if (i >= MODULE_SIZE)
            break; // break if
        temp = (unsigned char) ((unsigned long) hexData % 10);
        digitAssign(temp, i + 1); // digitAssign(temp,i);
        // displayBuffer[(MODULE_SIZE - 1) - i] = int2sevenSeg[(unsigned char)temp];
        hexData = (hexData - (unsigned long) temp) / 10;
        if (dotPosition < MODULE_SIZE)
            if (i >= dotPosition)
                if (hexData == 0)
                    break;
        i++;
    } while (i <= 10);

    if (dotPosition < MODULE_SIZE)
        directOrWithDisplay(0x80, dotPosition + 1); // directOrWithDisplay(0x80,dotPosition);
    // displayBuffer[(MODULE_SIZE - 1) - dotPosition] |= 0x80;
    // display();
}

void checkAndInrDcrLong(unsigned long *value) {

    unsigned char i;
    unsigned long tempUpCount;
    unsigned char digitValue[MAX_DIGIT];

    tempUpCount = *value;
    for (i = 0; i < MAX_DIGIT; i++) {
        digitValue[i] = (unsigned char) ((unsigned long) tempUpCount % 10);
        tempUpCount = (tempUpCount - (unsigned long) digitValue[i]) / 10;
    }

    if ((keyDown & DNKEY_MASK) || (keyHold & DNKEY_MASK)) {
        keyDown = 0;
        keyHold = 0;
        if (selectedIndx < MAX_DIGIT - 1) {
            selectedIndx++;
        } else {
            selectedIndx = 0;
        }
    }
    if (selectedIndx < MAX_DIGIT) {
        if ((keyDown & UPKEY_MASK) || (keyHold & UPKEY_MASK)) {
            keyDown = 0;
            keyHold = 0;
            if (digitValue[selectedIndx] < 9) {
                digitValue[selectedIndx]++;
            } else {
                digitValue[selectedIndx] = 0;
            }
        }
    }

    tempUpCount = 0;
    for (i = MAX_DIGIT - 1; i >= 1; i--) {
        tempUpCount += digitValue[i];
        tempUpCount = (unsigned long) tempUpCount * 10;
    }
    tempUpCount += digitValue[0];
    *value = tempUpCount;
}
#endif
#ifdef DISPLAY_INT

void displayInt(unsigned int hexData, unsigned char dotPosition) {
    unsigned char temp;
    unsigned char i = 0;
    clear();
    // i = 0;
    do {
        if (i >= MODULE_SIZE)
            break; // break if
        temp = (unsigned char) ((unsigned int) hexData % (unsigned int) 10);
        digitAssign(temp, i); // digitAssign(temp,i);
        //			displayBuffer[(MODULE_SIZE - 1) - i] = int2sevenSeg[(unsigned char)temp];
        hexData = (unsigned int) (hexData - (unsigned int) temp) / (unsigned int) 10;
        if (dotPosition < MODULE_SIZE)
            if (i >= dotPosition)
                if (hexData == 0)
                    break;
        i++;
    } while (i <= 5);

    if (dotPosition < MODULE_SIZE)
        directOrWithDisplay(0x80, dotPosition); // directOrWithDisplay(0x80,dotPosition);
    // displayBuffer[(MODULE_SIZE - 1) - dotPosition] |= 0x80;
    // display();
}
#endif
#ifdef DISPLAY_SIGNED_INT

void displaySignedInt(int hexData, unsigned char dotPosition) {
    unsigned char temp;
    unsigned char i = 0;
    bool isNegative = 0;
    // unsigned int hexData;
    clear();
    // i = 0;
    if (hexData < 0) {
        hexData = -hexData;
        isNegative = 1;
    } else {
        hexData = hexData;
        isNegative = 0;
    }
    do {
        if (i >= MODULE_SIZE)
            break; // break if
        temp = (unsigned char) ((unsigned int) hexData % (unsigned int) 10);
        digitAssign(temp, i); // digitAssign(temp,i);
        //			displayBuffer[(MODULE_SIZE - 1) - i] = int2sevenSeg[(unsigned char)temp];
        hexData = (unsigned int) ((unsigned int) hexData - (unsigned int) temp) / (unsigned int) 10;
        if (dotPosition < MODULE_SIZE)
            if (i >= dotPosition)
                if (hexData == 0)
                    break;
        i++;
    } while (i <= 5);

    if (isNegative)
        digitAssign(SEG_MINUS, i); // digitAssign(SEG_MINUS, i + 1);

    if (dotPosition < MODULE_SIZE)
        directOrWithDisplay(0x80, dotPosition); // directOrWithDisplay(0x80,dotPosition);
    // displayBuffer[(MODULE_SIZE - 1) - dotPosition] |= 0x80;
    // display();
}
#endif
#ifdef DISPLAY_CHAR_INPOS

void displayCharInPos(unsigned char hexData, unsigned char pos) {
    register unsigned char temp;

    directAssign(0x00, pos); // clear
    directAssign(0x00, pos + 1); // clear

    temp = (unsigned char) ((unsigned char) hexData % 10);
    digitAssign(temp, pos);
    // displayBuffer[(MODULE_SIZE - 1) - pos] = int2sevenSeg[(unsigned char)temp];
    hexData = (hexData - (unsigned char) temp) / 10;
    if (hexData != 0)
        digitAssign(hexData, (pos + 1));
    // displayBuffer[(MODULE_SIZE - 1) - (pos+1)] = int2sevenSeg[(unsigned char)hexData];
    // display();
}
#endif
#ifdef CHECK_AND_INR_DCR_CHAR

unsigned char checkAndInrDcrChar(unsigned char *value, unsigned char upperLimit, unsigned char lowerLimit) {
    if ((keyDown & UPKEY_MASK) || (keyHold & UPKEY_MASK)) { // up key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) < upperLimit)
            (*value)++;
        else
            (*value) = upperLimit;
        return 1;
    }
    if ((keyDown & DNKEY_MASK) || (keyHold & DNKEY_MASK)) { // down key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) > lowerLimit)
            (*value)--;
        else
            (*value) = lowerLimit;
        return 1;
    }
    return 0;
}
#endif
#ifdef CHECK_AND_INR_DCR_SIGNED_CHAR

unsigned char checkAndInrDcrSignedChar(char *value, char upperLimit, char lowerLimit) {
    if ((keyDown & UPKEY_MASK) || (keyHold & UPKEY_MASK)) { // up key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) < upperLimit)
            (*value)++;
        else
            (*value) = upperLimit;
        return 1;
    }
    if ((keyDown & DNKEY_MASK) || (keyHold & DNKEY_MASK)) { // down key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) > lowerLimit)
            (*value)--;
        else
            (*value) = lowerLimit;
        return 1;
    }
    return 0;
}
#endif
#ifdef CHECK_AND_INR_DCR_INT

void checkAndInrDcrInt(unsigned int *value, unsigned int upperLimit, unsigned int lowerLimit) {
    register unsigned int tempInt;

    if (keyDown & UPKEY_MASK) { // up key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) < upperLimit)
            (*value)++;
        else
            (*value) = upperLimit;
        return;
    }
    if (keyDown & DNKEY_MASK) { // down key
        keyDown = 0x00;
        keyHold = 0x00;
        if ((*value) > lowerLimit)
            (*value)--;
        else
            (*value) = lowerLimit;
        return;
    }
#ifdef CHECK_AND_INR_DCR_INT_SMOOTH
    if (keyHold & UPKEY_MASK) { // up key
        keyHold = 0x00;
        if ((*value) < upperLimit)
            (*value)++;
        else
            (*value) = upperLimit;
        return;
    }
    if (keyHold & DNKEY_MASK) { // down key
        keyHold = 0x00;
        if ((*value) > lowerLimit)
            (*value)--;
        else
            (*value) = lowerLimit;
        return;
    }
#else
    if (keyHold & UPKEY_MASK) { // up key
        keyHold = 0x00;
        if ((*value) < (upperLimit - 10)) {
            tempInt = 10 - (*value) % 10;
            tempInt = (tempInt != 0 ? tempInt : 10);
            (*value) += tempInt;
        } else
            (*value) = upperLimit;
        return;
    }
    if (keyHold & DNKEY_MASK) { // down key
        keyHold = 0x00;
        if ((*value) > (lowerLimit + 10)) {
            tempInt = (*value) % 10;
            tempInt = (tempInt != 0 ? tempInt : 10);
            (*value) = (*value) - tempInt;
        } else
            (*value) = lowerLimit;
        return;
    }
#endif
}
#endif
#ifdef CHECK_AND_INR_DCR_INT_SHIFT

void checkAndInrDcrIntShift(unsigned int *value) {

    unsigned char i;
    unsigned int tempUpCount;
    unsigned char digitValue[MAX_DIGIT];

    tempUpCount = *value;
    for (i = 0; i < MAX_DIGIT; i++) {
        digitValue[i] = (unsigned char) ((unsigned int) tempUpCount % 10);
        tempUpCount = (tempUpCount - (unsigned int) digitValue[i]) / 10;
    }

    if ((keyDown & DNKEY_MASK) || (keyHold & DNKEY_MASK)) {
        keyDown = 0;
        keyHold = 0;
        if (selectedIndx < MAX_DIGIT - 1) {
            selectedIndx++;
        } else {
            selectedIndx = 0;
        }
    }
    if ((keyDown & UPKEY_MASK) || (keyHold & UPKEY_MASK)) {
        keyDown = 0;
        keyHold = 0;
        if (digitValue[selectedIndx] < 9) {
            digitValue[selectedIndx]++;
        } else {
            digitValue[selectedIndx] = 0;
        }
    }

    tempUpCount = 0;
    for (i = MAX_DIGIT - 1; i >= 1; i--) {
        tempUpCount += digitValue[i];
        tempUpCount = (unsigned int) tempUpCount * 10;
    }
    tempUpCount += digitValue[0];
    *value = tempUpCount;
}
#endif

unsigned char minusOneWithModulo(unsigned char val, const unsigned char modulo_val) {
    if (val > 0) {
        return --val;
    }
    return modulo_val - 1;

}

unsigned char plusOneWithModulo(unsigned char val, const unsigned char modulo_val) {
    if (val < modulo_val - 1) {
        return ++val;
    }
    return 0;
}