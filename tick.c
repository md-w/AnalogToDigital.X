#include "tick.h"

uint16_t filterCount = 0;
uint16_t filter = 0;
uint8_t ucScanChannel = 0;
int16_t iADCValCh1;
int16_t iADCValCh2;
volatile uint8_t timer0ReloadVal;
#define FILTER_POINTS 32

void initTimer0(void) {
    /*
    T0CON:
    bit 7 
        TMR0ON: Timer0 On/Off Control bit
        1 = Enables Timer0
        0 = Stops Timer0
    bit 6 
        T08BIT: Timer0 8-bit/16-bit Control bit
        1 = Timer0 is configured as an 8-bit timer/counter
        0 = Timer0 is configured as a 16-bit timer/counter
    bit 5
        T0CS: Timer0 Clock Source Select bit
        1 = Transition on T0CKI pin
        0 = Internal instruction cycle clock (CLKO)
    bit 4
        T0SE: Timer0 Source Edge Select bit
        1 = Increment on high-to-low transition on T0CKI pin
        0 = Increment on low-to-high transition on T0CKI pin
    bit 3
        PSA: Timer0 Prescaler Assignment bit
        1 = TImer0 prescaler is NOT assigned. Timer0 clock input bypasses prescaler.
        0 = Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
    bit 2-0
        T0PS2:T0PS0: Timer0 Prescaler Select bits
        111 = 1:256 prescale value
        110 = 1:128 prescale value
        101 = 1:64 prescale value
        100 = 1:32 prescale value
        011 = 1:16 prescale value
        010 = 1:8 prescale value
        001 = 1:4 prescale value
        000 = 1:2 prescale value
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    TMR0L	Timer0 Module Low Byte Register									xxxx xxxx 	uuuu uuuu
    TMR0H	Timer0 Module High Byte Register								0000 0000	0000 0000
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    T0CON	TMR0ON	T08BIT	T0CS	T0SE	PSA		T0PS2	T0PS1	T0PS0	1111 1111	1111 1111
    TRISA 	? 		PORTA Data Direction Register 							-111 1111	-111 1111
     */
    timer0ReloadVal = 217;
    TMR0L = timer0ReloadVal;
    TMR0H = 0;
    T0CON = 0b01000111;
    INTCONbits.TMR0IF = 1;
    INTCONbits.TMR0IE = 1;
    INTCON2bits.TMR0IP = 0;
    //INTCONbits.GIEH = 1;
    //INTCONbits.GIEL = 1;
    T0CONbits.TMR0ON = 1;
}

void initTimer1(void) {
    /*	
    T1CON:
    bit 7
        RD16: 16-bit Read/Write Mode Enable bit
            1 = Enables register Read/Write of Timer1 in one 16-bit operation
            0 = Enables register Read/Write of Timer1 in two 8-bit operations
    bit 6 Unimplemented: Read as '0'
    bit 5-4
        T1CKPS1:T1CKPS0: Timer1 Input Clock Prescale Select bits
            11 = 1:8 Prescale value
            10 = 1:4 Prescale value
            01 = 1:2 Prescale value
            00 = 1:1 Prescale value
    bit 3
        T1OSCEN: Timer1 Oscillator Enable bit
            1 = Timer1 Oscillator is enabled
            0 = Timer1 Oscillator is shut-off
            The oscillator inverter and feedback resistor are turned off to eliminate power drain.
    bit 2
        T1SYNC: Timer1 External Clock Input Synchronization Select bit
            When TMR1CS = 1:
            1 = Do not synchronize external clock input
            0 = Synchronize external clock input
            When TMR1CS = 0:
            This bit is ignored. Timer1 uses the internal clock when TMR1CS = 0.
    bit 1
        TMR1CS: Timer1 Clock Source Select bit
            1 = External clock from pin RC0/T1OSO/T13CKI (on the rising edge)
            0 = Internal clock (FOSC/4)
    bit 0
        TMR1ON: Timer1 On bit
            1 = Enables Timer1
            0 = Stops Timer
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    PIR1	PSPIF	ADIF	RCIF	TXIF	SSPIF 	CP1IF	TMR2IF	TMR1IF	0000 0000	0000 0000
    PIE1	PSPIE	ADIE	RCIE	TXIE	SSPIE	CCP1IE	TMR2IE	TMR1IE	0000 0000	0000 0000
    IPR1	PSPIP	ADIP	RCIP	TXIP	SSPIP	CCP1IP	TMR2IP	TMR1IP	0000 0000	0000 0000
    TMR1L	Least Significant Byte of the 16-bit TMR1 Register				xxxx xxxx	uuuu uuuu
    TMR1H	Most Significant Byte of the 16-bit TMR1 Register				xxxx xxxx	uuuu uuuu
    T1CON 	RD16 	? 		T1CKPS1	T1CKPS0	T1OSCEN	T1SYNC	TMR1CS	TMR1ON	0-00 0000	u-uu uuuu
     */
}

void initTimer2(void) {
    /*
    T2CON:
    bit 7 Unimplemented: Read as '0'
    bit 6-3
        TOUTPS3:TOUTPS0: Timer2 Output Postscale Select bits
            0000 = 1:1 Postscale
            0001 = 1:2 Postscale
            ?
            ?
            ?
            1111 = 1:16 Postscale
    bit 2
        TMR2ON: Timer2 On bit
            1 = Timer2 is on
            0 = Timer2 is off
    bit 1-0
        T2CKPS1:T2CKPS0: Timer2 Clock Prescale Select bits
            00 = Prescaler is 1
            01 = Prescaler is 4
            1x = Prescaler is 16
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    PIR1	PSPIF	ADIF	RCIF	TXIF	SSPIF 	CP1IF	TMR2IF	TMR1IF	0000 0000	0000 0000
    PIE1	PSPIE	ADIE	RCIE	TXIE	SSPIE	CCP1IE	TMR2IE	TMR1IE	0000 0000	0000 0000
    IPR1	PSPIP	ADIP	RCIP	TXIP	SSPIP	CCP1IP	TMR2IP	TMR1IP	0000 0000	0000 0000
    TMR2	Timer2 Module Register											0000 0000	0000 0000
    T2CON 	? 		TOUTPS3	TOUTPS2	TOUTPS1	TOUTPS0	TMR2ON	T2CKPS1	T2CKPS0 -000 0000	-000 0000
    PR2		Timer2 Period Register											1111 1111	1111 1111
     */
}

void initTimer3(void) {
    /*
    T3CON:
    bit 7
        RD16: 16-bit Read/Write Mode Enable bit
            1 = Enables register Read/Write of Timer3 in one 16-bit operation
            0 = Enables register Read/Write of Timer3 in two 8-bit operations
    bit 6-3 *********
        T3CCP2:T3CCP1: Timer3 and Timer1 to CCPx Enable bits
            1x = Timer3 is the clock source for compare/capture CCP modules
            01 = Timer3 is the clock source for compare/capture of CCP2,
                 Timer1 is the clock source for compare/capture of CCP1
            00 = Timer1 is the clock source for compare/capture CCP modules
    bit 5-4 **********
        T3CKPS1:T3CKPS0: Timer3 Input Clock Prescale Select bits
            11 = 1:8 Prescale value
            10 = 1:4 Prescale value
            01 = 1:2 Prescale value
            00 = 1:1 Prescale value
    bit 2	
        T3SYNC: Timer3 External Clock Input Synchronization Control bit
        (Not usable if the system clock comes from Timer1/Timer3)
        When TMR3CS = 1:
            1 = Do not synchronize external clock input
            0 = Synchronize external clock input
        When TMR3CS = 0:
            This bit is ignored. Timer3 uses the internal clock when TMR3CS = 0.
    bit 1
        TMR3CS: Timer3 Clock Source Select bit
            1 = External clock input from Timer1 oscillator or T1CKI
                (on the rising edge after the first falling edge)
            0 = Internal clock (FOSC/4)
    bit 0
        TMR3ON: Timer3 On bit
            1 = Enables Timer3
            0 = Stops Timer3
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    PIR2	?		?		?		EEIF	BCLIF	LVDIF	TMR3IF	CCP2IF	---0 0000	---0 0000
    PIE2	?		?		?		EEIE	BCLIE	LVDIE	TMR3IE	CCP2IE ---0 0000 	---0 0000
    IPR2	?		?		?		EEIP	BCLIP	LVDIP	TMR3IP	CCP2IP ---1 1111	---1 1111
    TMR3L	Least Significant Byte of the 16-bit TMR3 Register				xxxx xxxx	uuuu uuuu
    TMR3H	Most Significant Byte of the 16-bit TMR3 Register				xxxx xxxx	uuuu uuuu
    T1CON	RD16 	?		T1CKPS1	T1CKPS0	T1OSCEN	T1SYNC	TMR1CS	TMR1ON	0-00 0000	u-uu uuuu
    T3CON	RD16	T3CCP2	T3CKPS1	T3CKPS0	T3CCP1	T3SYNC	TMR3CS	TMR3ON	0000 0000	uuuu uuuu
     */
    //76543210
    T3CON = 0b10110001;
    PIR2bits.TMR3IF = 0;
    TMR3L = 0x00;
    TMR3H = 0x00;
    IPR2bits.TMR3IP = 1;
    PIE2bits.TMR3IE = 1;
}

void initExternalInterrupt(void) {
    /*
    INTCON:
    bit 7 
        GIE/GIEH: Global Interrupt Enable bit
        When IPEN = 0:
            1 = Enables all unmasked interrupts
            0 = Disables all interrupts
        When IPEN = 1:
            1 = Enables all high priority interrupts
            0 = Disables all interrupts
    bit 6
        PEIE/GIEL: Peripheral Interrupt Enable bit
        When IPEN = 0:
            1 = Enables all unmasked peripheral interrupts
            0 = Disables all peripheral interrupts
        When IPEN = 1:
            1 = Enables all low priority peripheral interrupts
            0 = Disables all low priority peripheral interrupts
    bit 5
        TMR0IE: TMR0 Overflow Interrupt Enable bit
            1 = Enables the TMR0 overflow interrupt
            0 = Disables the TMR0 overflow interrupt
    bit 4
        INT0IE: INT0 External Interrupt Enable bit
            1 = Enables the INT0 external interrupt
            0 = Disables the INT0 external interrupt
    bit 3
        RBIE: RB Port Change Interrupt Enable bit
            1 = Enables the RB port change interrupt
            0 = Disables the RB port change interrupt
    bit 2
        TMR0IF: TMR0 Overflow Interrupt Flag bit
            1 = TMR0 register has overflowed (must be cleared in software)
            0 = TMR0 register did not overflow
    bit 1
        INT0IF: INT0 External Interrupt Flag bit
            1 = The INT0 external interrupt occurred (must be cleared in software)
            0 = The INT0 external interrupt did not occur
    bit 0
        RBIF: RB Port Change Interrupt Flag bit
            1 = At least one of the RB7:RB4 pins changed state (must be cleared in software)
            0 = None of the RB7:RB4 pins have changed state
            Note: A mismatch condition will continue to set this bit. Reading PORTB will end the
            mismatch condition and allow the bit to be cleared.
    INTCON2:
    bit 7
        RBPU: PORTB Pull-up Enable bit
            1 = All PORTB pull-ups are disabled
            0 = PORTB pull-ups are enabled by individual port latch values
    bit 6
        INTEDG0:External Interrupt0 Edge Select bit
            1 = Interrupt on rising edge
            0 = Interrupt on falling edge
    bit 5
        INTEDG1: External Interrupt1 Edge Select bit
            1 = Interrupt on rising edge
            0 = Interrupt on falling edge
    bit 4
        INTEDG2: External Interrupt2 Edge Select bit
            1 = Interrupt on rising edge
            0 = Interrupt on falling edge
    bit 3 Unimplemented: Read as '0'
    bit 2
        TMR0IP: TMR0 Overflow Interrupt Priority bit
            1 = High priority
            0 = Low priority
    bit 1 Unimplemented: Read as '0'
    bit 0 RBIP: RB Port Change Interrupt Priority bit
            1 = High priority
            0 = Low priority
    INTCON3:
    bit 7
        INT2IP: INT2 External Interrupt Priority bit
            1 = High priority
            0 = Low priority
    bit 6
        INT1IP: INT1 External Interrupt Priority bit
            1 = High priority
            0 = Low priority
    bit 5 Unimplemented: Read as '0'
    bit 4
        INT2IE: INT2 External Interrupt Enable bit
            1 = Enables the INT2 external interrupt
            0 = Disables the INT2 external interrupt
    bit 3
        INT1IE: INT1 External Interrupt Enable bit
            1 = Enables the INT1 external interrupt
            0 = Disables the INT1 external interrupt
    bit 2 Unimplemented: Read as '0'
    bit 1
        INT2IF: INT2 External Interrupt Flag bit
            1 = The INT2 external interrupt occurred (must be cleared in software)
            0 = The INT2 external interrupt did not occur
    bit 0
        INT1IF: INT1 External Interrupt Flag bit
            1 = The INT1 external interrupt occurred (must be cleared in software)
            0 = The INT1 external interrupt did not occur
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    INTCON2	RBPU	INTEDG0	INTEDG1	INTEDG2	?		TMR0IP	?		RBIP
    INTCON3	INT2IP	INT1IP	?		INT2IE	INT1IE	?		INT2IF	INT1IF
     */
    TRISBbits.TRISB0 = 1;
    INTCON2bits.INTEDG0 = 0;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;

}

void initPWM(void) {
    /*
    CCP1CON:
    bit 7-6 Unimplemented: Read as '0'
    bit 5-4
        DCxB1:DCxB0: PWM Duty Cycle bit1 and bit0
        Capture mode:Unused
        Compare mode:Unused
        PWM mode:
        These bits are the two LSbs (bit1 and bit0) of the 10-bit PWM duty cycle. The upper eight bits
        (DCx9:DCx2) of the duty cycle are found in CCPRxL.
    bit 3-0
        CCPxM3:CCPxM0: CCPx Mode Select bits
        0000 = Capture/Compare/PWM disabled (resets CCPx module)
        0001 = Reserved
        0010 = Compare mode, toggle output on match (CCPxIF bit is set)
        0011 = Reserved
        0100 = Capture mode, every falling edge
        0101 = Capture mode, every rising edge
        0110 = Capture mode, every 4th rising edge
        0111 = Capture mode, every 16th rising edge
        1000 = Compare mode,
                Initialize CCP pin Low, on compare match force CCP pin High (CCPIF bit is set)
        1001 = Compare mode,
                Initialize CCP pin High, on compare match force CCP pin Low (CCPIF bit is set)
        1010 = Compare mode,
                Generate software interrupt on compare match (CCPIF bit is set, CCP pin is unaffected)
        1011 = Compare mode,
                Trigger special event (CCPIF bit is set)
        11xx = PWM mode
    T2CON:
    bit 7 Unimplemented: Read as '0'
    bit 6-3
        TOUTPS3:TOUTPS0: Timer2 Output Postscale Select bits
            0000 = 1:1 Postscale
            0001 = 1:2 Postscale
            ?
            ?
            ?
            1111 = 1:16 Postscale
    bit 2
        TMR2ON: Timer2 On bit
            1 = Timer2 is on
            0 = Timer2 is off
    bit 1-0
        T2CKPS1:T2CKPS0: Timer2 Clock Prescale Select bits
            00 = Prescaler is 1
            01 = Prescaler is 4
            1x = Prescaler is 16
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    PIR1	PSPIF	ADIF	RCIF	TXIF	SSPIF 	CP1IF	TMR2IF	TMR1IF	0000 0000	0000 0000
    PIE1	PSPIE	ADIE	RCIE	TXIE	SSPIE	CCP1IE	TMR2IE	TMR1IE	0000 0000	0000 0000
    IPR1	PSPIP	ADIP	RCIP	TXIP	SSPIP	CCP1IP	TMR2IP	TMR1IP	0000 0000	0000 0000
    TMR2	Timer2 Module Register											0000 0000	0000 0000
    T2CON 	? 		TOUTPS3	TOUTPS2	TOUTPS1	TOUTPS0	TMR2ON	T2CKPS1	T2CKPS0 -000 0000	-000 0000
    PR2		Timer2 Period Register											1111 1111	1111 1111

     */
    PR2 = 0xFF;
    //76543210			   
    CCP1CON = 0b00001100;
    TRISCbits.TRISC2 = 0;
    //76543210
    T2CON = 0b00000110;
    PIR1bits.TMR2IF = 0; //enable timer2 interrupt
    PIE1bits.TMR2IE = 1;
    IPR1bits.TMR2IP = 1; //hi priority
    CCPR1L = 0xFF;
}

void setPWMDuty(unsigned int iVal) {
    if (iVal > 1023)
        iVal = 1023;
    /*
    if(iVal < 204)
        iVal = 204;
     */

    CCPR1L = iVal >> 2;
    switch (iVal % 2) {
        case 3:
            CCP1CONbits.DC1B1 = 1;
            CCP1CONbits.DC1B0 = 1;
            break;
        case 2:
            CCP1CONbits.DC1B1 = 1;
            CCP1CONbits.DC1B0 = 0;
            break;
        case 1:
            CCP1CONbits.DC1B1 = 0;
            CCP1CONbits.DC1B0 = 1;
            break;
        case 0:
            CCP1CONbits.DC1B1 = 0;
            CCP1CONbits.DC1B0 = 0;
            break;
    }
}

void initLVD(void) {
    /*
    1. Write the value to the LVDL3:LVDL0 bits (LVDCON register), which selects the desired LVD Trip Point.
    2. Ensure that LVD interrupts are disabled (the LVDIE bit is cleared or the GIE bit is cleared).
    3. Enable the LVD module (set the LVDEN bit in the LVDCON register).
    4. Wait for the LVD module to stabilize (the IRVST bit to become set).
    5. Clear the LVD interrupt flag, which may have falsely become set until the LVD module has stabilized (clear the LVDIF bit).
    6. Enable the LVD interrupt (set the LVDIE and the GIE bits).
    LVDCON:
    bit 7-6 Unimplemented:
        Read as '0'
    bit 5
    IRVST:Internal Reference Voltage Stable Flag bit
        1 = Indicates that the Low Voltage Detect logic will generate the interrupt flag at the specified voltage range
        0 = Indicates that the Low Voltage Detect logic will not generate the interrupt flag at the specified voltage range and the LVD interrupt should not be enabled
    bit 4
    LVDEN: Low Voltage Detect Power Enable bit
        1 = Enables LVD, powers up LVD circuit
        0 = Disables LVD, powers down LVD circuit
    bit 3-0
    LVDL3:LVDL0: Low Voltage Detection Limit bits
        1111 = External analog input is used (input comes from the LVDIN pin)
        1110 = 4.5V - 4.77V
        1101 = 4.2V - 4.45V
        1100 = 4.0V - 4.24V
        1011 = 3.8V - 4.03V
        1010 = 3.6V - 3.82V
        1001 = 3.5V - 3.71V
        1000 = 3.3V - 3.50V
        0111 = 3.0V - 3.18V
        0110 = 2.8V - 2.97V
        0101 = 2.7V - 2.86V
        0100 = 2.5V - 2.65V
        0011 = 2.4V - 2.54V
        0010 = 2.2V - 2.33V
        0001 = 2.0V - 2.12V
        0000 = Reserved
    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    LVDCON	?		?		IRVST	LVDEN	LVDL3	LVDL2	LVDL1	LVDL0	--00 0101	--00 0101
    PIE2	?		?		?		EEIE	BCLIE	LVDIE	TMR3IE	CCP2IE	---0 0000	---0 0000
    PIR2	?		?		?		EEIF	BCLIF	LVDIF	TMR3IF	CCP2IF	---0 0000	---0 0000
    IPR2	?		?		?		EEIP	BCLIP	LVDIP	TMR3IP	CCP2IP	---1 1111	---1 1111
     */
    LVDCON = 0b00001110;
    PIE2bits.LVDIE = 0;
    LVDCONbits.LVDEN = 1;
    //LVDCONbits.LVDEN = 1;
    while (!LVDCONbits.IRVST)
        ;
    PIR2bits.LVDIF = 0;
    PIE2bits.LVDIE = 1;
    IPR2bits.LVDIP = 1;

}

void initADC(void) {
#if defined(__18F4520)
    ADCON0 = 0b00000000;
    ADCON1 = 0b00011011; //ADCON1 = 0b00010000;
    ADCON2 = 0b10110101;
    //76543210
    TRISA |= 0b00001111;
    ADCON0bits.ADON = 1;
#else
    /*
    ADCON0 REGISTER
    bit 7-6 
    ADCS1:ADCS0: A/D Conversion Clock Select bits (ADCON0 bits in bold)
    ADCON1		ADCON0		Clock Conversion
    <ADCS2>	<ADCS1:ADCS0>
        0		00			FOSC/2
        0		01			FOSC/8
        0		10			FOSC/32		<-- Recomended 2
        0		11			FRC (clock derived from the internal A/D RC oscillator)
        1		00			FOSC/4
        1		01			FOSC/16
        1		10			FOSC/64		<-- Recomended 1 for 20 MHz Fosc
        1		11			FRC (clock derived from the internal A/D RC oscillator)
    bit 5-3
    CHS2:CHS0: Analog Channel Select bits
        000 = channel 0, (AN0)
        001 = channel 1, (AN1)
        010 = channel 2, (AN2)
        011 = channel 3, (AN3)
        100 = channel 4, (AN4)
        101 = channel 5, (AN5)
        110 = channel 6, (AN6)
        111 = channel 7, (AN7)
    bit 2
    GO/DONE: A/D Conversion Status bit
        When ADON = 1:
        1 = A/D conversion in progress (setting this bit starts the A/D conversion which is automatically
            cleared by hardware when the A/D conversion is complete)
        0 = A/D conversion not in progress
    bit 1 Unimplemented: Read as '0'
    bit 0 ADON: A/D On bit
        1 = A/D converter module is powered up
        0 = A/D converter module is shut-off and consumes no operating current
            // 76543210
    ADCON0 = 0b10000001;
	
    ADCON1 REGISTER
    bit 7
    ADFM: A/D Result Format Select bit
        1 = Right justified. Six (6) Most Significant bits of ADRESH are read as ?0?.
        0 = Left justified. Six (6) Least Significant bits of ADRESL are read as ?0?.
    bit 6 
    ADCS2: A/D Conversion Clock Select bit (ADCON1 bits in bold)
    ADCON1		ADCON0		Clock Conversion
    <ADCS2>	<ADCS1:ADCS0>
        1		10			FOSC/64		<-- Recomended 1 for 20 MHz Fosc
    bit 5-4 Unimplemented: Read as '0'
    bit 3-0
    PCFG3:PCFG0: A/D Port Configuration Control bits
    PCFG	AN7		AN6		AN5		AN4		AN3		AN2		AN1		AN0		VREF+	VREF- 	 C/R
            RE2 	RE1		RE0		LVDIN/	RA3		RA2		RA1		RA0		
    <3:0>	/CS_					RA5
    0000	 A		 A		 A		 A		 A		 A		 A		 A		 VDD	 VSS	 8/0
    0001	 A		 A		 A		 A		 VREF+	 A		 A		 A		 AN3	 VSS	 7/1
    0010	 D		 D		 D		 A		 A		 A		 A		 A		 VDD	 VSS	 5/0
    0011	 D		 D		 D		 A		 VREF+	 A		 A		 A		 AN3	 VSS	 4/1
    0100	 D		 D		 D		 D		 A		 D		 A		 A		 VDD	 VSS	 3/0	<--Suitable for universal controller (Internal ref)
    0101	 D		 D		 D		 D		 VREF+	 D		 A		 A		 AN3	 VSS	 2/1
    011x	 D		 D		 D		 D		 D		 D		 D		 D		 ?		 ?		 0/0
    1000	 A		 A		 A		 A		 VREF+	 VREF-	 A		 A		 AN3	 AN2	 6/2
    1001	 D		 D		 A		 A		 A		 A		 A		 A		 VDD	 VSS	 6/0
    1010	 D		 D		 A		 A		 VREF+	 A		 A		 A		 AN3	 VSS	 5/1
    1011	 D		 D		 A		 A		 VREF+	 VREF-	 A		 A		 AN3	 AN2	 4/2	<--Suitable for universal controller (Exteranl ref) 1
    1100	 D		 D		 D		 A		 VREF+	 VREF-	 A		 A		 AN3	 AN2	 3/2		(but 3rd channel conflicting with LVDIN)
    1101	 D		 D		 D		 D		 VREF+	 VREF-	 A		 A		 AN3	 AN2	 2/2	<--Suitable for universal controller (Exteranl ref)
    1110	 D		 D		 D		 D		 D		 D		 D		 A		 VDD	 VSS	 1/0		(LVDIN can be enabled)
    1111	 D		 D		 D		 D		 VREF+	 VREF-	 D		 A		 AN3	 AN2	 1/2

    Name 	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	POR/BOR		RESET
    INTCON	GIE		PEIE	TMR0IE	INT0IE	RBIE	TMR0IF	INT0IF	RBIF	0000 000x	0000 000u
            /GIEH	/GIEL
    PIR1	PSPIF	ADIF	RCIF	TXIF	SSPIF 	CP1IF	TMR2IF	TMR1IF	0000 0000	0000 0000
    PIE1	PSPIE	ADIE	RCIE	TXIE	SSPIE	CCP1IE	TMR2IE	TMR1IE	0000 0000	0000 0000
    IPR1	PSPIP	ADIP	RCIP	TXIP	SSPIP	CCP1IP	TMR2IP	TMR1IP	0000 0000	0000 0000
    PIR2	?		?		?		EEIF	BCLIF	LVDIF	TMR3IF	CCP2IF	---0 0000	---0 0000
    PIE2	?		?		?		EEIE	BCLIE	LVDIE	TMR3IE	CCP2IE  ---0 0000 	---0 0000
    IPR2	?		?		?		EEIP	BCLIP	LVDIP	TMR3IP	CCP2IP  ---1 1111	---1 1111
    ADRESH A/D Result Register												xxxx xxxx	uuuu uuuu
    ADRESL A/D Result Register												xxxx xxxx	uuuu uuuu
    ADCON0 	ADCS1 	ADCS0	CHS2	CHS1	CHS0	GO/DONE	?		ADON	0000 00-0	0000 00-0
    ADCON1 	ADFM	ADCS2	?		?		PCFG3	PCFG2	PCFG1	PCFG0	---- -000	---- -000
    PORTA	?		RA6		RA5		RA4		RA3		RA2		RA1		RA0		--0x 0000	--0u 0000
    TRISA	?	PORTA Data Direction Register								--11 1111	--11 1111
    PORTE	?		?		?		?		?		RE2		RE1		RE0		---- -000	---- -000
    LATE	?		?		?		?		?		LATE2	LATE1	LATE0	---- -xxx	---- -uuu
    TRISE	IBF		OBF		IBOV	PSPMODE	?		PORTE Data Direction 	0000 -111	0000 -111
     */
    // 76543210
    ADCON1 = 0b11001101; //right justified, Fosc/64, external reference, 2 I/P
    //ADCON1 = 0b11000100;	//right justified, Fosc/64, internal reference, 3 I/P

    ADCON0 = 0b10000000;
    TRISA = 0b00001111;
    TRISE = 0b00000000;
    LATE = 0b00000000;
    ADCON0bits.ADON = 1;
#endif
}
//void selectChannel(unsigned char ucChannel)
//{
//	switch(ucChannel)
//	{
//		case 0:					//AN0/RA0/pin 2
//			ADCON0bits.CHS2 = 0;
//			ADCON0bits.CHS1 = 0;
//			ADCON0bits.CHS0 = 0;			
//			TRISAbits.TRISA0 = 1;
//		break;
//		case 1:					//AN1/RA1/pin 3
//			ADCON0bits.CHS2 = 0;
//			ADCON0bits.CHS1 = 0;
//			ADCON0bits.CHS0 = 1;			
//			TRISAbits.TRISA1 = 1;
//		break;
//		case 2:					//AN4/RA5/pin 7
//			ADCON0bits.CHS2 = 0;
//			ADCON0bits.CHS1 = 1;
//			ADCON0bits.CHS0 = 0;
//			TRISAbits.TRISA5 = 1;
//		break;
//		case 3:					//AN5/RE0/pin 8
//			ADCON0bits.CHS2 = 1;
//			ADCON0bits.CHS1 = 0;
//			ADCON0bits.CHS0 = 1;
//			TRISEbits.TRISE0 = 1;
//		break;
//		case 4:					//AN3/RA3/pin 5	(AN2/RA)
//			ADCON0bits.CHS2 = 0;
//			ADCON0bits.CHS1 = 1;
//			ADCON0bits.CHS0 = 1;
//			TRISAbits.TRISA3 = 1;
//		break;		
//		default:				//AN0/RA0/pin 2
//			ADCON0bits.CHS2 = 0;
//			ADCON0bits.CHS1 = 0;
//			ADCON0bits.CHS0 = 0;
//			TRISAbits.TRISA0 = 1;
//		break;
//	}
//}
//unsigned int getAdcValue1(void)
//{	
//	//ADCON0bits.ADON = 1;
//	ADCON0bits.GO = 1;
//	while(ADCON0bits.GO)
//		;
//	return ADRES;
//}
//unsigned int getAdcValue(unsigned char ucChannel)
//{
//	selectChannel(ucChannel);
//	return getAdcValue1();	
//}	

void adcTask(void) {
    if (ADCON0bits.ADON) {
        if (filterCount < FILTER_POINTS - 1) {
            filterCount++;
            //ADRES = 0;
            ADCON0bits.GO = 1;
            while (ADCON0bits.GO)
                ;
            filter += ADRES;
        } else {
            //ADRES = 0;
            ADCON0bits.GO = 1;
            while (ADCON0bits.GO)
                ;
            filter += ADRES;
            switch (ucScanChannel) {
                case 0:
                    iADCValCh1 = filter / FILTER_POINTS;
                    ADCON0bits.CHS2 = 0;
                    ADCON0bits.CHS1 = 0;
                    ADCON0bits.CHS0 = 1;
                    TRISAbits.TRISA1 = 1; //AN1/RA1/pin 3
                    ucScanChannel++;
                    break;
                case 1:
                    iADCValCh2 = filter / FILTER_POINTS;
                    ADCON0bits.CHS2 = 0;
                    ADCON0bits.CHS1 = 0;
                    ADCON0bits.CHS0 = 0;
                    TRISAbits.TRISA0 = 1; //AN0/RA0/pin 2
                    ucScanChannel = 0;
                    break;
                default:
                    ADCON0bits.CHS2 = 0;
                    ADCON0bits.CHS1 = 0;
                    ADCON0bits.CHS0 = 0;
                    TRISAbits.TRISA0 = 1; //AN0/RA0/pin 2
                    ucScanChannel = 0;
                    break;
            }
            filterCount = 0;
            filter = 0;
        }
    }
}

void SerialInit(void) {


    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
    //PIE1bits.TXIE = 0;

    IPR1bits.RCIP = 0;
    //IPR1bits.TXIP = 1;
    /*
     * Configure USART for Asynch, 8-bit mode.
     */
    //TXSTA = 0b00100100;	//USART_USE_BRGH
    TXSTAbits.CSRC = 0;
    TXSTAbits.TX9 = 0;
    TXSTAbits.TXEN = 1;
    TXSTAbits.SYNC = 0;
    // reserve bits=0
    TXSTAbits.BRGH = 1;
    TXSTAbits.TRMT = 0;
    TXSTAbits.TX9D = 0;


    //RCSTA = 0b10010000;
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.SREN = 0;
    RCSTAbits.CREN = 1;
    RCSTAbits.ADDEN = 0;
    RCSTAbits.FERR = 0;
    RCSTAbits.OERR = 0;
    RCSTAbits.RX9D = 0;


    /*
    clock	baudrate	when 	SPBRG
    22118400	9600	16		144	143
    22118400	19200	16		72	71
    22118400	38400	16		36	35
    22118400	57600	16		24	23
    22118400	115200	16		12	11
    22118400	230400	16		6	5
    22118400	460800	16		3	2
    22118400			
    22118400	2400	64		144	143
    22118400	4800	64		72	71
    22118400	9600	64		36	35
    22118400	19200	64		18	17
    22118400	38400	64		9	8
    22118400	57600	64		6	5
    22118400	115200	64		3	2
     */

    SPBRG = 129; // 9600 baudrate at 20MHz clock when BRGH=1
    //SPBRG = 143;  //  FOR  9600 baud at 22118400 MHz crystal

    TRISCbits.TRISC6 = 0; // SET TX   pin=25
    TRISCbits.TRISC7 = 1; // SET RX	pin=26

}
