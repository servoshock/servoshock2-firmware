#include "functional_test.h"
#include "main.h"
#include <p24Fxxxx.h>
#include <stdlib.h>

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "uart2.h"

/*************************************************************************
 * Function:        TestCases
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         None
 *
 * Side Effects:    None
 *
 * Overview:        This routine is used for use with the automatic testbed.
 *                  clocking the PGC pin will toggle the outputs on and off
 *                  one by one.
 *************************************************************************/
void TestCases(void) {
    unsigned char pinNum = 0;
    while (UART2GetChar() != 0x02)
    {
        UART2ClrError();
    }
    pinNum = (unsigned char)UART2GetChar()-48;
    UART2PrintString("Test Case: ");
    UART2PutDec(pinNum);
    UART2PrintString("\n\r");

    //Disable Pull-up on RD11 Slave Select;
    CNPU4bits.CN56PUE = 0;

    //Set all pins to inputs in case of shorting
    
    TRISB = 0xFFFF;
    TRISC = 0xFFFF;
    TRISD = 0xFFFF;
    TRISE = 0xFFFF;
    TRISF = 0xFFFF;
    TRISG = 0xFFFF;

    
    TRISB = 0xFFFF;
    TRISC = 0xFFFF;
    TRISD = 0xFFFF;
    TRISE = 0xFFFF;
    TRISF = 0xFFFF;
    TRISG = 0;

    
    LATB = 0;
    LATC = 0;
    LATD = 0;
    LATE = 0;
    LATF = 0;
    LATG = 0;

    switch (pinNum) {
        case 0: //PWM0
            _TRISG6 = 0;
            _LATG6 = 1;
            break;
        case 1: //PWM1
            _TRISG7 = 0;
            _LATG7 = 1;
            break;
        case 2: //PWM2
            _TRISG8 = 0;
            _LATG8 = 1;
            break;
        case 3: //PWM3
            _TRISG9 = 0;
            _LATG9 = 1;
            break;
        case 4: //PWM4
            _TRISB5 = 0;
            _LATB5 = 1;
            break;
        case 5: //PWM5
            _TRISB4 = 0;
            _LATB4 = 1;
            break;
        case 6: //PWM6
            _TRISB2 = 0;
            _LATB2 = 1;
            break;
        case 7: //PWM7
            _TRISB1 = 0;
            _LATB1 = 1;
            break;
        case 8: //PWM8
            _TRISB0 = 0;
            _LATB0 = 1;
            break;
        case 9: //PWM9
            _TRISB8 = 0;
            _LATB8 = 1;
            break;
        case 10: //PWM10
            _TRISB12 = 0;
            _LATB12 = 1;
            break;
        case 11: //PWM11
            _TRISB13 = 0;
            _LATB13 = 1;
            break;
        case 12: //B0
            _TRISF3 = 0;
            _LATF3 = 1;
            break;
        case 13: //B1
            _TRISC13 = 0;
            _LATC13 = 1;
            break;
        case 14: //B2
            _TRISC14 = 0;
            _LATC14 = 1;
            break;
        case 15: //B3
            _TRISD1 = 0;
            _LATD1 = 1;
            break;
        case 16: //B4
            _TRISD2 = 0;
            _LATD2 = 1;
            break;
        case 17: //B5
            _TRISD3 = 0;
            _LATD3 = 1;
            break;
        case 18: //B6
            _TRISD4 = 0;
            _LATD4 = 1;
            break;
        case 19: //B7
            _TRISD5 = 0;
            _LATD5 = 1;
            break;
        case 20: //B8
            _TRISD6 = 0;
            _LATD6 = 1;
            break;
        case 21: //B9
            _TRISD7 = 0;
            _LATD7 = 1;
            break;
        case 22: //B10
            _TRISF0 = 0;
            _LATF0 = 1;
            break;
        case 23: //B11
            _TRISF1 = 0;
            _LATF1 = 1;
            break;
        case 24: //B12
            _TRISE0 = 0;
            _LATE0 = 1;
            break;
        case 25: //B13
            _TRISE1 = 0;
            _LATE1 = 1;
            break;
        case 26: //B14
            _TRISE2 = 0;
            _LATE2 = 1;
            break;
        case 27: //B15
            _TRISE3 = 0;
            _LATE3 = 1;
            break;
        case 28: //B16
            _TRISE4 = 0;
            _LATE4 = 1;
            break;
        case 29: //B17
            _TRISB3 = 0;
            _LATB3 = 1;
            break;
        case 30: //SCKIN
            _TRISD8 = 0;
            _LATD8 = 1;
            break;
        case 31: //SDI1
            _TRISD9 = 0;
            _LATD9 = 1;
            break;
        case 32: //SDO1
            _TRISD10 = 0;
            _LATD10 = 1;
            break;
        case 33: //SSIN
            _TRISD11 = 0;
            _LATD11 = 1;
            break;
        case 34: //LED
            _TRISB9 = 0;
            _LATB9 = 1;
            break;
        case 35: //RumbleL
            _TRISB10 = 0;
            _LATB10 = 1;
            break;
        case 36: //RumbleH
            _TRISB11 = 0;
            _LATB11 = 1;
            break;
        default:
            UART2PrintString("Unknown Case\n\r");
            break;
    }
}

