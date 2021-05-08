// HardwareProfile.h

#ifndef _HARDWAREPROFILE_H_
#define _HARDWAREPROFILE_H_

#if defined( __PIC32MX__)
#endif

//#define USE_USB_PLL

#if defined (__C30__)
#if defined (__dsPIC33EP512MU810__)||defined(__PIC24EP512GU810__)
#define GetSystemClock()            40000000UL
#define GetPeripheralClock()        (GetSystemClock())
#define GetInstructionClock()       (GetSystemClock() / 2)
#else
// Various clock values
#define GetSystemClock()            32000000UL
#define GetPeripheralClock()        (GetSystemClock())
#define GetInstructionClock()       (GetSystemClock() / 2)
#endif
#endif

/*
 delay_us(x) and delay_ms(x) for C30
 */
#ifndef __DELAY_H
#define FOSC  32000000LL  // clock-frequecy in Hz with suffix LL (64-bit-long), eg. 32000000LL for 32MHz
#define FCY       (FOSC/2)  // MCU is running at FCY MIPS
#define DelayUs(x) __delay32(((x*FCY)/1000000L)) // delays x us
#define DelayMs(x) __delay32(((x*FCY)/1000L))  // delays x ms
#define __DELAY_H 1
#include <libpic30.h>
#endif


// Define the baud rate constants
#define BAUDRATE2       115200UL //19200
#define BRG_DIV2        4 //16
#define BRGH2           1 //0

#define DEMO_TIMEOUT_LIMIT  0xF000

#if defined(__PIC24F__) || defined(__PIC24H__)
#include <p24fxxxx.h>
#include <uart2.h>

#endif


/** TRIS ***********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0



#endif  

