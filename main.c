//#define PRINT_ACL_PACKET//
//#define PRINT_HCI_PACKET
//#define DEBUG_MODE


/******************************************************************************
 * Filename:        main.c
 * Dependancies:    USB Host Driver with Generic Client Driver
 * Processor:       PIC24F256GB1xx
 * Hardware:        Explorer 16 with USB PICtail Plus
 * Compiler:        C30 v3.31
 * Company:         Microchip Technology, Inc.

 *Revisions:
 *1.0 04/01/18 Initial Release
 *1.1 05/10/18 Fixed servo channel 2 SPI bus mapping. Minor fixes in serial text help dialog
 *1.2 06/29/18 Fixed bug that affected EEPROM when updating firmware
 *2.0 12/14/18 Changed servo and button output states in the SPI packet to be
 the processed values that WOULD have been output if not overridden instead of 
 the actual output register value.  This was changed because the actual output can be 
 overwritten by the Arduino.
 *3.01 5/10/21 Added function to allow servo to be remapped to buttons
 *3.11 03/08/22 Fixed bug in servo button remapping code, fixed bug in tilt incremental mode initial servo state
Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the ìCompanyÅE for its PICmicroÆ Microcontroller is intended and
supplied to you, the Companyís customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN ìAS ISÅECONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.


 *******************************************************************************/
 /******************************************************************************
            Acknowledgements

 * This project was  possible thanks to previous work from many people:
 * 
 * The Microchip PIC USB Bluetooth dongle driver framework was based off of work by
 * Yuhji Tsujimi "yts" on the Microchip forums.
 *  http://www.microchip.com/forums/m489233.aspx
 *
 * A lot of Bluetooth and Sony DualShock driver work was done by the following:
 * Richard Ibbotson/Oleg Mazurov
 * http://www.circuitsathome.com/mcu/ps3-and-wiimote-game-controllers-on-the-arduino-host-shield-part-1
 *
 * Kristian Lauszus/Oleg Mazurov/Alexei Glushchenko/Andrew Kroll/guruthree/Yuuichi Akagawa
 * https://github.com/felis/USB_Host_Shield_2.0
 *
 * Guillem ViÒals Gangolells
 * http://www.guillem.co.uk/projects
 *
 * "Acxie"
 * http://www.motioninjoy.com/
 *
 * PS4 packet protocol decoding work was made possible by various contributors at:
 * http://www.psdevwiki.com/ps4/DS4-BT
 * http://eleccelerator.com/wiki/index.php?title=DualShock_4
 *
 * The bootloader is Mikael Gustafsson's excellent ds30 bootloader.
 * http://mrmackey.no-ip.org/elektronik/ds30loader/
 *
 * Lots of protocol decoding was done with WireShark:
 * http://www.wireshark.org/
******************************************************************************/
/*****************************************************************************
    PS4 USB Host
    Copyright (C) 2013 Cross Product Creations
    The GNU Public License applies to code NOT from Microchip.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/


#include "main.h"
#include <p24Fxxxx.h>
#include <stdlib.h>

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb_host_generic_PS4.h"
#include "DEE Emulation 16-bit/DEE Emulation 16-bit.h"

#include "PS4_controller.h"
#include "outputs.h"
#include "PS4_SPI.h"

#include "functional_test.h"
//firmware revision


/////////////////Library configuration//////////////
#define USE_AND_OR
// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************
#define STOP_TIMER_IN_IDLE_MODE     0x2000
#define TIMER_SOURCE_INTERNAL       0x0000
#define TIMER_ON                    0x8000
#define GATED_TIME_DISABLED         0x0000
#define TIMER_16BIT_MODE            0x0000

#define TIMER_PRESCALER_1           0x0000
#define TIMER_PRESCALER_8           0x0010
#define TIMER_PRESCALER_64          0x0020
#define TIMER_PRESCALER_256         0x0030
#define TIMER_INTERRUPT_PRIORITY    0x0001

_CONFIG2(IESO_OFF & PLL_96MHZ_ON & PLLDIV_DIV5 & FNOSC_PRIPLL & POSCMOD_HS) // Primary HS OSC with PLL, USBPLL /5
_CONFIG1(JTAGEN_OFF & ICS_PGx2 & FWDTEN_OFF) // JTAG off, watchdog timer off

// *****************************************************************************


// Application States

typedef enum {
    BT_INITIALIZE = 0, // Initialize the app when a device is attached
    BT_STATE_PROCESS,
    BT_STATE_READ_DATA
} BT_STATE;


// Hci States

typedef enum {
    HCI_CMD_RESET = 0, // Initialize the hci when a device is attached
    HCI_CMD_READ_BD_ADDR,
    HCI_CMD_LOCAL_NAME,
    HCI_CMD_CLASS_DEVICE,
    HCI_CMD_SCAN_ENABLE,
    HCI_CMD_INQUIRY,
    HCI_CMD_SCAN_ENABLE_WAIT,
    HCI_CMD_CREATE_CONNECTION,
    HCI_CMD_INCOMING_ACCEPT,
    HCI_ROLE_CHANGE_WAIT,
    HCI_CMD_CONNECTION_ACCEPTED,
    HCI_CMD_SCAN_DISABLE,
    LINK_KEY_REPLY,

    //L2CAP_CONNECT_REQ01,
    L2CAP_CONNECT_RESP01,
    L2CAP_CONFIG_REQ01,
    L2CAP_CONFIG_RESP01,
    L2CAP_DISCONNECT_SDP,
    L2CAP_DISCONNECT_RESP01,

    L2CAP_CONNECT_RESP11,
    L2CAP_CONNECT_REQ11,
    L2CAP_CONFIG_REQ11,
    L2CAP_CONFIG_RESP11,
    L2CAP_CONNECT_RESP13,
    L2CAP_CONFIG_REQ13,
    L2CAP_CONFIG_RESP13,

    PS4_BT_INIT,
    PS4_BT_RUNNING,

    L2CAP_DISCONNECT_DATA_RESP,
    L2CAP_DISCONNECT_CTRL_RESP,
    L2CAP_DISCONNECT_DATA,
    L2CAP_DISCONNECT_CTRL,
    HCI_DISCONNECT,

    HCI_READ,

} HCI_STATE;

typedef enum {
    PS4_WIRE_READ_BD_ADDR1,
    PS4_WIRE_WRITE_BD_ADDR,
    SET_0x14,
    PS4_WIRE_READ_BD_ADDR2,
    PS4_WIRE_CONTROLLER_INIT,
    PS4_WIRE_RUNNING
} PS4_WIRE_STATE;

typedef enum {
    SPACE_NAV_INIT,
    SPACE_NAV_RUNNING
} SPACE_NAV_STATE;

// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************


BYTE deviceAddress; // Address of the device on the USB
BT_STATE DemoState; // Current state of the demo application
HCI_STATE HciState; // Current state of the demo application
PS4_WIRE_STATE PS4WireState; //state machine when usb controller is attached.
SPACE_NAV_STATE SpaceNavState;

int data_num;

union __PACKED {
    unsigned char bytes[6]; //
    unsigned int ints[3];
} localBdAddr;

BYTE remote_bd_addr[6]; //
BYTE acl_buf[DATA_PACKET_LENGTH];
BYTE hci_buf[DATA_PACKET_LENGTH];

BYTE handle[2]; //a handle for ACL
BYTE cid_dev_ctrl[2];
const BYTE cid_host_ctrl[2] = {0x70, 0x00}; //assign this channel id
BYTE cid_dev_data[2];
const BYTE cid_host_data[2] = {0x71, 0x00};
BYTE cid_dev_sdp[2];
const BYTE cid_host_sdp[2] = {0x72, 0x00};
BYTE packet_id;

char message[64];
static volatile int spi_index = 0;

//variables to access PS4 packets
CONTROLLER_IN *BTControllerInPtr = (CONTROLLER_IN*) (acl_buf + 11); //the actual data starts on byte 11, the header is 10 bytes long


unsigned int i; //general purpose index counter
static volatile unsigned int loopCounter; //counts the numerber of times the loop is executed, used for scheduling output packets
static volatile unsigned int timeoutCounter = 0; //counter for idle disconnect
static volatile BOOL configFlag = 0; //TRUE if we're in configuration mode

unsigned char vibeL;
unsigned char vibeH;
unsigned char LedAdc = 0;


char temp; //general purpose temp char
//******************************************************************************
//******************************************************************************
// Local Routines
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        ResetStates
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
 * Overview:        This routine will reset the internal states to the
 *                  power-on state.
 *************************************************************************/
void ResetStates(void) {
    UART2PrintString("***Reset States.***\n\r");
    DemoState = BT_INITIALIZE;
    HciState = HCI_CMD_RESET;
    PS4WireState = PS4_WIRE_CONTROLLER_INIT;
    SpaceNavState = SPACE_NAV_INIT;
    ResetOutputs(BTControllerInPtr);
}

/*************************************************************************
 * Function:        InitializeSystem
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         TRUE if successful, FALSE if not.
 *
 * Side Effects:    See below
 *
 * Overview:        This routine initializes the processor and peripheral,
 *                  setting clock speeds and enabling any required
 *                  features.
 *************************************************************************/
BOOL InitializeSystem(void) {
    OSCCON = 0b1111001100000000; // Enable secondary oscillator
    CLKDIV = 0x0000; // Set PLL prescaler (1:1)

    //Initialize Output pins to 0 to prevent floating outputs:
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    PORTF = 0;
    PORTG = 0;

    TRISB = 0b1100111011000000;
    TRISC = 0b1001111111111111;
    TRISD = 0b1111111100000001;
    TRISE = 0b1111111111000000;
    TRISF = 0b1111111111110100;
    TRISG = 0b1111110000111111;

    //Make PGC and PGD inputs
    _TRISB6 = 1; //PGC
    _TRISB7 = 1; //PGD
    ////////////////Map SPI Bus///////////////////////////////

    RPINR20bits.SCK1R = 2; //map SCK1IN to RP2
    RPINR20bits.SDI1R = 4; //map SDI1 to RP4
    RPINR21bits.SS1R = 12; //map SS1R to RP12
    RPOR1bits.RP3R = 7; //map RP3 to SDO1

    //Map interrupt for /SS line
    _TRISD0 = 1;
    RPINR0bits.INT1R = 11;
    IPC5bits.INT1IP = 5; //set slave select interrupt priority
    INTCON2bits.INT1EP = 0; //interrupt on positive edge
    IFS1bits.INT1IF = 0;

    ////////////////Set internal pull-up/pull-down resistors//////////////
    //Pull down programming lines
    CNPD2bits.CN24PDE = 1;
    CNPD2bits.CN25PDE = 1;

    //Pull down LED0, LED1
    CNPD2bits.CN30PDE = 1; //use resistors now in final rev
    CNPD2bits.CN31PDE = 1;
    
    //Pull Down rumble lines
    CNPD2bits.CN28PDE = 1;
    CNPD2bits.CN29PDE = 1;


    //Pull up Slave Select line
    CNPU4bits.CN56PUE = 1;


    /////////////Use Timer4 for output poll period
    T4CON = 0; //clear state
    T4CONbits.TSIDL = 0; //Continue in Idle
    T4CONbits.TGATE = 0; //No gate accumulation
    T4CONbits.TCKPS = 0b10; //Prescale 1:64
    T4CONbits.T32 = 0; //16-bit mode
    T4CONbits.TCS = 0; //Clock Source: Internal Clock (Fosc/2 = 16MHz)
    TMR4 = 0;
    PR4 = 2500; //2500 with presacle 64 is 10ms
    T4CONbits.TON = 1;
    //////////////////////////////////

    // Configure UART2 Functions
    //Assign U2RX To Pin RP10,RF4
    RPINR19bits.U2RXR = 10;
    //Assign U2TX To Pin RP17,RF5
    RPOR8bits.RP17R = 5;

    UART2Init();
    UART2PrintString("UART Initialized/.\n\r");

    ///////////////SETUP ADC////////////////////
    AD1PCFGL = 0b1111000111111111; // Configure AN9 AN10  AN11 as analog, on rev1 boards it was 10/11/12
    AD1CSSL = ~AD1PCFGL; // Include  AN9 AN10 AN11 in scan,
    AD1CON1bits.FORM = 0b00; // Fractional
    AD1CON1bits.SSRC = 0b111; //Internal counter triggers conversion
    AD1CON1bits.ASAM = 0; //manual sampling start
    AD1CON1bits.SAMP = 1;


    AD1CON2bits.CSCNA = 1; //scan channels in AD1CSSL
    AD1CON2bits.VCFG = 0; //Use AVdd and AVss as references
    AD1CON2bits.SMPI = 0b10; //interrupts after every 3 samples
    AD1CON2bits.BUFM = 0; //16bit buffer
    AD1CON2bits.ALTS = 0; //only use MUX A

    AD1CON3 = 0b0001111100111111; // Sample time = 15Tad, Tad = Tcy
    //IEC0bits.AD1IE = 1; //enable interrupt, since the DONE flag seems to be bugged.
    AD1CON1bits.ADON = 1; // turn ADC ON


    // Set Default demo state
#ifdef DEBUG_MODE
    UART2PrintString("Reset States on Init.\n\r");
#endif
    ResetStates();
    return TRUE;
} // InitializeSystem

/*************************************************************************
 * Function:        CheckForNewAttach
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          deviceAddress (global)
 *                  Updates the device address when an attach is found.
 *
 * Returns:         TRUE if a new device has been attached.  FALSE,
 *                  otherwise.
 *
 * Side Effects:    Prints attach message
 *
 * Overview:        This routine checks to see if a new device has been
 *                  attached.  If it has, it records the address.
 *************************************************************************/
/*
BOOL CheckForNewAttach ( void )
{
    // Try to get the device address, if we don't have one.
    if (deviceAddress == 0)
    {
        GENERIC_DEVICE_ID DevID;

        #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
            DevID.serialNumberLength = 0;
            DevID.serialNumber = NULL;
        #endif

        if (USBHostGenericGetDeviceAddress(&DevID))
        {
            deviceAddress = DevID.deviceAddress;
            UART2PrintString( "Generic demo device attached - polled, deviceAddress=" );
            UART2PutDec( deviceAddress );
            UART2PrintString( "\r\n" );
            return TRUE;
        }
    }

    return FALSE;

} // CheckForNewAttach
 */

//Parse HCI packets

void ParseRx1PS4BT(void) {
    unsigned int opCode;

#ifdef PRINT_HCI_PACKET
    unsigned int data_num;
    UART2PrintString("HCI_RX: ");
    for (data_num = 0; data_num < USBHostGenericGetRx1Length(deviceAddress); data_num++) {
        UART2PutHex(hci_buf[data_num]);
        UART2PutChar(' ');
    }
    UART2PrintString("-- ");
#endif
    DemoState = BT_STATE_PROCESS; //send program back to BT_STATE_PROCESS by default
    opCode = ((unsigned int) hci_buf[3])*256 + hci_buf[4];
    switch (hci_buf[0]) { //Switch by event code
        case 0x0E: //command complete event
            DemoState = BT_STATE_PROCESS;
            UART2PrintString("CMD complete: ");
            switch (opCode) {
                case 0x030c: //if (hci_buf[3] == 0x03 && hci_buf[4]==0x0c) //Reset complete code.  Assumes success/.
                    UART2PrintString("Reset.\n\r");
                    HciState = HCI_CMD_READ_BD_ADDR;
                    break;

                case 0x0910:
                    UART2PrintString("Read Address. Local BD_ADDR: ");
                    //save address
                    for (data_num = 0; data_num < 6; data_num++) {
                        localBdAddr.bytes[data_num] = hci_buf[6 + data_num];
                        UART2PutHex(localBdAddr.bytes[data_num]);
                        UART2PutChar(' ');
                    }
                    UART2PrintString(". Storing address.\n\r");
                    DataEEWrite(localBdAddr.ints[0], EEPROM_BT_ADDR_0);
                    DataEEWrite(localBdAddr.ints[1], EEPROM_BT_ADDR_1);
                    DataEEWrite(localBdAddr.ints[2], EEPROM_BT_ADDR_2);
                    UART2PutHexWord(DataEERead(EEPROM_BT_ADDR_0));
                    UART2PutHexWord(DataEERead(EEPROM_BT_ADDR_1));
                    UART2PutHexWord(DataEERead(EEPROM_BT_ADDR_2));
                    UART2PrintString("\n\r");

                    break;
                case 0x130c:
                    UART2PrintString("Local Name.\n\r");
                    break;

                case 0x240c:
                    UART2PrintString("Write Class.\n\r");
                    break;

                case 0x1a0c:
                    UART2PrintString("Write Scan Enable/Disable.\n\r");
                    break;
                case 0x0B04:
                    UART2PrintString("Link Key Reply.\n\r");
                    break;
                default:
                    UART2PrintString("Other.\n\r");
                    break;
            }
            break;

        case 0x12: //Event: Role Change Status
            if (hci_buf[2] == 0x00) {
                UART2PrintString("Role Change Successful.\n\r");
                HciState = HCI_CMD_CONNECTION_ACCEPTED;
            } else {
                UART2PrintString("Role Change Failed.  Error: ");
                UART2PutHex(hci_buf[2]);
                UART2PrintString("\n\r");
                HciState = HCI_CMD_RESET;
            }
            break;

        case 0x01:
            UART2PrintString("Inquiry Complete\n\r");
            break;

        case 0x02:
            remote_bd_addr[0] = hci_buf[3];
            remote_bd_addr[1] = hci_buf[4];
            remote_bd_addr[2] = hci_buf[5];
            remote_bd_addr[3] = hci_buf[6];
            remote_bd_addr[4] = hci_buf[7];
            remote_bd_addr[5] = hci_buf[8];

            UART2PrintString("Inquiry Result, remote BD_ADDR: ");
            for (data_num = 0; data_num < 6; data_num++) {
                UART2PutHex(remote_bd_addr[data_num]);
                UART2PutChar(' ');
            }
            UART2PrintString("\n\r");
            HciState = HCI_CMD_CREATE_CONNECTION;
            break;

        case 0x03: //connection complete
            UART2PrintString("Connection Accepted Event: ");
            if (hci_buf[2] == 0) {
                handle[0] = hci_buf[3];
                handle[1] = hci_buf[4]; //save connection handle and add (PB flag + BC flag)
                UART2PrintString("Success. Handle = ");
                UART2PutHex(hci_buf[4]);
                UART2PutHex(hci_buf[3]);
                HciState = HCI_READ; //wait for incoming request from controller
            } else {
                UART2PrintString("Error: ");
                UART2PutHex(hci_buf[2]);
                HciState = HCI_CMD_RESET;
            }
            UART2PrintString("\n\r");
            break;

        case 0x04: //incoming connection attempt
            if (HciState == HCI_CMD_SCAN_ENABLE_WAIT) {
                UART2PrintString("Incoming Connection: Remote_Bd_Addr = ");
                remote_bd_addr[0] = hci_buf[2];
                remote_bd_addr[1] = hci_buf[3];
                remote_bd_addr[2] = hci_buf[4];
                remote_bd_addr[3] = hci_buf[5];
                remote_bd_addr[4] = hci_buf[6];
                remote_bd_addr[5] = hci_buf[7];

                for (data_num = 0; data_num < 6; data_num++) {
                    UART2PutHex(remote_bd_addr[data_num]);
                    UART2PutChar(' ');
                }
                UART2PrintString("\n\r");
                HciState = HCI_CMD_INCOMING_ACCEPT;
            }
            break;

        case 0x05:
            UART2PrintString("Disconnect Event: ");
            if (hci_buf[2] == 0) {
                UART2PrintString("Success.");
                HciState = HCI_CMD_RESET;
            } else {
                UART2PrintString("Error.");
            }
            UART2PrintString("\n\r");
            ResetStates();
            break;

        case 0x08:
            UART2PrintString("Encryption change:");
            if (hci_buf[2] == 0 && hci_buf[5] == 1) {
                UART2PrintString("Channel Encryption On. Handle=");
                UART2PutHex(hci_buf[3]);
                UART2PutHex(hci_buf[4]);
            } else {
                UART2PrintString("Error.");
            }
            UART2PrintString("\n\r");
            break;

        case 0x0F:
            UART2PrintString("Command Status Event: ");
            switch (hci_buf[2]) {
                case 0x00:
                    UART2PrintString("Pending.");
                    DemoState = BT_STATE_READ_DATA;
                    break;

                case 0x02:
                    UART2PrintString("Error 0x02: Unknown Connection ID.");
                    HciState = HCI_CMD_RESET;
                    break;

                default:
                    UART2PrintString("Error Code:");
                    UART2PutHex(hci_buf[2]);
                    HciState = HCI_CMD_RESET;
                    break;
            }

            UART2PrintString("\n\r");
            break;

        case 0x13:
#ifdef PRINT_HCI_PACKET
            UART2PrintString("Completed Packets.\n\r");
            DemoState = BT_STATE_READ_DATA;
#endif
            break;

        case 0x17:
            UART2PrintString("Link Key Request.  BD_ADDR:");
            for (data_num = 0; data_num < 6; data_num++) {
                UART2PutHex(hci_buf[data_num + 1]);
                UART2PutChar(' ');
            }
            UART2PrintString("\n\r");
            HciState = LINK_KEY_REPLY;
            break;

        case 0x1B:
            UART2PrintString("Max Slots Change.\n\r");
            HciState = HCI_READ;
            break;
            
#ifdef DEBUG_MODE
        case 0x20:
            UART2PrintString("Page Scan Repetition Mode Change.\n\r");
            break;
#endif

        default:
#ifdef PRINT_HCI_PACKET
            UART2PrintString("Unknown.\n\r");
#endif

            DemoState = BT_STATE_READ_DATA;
            break;
    }

}


//Parse ACL Packet

void ParseRx2PS4BT(void) {
#ifdef PRINT_ACL_PACKET
    UART2PrintString("ACL_RX: ");
    for (data_num = 0; data_num < USBHostGenericGetRx2Length(deviceAddress); data_num++) {
        UART2PutHex(acl_buf[data_num]);
        UART2PutChar(' ');
    }
    UART2PrintString("-- ");
    UART2PrintString("CID=0x");
    UART2PutHex(acl_buf[7]);
    UART2PutHex(acl_buf[6]);
    UART2PrintString(". ");
#endif

    DemoState = BT_STATE_PROCESS; //send program back to BT_STATE_PROCESS

    packet_id = acl_buf[9];

    if (acl_buf[6] == 1) //control channel
    {
        switch (acl_buf[8]) {
            case 0x01:
                UART2PrintString("Code=0x01. Command Rejected.\n\r");
                break;

            case 0x02:
                UART2PrintString("Code=0x02.(Connection Req).");
                if (acl_buf[12] == 0x01 && acl_buf[13] == 0x00) {
                    UART2PrintString("PSM=0x0001 (SDP). SCID=");
                    UART2PutHex(acl_buf[15]);
                    UART2PutHex(acl_buf[14]);
                    UART2PrintString("\n\r");
                    cid_dev_sdp[0] = acl_buf[14];
                    cid_dev_sdp[1] = acl_buf[15];
                    HciState = L2CAP_CONNECT_RESP01;
                } else if (acl_buf[12] == 0x11 && acl_buf[13] == 0x00) {
                    UART2PrintString("PSM=0x0011 (HID_CTRL). SCID=");
                    UART2PutHex(acl_buf[15]);
                    UART2PutHex(acl_buf[14]);
                    UART2PrintString("\n\r");
                    cid_dev_ctrl[0] = acl_buf[14];
                    cid_dev_ctrl[1] = acl_buf[15];
                    HciState = L2CAP_CONNECT_RESP11;
                } else if (acl_buf[12] == 0x13 && acl_buf[13] == 0x00) {
                    UART2PrintString("PSM=0x0013. SCID=");
                    UART2PutHex(acl_buf[15]);
                    UART2PutHex(acl_buf[14]);
                    UART2PrintString("\n\r");
                    cid_dev_data[0] = acl_buf[14];
                    cid_dev_data[1] = acl_buf[15];
                    HciState = L2CAP_CONNECT_RESP13;
                }
                else {
                    UART2PrintString("!!!Error: PSM not supported.\n\r");
                }
                break;

            case 0x03:
                UART2PrintString("Connection Complete:");
                if (acl_buf[16] == 0) {
                    UART2PrintString("Success. ");
                    if (acl_buf[14] == cid_host_ctrl[0] && acl_buf[15] == cid_host_ctrl[1]) {
                        UART2PrintString("HID CTRL (PSM 0x11)\n\r");
                        cid_dev_ctrl[0] = acl_buf[12];
                        cid_dev_ctrl[1] = acl_buf[13];
                        HciState = L2CAP_CONNECT_RESP11;
                    } else if (acl_buf[14] == cid_host_data[0] && acl_buf[15] == cid_host_data[1]) {
                        UART2PrintString("HID DATA (PSM 0x13)\n\r");
                        cid_dev_data[0] = acl_buf[12];
                        cid_dev_data[1] = acl_buf[13];
                        HciState = L2CAP_CONNECT_RESP13;
                    } else {
                        UART2PrintString("Unknown\n\r");
                    }
                } else if (acl_buf[16] == 1) {
                    UART2PrintString("Pending...\n\r");
                } else {
                    UART2PrintString("Failure. Error:");
                    UART2PutHex(acl_buf[16]);
                    UART2PrintString("\n\r");
                }
                break;

            case 0x04:
                UART2PrintString("Code=0x04(Config Req):");
                if (acl_buf[12] == cid_host_ctrl[0] && acl_buf[13] == cid_host_ctrl[1]) {
                    UART2PrintString("HID Control Channel.\n\r");
                    HciState = L2CAP_CONFIG_RESP11;
                } else if (acl_buf[12] == cid_host_data[0] && acl_buf[13] == cid_host_data[1]) {
                    UART2PrintString("HID Data Channel.\n\r");
                    HciState = L2CAP_CONFIG_RESP13;
                } else if (acl_buf[12] == cid_host_sdp[0] && acl_buf[13] == cid_host_sdp[1]) {
                    UART2PrintString("SDP Channel.\n\r");
                    HciState = L2CAP_CONFIG_RESP01;
                }
                else {
                    UART2PrintString("!!!Error: Unknown Channel ID.\n\r");
                }
                break;

            case 0x05:
                UART2PrintString("Code=0x05(Config Resp). ");
                if (acl_buf[16] == 0) {
                    UART2PrintString("Success\n\r");
                } else {
                    UART2PrintString("Fail.\n\r");
                }
                break;

            case 0x06:
                UART2PrintString("Disconnect request:");
                if (acl_buf[12] == cid_host_data[0] && acl_buf[13] == cid_host_data[1]) {
                    UART2PutHex(cid_host_data[0]);
                    UART2PrintString(" DATA.\n\r");
                    HciState = L2CAP_DISCONNECT_DATA_RESP;
                } else if (acl_buf[12] == cid_host_ctrl[0] && acl_buf[13] == cid_host_ctrl[1]) {
                    UART2PutHex(cid_host_ctrl[0]);
                    UART2PrintString(" CTRL.\n\r");
                    HciState = L2CAP_DISCONNECT_CTRL_RESP;
                } else if (acl_buf[12] == cid_host_sdp[0] && acl_buf[13] == cid_host_sdp[1]) {
                    UART2PutHex(cid_host_sdp[0]);
                    UART2PrintString(" SDP.\n\r");
                    HciState = L2CAP_DISCONNECT_RESP01;
                }
                else {
                    UART2PrintString(" Unknown Channel.\n\r");
                }
                break;

            case 0x07:
                UART2PrintString("Disconnect Response:");
                if (acl_buf[12] == cid_dev_data[0] && acl_buf[13] == cid_dev_data[1]) {
                    UART2PutHex(cid_dev_data[0]);
                    UART2PrintString(" DATA.\n\r");
                    HciState = L2CAP_DISCONNECT_CTRL;
                } else if (acl_buf[12] == cid_dev_ctrl[0] && acl_buf[13] == cid_dev_ctrl[1]) {
                    UART2PutHex(cid_dev_ctrl[0]);
                    UART2PrintString(" CTRL.\n\r");
                    HciState = HCI_DISCONNECT;
                } else if (acl_buf[12] == cid_dev_sdp[0] && acl_buf[13] == cid_dev_sdp[1]) {
                    UART2PutHex(cid_dev_sdp[0]);
                    UART2PrintString(" SDP.\n\r");
                    HciState = HCI_READ;
                }
                break;
            default:
                UART2PrintString("\n\r");
                break;

        }
    } 

    else if (acl_buf[6] == cid_host_sdp[0]) {
        UART2PrintString("Ch:SDP.\n\r");
        switch (acl_buf[8]) {
            case 0x06:
                UART2PrintString("SDP Attribute Request.\n\r");
                break;
        }
        
    } else if (acl_buf[6] == cid_host_data[0]) {
#ifdef PRINT_ACL_PACKET
        UART2PrintString("Ch:HID_DATA.\n\r");
#endif
        switch (acl_buf[8]) {
            case 0xA1: //Data from controller
                if (acl_buf[9] == 0x11) //Input Report
                {
                    CopyPS4Report(BTControllerInPtr);
                }
                break;
        }
    } else if (acl_buf[6] == cid_host_ctrl[0]) {
#ifdef PRINT_ACL_PACKET
        UART2PrintString("Ch:HID_CTRL.\n\r");
#endif
        switch (acl_buf[8]) {
        }
    } else {
#ifdef PRINT_ACL_PACKET
        UART2PrintString("Ch:Other.\n\r");
#endif
    }
}

void ParseRx1PS4Wire(void) {
#ifdef PRINT_HCI_PACKET
    unsigned int data_num;
    UART2PrintString("HCI_RX: ");
    for (data_num = 0; data_num < 64; data_num++) {
        UART2PutHex(hci_buf[data_num]);
        UART2PutChar(' ');
    }
    UART2PrintString("-- ");
#endif
    if (hci_buf[0] == 0x01) {
#ifdef PRINT_HCI_PACKET
        UART2PrintString("Input Report.");
#endif
        CopyPS4Report((CONTROLLER_IN*) hci_buf);
    }

#ifdef PRINT_HCI_PACKET
    UART2PrintString("\n\r");
#endif
}
void ParseRx1SpaceNavigator(void)
{

#ifdef PRINT_HCI_PACKET
    unsigned int data_num;
    UART2PrintString("HCI_RX: ");
    for (data_num = 0; data_num < USBHostGenericGetRx1Length(deviceAddress); data_num++) {
        UART2PutHex(hci_buf[data_num]);
        UART2PutChar(' ');
    }
    UART2PrintString("\n\r");
#endif
    CopySpaceNavigatorReport(hci_buf, USBHostGenericGetRx1Length(deviceAddress) );
}

void ParseRx1Thrustmaster(void) {
#ifdef PRINT_HCI_PACKET
    unsigned int data_num;
    UART2PrintString("HCI_RX: ");
    for (data_num = 0; data_num < 64; data_num++) {
        UART2PutHex(hci_buf[data_num]);
        UART2PutChar(' ');
    }
    UART2PrintString("-- ");
#endif
    if (hci_buf[0] == 0x01) {
#ifdef PRINT_HCI_PACKET
        UART2PrintString("Input Report.");
#endif
        CopyThrustmasterReport((CONTROLLER_IN*) hci_buf);
    }

#ifdef PRINT_HCI_PACKET
    UART2PrintString("\n\r");
#endif
}

/*Write functions for USB*/
BYTE Write_HCI_Command(char *message, BYTE *hci_buf, unsigned int data_size1) {
    BYTE returnVal;

    returnVal = USBHostGenericClassRequest(deviceAddress, hci_buf, data_size1);
    if (returnVal == USB_SUCCESS) {
#ifdef DEBUG_MODE
        unsigned int data_num;
        UART2PrintString(message);
        for (data_num = 0; data_num < data_size1; data_num++) {
            UART2PutHex(hci_buf[data_num]);
            UART2PutChar(' ');
        }
        UART2PrintString("\r\n");
#endif
        //UART2PrintString( "HCI COMMAND SENT\r\n" );
    } else {
        UART2PrintString("Write Class Error: ");
        UART2PutHex(returnVal);
        UART2PrintString("\n\r");
    }

    return returnVal;
}

BYTE Write_ACL_Command(char *message, BYTE *acl_buf, int data_size2) {
    BYTE retVal = USB_BUSY;
    retVal = USBHostGenericAclWrite(deviceAddress, acl_buf, data_size2);
    if (retVal == USB_SUCCESS) {
#ifdef DEBUG_MODE
        unsigned int data_num;
        UART2PrintString(message);
        for (data_num = 0; data_num < data_size2; data_num++) {
            UART2PutHex(acl_buf[data_num]);
            UART2PutChar(' ');
        }
        UART2PrintString("\r\n");
#endif
        // UART2PrintString( "ACL_DATA_SENT!\n\r" );
    } 
    else {
        UART2PutHex(retVal);
        UART2PrintString(" - ACL cannot be written ");
#ifdef DEBUG_MODE 
        UART2PrintString("DemoState:");
        UART2PutDec(DemoState);
        UART2PrintString("HciState:");
        UART2PutDec(HciState);
        UART2PrintString("\r\n");
        for (data_num = 0; data_num < data_size2; data_num++) {
            UART2PutHex(acl_buf[data_num]);
            UART2PutChar(' ');
        }
        UART2PrintString("end debug\n\r");
#endif
    }
    DelayMs(1);
    return retVal;
}

BYTE WriteOutputReport(void) {
    static unsigned char brightness = 0;
    BYTE retVal;
    brightness++;
    switch (GetConnectedDeviceType()) {
        case BLUETOOTH:
            strcpy(message, "BT Output: ");
            retVal = Write_ACL_Command(message, GetOutputReportBT(), 87);
            break;

        case CONTROLLER:
            retVal = USBHostGenericWrite(deviceAddress, GetOutputReportWire(), 32);
            break;

        default:
            UART2PrintString("Error: Cannot find correct type.\n\r");
            retVal = USB_ILLEGAL_REQUEST;
            break;
    }
    return retVal;
}

/*************************************************************************
 * Function:        ManageStateBluetooth
 *
 * Preconditions:   The DemoState global variable must be initialized to
 *                  DEMO_STATE_IDLE (0).  (This occurs on reset.)
 *
 * Input:           DemoState (global)
 *                  Actions selected based value of DemoState on function
 *                  entry.
 *
 *                  deviceAddress (global)
 *                  May use device address to access device, depending on
 *                  state.
 *
 *                  DataPacket (global)
 *                  May read data from packet buffer, depending on state.
 *
 * Output:          DemoState (global)
 *                  Updates demo state as appropriate.
 *
 *                  DataPacket (global)
 *                  May cause data in the packet buffer to be updated,
 *                  depending on state.
 *
 * Returns:         None
 *
 * Side Effects:    Depend on state transition
 *
 * Overview:        This routine maintains the state of the application,
 *                  updateing global data and taking actions as necessary
 *                  to maintain the custom demo operations.
 *************************************************************************/
void ManageStateBluetooth(void) {
    packet_id++;
    /*
        UART2PutHex(DemoState);
        UART2PutChar('-');
        UART2PutHex(HciState);
        UART2PutChar(' ');
     */
    // Watch for device detaching
    if (USBHostGenericDeviceDetached(deviceAddress) && deviceAddress != 0) {
#ifdef DEBUG_MODE
        UART2PrintString("Bluetooth adapter detached - polled\r\n");
#endif
        ResetStates();
        SetConnectedDeviceType(NONE);
    }

    switch (DemoState) {
        case BT_INITIALIZE:
            UART2PrintString("INIT\n\r");
            DemoState = BT_STATE_PROCESS;
            HciState = HCI_CMD_RESET; //MH
            break;


        case BT_STATE_PROCESS:
            switch (HciState) {
                    //HCI layer***********************************************************************
                case HCI_CMD_RESET:
                    timeoutCounter = 0;
                    ResetOutputs(BTControllerInPtr);
                    DelayMs(100); //pause a bit to let whatever transactions were going on finish.
                    hci_buf[0] = 0x03;
                    hci_buf[1] = 0x0c;
                    hci_buf[2] = 0;

                    strcpy(message, "HCI_CMD_RESET: "); //message for BT_STATE_READ_HCI
                    if (Write_HCI_Command(message, hci_buf, 3) == USB_SUCCESS) {
                        HciState = HCI_CMD_READ_BD_ADDR;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                case HCI_CMD_READ_BD_ADDR:
                    hci_buf[0] = 0x09;
                    hci_buf[1] = 0x10;
                    hci_buf[2] = 0;

                    strcpy(message, "HCI_CMD_READ_BD_ADDR: ");
                    if (Write_HCI_Command(message, hci_buf, 3) == USB_SUCCESS) {
                        HciState = HCI_CMD_LOCAL_NAME;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                    //********************************************************************************
                case HCI_CMD_LOCAL_NAME:
                    hci_buf[0] = 0x13;
                    hci_buf[1] = 0x0c;
                    hci_buf[2] = 0x04;
                    hci_buf[3] = 'C';
                    hci_buf[4] = 'P';
                    hci_buf[5] = 'C';
                    hci_buf[6] = 0x00;

                    strcpy(message, "HCI_CMD_LOCAL_NAME: ");
                    if (Write_HCI_Command(message, hci_buf, 7) == USB_SUCCESS) {
                        HciState = HCI_CMD_CLASS_DEVICE;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                    //********************************************************************************
                case HCI_CMD_CLASS_DEVICE:
                    hci_buf[0] = 0x24;
                    hci_buf[1] = 0x0c;
                    hci_buf[2] = 0x03;
                    hci_buf[3] = 0x08; //gamepad
                    hci_buf[4] = 0x05; //joystick
                    hci_buf[5] = 0x00;

                    strcpy(message, "HCI_CMD_CLASS_DEVICE: ");
                    if (Write_HCI_Command(message, hci_buf, 6) == USB_SUCCESS) {
                        HciState = HCI_CMD_SCAN_ENABLE;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                case HCI_CMD_SCAN_ENABLE:
                    hci_buf[0] = 0x1a;
                    hci_buf[1] = 0x0c;
                    hci_buf[2] = 0x01;
                    hci_buf[3] = 0x03; //enable page

                    strcpy(message, "HCI_CMD_SCAN_ENABLE: ");
                    if (Write_HCI_Command(message, hci_buf, 4) == USB_SUCCESS) {
                        HciState = HCI_CMD_SCAN_ENABLE_WAIT;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                case HCI_CMD_INQUIRY:
                    hci_buf[0] = 0x01;
                    hci_buf[1] = 0x01 << 2;
                    hci_buf[2] = 0x05; // Parameter Total Length = 5
                    hci_buf[3] = 0x33; // LAP: Genera/Unlimited Inquiry Access Code (GIAC = 0x9E8B33) - see https://www.bluetooth.org/Technical/AssignedNumbers/baseband.htm
                    hci_buf[4] = 0x8B;
                    hci_buf[5] = 0x9E;
                    hci_buf[6] = 0x30; // Inquiry time = 61.44 sec (maximum)
                    hci_buf[7] = 0x0A; // 10 number of responses


                    strcpy(message, "HCI_CMD_INQUIRY: ");
                    if (Write_HCI_Command(message, hci_buf, 8) == USB_SUCCESS) {
                        HciState = HCI_CMD_SCAN_ENABLE_WAIT;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;


                case HCI_CMD_SCAN_ENABLE_WAIT:
                    USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
                    DemoState = BT_STATE_READ_DATA;
                    break;

                case HCI_CMD_CREATE_CONNECTION:
                    hci_buf[0] = 0x05;
                    hci_buf[1] = 0x01 << 2; // HCI OGF = 1
                    hci_buf[2] = 0x0D; // parameter Total Length = 13
                    hci_buf[3] = remote_bd_addr[0]; // 6 octet bdaddr (LSB)
                    hci_buf[4] = remote_bd_addr[1];
                    hci_buf[5] = remote_bd_addr[2];
                    hci_buf[6] = remote_bd_addr[3];
                    hci_buf[7] = remote_bd_addr[4];
                    hci_buf[8] = remote_bd_addr[5];
                    hci_buf[9] = 0x18; // DM1 or DH1 may be used
                    hci_buf[10] = 0xCC; // DM3, DH3, DM5, DH5 may be used
                    hci_buf[11] = 0x01; // Page repetition mode R1
                    hci_buf[12] = 0x00; // Reserved
                    hci_buf[13] = 0x00; // Clock offset
                    hci_buf[14] = 0x00; // Invalid clock offset
                    hci_buf[15] = 0x00; // Do not allow role switch

                    strcpy(message, "HCI_CMD_CREATE_CONNECTION: ");
                    if (Write_HCI_Command(message, hci_buf, 16) == USB_SUCCESS) {
                        HciState = HCI_CMD_SCAN_ENABLE_WAIT;
                        DemoState = BT_STATE_READ_DATA;
                    };

                    break;

                case HCI_CMD_INCOMING_ACCEPT: //Accept connection, role change to master
                    hci_buf[0] = 0x09;
                    hci_buf[1] = 0x04;
                    hci_buf[2] = 0x07;
                    hci_buf[3] = remote_bd_addr[0]; //******************************************
                    hci_buf[4] = remote_bd_addr[1]; //BD address (6 octets) of the slave bluetooth
                    hci_buf[5] = remote_bd_addr[2]; //
                    hci_buf[6] = remote_bd_addr[3]; //
                    hci_buf[7] = remote_bd_addr[4]; //
                    hci_buf[8] = remote_bd_addr[5]; //******************************************
                    hci_buf[9] = 0x00;

                    strcpy(message, "HCI_CMD_ACCEPT_INCOMING. ROLE CHANGE REQUESTED: ");
                    if (Write_HCI_Command(message, hci_buf, 10) == USB_SUCCESS) {
                        HciState = HCI_ROLE_CHANGE_WAIT;
                        DemoState = BT_STATE_READ_DATA;
                    };
                    break;

                case HCI_ROLE_CHANGE_WAIT:
                    USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
                    break;

                case HCI_CMD_CONNECTION_ACCEPTED:
                    HciState = HCI_CMD_SCAN_DISABLE;
                    break;

                case HCI_CMD_SCAN_DISABLE: //wait for controller to initiate L2CAP Connection Request
                    hci_buf[0] = 0x1a;
                    hci_buf[1] = 0x0c;
                    hci_buf[2] = 0x01;
                    hci_buf[3] = 0x00; //disable scan
                    strcpy(message, "HCI_CMD_SCAN_DISABLE: ");
                    if (Write_HCI_Command(message, hci_buf, 4) == USB_SUCCESS) {
                        HciState = HCI_READ;
                    };
                    break;

                case LINK_KEY_REPLY: //wait for controller to initiate L2CAP Connection Request
                    hci_buf[0] = 0x0B;
                    hci_buf[1] = 0x01 << 2;
                    hci_buf[2] = 22;
                    hci_buf[3] = remote_bd_addr[0];
                    hci_buf[4] = remote_bd_addr[1];
                    hci_buf[5] = remote_bd_addr[2];
                    hci_buf[6] = remote_bd_addr[3];
                    hci_buf[7] = remote_bd_addr[4];
                    hci_buf[8] = remote_bd_addr[5];
                    hci_buf[9] = 0x56;
                    hci_buf[10] = 0xE8;
                    hci_buf[11] = 0x81;
                    hci_buf[12] = 0x38;
                    hci_buf[13] = 0x08;
                    hci_buf[14] = 0x06;
                    hci_buf[15] = 0x51;
                    hci_buf[16] = 0x41;
                    hci_buf[17] = 0xC0;
                    hci_buf[18] = 0x7F;
                    hci_buf[19] = 0x12;
                    hci_buf[20] = 0xAA;
                    hci_buf[21] = 0xD9;
                    hci_buf[22] = 0x66;
                    hci_buf[23] = 0x3C;
                    hci_buf[24] = 0xCE;


                    strcpy(message, "LINK_KEY_REPLY: ");
                    if (Write_HCI_Command(message, hci_buf, 25) == USB_SUCCESS) {
                        HciState = HCI_READ;
                    };
                    break;
                    //HCI_ACL ************************************************************************
                    //Respond to PS4 controller's request to open a connection channel for HID_Control 0x0011
                    /*              
                              case L2CAP_CONNECT_REQ01:
                                  packet_id = 1;
                                  acl_buf[0]=handle[0];
                                  acl_buf[1]=handle[1]+0x20;   
                                  acl_buf[2]=0x0C;//length
                                  acl_buf[3]=0x00;
                                  acl_buf[4]=0x08; //length
                                  acl_buf[5]=0x00;
                                  acl_buf[6]=0x01; //cid
                                  acl_buf[7]=0x00;
                                  acl_buf[8]=0x02; //code
                                  acl_buf[9]=0x01; //packet id
                                  acl_buf[10]=0x04; //length
                                  acl_buf[11]=0x00;
                                  acl_buf[12]=0x11;//HID PSM
                                  acl_buf[13]=0x00;
                                  acl_buf[14]=cid_host_ctrl[0];
                                  acl_buf[15]=cid_host_ctrl[1];

                                  strcpy(message,"L2CAP_CONNECT_REQ01 ");
                                  if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS)
                                  {
                                      HciState = HCI_READ; 
                                  }
                
                                  break;
                     */
                case L2CAP_CONNECT_RESP01:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x03;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_sdp[0];
                    acl_buf[13] = cid_host_sdp[1];
                    acl_buf[14] = cid_dev_sdp[0];
                    acl_buf[15] = cid_dev_sdp[1];
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;
                    acl_buf[18] = 0x00;
                    acl_buf[19] = 0x00;

                    UART2PrintString("SCID CTRL = ");
                    UART2PutHex(cid_host_ctrl[0]);
                    UART2PrintString("\n\r");
                    strcpy(message, "L2CAP_CONNECT_RESP01: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = L2CAP_CONFIG_REQ01; //No response from controller, go to config request
                    }

                    break;
                    //Config request for Ch 0x0001 (SDP
                case L2CAP_CONFIG_REQ01: //config request
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10; //length
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c; //length
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01; //CID
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x04;
                    acl_buf[9] = 0x01;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_sdp[0];
                    acl_buf[13] = cid_dev_sdp[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x01;
                    acl_buf[17] = 0x02;
                    acl_buf[18] = 0x40;
                    acl_buf[19] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_REQ01: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = HCI_READ; //Wait for response
                    }
                    break;

                case L2CAP_CONFIG_RESP01:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0e;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0a;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x05;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x06;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_sdp[0];
                    acl_buf[13] = cid_dev_sdp[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_RESP01: ");
                    if (Write_ACL_Command(message, acl_buf, 18) == USB_SUCCESS) {
                        HciState = L2CAP_DISCONNECT_SDP; //wait for response
                        DemoState = BT_STATE_READ_DATA;
                    }
                    break;

                case L2CAP_DISCONNECT_SDP:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x06; //disconnect command
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_sdp[0];
                    acl_buf[13] = cid_dev_sdp[1];
                    acl_buf[14] = cid_host_sdp[0];
                    acl_buf[15] = cid_host_sdp[1];
                    strcpy(message, "L2CAP_DISCONNECT_SDP_REQ: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = HCI_READ;
                    }
                    break;

                case L2CAP_DISCONNECT_RESP01:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x06;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_sdp[0];
                    acl_buf[13] = cid_host_sdp[1];
                    acl_buf[14] = cid_dev_sdp[0];
                    acl_buf[15] = cid_dev_sdp[1];

                    strcpy(message, "L2CAP_DISCONNECT_RESP01: ");
                    if (Write_ACL_Command(message, acl_buf, 18) == USB_SUCCESS) {
                        HciState = HCI_READ; //wait for response
                    }
                    break;

                case L2CAP_CONNECT_REQ11:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0C;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x02;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = 0x011; //PSM = 0x0011
                    acl_buf[13] = 0x00;
                    acl_buf[14] = cid_host_ctrl[0];
                    acl_buf[15] = cid_host_ctrl[1];

                    UART2PrintString("SCID CTRL = ");
                    UART2PutHex(cid_host_ctrl[0]);
                    UART2PrintString("\n\r");
                    strcpy(message, "L2CAP_CONNECT_REQ11: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = HCI_READ; //No response from controller, go to config request
                    }
                    break;

                case L2CAP_CONNECT_RESP11:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x03;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_ctrl[0];
                    acl_buf[13] = cid_host_ctrl[1];
                    acl_buf[14] = cid_dev_ctrl[0];
                    acl_buf[15] = cid_dev_ctrl[1];
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;
                    acl_buf[18] = 0x00;
                    acl_buf[19] = 0x00;

                    UART2PrintString("SCID CTRL = ");
                    UART2PutHex(cid_host_ctrl[0]);
                    UART2PrintString("\n\r");
                    strcpy(message, "L2CAP_CONNECT_RESP11: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = L2CAP_CONFIG_REQ11; //No response from controller, go to config request
                    }
                    break;

                    //Config request for Ch 0x0011
                case L2CAP_CONFIG_REQ11: //config request
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x04;
                    acl_buf[9] = 0x01;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_ctrl[0];
                    acl_buf[13] = cid_dev_ctrl[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x01;
                    acl_buf[17] = 0x02;
                    acl_buf[18] = 0x40;
                    acl_buf[19] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_REQ11: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = HCI_READ; //Wait for response
                    }
                    break;

                    //Respond to config request for Ch 0x0011
                case L2CAP_CONFIG_RESP11:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0e;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0a;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x05;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x06;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_ctrl[0];
                    acl_buf[13] = cid_dev_ctrl[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_RESP11: ");
                    if (Write_ACL_Command(message, acl_buf, 18) == USB_SUCCESS) {
                        HciState = HCI_READ; //wait for response
                    }
                    break;

                    //Respond to PS4 controller's request to open a connection channel for HID_Interrupt 0x0013
                case L2CAP_CONNECT_RESP13:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x03;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_data[0];
                    acl_buf[13] = cid_host_data[1];
                    acl_buf[14] = cid_dev_data[0];
                    acl_buf[15] = cid_dev_data[1];
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;
                    acl_buf[18] = 0x00;
                    acl_buf[19] = 0x00;

                    UART2PrintString("SCID DATA = ");
                    UART2PutHex(cid_host_data[0]);
                    UART2PrintString("\n\r");
                    strcpy(message, "L2CAP_CONNECT_RESP13: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = L2CAP_CONFIG_REQ13;
                    }
                    break;

                case L2CAP_CONFIG_REQ13:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x10;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0c;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x04;
                    acl_buf[9] = 0x02;
                    acl_buf[10] = 0x08;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_data[0];
                    acl_buf[13] = cid_dev_data[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x01;
                    acl_buf[17] = 0x02;
                    acl_buf[18] = 0x40;
                    acl_buf[19] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_REQ13: ");
                    if (Write_ACL_Command(message, acl_buf, 20) == USB_SUCCESS) {
                        HciState = HCI_READ; //wait for response
                    }
                    break;

                case L2CAP_CONFIG_RESP13:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0e;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x0a;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x05;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x06;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_data[0];
                    acl_buf[13] = cid_dev_data[1];
                    acl_buf[14] = 0x00;
                    acl_buf[15] = 0x00;
                    acl_buf[16] = 0x00;
                    acl_buf[17] = 0x00;

                    strcpy(message, "L2CAP_CONFIG_RESP13: ");
                    if (Write_ACL_Command(message, acl_buf, 18) == USB_SUCCESS) {
                        HciState = PS4_BT_INIT;
                        DemoState = BT_STATE_READ_DATA;
                    }
                    break;

                case PS4_BT_INIT:
                    //Initialize output packet values
                    GetOutputReportBT()[0] = handle[0];
                    GetOutputReportBT()[1] = handle[1] + 0x20;
                    GetOutputReportBT()[2] = 83;
                    GetOutputReportBT()[3] = 0x00;
                    GetOutputReportBT()[4] = 79;
                    GetOutputReportBT()[5] = 0x00;
                    GetOutputReportBT()[6] = cid_dev_ctrl[0];
                    GetOutputReportBT()[7] = cid_dev_ctrl[1];
                    GetOutputReportBT()[8] = 0x52; //0 HID BT Set_report (0x50) | Report Type (Output 0x02)
                    GetOutputReportBT()[9] = 0x11; //1 Report ID
                    GetOutputReportBT()[10] = 0x80; //2
                    GetOutputReportBT()[11] = 0x00; //3
                    GetOutputReportBT()[12] = 0xff; //4 enable
                    GetOutputReportBT()[13] = 0x00; //5
                    GetOutputReportBT()[14] = 0x00; //6


                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x06;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x02;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = cid_dev_ctrl[0];
                    acl_buf[7] = cid_dev_ctrl[1];
                    acl_buf[8] = 0x43;
                    acl_buf[9] = 0x02;

                    strcpy(message, "GET FEATURE 0x02 (PS4_BT_INIT): ");
                    if (Write_ACL_Command(message, acl_buf, 10) == USB_SUCCESS) {
                        HciState = PS4_BT_RUNNING;
                    }
                    break;

                case PS4_BT_RUNNING:
                    USBHostGenericAclRead(deviceAddress, acl_buf, DATA_PACKET_LENGTH);
                    USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
                    break;

                case L2CAP_DISCONNECT_DATA_RESP:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x07;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_data[0];
                    acl_buf[13] = cid_host_data[1];
                    acl_buf[14] = cid_dev_data[0];
                    acl_buf[15] = cid_dev_data[1];
                    strcpy(message, "L2CAP_DISCONNECT_DATA_RESP: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = PS4_BT_RUNNING;
                    }
                    break;

                case L2CAP_DISCONNECT_CTRL_RESP:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x07;
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_host_ctrl[0];
                    acl_buf[13] = cid_host_ctrl[1];
                    acl_buf[14] = cid_dev_ctrl[0];
                    acl_buf[15] = cid_dev_ctrl[1];
                    strcpy(message, "L2CAP_DISCONNECT_CTRL_RESP: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = PS4_BT_RUNNING;
                    }
                    break;

                case L2CAP_DISCONNECT_DATA:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x06; //disconnect command
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_data[0];
                    acl_buf[13] = cid_dev_data[1];
                    acl_buf[14] = cid_host_data[0];
                    acl_buf[15] = cid_host_data[1];
                    strcpy(message, "L2CAP_DISCONNECT_DATA: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = HCI_READ;
                        DemoState = BT_STATE_READ_DATA;
                    }
                    break;

                case L2CAP_DISCONNECT_CTRL:
                    acl_buf[0] = handle[0];
                    acl_buf[1] = handle[1] + 0x20;
                    acl_buf[2] = 0x0c;
                    acl_buf[3] = 0x00;
                    acl_buf[4] = 0x08;
                    acl_buf[5] = 0x00;
                    acl_buf[6] = 0x01;
                    acl_buf[7] = 0x00;
                    acl_buf[8] = 0x06; //disconnect command
                    acl_buf[9] = packet_id;
                    acl_buf[10] = 0x04;
                    acl_buf[11] = 0x00;
                    acl_buf[12] = cid_dev_ctrl[0];
                    acl_buf[13] = cid_dev_ctrl[1];
                    acl_buf[14] = cid_host_ctrl[0];
                    acl_buf[15] = cid_host_ctrl[1];
                    strcpy(message, "L2CAP_DISCONNECT_DATA: ");
                    if (Write_ACL_Command(message, acl_buf, 16) == USB_SUCCESS) {
                        HciState = HCI_READ;
                        DemoState = BT_STATE_READ_DATA;
                    }
                    break;

                case HCI_DISCONNECT:
                    hci_buf[0] = 0x06;
                    hci_buf[1] = 0x01 << 2;
                    hci_buf[2] = 0x03;
                    hci_buf[3] = handle[0];
                    hci_buf[4] = handle[1];
                    hci_buf[5] = 0x13; //user terminated connection

                    strcpy(message, "HCI_DISCONNECT: ");
                    if (Write_HCI_Command(message, hci_buf, 6) == USB_SUCCESS) {
                        ResetStates();
                    }
                    break;

                case HCI_READ:
                    USBHostGenericAclRead(deviceAddress, acl_buf, DATA_PACKET_LENGTH);
                    USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
                    break;
            }
            break;

        case BT_STATE_READ_DATA:
            USBHostGenericAclRead(deviceAddress, acl_buf, DATA_PACKET_LENGTH);
            USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
            break;

        default:
            UART2PrintString("Unknown Demo State.\n\r");
            ResetStates();
            break;
    }
    //DelayMs(1); // 1ms delay.  Not sure why we need this, but seems to prevent certain errors.

} // ManageStateBluetooth

void ManageStateWire(void) {
    unsigned long bytes;
    BYTE errorCode;
    BYTE usbReturnVal;
    if (USBHostGenericDeviceDetached(deviceAddress) && deviceAddress != 0) {
#ifdef DEBUG_MODE
        UART2PrintString("Controller detached - polled\r\n");
#endif
        ResetStates();
        SetConnectedDeviceType(NONE);
    }
    switch (PS4WireState) {
        case PS4_WIRE_READ_BD_ADDR1:
            UART2PrintString("Read Back ADDR.\n\r");
            if (USB_SUCCESS == USBHostIssueDeviceRequest(deviceAddress, 0xA1, 0x01, 0x0312, 0, 16, hci_buf, USB_DEVICE_REQUEST_GET, 0)) {
                while (!USBHostTransferIsComplete(deviceAddress, 0, &errorCode, &bytes)) {
                    UART2PrintString("Waiting for read to complete...");
                }
                PS4WireState = PS4_WIRE_WRITE_BD_ADDR;
                for (data_num = 0; data_num < 16; data_num++) {
                    UART2PutHex(hci_buf[data_num]);
                    UART2PutChar(' ');
                }
                UART2PrintString("\n\r");
            }
            break;
        case PS4_WIRE_WRITE_BD_ADDR:
            UART2PrintString("Pair Controller with last Bluetooth dongle...\n\r");
            localBdAddr.ints[0] = DataEERead(EEPROM_BT_ADDR_0);
            localBdAddr.ints[1] = DataEERead(EEPROM_BT_ADDR_1);
            localBdAddr.ints[2] = DataEERead(EEPROM_BT_ADDR_2);

            hci_buf[0] = 0x13;
            hci_buf[1] = localBdAddr.bytes[0];
            hci_buf[2] = localBdAddr.bytes[1];
            hci_buf[3] = localBdAddr.bytes[2];
            hci_buf[4] = localBdAddr.bytes[3];
            hci_buf[5] = localBdAddr.bytes[4];
            hci_buf[6] = localBdAddr.bytes[5];
            hci_buf[7] = 0x56;
            hci_buf[8] = 0xE8;
            hci_buf[9] = 0x81;
            hci_buf[10] = 0x38;
            hci_buf[11] = 0x08;
            hci_buf[12] = 0x06;
            hci_buf[13] = 0x51;
            hci_buf[14] = 0x41;
            hci_buf[15] = 0xC0;
            hci_buf[16] = 0x7F;
            hci_buf[17] = 0x12;
            hci_buf[18] = 0xAA;
            hci_buf[19] = 0xD9;
            hci_buf[20] = 0x66;
            hci_buf[21] = 0x3C;
            hci_buf[22] = 0xCE;

            if (USB_SUCCESS == USBHostIssueDeviceRequest(deviceAddress, 0x21, 0x09, 0x0313, 0, 23, hci_buf, USB_DEVICE_REQUEST_SET, 0)) {
                while (!USBHostTransferIsComplete(deviceAddress, 0, &errorCode, &bytes)) {
                    UART2PrintString("Waiting for write to complete...");
                }
                UART2PrintString("Done. ");
                PS4WireState = SET_0x14;
            } else {
                UART2PrintString("Device Write Error !\r\n");
            }
            for (data_num = 0; data_num < 23; data_num++) {
                UART2PutHex(hci_buf[data_num]);
                UART2PutChar(' ');
            }
            UART2PrintString("\n\r");
            break;

        case SET_0x14:
            UART2PrintString("Set Report 0x14:");
            hci_buf[0] = 0x14;
            hci_buf[1] = 0x02;
            hci_buf[2] = 0;
            hci_buf[3] = 0;
            hci_buf[4] = 0;
            hci_buf[5] = 0;
            hci_buf[6] = 0;
            hci_buf[7] = 0;
            hci_buf[8] = 0;
            hci_buf[9] = 0;
            hci_buf[10] = 0;
            hci_buf[11] = 0;
            hci_buf[12] = 0;
            hci_buf[13] = 0;
            hci_buf[14] = 0;
            hci_buf[15] = 0;
            hci_buf[16] = 0;
            if (USB_SUCCESS == USBHostIssueDeviceRequest(deviceAddress, 0x21, 0x09, 0x0314, 0, 17, hci_buf, USB_DEVICE_REQUEST_SET, 0)) {
                PS4WireState = PS4_WIRE_READ_BD_ADDR2;
                UART2PrintString("Success.\n\r");
            }

            break;

        case PS4_WIRE_READ_BD_ADDR2:
            UART2PrintString("Read Back ADDR.\n\r");
            if (USB_SUCCESS == USBHostIssueDeviceRequest(deviceAddress, 0xA1, 0x01, 0x0312, 0, 16, hci_buf, USB_DEVICE_REQUEST_GET, 0)) {
                while (!USBHostTransferIsComplete(deviceAddress, 0, &errorCode, &bytes)) {
                    UART2PrintString("Waiting for read to complete...");
                }
                PS4WireState = PS4_WIRE_CONTROLLER_INIT;
                for (data_num = 0; data_num < 16; data_num++) {
                    UART2PutHex(hci_buf[data_num]);
                    UART2PutChar(' ');
                }
                UART2PrintString("\n\r");
            }
            break;

        case PS4_WIRE_CONTROLLER_INIT:
            GetOutputReportWire()[0] = 0x05; // Report ID
            GetOutputReportWire()[1] = 0xFF;
            GetOutputReportWire()[2] = 0x00;
            GetOutputReportWire()[3] = 0x00;

            SetLEDColor(WHITE, GetLEDBrightness(), 0, 0);
            SetRumbleL(0);
            SetRumbleH(0);

            usbReturnVal = USBHostGenericWrite(deviceAddress, GetOutputReportWire(), 32);
            UART2PrintString("Initialize Controller.\n\r");
            if (USB_SUCCESS == usbReturnVal) {
                PS4WireState = PS4_WIRE_RUNNING;
            } else {
                UART2PutHex(usbReturnVal);
                UART2PrintString("Device Write Error !\r\n");
            }
            break;

        case PS4_WIRE_RUNNING:
            usbReturnVal = USBHostGenericRead(deviceAddress, hci_buf, DATA_PACKET_LENGTH);
            //UART2PrintString("Running\n\r");
            if (usbReturnVal != USB_SUCCESS && usbReturnVal != USB_BUSY) {
                UART2PutHex(usbReturnVal);
                UART2PrintString(" - Device Read Error !\r\n");
            }
            break;

        default:
            break;
    }
    DelayMs(5); // 1ms delay, otherwise seems to get endpoint busy/usb busy errors.
} // ManageStateWire

void ManageStateSpaceNavigator(void){
    BYTE usbReturnVal;
    switch(SpaceNavState)
    {
        case SPACE_NAV_INIT:
            GetPS4Report()[5] = 0; //have to clear this since the PS4 has bit 7 as 1 by default.
            SpaceNavState = SPACE_NAV_RUNNING;
            break;
        case SPACE_NAV_RUNNING:
            usbReturnVal = USBHostGenericRead(deviceAddress, hci_buf, 32);
            break;
        default:
            break;
    }
}
//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        USB_ApplicationEventHandler
 *S
 * Preconditions:   The USB must be initialized.
 *
 * Input:           event       Identifies the bus event that occured
 *
 *                  data        Pointer to event-specific data
 *
 *                  size        Size of the event-specific data
 *
 * Output:          deviceAddress (global)
 *                  Updates device address when an attach or detach occurs.
 *
 *                  DemoState (global)
 *                  Updates the demo state as appropriate when events occur.
 *
 * Returns:         TRUE if the event was handled, FALSE if not
 *
 * Side Effects:    Event-specific actions have been taken.
 *
 * Overview:        This routine is called by the Host layer or client
 *                  driver to notify the application of events that occur.
 *                  If the event is recognized, it is handled and the
 *                  routine returns TRUE.  Otherwise, it is ignored (or
 *                  just "sniffed" and the routine returns FALSE.
 *************************************************************************/

BOOL USB_ApplicationEventHandler(BYTE address, USB_EVENT event, void *data, DWORD size) {
#ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
    BYTE i;
#endif

    // Handle specific events.
    switch (event) {
        case EVENT_GENERIC_ATTACH:
            if (size == sizeof (GENERIC_DEVICE_ID)) {
                deviceAddress = ((GENERIC_DEVICE_ID *) data)->deviceAddress;
                DemoState = BT_STATE_PROCESS;
                HciState = HCI_CMD_RESET; //YTS !!!!!!!!!!!!!!!!!
                PS4WireState = PS4_WIRE_READ_BD_ADDR1; //MH

                UART2PrintString("Generic demo device attached - event, deviceAddress=");
                UART2PutDec(deviceAddress);
                UART2PrintString("\r\n");
#ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
                for (i = 1; i < ((GENERIC_DEVICE_ID *) data)->serialNumberLength; i++) {
                    UART2PutChar(((GENERIC_DEVICE_ID *) data)->serialNumber[i]);
                }
#endif
                UART2PrintString("\r\n");
                return TRUE;
            }
            break;

        case EVENT_GENERIC_DETACH:
            UART2PrintString("Generic demo device detached - event\r\n");
            ResetStates();
            SetConnectedDeviceType(NONE);
            return TRUE;
            break;

        case EVENT_GENERIC_TX_DONE: // The main state machine will poll the driver.
            break;

        case EVENT_GENERIC_TX2_DONE:
            break;

        case EVENT_GENERIC_RX1_DONE://YTS
            if (USBHostGenericGetRx1Length(deviceAddress) > 0) {
                switch (GetConnectedDeviceType()) {
                    case BLUETOOTH:
                        ParseRx1PS4BT();
                        break;
                    case CONTROLLER:
                        ParseRx1PS4Wire();
                        break;
                    case SPACE_MOUSE:
                        ParseRx1SpaceNavigator();
                        break;
                    case THRUSTMASTER:
                        ParseRx1Thrustmaster();
                        break;
                    default:
                        break;
                }
            }
            return TRUE;
            break;

        case EVENT_GENERIC_RX2_DONE://YTS
            if (USBHostGenericGetRx2Length(deviceAddress) > 0) {
                switch (GetConnectedDeviceType()) {
                    case BLUETOOTH:
                        ParseRx2PS4BT();
                        break;
                    default:
                        break;
                }
            }
            return TRUE;
            break;

        case EVENT_VBUS_REQUEST_POWER:
            // We'll let anything attach.
            return TRUE;
            break;

        case EVENT_VBUS_RELEASE_POWER:
            // We aren't keeping track of power.
            return TRUE;
            break;

        case EVENT_HUB_ATTACH:
            UART2PrintString("\r\n***** USB Error - hubs are not supported *****\r\n");
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            UART2PrintString("\r\n***** USB Error - device is not supported *****\r\n");
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            UART2PrintString("\r\n***** USB Error - cannot enumerate device *****\r\n");
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            UART2PrintString("\r\n***** USB Error - client driver initialization error *****\r\n");
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            UART2PrintString("\r\n***** USB Error - out of heap memory *****\r\n");
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR: // This should never be generated.
            UART2PrintString("\r\n***** USB Error - unspecified *****\r\n");
            return TRUE;
            break;

        case EVENT_SUSPEND:
        case EVENT_DETACH:
        case EVENT_RESUME:
        case EVENT_BUS_ERROR:
            return TRUE;
            break;

        default:
            break;
    }
    return FALSE;
} // USB_ApplicationEventHandler



//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        main
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         Never exits
 *
 * Side Effects:    Runs the application
 *
 * Overview:        This is the USB Custom Demo Application's main entry
 *                  point.
 *************************************************************************/

int main(void) {
    unsigned char errorCode;
    unsigned char uartIn;
    unsigned char charbuf[2] = {0};
    // Initialize the processor and peripherals.
    if (InitializeSystem() != TRUE) {
        UART2PrintString(FIRMWARE_REV);
        UART2PrintString(": Could not initialize system.  Halting.");
        while (1);
    }
    if (USBHostInit(0) == TRUE) {
        UART2PrintString("\r\n\r\n* ServoShock 2 Firmware Rev ");
        UART2PrintString(FIRMWARE_REV);
        UART2PrintString(" *\r\n");
        UART2PrintString("=Press 'Esc' in console to display quick reference guide=\n\r\n\r");
    } else {
        UART2PrintString(FIRMWARE_REV);
        UART2PrintString(" Could not initialize USB.  Halting.");
        while (1);
    }

    //Enter test mode
    if (_PGD == 1) {
        UART2PrintString("Circuit Test Mode.\n\r");
        while (1) {
            TestCases();
        }
    }

    //Initialize non-volatile memory
    DataEEInit();
    dataEEFlags.val = 0;
    UART2PrintString("Firmware Code: ");
    UART2PutDecInt(DataEERead(DEFAULTS_INITIALIZED_FLAG));
    UART2PrintString("\n\r");
    if (DataEERead(DEFAULTS_INITIALIZED_FLAG) != VERSION_CODE) {        
        LoadDefaultSettings(TRUE, 0);
        errorCode = DataEEWrite(VERSION_CODE, DEFAULTS_INITIALIZED_FLAG);
        if (errorCode == 0) {
            UART2PrintString("New FW Detected.\n\r");
            SaveSettings();
            Reset();
        } 
    } 
    else {
        LoadSavedSettings();
        UART2PrintString("Loaded Saved Settings. \n\r");
    }

    ResetOutputs(BTControllerInPtr); //initialize outputs and put in an artificial nominal input so it doesn't read all zeros.


    //enable timer4 interrupt
    IEC1bits.T4IE = 1;
    IPC6bits.T4IP = 1; //lowest priority interrupt

    //SPI setup
    InitSPI();
    IEC1bits.INT1IE = 1;

    InitOutputs(); //enable outputs

    // Main Processing Loop
    while (1) {
        UART2ClrError();
        if (UART2DataReceived())
        {
            uartIn = U2RXREG;
            switch(uartIn){
                case 0x1B:
                    PrintMenu();
                    break;
                case 'T':
                    if (charbuf[1] == 'R' && charbuf[0] == 'S' && uartIn == 'T') Reset();
                    break;
                case 'l':
                    LoadSavedSettings();
            }
            charbuf[1] = charbuf[0];
            charbuf[0] = uartIn;
        }

#ifdef DEBUG_MODE
        if (LED_SEL_0 == 1) {
            UART2PutHex(DemoState);
            UART2PutChar('-');
            UART2PutHex(HciState);
            UART2PutChar(' ');
        }
#endif
        // Maintain USB Host State
        USBHostTasks();
        switch (GetConnectedDeviceType()) {
            case CONTROLLER:
                ManageStateWire();
                break;
            case BLUETOOTH:
                ManageStateBluetooth();
                break;
            case SPACE_MOUSE:
                ManageStateSpaceNavigator();
                break;
            case THRUSTMASTER:
                ManageStateWire();
                break;
            default:
                break;
        }

        //Process buttons here
        if (HciState == PS4_BT_RUNNING || PS4WireState == PS4_WIRE_RUNNING || SpaceNavState == SPACE_NAV_RUNNING){
            INDICATOR_LED = 0;
            //Process Device Type specific tasks
            switch (GetConnectedDeviceType()) {
                case BLUETOOTH:

                    if (GetPressDuration()->share > 300 && GetPressDuration()->options > 300) {
                        UART2PrintString("Disconnecting...\n\r");
                        HciState = L2CAP_DISCONNECT_DATA;
                    }
                    if ((GetTimeoutSetting() > 0) && (GetIdleTimer() > GetTimeoutSetting())) {
                        UART2PrintString("Idle Timeout.\n\r");
                        HciState = L2CAP_DISCONNECT_DATA;
                    }
                    break;

                case CONTROLLER:
                    break;

                default:
                    break;
            }
            
            if (GetSpiInputPacket()->overrideRumbleL) {
                vibeL = GetSpiInputPacket()->RumbleL;
            } 
            else {
                vibeL = (unsigned char)(ADC1BUF1 >> 2); //convert 10-bit to 8-bit
            }

            if (GetSpiInputPacket()->overrideRumbleH) {
                vibeH = GetSpiInputPacket()->RumbleH;
            } 
            else {
                vibeH = (unsigned char)(ADC1BUF2 >> 2);           
            }
            LedAdc = (unsigned char)(ADC1BUF0/103); //convert 1023 to 0-9 number

            ////////////100ms loop for writing LEDs and rumble motor values to controller////////////////
            if (loopCounter >= 10) //time the outputs to avoid spamming the controller.  Spamming will cause it to lag or freeze//
            {
                INDICATOR_LED = 1;
                loopCounter = 0;

                if (configFlag == FALSE)
                {
                    SetRumbleL(vibeL);
                    SetRumbleH(vibeH);
                    if (GetSpiInputPacket()->overrideLED) {//Control LED with SPI
                        SetLEDColorRGB(GetSpiInputPacket()->LEDRed, 
                                GetSpiInputPacket()->LEDGreen, 
                                GetSpiInputPacket()->LEDBlue,
                                GetSpiInputPacket()->LEDBlinkOnDuration,
                                GetSpiInputPacket()->LEDBlinkOffDuration);
                    }
                    else{
                        switch (GetLEDColorSetting())
                        {
                            case COLOR_ADC:
                                SetLEDColor(LedAdc, GetLEDBrightness(), 0, 0);
                                break;
                            case BATTERY:
                                UpdateBatteryDisplay(TRUE);
                                break;
                            default:
                                SetLEDColor(GetLEDColorSetting(), GetLEDBrightness(), 0, 0);
                                break;
                        }
                    }
                }
                
                UpdateRumbleFeedback(0); //decrease rumble timer if it's set.
                UpdateBatteryDisplay(FALSE); //display battery levels on LEDs if the command was activated
                
                if (GetConnectedDeviceType() == BLUETOOTH || GetConnectedDeviceType() == CONTROLLER)
                {
                    if (WriteOutputReport() == USB_SUCCESS)
                    {
                        #ifdef DEBUG_MODE
                        UART2PrintString("OK:");
                        #endif
                    }
                }
            }
        }            
        //Stuff to do when the controllers are not connected
        //exit the config state if we were left in that state when the controller was connected
        else {
            INDICATOR_LED = 0;
            ResetIdleTimer(); //need to reset the idle timer because being disconnected makes it appear idle and can trigger the auto-disconnect.
            GetPS4Report()[0] = 0xFF; //set first byte to 0xFF so anything reading the packs from SPI bus can tell it's disconnected
            if (configFlag == TRUE) {
                configFlag = FALSE;
                ConfigOutput(configFlag);
            }
            if (loopCounter == 150)
            {
                INDICATOR_LED = 1;
            }
            if (loopCounter > 160)
            {
                loopCounter = 0;
            }
        }
        DelayMs(1);
    }
    return 0;
} // main

//10ms interrupt to poll/update states

void __attribute__((__interrupt__, auto_psv))_T4Interrupt(void) {
    UpdateNewPress(); //Update record of button presses first thing to reduce jitter
    loopCounter++; //counts how many times we enter this ISR routine.
    
    UpdateCurrentStateBuffer((CONTROLLER_IN *)GetPS4Report());
    
    UpdateServoOutputs(); //Update servo outputs
    
    CheckIdle();
    
    //enter or exit config mode here
    if (GetPressDuration()->share > 300 && GetPressDuration()->psButton > 300) {
        configFlag = TRUE;
    }
    if (configFlag == TRUE) {
        configFlag = ConfigOutput(configFlag); //configFlag will be set to false when we want to exit config, and breaks this loop.
    } 
    else{
        UpdateButtonOutputs(FALSE, 0); //don't update button outputs when in config mode because buttons are used to change settings.
    }    
    IFS0bits.AD1IF = 0; //clear interrupt
    AD1CON1bits.SAMP = 1; //sample ADC
    IFS1bits.T4IF = 0;
}

//SPI bus interrupt to handle SPI packets

void __attribute__((__interrupt__, auto_psv))_SPI1Interrupt(void) {
    while (!SPI1STATbits.SPITBF && !SPI1STATbits.SRXMPT) {
        if (spi_index < PS4_REPORT_LENGTH) {
            GetSpiInputPacket()->array[spi_index] = SPI1BUF;
            SPI1BUF = GetPS4Report()[spi_index]; //fill output buffer

        } 
        else if (spi_index < (PS4_REPORT_LENGTH + SPI_OUTPUT_STATE_PACKET_LENGTH)) {
            SPI1BUF = GetSpiOutputStatePacket()->array[spi_index-PS4_REPORT_LENGTH];
            temp = SPI1BUF;
        }
        else {
            SPI1BUF = 0xAC; //padding
            temp = SPI1BUF; //burn off bytes
        }
        spi_index++;
    }
    IFS0bits.SPI1IF = 0;
}

//INT1Interrupt: The INT1Interrupt pin (pin 46) is tied to the SPI slave select pin (pin 45).
//execute when SPI slave select is pulled high to clear/prime the input/output buffers, so the messages will always be the same length

void __attribute__((__interrupt__, auto_psv))_INT1Interrupt(void) {
    if (SPI1STATbits.SPIEN) //if we try to run the code in the interrupt when SPI bus is disabled, it will get stuck!
    {
        while (!SPI1STATbits.SPITBF) {
            //The PIC can't manually flush the 8-byte output buffer, so we'll  fill it up so that we
            //start the packet on the 9th byte every time.  Discard the first 8 byes
            //on the receiving end.
            SPI1BUF = 0xAB;
        }

        while (!SPI1STATbits.SRXMPT) {
            temp = SPI1BUF; //flush the input buffer.
        }
        spi_index = 0;
        SPI1STATbits.SPIROV = 0;
        IFS1bits.INT1IF = 0;
    }
}


//Error Traps

void __attribute__((__interrupt__, auto_psv))_AddressError(void) {
    UART2PutChar('E');
    {
        __asm__ volatile ("nop");
    }
}

void __attribute__((__interrupt__, auto_psv))_StackError(void) {
    UART2PrintString("StackError");
    {
        __asm__ volatile ("retfie");
    }
}

void __attribute__((__interrupt__, auto_psv))_MathError(void) {
    UART2PrintString("Math Error");
    {
        __asm__ volatile ("retfie");
    }
}
/*************************************************************************
 * EOF main.c
 */

