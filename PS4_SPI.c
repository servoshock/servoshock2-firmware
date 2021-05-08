/***********************************************************************
    PS4 USB Host
    Copyright (C) 2013 Cross Product Creations

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

#include "PS4_SPI.h"
#include "outputs.h"

volatile BYTE spiInputBuf[SPI_INPUT_PACKET_LENGTH];
SPI_OUTPUT_STATE_PACKET spiOutputStatePacket;

void InitSPI(void) {
    BYTE temp;

    RPINR20bits.SCK1R = 2; //map SCK1IN to RP2
    RPINR20bits.SDI1R = 4; //map SDI1 to RP4
    RPINR21bits.SS1R = 12; //map SS1R to RP12
    RPOR1bits.RP3R = 7; //map RP3 to SDO1

    //Map interrupt for /SS line
    _TRISD0 = 1;
    RPINR0bits.INT1R = 11;
    IPC5bits.INT1IP = 6; //set slave select interrupt priority
    INTCON2bits.INT1EP = 0; //interrupt on positive edge
    IFS1bits.INT1IF = 0;

    SPI1STATbits.SISEL = 0b001; //interrupt when data is available.
    SPI1CON1 = 0;
    SPI1CON1bits.SMP = 0;
    SPI1CON1bits.SSEN = 1;
    SPI1CON1bits.MSTEN = 0; //slave mode
    SPI1CON2 = 0;
    SPI1CON2bits.SPIBEN = 1;
    switch (GetSpiBusMode()) {
        case 0:
            UART2PrintString("SPI Bus Mode 0\n\r");
            SPI1CON1bits.CKP = 0;
            SPI1CON1bits.CKE = 1;
            break;

        case 1:
            UART2PrintString("SPI Bus Mode 1\n\r");
            SPI1CON1bits.CKP = 0;
            SPI1CON1bits.CKE = 0;
            break;

        case 2:
            UART2PrintString("SPI Bus Mode 2\n\r");
            SPI1CON1bits.CKP = 1;
            SPI1CON1bits.CKE = 1;
            break;

        case 3:
            UART2PrintString("SPI Bus Mode 3\n\r");
            SPI1CON1bits.CKP = 1;
            SPI1CON1bits.CKE = 0;
            break;

        default:
            UART2PrintString("ERROR SPI BUS MODE\n\r");
            break;
    }

    IFS0bits.SPI1IF = 0; //clear spi interrupt flag
    IPC2bits.SPI1IP = 6; ///set interrupt priority
    IEC0bits.SPI1IE = 1; //enable interrupts

    SPI1STATbits.SPIEN = 1;

    while (!SPI1STATbits.SPITBF) {
        SPI1BUF = 0xAC; //fill the buffer so we always start on byte 9
    }

    while (!SPI1STATbits.SRXMPT) {
        temp = SPI1BUF; //burn off bytes to empty the input buffer.
    }

}

void LoadOutputStatePacketButtons( BUTTONS* pButtons )
{   
    spiOutputStatePacket.dPadUpState = pButtons->dPadUp; //byte 24
    spiOutputStatePacket.dPadRightState = pButtons->dPadRight;
    spiOutputStatePacket.dPadDownState = pButtons->dPadDown;
    spiOutputStatePacket.dPadLeftState = pButtons->dPadLeft;
    spiOutputStatePacket.triangleState = pButtons->triangle;
    spiOutputStatePacket.circleState = pButtons->circle;
    spiOutputStatePacket.crossState = pButtons->cross;
    spiOutputStatePacket.squareState = pButtons->square;

    spiOutputStatePacket.lBumperState = pButtons->lBumper; //byte 25
    spiOutputStatePacket.rBumperState = pButtons->rBumper;
    spiOutputStatePacket.lTriggerDigitalState = pButtons->lTriggerPull;
    spiOutputStatePacket.rTriggerDigitalState = pButtons->rTriggerPull;
    spiOutputStatePacket.lStickPressState = pButtons->lStickPress;
    spiOutputStatePacket.rStickPressState = pButtons->rStickPress;
    spiOutputStatePacket.shareState = pButtons->share;
    spiOutputStatePacket.optionsState = pButtons->options;

    spiOutputStatePacket.tpadClickState = pButtons->tpadClick; //byte 26
    spiOutputStatePacket.psButtonState = pButtons->psButton;
}

void LoadOutputStatePacketServos( unsigned int* pServos )
{
    spiOutputStatePacket.lStickXState_uS = pServos[L_STICK_X]/2; //byte 0-1
    spiOutputStatePacket.lStickYState_uS = pServos[L_STICK_Y]/2; //2-3
    spiOutputStatePacket.rStickXState_uS = pServos[R_STICK_X]/2; //4-5
    spiOutputStatePacket.rStickYState_uS = pServos[R_STICK_Y]/2; //6-7
    spiOutputStatePacket.lTriggerAnalogState_uS = pServos[L_TRIG_A]/2; //8-9
    spiOutputStatePacket.rTriggerAnalogState_uS = pServos[R_TRIG_A]/2; //10-11
    spiOutputStatePacket.lTpadXState_uS = pServos[L_TPAD_X]/2; //12-13
    spiOutputStatePacket.lTpadYState_uS = pServos[L_TPAD_Y]/2; //14-15
    spiOutputStatePacket.rTpadXState_uS = pServos[R_TPAD_X]/2; //16-17
    spiOutputStatePacket.rTpadYState_uS = pServos[R_TPAD_Y]/2; //18-19 
    spiOutputStatePacket.tiltXState_uS = pServos[TILT_X]/2; //20-21
    spiOutputStatePacket.tiltYState_uS =  pServos[TILT_Y]/2; //22-23
            
}

SPI_INPUT_PACKET* GetSpiInputPacket(void) {
    return (SPI_INPUT_PACKET*)&spiInputBuf;
}

SPI_OUTPUT_STATE_PACKET* GetSpiOutputStatePacket(void) {
    return (SPI_OUTPUT_STATE_PACKET*)&spiOutputStatePacket;
}
