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

#ifndef __PS4_SPI_H_
#define __PS4_SPI_H_

#include <GenericTypeDefs.h>
#include "outputs.h"
#include <p24Fxxxx.h>
#include "uart2.h"
#include "PS4_controller.h"

#define SPI_INPUT_PACKET_LENGTH 39
#define SPI_OUTPUT_STATE_PACKET_LENGTH 27

typedef union  __PACKED _SPI_OUTPUT_STATE_PACKET {
    struct{
        unsigned int lStickXState_uS; //byte 0-1
        unsigned int lStickYState_uS; //2-3
        unsigned int rStickXState_uS; //4-5
        unsigned int rStickYState_uS; //6-7
        unsigned int lTriggerAnalogState_uS; //8-9
        unsigned int rTriggerAnalogState_uS; //10-11
        unsigned int lTpadXState_uS; //12-13
        unsigned int lTpadYState_uS; //14-15
        unsigned int rTpadXState_uS; //16-17
        unsigned int rTpadYState_uS; //18-19 
        unsigned int tiltXState_uS; //20-21
        unsigned int tiltYState_uS; //22-23

        unsigned dPadUpState : 1; //byte 24
        unsigned dPadRightState : 1;
        unsigned dPadDownState : 1;
        unsigned dPadLeftState : 1;
        unsigned triangleState : 1;
        unsigned circleState : 1;
        unsigned crossState : 1;
        unsigned squareState : 1;

        unsigned lBumperState : 1; //byte 25
        unsigned rBumperState : 1;
        unsigned lTriggerDigitalState : 1;
        unsigned rTriggerDigitalState : 1;
        unsigned lStickPressState : 1;
        unsigned rStickPressState : 1;
        unsigned shareState : 1;
        unsigned optionsState : 1;

        unsigned tpadClickState : 1; //byte 26
        unsigned psButtonState : 1;
        unsigned : 6;
    };
    BYTE array[SPI_OUTPUT_STATE_PACKET_LENGTH];
} SPI_OUTPUT_STATE_PACKET;
        

        

typedef struct __PACKED _SPI_INPUT_PACKET {
    union {
        struct {
            //override feedback
            unsigned overrideLED : 1; //byte 0 bit 0
            unsigned overrideRumbleL : 1;
            unsigned overrideRumbleH : 1;
            unsigned : 5; //bit 7
            
            unsigned char LEDRed;  //byte 1
            unsigned char LEDGreen;
            unsigned char LEDBlue;
            unsigned char LEDBlinkOnDuration;
            unsigned char LEDBlinkOffDuration;
            unsigned char RumbleL;
            unsigned char RumbleH; //byte 7
            
            unsigned overrideLStickX : 1; //byte 8
            unsigned overrideLStickY : 1;
            unsigned overrideRStickX : 1;
            unsigned overrideRStickY : 1;
            unsigned overrideLTriggerAnalog : 1;
            unsigned overrideRTriggerAnalog : 1;
            unsigned overrideLTpadX : 1;
            unsigned overrideLTpadY : 1;
            
            unsigned overrideRTpadX : 1; //byte 9
            unsigned overrideRTpadY : 1;
            unsigned overrideTiltX : 1;
            unsigned overrideTiltY : 1;
            unsigned overrideDPadUp : 1;
            unsigned overrideDPadRight : 1;
            unsigned overrideDPadDown : 1;
            unsigned overrideDPadLeft : 1; 
            
            unsigned overrideTriangle : 1; //byte 10
            unsigned overrideCircle : 1;
            unsigned overrideCross : 1;
            unsigned overrideSquare : 1;
            unsigned overrideLBumper : 1;
            unsigned overrideRBumper : 1;
            unsigned overrideLTriggerPull : 1; 
            unsigned overrideRTriggerPull : 1; 
            
            unsigned overrideLStickPress : 1; //byte 11
            unsigned overrideRStickPress : 1;
            unsigned overrideShare : 1; 
            unsigned overrideOptions : 1;
            unsigned overrideTpadClick : 1; 
            unsigned overridePsButton : 1; 
            unsigned : 2; 
            
            unsigned int lStickX; //byte 12-13
            unsigned int lStickY; 
            unsigned int rStickX;
            unsigned int rStickY;
            unsigned int lTriggerAnalog; //byte 20-21
            unsigned int rTriggerAnalog;
            unsigned int lTpadX;
            unsigned int lTpadY;
            unsigned int rTpadX; 
            unsigned int rTpadY; //byte 30-31
            unsigned int tiltX;
            unsigned int tiltY;//byte 34-35

            unsigned dPadUp : 1; //byte 36
            unsigned dPadRight : 1;
            unsigned dPadDown : 1;
            unsigned dPadLeft : 1;
            unsigned triangle : 1;
            unsigned circle : 1;
            unsigned cross : 1;
            unsigned square : 1;
            
            unsigned lBumper : 1; //byte 37
            unsigned rBumper : 1;
            unsigned lTriggerPull : 1;
            unsigned rTriggerPull : 1;
            unsigned lStickPress : 1;
            unsigned rStickPress : 1;
            unsigned share : 1;
            unsigned options : 1;
            
            unsigned tpadClick : 1; //byte 38
            unsigned psButton : 1;
            unsigned : 6;
        };
        BYTE array[SPI_INPUT_PACKET_LENGTH];
    };
} SPI_INPUT_PACKET;

void InitSPI(void);
void LoadOutputStatePacketServos( unsigned int* pServos );
void LoadOutputStatePacketButtons( BUTTONS* pButtons );
SPI_OUTPUT_STATE_PACKET* GetSpiOutputStatePacket(void);
SPI_INPUT_PACKET* GetSpiInputPacket(void);


#endif
