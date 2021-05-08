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

#ifndef __PS4_CONTROLLER_H_
#define __PS4_CONTROLLER_H_

#include "GenericTypeDefs.h"

#define PS4_REPORT_LENGTH 61 //this is the length of _CONTROLLER_IN structure in bytes.

//touchpad dimensions are 1920 x 940
#define TOUCHPAD_ORIGIN_Y 470
#define TOUCHPAD_ORIGIN_X_WHOLE 960
#define TOUCHPAD_ORIGIN_X_LEFT 320  //left and right origins a bit towards the edges so thumbs won't collide
#define TOUCHPAD_ORIGIN_X_RIGHT 1600

typedef struct __PACKED _TOUCHPAD_RAW_DATA {
    unsigned char packetCounter;

    struct __PACKED {
        unsigned char touchID : 7; // Increments every time a finger is touching the touchpad
        unsigned char noFinger : 1; // The top bit is cleared if the finger is touching the touchpad
        unsigned int x : 12;
        unsigned int y : 12;
    } finger[2]; // 0 = first finger, 1 = second finger
} TOUCHPAD_RAW_DATA; //9 bytes long

typedef struct __PACKED _TOUCHPAD {
    signed int absoluteX;
    signed int absoluteY;
    signed int displacementX;
    signed int displacementY;
    signed int originX;
    signed int originY;
    unsigned char originTouchID;
    signed int incrementX;
    signed int incrementY;
    BOOL fingerActive;
} TOUCHPAD;

typedef struct __PACKED _TOUCHPAD_STRUCT {
    TOUCHPAD wholePad;
    TOUCHPAD leftSide;
    TOUCHPAD rightSide;
} TOUCHPAD_STRUCT;

typedef struct __PACKED _CONTROLLER_IN //WARNING: if you edit this, you need to change the PS4_REPORT_LENGTH!!!
{
    //bluetooth packets have 10 bytes of header.

    //start of packet in both USB and BT modes
    unsigned char reportID; //byte 0
    unsigned char lStickX;
    unsigned char lStickY;
    unsigned char rStickX;
    unsigned char rStickY;

    unsigned dPad : 4; //byte 5 (BT:15)
    unsigned square : 1;
    unsigned cross : 1;
    unsigned circle : 1;
    unsigned triangle : 1;

    unsigned lBumper : 1;
    unsigned rBumper : 1;
    unsigned lTriggerPull : 1;
    unsigned rTriggerPull : 1;
    unsigned share : 1;
    unsigned options : 1;
    unsigned lStickPress : 1;
    unsigned rStickPress : 1;

    unsigned psButton : 1; //byte 7
    unsigned tpadClick : 1;
    unsigned counter : 6;

    unsigned char lTriggerAnalog;
    unsigned char rTriggerAnalog;
    unsigned char bytes10_12[3];

    signed int gyroX;
    signed int gyroY;
    signed int gyroZ;
    signed int accelX;
    signed int accelY;
    signed int accelZ;

    unsigned char bytes25_29[5]; //25-29
    unsigned char battery : 4;
    unsigned char USBData :1; //usb plugged in
    unsigned char :3; //

    unsigned char bytes31_32[2]; //31-32
    unsigned char numTpadPackets; //byte 33
    TOUCHPAD_RAW_DATA tpad[3]; //bytes 34-60
} CONTROLLER_IN; //61 bytes total

typedef struct __PACKED _CONTROLLER_OUTPUT {
    unsigned char header[15];
    unsigned char rumbleHPower;
    unsigned char rumbleLPower;
    unsigned char LEDRed;
    unsigned char LEDGreen;
    unsigned char LEDBlue;
    unsigned char blinkOnDuration; //255 = 2.5s
    unsigned char blinkOffDuration;
} CONTROLLER_OUTPUT;

typedef union __PACKED _BUTTONS {

    struct {
        unsigned int share;
        unsigned int lStickPress;
        unsigned int rStickPress;
        unsigned int options;
        unsigned int dPadUp;
        unsigned int dPadRight;
        unsigned int dPadDown;
        unsigned int dPadLeft;
        unsigned int lTriggerPull;
        unsigned int rTriggerPull;
        unsigned int lBumper;
        unsigned int rBumper;
        unsigned int triangle;
        unsigned int circle;
        unsigned int cross;
        unsigned int square;
        unsigned int psButton;
        unsigned int tpadClick;
    };
    unsigned int array[18];
} BUTTONS;

typedef union __PACKED _ANALOGS {

    struct {
        signed int lStickX;
        signed int lStickY;
        signed int rStickX;
        signed int rStickY;
        signed int lTriggerAnalog;
        signed int rTriggerAnalog;
        signed int lTpadX;
        signed int lTpadY;
        signed int rTpadX;
        signed int rTpadY;
        signed int tiltX;
        signed int tiltZ;
    };
    signed int array[12];
} ANALOGS;

typedef struct __PACKED _CONTROLLER_STATE_BUFFER {
    //buttons
    BUTTONS Buttons;
    ANALOGS Analogs;
    TOUCHPAD_STRUCT Tpad;
    unsigned char battery;
} CONTROLLER_STATE_BUFFER;

typedef struct __PACKED _SPACE_NAV_PACKET6{
    unsigned char id;
    signed int x;
    signed int y;
    signed int z;        
} SPACE_NAV_PACKET6;

typedef struct __PACKED _SPACE_NAV_PACKET13{
    unsigned char id;
    signed int x1;
    signed int y1;
    signed int z1;    
    signed int x2;
    signed int y2;
    signed int z2;
} SPACE_NAV_PACKET13;

typedef struct __PACKED _SPACE_NAV_BUTTONS{ //Space Explorer Buttons
    unsigned char id;
    unsigned one :1;
    unsigned two :1;
    unsigned T :1;
    unsigned L :1;
    unsigned R :1;
    unsigned F :1;
    unsigned esc :1;
    unsigned alt :1;
    
    unsigned shift :1;
    unsigned ctrl :1;
    unsigned fit :1;
    unsigned panel :1;
    unsigned plus :1;
    unsigned minus :1;
    unsigned twoD :1;
    unsigned :1;
    
} SPACE_NAV_BUTTONS;

typedef enum {
    RED = 0,
    ORANGE = 1,
    YELLOW = 2,
    LIME = 3,
    GREEN = 4,
    TURQUOISE = 5,
    BLUE = 6,
    PURPLE = 7,
    PINK = 8,
    WHITE = 9,
    COLOR_ADC = 10,
    BATTERY = 11
}LED_COLORS;

//////////////////////////////UPDATE BUTTONS/////////////////////////////
void CopyPS4Report(CONTROLLER_IN *ControllerIn);
void CopySpaceNavigatorReport(BYTE* packet, unsigned char length);
void UpdateCurrentStateBuffer(CONTROLLER_IN *ControllerIn);
void CopyThrustmasterReport(CONTROLLER_IN* ControllerIn);
BOOL CheckIdle(void);
void UpdateNewPress(void);
//void UpdatePS4ButtonDuration(void);

//////////////////////////////GET BUTTON INFO//////////////////////////////////

BUTTONS* GetButtonPress(void); //For checking if a button is pressed

BUTTONS* GetPressDuration(void); //For checking how long a button is pressed

BUTTONS* GetNewPress(void); //For checking if a button was pressed since the last poll

ANALOGS* GetAnalogs(void); //For checking the analog inputs

TOUCHPAD_STRUCT* GetTouchpads(void); //get Touchpads

BYTE* GetPS4Report(void);

CONTROLLER_OUTPUT* GetOutputPacket(void);

BYTE* GetOutputReportWire(void);

BYTE* GetOutputReportBT(void);

////////////////////////SET LED FUNCTIONS//////////////////////////////
void SetLEDColor(LED_COLORS color, unsigned char intensity, unsigned char blinkOn, unsigned char blinkOff);
void SetLEDColorRGB(unsigned char red, unsigned char green, unsigned char blue, unsigned char blinkOn, unsigned char blinkOff);
////////////////////////Vibe SET FUNCTIONS/////////////////////////
void SetRumbleH(unsigned char level);
void SetRumbleL(unsigned char level);

void UpdateBatteryDisplay(BOOL override);
void UpdateRumbleFeedback(unsigned int duration);

///////////////////////MISC FUNCTIONS///////////////////////////////
unsigned int GetIdleTimer(void);
void ResetIdleTimer(void);

#endif

