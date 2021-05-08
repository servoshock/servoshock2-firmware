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

#include "USB\usb_host_generic_PS4.h"
#include "PS4_controller.h"
#include "uart2.h"
#include "PS4_SPI.h"
#include "main.h"


volatile BYTE outputReportBT[DATA_PACKET_LENGTH] = {0};

//USB Wire report is shorter, doesn't have the Bluetooth headers
static volatile BYTE *outputReportWire = &outputReportBT[11]; //USB version of the packet has smaller header, remember to edit the header for USB
static volatile CONTROLLER_OUTPUT *PS4OutputPtr = (CONTROLLER_OUTPUT*) outputReportBT;

//////////////////////////////PRIVATE VARIABLES//////////////////////////////////
static volatile BUTTONS LastPress;
static volatile BUTTONS NewPress;
static volatile BUTTONS PressDuration;
static volatile CONTROLLER_STATE_BUFFER CurrentStateBuffer;

//static volatile TOUCHPAD_STRUCT Touchpads;

static volatile BYTE PS4_report_buf[PS4_REPORT_LENGTH];
static volatile unsigned int idleTimer = 0;

void CopyPS4Report(CONTROLLER_IN* ControllerIn){
    unsigned int i = 0;
    for (i = 0; i < PS4_REPORT_LENGTH; i++) {  //have to do a deep copy
        PS4_report_buf[i] = ((BYTE*) ControllerIn)[i]; //need this for SPI data; copy this over because hci_buf and acl_buf don't always contain dualshock packets
    }
}


void CopySpaceNavigatorReport(BYTE packet[], unsigned char length){
    int x,y,z;
    switch (((SPACE_NAV_PACKET6*)packet)->id)
    {
        case 1:
            if (length == 7 || length == 13)
            {
                x = ((SPACE_NAV_PACKET6*)packet)->x/2+127;
                if (x > 255) x = 255;
                else if (x < 0) x = 0;
                
                y = ((SPACE_NAV_PACKET6*)packet)->y/2+127;
                if (y > 255) y = 255;
                else if (y < 0) y = 0;
                
                z = ((SPACE_NAV_PACKET6*)packet)->z/2+127;
                if (z > 255) z = 255;
                else if (z < 0) z = 0;
                
                ((CONTROLLER_IN*)PS4_report_buf)->lStickX = (unsigned char)x;
                ((CONTROLLER_IN*)PS4_report_buf)->lStickY = (unsigned char)y;
                ((CONTROLLER_IN*)PS4_report_buf)->rStickX = (unsigned char)z;
                
                if (length == 13)
                {
                    x = ((SPACE_NAV_PACKET13*)packet)->x2/2+127;
                    if (x > 255) x = 255;
                    else if (x < 0) x = 0;

                    y = ((SPACE_NAV_PACKET13*)packet)->y2/2+127;
                    if (y > 255) y = 255;
                    else if (y < 0) y = 0;

                    z = ((SPACE_NAV_PACKET13*)packet)->z2/2+127;
                    if (z > 255) z = 255;
                    else if (z < 0) z = 0;

                    ((CONTROLLER_IN*)PS4_report_buf)->rStickY = (unsigned char)x;
                    ((CONTROLLER_IN*)PS4_report_buf)->lTriggerAnalog = (unsigned char)y;
                    ((CONTROLLER_IN*)PS4_report_buf)->rTriggerAnalog = (unsigned char)z;       

                }
            }

            else{
#ifdef DEBUG_MODE
                UART2PrintString("packet length?\n\r");
#endif
            }
            break;
        case 2:
            x = ((SPACE_NAV_PACKET6*)packet)->x/2+127;
            if (x > 255) x = 255;
            else if (x < 0) x = 0;

            y = ((SPACE_NAV_PACKET6*)packet)->y/2+127;
            if (y > 255) y = 255;
            else if (y < 0) y = 0;

            z = ((SPACE_NAV_PACKET6*)packet)->z/2+127;
            if (z > 255) z = 255;
            else if (z < 0) z = 0;

            ((CONTROLLER_IN*)PS4_report_buf)->rStickY = (unsigned char)x;
            ((CONTROLLER_IN*)PS4_report_buf)->lTriggerAnalog = (unsigned char)y;
            ((CONTROLLER_IN*)PS4_report_buf)->rTriggerAnalog = (unsigned char)z;
            break;
        case 3:
            if (length > 2)
            {
                if ( ((SPACE_NAV_BUTTONS*)packet)->esc ) ((CONTROLLER_IN*)PS4_report_buf)->dPad |= 0b0001;
                else ((CONTROLLER_IN*)PS4_report_buf)->dPad &= 0b1110;
                if ( ((SPACE_NAV_BUTTONS*)packet)->ctrl ) ((CONTROLLER_IN*)PS4_report_buf)->dPad |= 0b0010;
                else ((CONTROLLER_IN*)PS4_report_buf)->dPad &= 0b1101;
                if ( ((SPACE_NAV_BUTTONS*)packet)->alt ) ((CONTROLLER_IN*)PS4_report_buf)->dPad |= 0b0100;
                else ((CONTROLLER_IN*)PS4_report_buf)->dPad &= 0b1011;
                if ( ((SPACE_NAV_BUTTONS*)packet)->shift ) ((CONTROLLER_IN*)PS4_report_buf)->dPad |= 0b1000;
                else ((CONTROLLER_IN*)PS4_report_buf)->dPad &= 0b0111;
                ((CONTROLLER_IN*)PS4_report_buf)->triangle = ((SPACE_NAV_BUTTONS*)packet)->T;
                ((CONTROLLER_IN*)PS4_report_buf)->circle = ((SPACE_NAV_BUTTONS*)packet)->R;
                ((CONTROLLER_IN*)PS4_report_buf)->cross = ((SPACE_NAV_BUTTONS*)packet)->F;
                ((CONTROLLER_IN*)PS4_report_buf)->square = ((SPACE_NAV_BUTTONS*)packet)->L;
                ((CONTROLLER_IN*)PS4_report_buf)->lBumper = ((SPACE_NAV_BUTTONS*)packet)->one;
                ((CONTROLLER_IN*)PS4_report_buf)->rBumper = ((SPACE_NAV_BUTTONS*)packet)->two;
                ((CONTROLLER_IN*)PS4_report_buf)->lStickPress = ((SPACE_NAV_BUTTONS*)packet)->plus;
                ((CONTROLLER_IN*)PS4_report_buf)->rStickPress = ((SPACE_NAV_BUTTONS*)packet)->minus;
                ((CONTROLLER_IN*)PS4_report_buf)->options = ((SPACE_NAV_BUTTONS*)packet)->fit;
                ((CONTROLLER_IN*)PS4_report_buf)->share = ((SPACE_NAV_BUTTONS*)packet)->panel;
                ((CONTROLLER_IN*)PS4_report_buf)->psButton = ((SPACE_NAV_BUTTONS*)packet)->twoD;
                
            }
            break;
        default:
#ifdef DEBUG_MODE
            UART2PrintString("Unknown packet\n\r");
#endif
            break;
    }   
} 

 void CopyThrustmasterReport(CONTROLLER_IN* ControllerIn){
    unsigned int i = 0;
    /*
     ch0: x-axis
     ch1: y-axis
     ch2: z-twist
     ch3: throttle
     ch4: throttle paddle
     ch5: R2/L2
     ch6: POV hat L/R
     ch7: POV hat U/D
     ch8: circle/square
     ch9: triangle/cross
     ch10: R1/R3
     ch11: L1/L3
    */
    
    ControllerIn->rStickX = ((BYTE*) ControllerIn)[47];//stick twist
    ControllerIn->rStickY = ((BYTE*) ControllerIn)[48];//throttle
    ControllerIn->lTriggerAnalog = ((BYTE*) ControllerIn)[49];//r
    for (i = 0; i < PS4_REPORT_LENGTH; i++) {  //have to do a deep copy
        PS4_report_buf[i] = ((BYTE*) ControllerIn)[i]; //need this for SPI data; copy this over because hci_buf and acl_buf don't always contain dualshock packets
    }
}

void UpdateCurrentStateBuffer(CONTROLLER_IN *ControllerIn) {
    
   /*********WARNING********
    * Button/Servo update functions may not be synchronized to this buffer update function.
    * Therefore, make a working copy of CurrentStateBuffer, or else you'll have 
    * to keep track very carefully that each member of the struct is only written to a
    * single time, since an interrupt could interrupt this function.
    *************************/
    CONTROLLER_STATE_BUFFER Temp = CurrentStateBuffer; //make copy of current state, touchpad is a state machine so we need this
    static signed long int xAngle=0;
    static signed long int zAngle=0;
    
    Temp.Buttons.triangle = ControllerIn->triangle;
    Temp.Buttons.circle = ControllerIn->circle;
    Temp.Buttons.cross = ControllerIn->cross;
    Temp.Buttons.square = ControllerIn->square;


    if (GetConnectedDeviceType() == SPACE_MOUSE)
    {
        Temp.Buttons.dPadUp = (BYTE)(ControllerIn->dPad)&0b00000001;
        Temp.Buttons.dPadRight = (BYTE)(ControllerIn->dPad)&0b00000010;
        Temp.Buttons.dPadDown = (BYTE)(ControllerIn->dPad)&0b00000100;
        Temp.Buttons.dPadLeft = (BYTE)(ControllerIn->dPad)&0b00001000;
    }
    else
    {

        Temp.Buttons.dPadUp = ControllerIn->dPad == 7 || ControllerIn->dPad == 0 || ControllerIn->dPad == 1;
        Temp.Buttons.dPadRight = ControllerIn->dPad == 1 || ControllerIn->dPad == 2 || ControllerIn->dPad == 3;
        Temp.Buttons.dPadDown = ControllerIn->dPad == 3 || ControllerIn->dPad == 4 || ControllerIn->dPad == 5;
        Temp.Buttons.dPadLeft = ControllerIn->dPad == 5 || ControllerIn->dPad == 6 || ControllerIn->dPad == 7;
        
    }

    Temp.Buttons.lStickPress = ControllerIn->lStickPress;
    Temp.Buttons.rStickPress = ControllerIn->rStickPress;
    Temp.Buttons.options = ControllerIn->options;
    Temp.Buttons.share = ControllerIn->share;
    Temp.Buttons.lTriggerPull = ControllerIn->lTriggerPull;
    Temp.Buttons.rTriggerPull = ControllerIn->rTriggerPull;
    Temp.Buttons.lBumper = ControllerIn->lBumper;
    Temp.Buttons.rBumper = ControllerIn->rBumper;
    Temp.Buttons.psButton = ControllerIn->psButton;
    Temp.Buttons.tpadClick = ControllerIn->tpadClick;
    Temp.Analogs.lStickX = (signed int)ControllerIn->lStickX;
    Temp.Analogs.lStickY = (signed int)ControllerIn->lStickY;
    Temp.Analogs.rStickX = (signed int)ControllerIn->rStickX;
    Temp.Analogs.rStickY = (signed int)ControllerIn->rStickY;
    Temp.Analogs.lTriggerAnalog = (signed int)ControllerIn->lTriggerAnalog;
    Temp.Analogs.rTriggerAnalog = (signed int)ControllerIn->rTriggerAnalog;
    Temp.battery = ControllerIn->battery;

    
    //Process Tilt
#define COEFF_GYRO 980
#define COEFF_ACCEL 20
#define ACCEL_GAIN 15
#define GYRO_GAIN 1
    xAngle = (COEFF_GYRO*((signed long)xAngle + GYRO_GAIN*(ControllerIn->gyroZ/10))+(COEFF_ACCEL*ACCEL_GAIN*(signed long)ControllerIn->accelX)/10)/1000;
    zAngle = (COEFF_GYRO*((signed long)zAngle + GYRO_GAIN*(ControllerIn->gyroX/10))-(COEFF_ACCEL*ACCEL_GAIN*(signed long)ControllerIn->accelZ)/10)/1000;

    Temp.Analogs.tiltX = xAngle/100;
    Temp.Analogs.tiltZ = zAngle/100;
    
    /*
    UART2PutDecSLong(ControllerIn->gyroX);
    UART2PutChar('\t');
    UART2PutDecSLong(ControllerIn->accelZ);
    UART2PutChar('\t');
     */
    
    //if no fingers detected, reset active flags, otherwise it won't initialized properly because the first Touch ID is unknown.
    if (ControllerIn->tpad[0].finger[0].noFinger && ControllerIn->tpad[0].finger[1].noFinger) 
    {
        Temp.Tpad.leftSide.fingerActive = 0;
        Temp.Tpad.rightSide.fingerActive = 0;
        Temp.Tpad.leftSide.originTouchID = 255; //some controllers don't increment the first touch ID from 0, set to another number
        Temp.Tpad.rightSide.originTouchID = 254;
    }
    
    //Process Touchpads
    //check finger 0
    Temp.Tpad.wholePad.fingerActive = !ControllerIn->tpad[0].finger[0].noFinger;
    if (Temp.Tpad.leftSide.originTouchID == ControllerIn->tpad[0].finger[0].touchID) //if the left half is most recently associated with finger 0
    {
        Temp.Tpad.leftSide.fingerActive = !ControllerIn->tpad[0].finger[0].noFinger;
    }
    if (Temp.Tpad.rightSide.originTouchID == ControllerIn->tpad[0].finger[0].touchID) //if the right half is most recently associated with finger 0
    {
        Temp.Tpad.rightSide.fingerActive = !ControllerIn->tpad[0].finger[0].noFinger;
    }
    //Check finger 1
    if (Temp.Tpad.leftSide.originTouchID == ControllerIn->tpad[0].finger[1].touchID) //if the left half is most recently associated with finger 1
    {
        Temp.Tpad.leftSide.fingerActive = !ControllerIn->tpad[0].finger[1].noFinger;
    }
    if (Temp.Tpad.rightSide.originTouchID == ControllerIn->tpad[0].finger[1].touchID) //if the right half is most recently associated with finger 1
    {
        Temp.Tpad.rightSide.fingerActive = !ControllerIn->tpad[0].finger[1].noFinger;
    }

    if (!ControllerIn->tpad[0].finger[0].noFinger) //process finger 0
    {
        //Update parameters for whole touchpad mode
        //Check to see if it can be traced back to an existing touch.  If not, it's a new touch, update origin
        if (ControllerIn->tpad[0].finger[0].touchID != Temp.Tpad.wholePad.originTouchID) //if this is a new touch, set the origin coordinates
        {
            Temp.Tpad.wholePad.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_WHOLE;
            Temp.Tpad.wholePad.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
            Temp.Tpad.wholePad.originTouchID = ControllerIn->tpad[0].finger[0].touchID;
            Temp.Tpad.wholePad.originX = ControllerIn->tpad[0].finger[0].x;
            Temp.Tpad.wholePad.originY = ControllerIn->tpad[0].finger[0].y;
        }
        //Update parameters
        Temp.Tpad.wholePad.incrementX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_WHOLE - Temp.Tpad.wholePad.absoluteX;
        Temp.Tpad.wholePad.incrementY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y - Temp.Tpad.wholePad.absoluteY;
        Temp.Tpad.wholePad.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_WHOLE;
        Temp.Tpad.wholePad.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
        Temp.Tpad.wholePad.displacementX = ControllerIn->tpad[0].finger[0].x - Temp.Tpad.wholePad.originX;
        Temp.Tpad.wholePad.displacementY = ControllerIn->tpad[0].finger[0].y - Temp.Tpad.wholePad.originY;

        //Update parameters for touchpad left/right mode.

        //Process Finger 0
        if (ControllerIn->tpad[0].finger[0].touchID != Temp.Tpad.leftSide.originTouchID &&\
                ControllerIn->tpad[0].finger[0].touchID != Temp.Tpad.rightSide.originTouchID) //if the touch is not a continuation of either joystick, it a new touch
        {
            if (ControllerIn->tpad[0].finger[0].x < 960)
            {
                Temp.Tpad.leftSide.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_LEFT;
                Temp.Tpad.leftSide.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
                Temp.Tpad.leftSide.originX = ControllerIn->tpad[0].finger[0].x;
                Temp.Tpad.leftSide.originY = ControllerIn->tpad[0].finger[0].y;
                Temp.Tpad.leftSide.originTouchID = ControllerIn->tpad[0].finger[0].touchID;
            }
            else if (ControllerIn->tpad[0].finger[0].x >= 960) {
                Temp.Tpad.rightSide.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_RIGHT;
                Temp.Tpad.rightSide.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
                Temp.Tpad.rightSide.originTouchID = ControllerIn->tpad[0].finger[0].touchID;
                Temp.Tpad.rightSide.originX = ControllerIn->tpad[0].finger[0].x;
                Temp.Tpad.rightSide.originY = ControllerIn->tpad[0].finger[0].y;
            }
            else
            {
                UART2PrintString("Unknown Side\n\r");
            }
        }
        if (ControllerIn->tpad[0].finger[0].touchID == Temp.Tpad.leftSide.originTouchID){
            Temp.Tpad.leftSide.incrementX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_LEFT - Temp.Tpad.leftSide.absoluteX;
            Temp.Tpad.leftSide.incrementY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y - Temp.Tpad.leftSide.absoluteY;
            Temp.Tpad.leftSide.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_LEFT;
            Temp.Tpad.leftSide.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
            Temp.Tpad.leftSide.displacementX = ControllerIn->tpad[0].finger[0].x - Temp.Tpad.leftSide.originX;
            Temp.Tpad.leftSide.displacementY = ControllerIn->tpad[0].finger[0].y - Temp.Tpad.leftSide.originY;
        }
        if (ControllerIn->tpad[0].finger[0].touchID == Temp.Tpad.rightSide.originTouchID){
            Temp.Tpad.rightSide.incrementX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_RIGHT - Temp.Tpad.rightSide.absoluteX;
            Temp.Tpad.rightSide.incrementY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y - Temp.Tpad.rightSide.absoluteY;
            Temp.Tpad.rightSide.absoluteX = ControllerIn->tpad[0].finger[0].x - TOUCHPAD_ORIGIN_X_RIGHT;
            Temp.Tpad.rightSide.absoluteY = ControllerIn->tpad[0].finger[0].y - TOUCHPAD_ORIGIN_Y;
            Temp.Tpad.rightSide.displacementX = ControllerIn->tpad[0].finger[0].x - Temp.Tpad.rightSide.originX;
            Temp.Tpad.rightSide.displacementY = ControllerIn->tpad[0].finger[0].y - Temp.Tpad.rightSide.originY;
        }
    } //end processing finger 0


    if (!ControllerIn->tpad[0].finger[1].noFinger) //process finger 1
    {
        if (ControllerIn->tpad[0].finger[1].touchID != Temp.Tpad.leftSide.originTouchID &&\
                ControllerIn->tpad[0].finger[1].touchID != Temp.Tpad.rightSide.originTouchID) //if the touch is not a continuation of either joystick, it a new touch
        {
            if (ControllerIn->tpad[0].finger[1].x < 960)
            {
                Temp.Tpad.leftSide.absoluteX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_LEFT;
                Temp.Tpad.leftSide.absoluteY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y;
                Temp.Tpad.leftSide.originX = ControllerIn->tpad[0].finger[1].x;
                Temp.Tpad.leftSide.originY = ControllerIn->tpad[0].finger[1].y;
                Temp.Tpad.leftSide.originTouchID = ControllerIn->tpad[0].finger[1].touchID;
            }
            else if (ControllerIn->tpad[0].finger[1].x >= 960) {
                Temp.Tpad.rightSide.absoluteX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_RIGHT;
                Temp.Tpad.rightSide.absoluteY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y;
                Temp.Tpad.rightSide.originTouchID = ControllerIn->tpad[0].finger[1].touchID;
                Temp.Tpad.rightSide.originX = ControllerIn->tpad[0].finger[1].x;
                Temp.Tpad.rightSide.originY = ControllerIn->tpad[0].finger[1].y;
            }
            else
            {
                UART2PrintString("Unknown Side\n\r");;
            }
        }
        if (ControllerIn->tpad[0].finger[1].touchID == Temp.Tpad.leftSide.originTouchID){
            Temp.Tpad.leftSide.incrementX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_LEFT - Temp.Tpad.leftSide.absoluteX;
            Temp.Tpad.leftSide.incrementY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y - Temp.Tpad.leftSide.absoluteY;
            Temp.Tpad.leftSide.absoluteX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_LEFT;
            Temp.Tpad.leftSide.absoluteY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y;
            Temp.Tpad.leftSide.displacementX = ControllerIn->tpad[0].finger[1].x - Temp.Tpad.leftSide.originX;
            Temp.Tpad.leftSide.displacementY = ControllerIn->tpad[0].finger[1].y - Temp.Tpad.leftSide.originY;
        }
        if (ControllerIn->tpad[0].finger[1].touchID == Temp.Tpad.rightSide.originTouchID){
            Temp.Tpad.rightSide.incrementX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_RIGHT - Temp.Tpad.rightSide.absoluteX;
            Temp.Tpad.rightSide.incrementY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y - Temp.Tpad.rightSide.absoluteY;
            Temp.Tpad.rightSide.absoluteX = ControllerIn->tpad[0].finger[1].x - TOUCHPAD_ORIGIN_X_RIGHT;
            Temp.Tpad.rightSide.absoluteY = ControllerIn->tpad[0].finger[1].y - TOUCHPAD_ORIGIN_Y;
            Temp.Tpad.rightSide.displacementX = ControllerIn->tpad[0].finger[1].x - Temp.Tpad.rightSide.originX;
            Temp.Tpad.rightSide.displacementY = ControllerIn->tpad[0].finger[1].y - Temp.Tpad.rightSide.originY;
        }
    } //end processing finger 1

    if (!Temp.Tpad.wholePad.fingerActive) {
        Temp.Tpad.wholePad.displacementX = 0;
        Temp.Tpad.wholePad.displacementY = 0;
        Temp.Tpad.wholePad.incrementX = 0;
        Temp.Tpad.wholePad.incrementY = 0;
    }
    if (!Temp.Tpad.leftSide.fingerActive) {
        Temp.Tpad.leftSide.displacementX = 0;
        Temp.Tpad.leftSide.displacementY = 0;
        Temp.Tpad.leftSide.incrementX = 0;
        Temp.Tpad.leftSide.incrementY = 0;
    }
    if (!Temp.Tpad.rightSide.fingerActive) {
        Temp.Tpad.rightSide.displacementX = 0;
        Temp.Tpad.rightSide.displacementY = 0;
        Temp.Tpad.rightSide.incrementX = 0;
        Temp.Tpad.rightSide.incrementY = 0;    }
    CurrentStateBuffer = Temp;
}

//check to see if controller is idle, if there is change, then reset the idle timer.
//probably only need the accels, could save some processing here.
BOOL CheckIdle(void) {
    unsigned int i;
    static volatile ANALOGS LastAnalogs; //for checking for change
    static volatile BUTTONS LastButtons; //for checking for change
    BOOL changeFlag = FALSE;
    signed int analogChange;
    
    //check for change
    for (i=0; i<NUM_BUTTONS; i++){
        if (LastButtons.array[i] != CurrentStateBuffer.Buttons.array[i]){
            changeFlag = TRUE;
        }
        LastButtons.array[i] = CurrentStateBuffer.Buttons.array[i];
    }
    
    for (i = 0; i<NUM_SERVOS; i++){
        if (CurrentStateBuffer.Analogs.array[i] > LastAnalogs.array[i]) {
            analogChange = CurrentStateBuffer.Analogs.array[i] - LastAnalogs.array[i];
        }
        else {
            analogChange = LastAnalogs.array[i] - CurrentStateBuffer.Analogs.array[i];
        }

        if (i == TILT_X || i ==TILT_Y) {
            if (analogChange > 200) {
                changeFlag = TRUE;
            }
        } else {
            if (analogChange > 5) {
                changeFlag = TRUE;
            }
        }
        LastAnalogs.array[i] = CurrentStateBuffer.Analogs.array[i];
    }
    
    if (CurrentStateBuffer.Tpad.wholePad.incrementX != 0 || CurrentStateBuffer.Tpad.wholePad.incrementY!= 0) {
        //changeFlag = TRUE;
    }
    //update idle timer if no input change.
    if (changeFlag == FALSE) {
        if (idleTimer < 0xFFFF) {
            idleTimer++;
        }
    }
    else {
        ResetIdleTimer();
    }
    /*
    if (changeFlag){
        UART2PutHex(changeFlag);
    }
    UART2PutDecInt(idleTimer);
    UART2PrintString("\n\r");
     */
    return !changeFlag;

}


/*******************************************************************************
Function:  UpdateNewPress(void)

Precondition:
    None.

Overview:
    This routine updates the states for button presses 
 * PressDuration: how long the button has been held down (10ms per count)
 * NewPress: if this is a new button press

Input: level: 0-255, determines rumble strength.  6 levels available.

Output: None

 *******************************************************************************/
void UpdateNewPress(void) {
    unsigned int i;
    //BOOL changeFlag = FALSE; //this flag is set if there there is change in
    //unsigned int analogChange;

    //Update Button state record
    for (i = 0; i < NUM_BUTTONS; i++) {
        //Set press duration counter to 1 immediately after being pressed or 0 after being released, or else it won't update until the Timer4 flag is set
        if (CurrentStateBuffer.Buttons.array[i] > 0) {
            PressDuration.array[i]++;
        }
        else {
            PressDuration.array[i] = 0;
        }
        //detect if a button is newly pressed
        if (CurrentStateBuffer.Buttons.array[i] > 0 && LastPress.array[i] == 0) {
            NewPress.array[i] = 1;
        }
        else {
            NewPress.array[i] = 0;
        }
        LastPress.array[i] = CurrentStateBuffer.Buttons.array[i];
    }
}
////////////////////////GET functions////////////////////////////////////

BUTTONS* GetButtonPress(void) {
    return (BUTTONS*)&(CurrentStateBuffer.Buttons);
}

BUTTONS* GetPressDuration(void) {
    return (BUTTONS*)&(PressDuration);
}

BUTTONS* GetNewPress(void) {
    return (BUTTONS*)&(NewPress);
}

ANALOGS* GetAnalogs(void) {
    return (ANALOGS*)&(CurrentStateBuffer.Analogs);
}

TOUCHPAD_STRUCT* GetTouchpads(void) {
    return (TOUCHPAD_STRUCT*)&(CurrentStateBuffer.Tpad);
}

//Returns PS4 input report.  We first byte to 0xFF to identify stagnant data.

BYTE* GetPS4Report(void) {
    return (BYTE*) PS4_report_buf;
}

CONTROLLER_OUTPUT* GetOutputPacket(void) {
    return (CONTROLLER_OUTPUT*) PS4OutputPtr;
}

BYTE* GetOutputReportWire(void) {
    return (BYTE*) outputReportWire;
}

BYTE* GetOutputReportBT(void) {
    return (BYTE*) outputReportBT;
}

void SetLEDColor(LED_COLORS color, unsigned char intensity, unsigned char blinkOn, unsigned char blinkOff) {
    //Beware that the intensity parameter can cause overflow, too lazy to put in checks.  Use carefully!
    /*
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
     */
    if (intensity >= 10)
    {
        intensity = 10;
    }
    
    PS4OutputPtr->blinkOnDuration = blinkOn;
    PS4OutputPtr->blinkOffDuration = blinkOff;
    
    switch (color) {
        case BLUE: //blue
            PS4OutputPtr->LEDRed = 0;
            PS4OutputPtr->LEDGreen = 0;
            PS4OutputPtr->LEDBlue = 20 * intensity;
            break;
        case RED: //red
            PS4OutputPtr->LEDRed = 20 * intensity;
            PS4OutputPtr->LEDGreen = 0;
            PS4OutputPtr->LEDBlue = 0;
            break;
        case ORANGE: //orange
            PS4OutputPtr->LEDRed = 15 * intensity;
            PS4OutputPtr->LEDGreen = 5 * intensity;
            PS4OutputPtr->LEDBlue = 0;
            break;
        case YELLOW: //yellow
            PS4OutputPtr->LEDRed = 10 * intensity;
            PS4OutputPtr->LEDGreen = 10 * intensity;
            PS4OutputPtr->LEDBlue = 0;
            break;
        case LIME://lime
            PS4OutputPtr->LEDRed = 5 * intensity;
            PS4OutputPtr->LEDGreen = 15 * intensity;
            PS4OutputPtr->LEDBlue = 0;
            break;
        case GREEN: //green
            PS4OutputPtr->LEDRed = 0;
            PS4OutputPtr->LEDGreen = 20 * intensity;
            PS4OutputPtr->LEDBlue = 0;
            break;
        case TURQUOISE: //turquoise
            PS4OutputPtr->LEDRed = 0;
            PS4OutputPtr->LEDGreen = 15 * intensity;
            PS4OutputPtr->LEDBlue = 5 * intensity;
            break;
        case PURPLE: //purple
            PS4OutputPtr->LEDRed = 5 * intensity;
            PS4OutputPtr->LEDGreen = 0;
            PS4OutputPtr->LEDBlue = 15 * intensity;
            break;
        case PINK: //pink
            PS4OutputPtr->LEDRed = 15 * intensity;
            PS4OutputPtr->LEDGreen = 0;
            PS4OutputPtr->LEDBlue = 5 * intensity;
            break;
        case WHITE: //white
            PS4OutputPtr->LEDRed = 5 * intensity;
            PS4OutputPtr->LEDGreen = 5 * intensity;
            PS4OutputPtr->LEDBlue = 5 * intensity;
            break;
        default:
            PS4OutputPtr->LEDRed = 0;
            PS4OutputPtr->LEDGreen = 0;
            PS4OutputPtr->LEDBlue = 0;

            break;
    }
}

void SetLEDColorRGB(unsigned char red, unsigned char green, unsigned char blue, unsigned char blinkOn, unsigned char blinkOff) {
    PS4OutputPtr->LEDRed = red;
    PS4OutputPtr->LEDGreen = green;
    PS4OutputPtr->LEDBlue = blue;
    PS4OutputPtr->blinkOnDuration = blinkOn;
    PS4OutputPtr->blinkOffDuration = blinkOff;
}
/*******************************************************************************
Function: SetRumbleH(unsigned char level)

Precondition:
    None.

Overview:
    This turns on the fast rumble

Input: level: 0-255, determines rumble strength. 
Output: None

 *******************************************************************************/
void SetRumbleH(unsigned char level) {
    if (level < 10) level = 0;
    PS4OutputPtr->rumbleHPower = level;
}

/*******************************************************************************
Function: SetRumbleL(unsigned char level)

Precondition:
    None.

Overview:
    This turns on the slow rumble

Input: level: 0-255, determines rumble strength.

Output: None

 *******************************************************************************/
void SetRumbleL(unsigned char level) {
    if (level < 10) level = 0;
    PS4OutputPtr->rumbleLPower = level;
}

void UpdateBatteryDisplay(BOOL override) { //override will display battery even if psButton isn't pressed
    static unsigned int displayBattCounter;
    
    //override LED state with battery meter if PS button is pressed.
    if (CurrentStateBuffer.Buttons.psButton == 1 || override == TRUE) {
        displayBattCounter = 10;
    }
    if (displayBattCounter > 0) {
        if (CurrentStateBuffer.battery >= 4) 
        {
            SetLEDColor(GREEN,GetLEDBrightness(),0,0);
        }
        else if (CurrentStateBuffer.battery >= 2)
        {
            SetLEDColor(YELLOW,GetLEDBrightness(),0,0);
        }
        else
        {
            SetLEDColor(RED,GetLEDBrightness(),0,0);
        }
        //UART2PutHex(CurrentStateBuffer.battery);
        //UART2PrintString("\n\r");
        displayBattCounter--;

    }
}

//This function is used to control timing for rumble feedback when setting configurations.

void UpdateRumbleFeedback(unsigned int duration) {
    static unsigned int rumbleCounter;

    // if duration > 0, then it is a set operation
    //if duration == 0, then decrement the counter
    if (duration > 0) //set duration
    {
        rumbleCounter = duration;
        SetRumbleH(0xFF);
    }
    else if (duration == 0) //update timer
    {
        if (rumbleCounter > 0) {
            SetRumbleH(0xFF);
            rumbleCounter--;
            if (rumbleCounter == 0) {
                SetRumbleH(0);
            }
        }
    }
}

//Check idle timer used for auto-disconnect

unsigned int GetIdleTimer(void) {
    return idleTimer;
}

//reset idle timer used for auto-disconnect

void ResetIdleTimer(void) {
    idleTimer = 0;
}


