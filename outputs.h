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

//This file holds the functions for configuring and setting the outputs.

#ifndef OUTPUTS_H
#define	OUTPUTS_H

#include <GenericTypeDefs.h>
#include <p24Fxxxx.h>
#include "PS4_controller.h"

#define AUTOCLICK_TIMER 2 //MUST be greater than 0, determines how fast the increment/decrement go when button is held
#define AUTOCLICK_DELAY 50 //dead time before autoclick kicks in
#define FAST_AUTOCLICK_DELAY 1000 //dead time before faster autoclick kicks in (for parameters with potentially large values)
#define FAST_AUTOCLICK_SIZE 10 //how many steps the fast increment takes at a time.

#define NUM_SERVOS 12
#define NUM_BUTTONS 18
#define NUM_ANALOGS 12


//Servo parameter values.   When choosing values, beware of overflow if the values are multiplied by a scalar.
/*
    0:Share button 
    1:L-Stick button 
    2:R-Stick button 
    3:Options button 
    4:D-Pad Up 
    5:D-Pad Right 
    6:D-Pad Down 
    7:D-Pad Left 
    8:L-Bumper
    9:R-Bumper
    10:L-Trig button
    11:R-Trig button
    12:Triangle button
    13:Circle button
    14:Cross button
    15:Square button
    16:PS button
    17:Touchpad click
*/

#define OFFSET_MAX 200
#define OFFSET_MIN -200
#define RANGE_MAX 250
#define RANGE_MIN 0
#define SENSITIVITY_MAX 200
#define SENSITIVITY_MIN 1
#define DEADBAND_MIN 0
#define DEADBAND_MAX 125
#define STOP_LOW 500 //hard stop low at pulse width 0.025 ms.  This is not a user configurable setting.
#define STOP_HIGH 5500 //hard stop high at pulse width 0.275 ms. This is not a user configurable setting.
#define TRIM_MAX 200
#define TRIM_MIN -200

#define AUTOFIRE_PERIOD_MAX 60000
#define AUTOFIRE_PERIOD_MIN 2
#define PULSE_WIDTH_MAX 60000
#define PULSE_WIDTH_MIN 1

/////////////////////
//#define SERVO_PERIOD_MS_MAX 20 //20ms with a 2MHz counter
//#define SERVO_PERIOD_MS_MIN 10

//#define PWM_FREQ_MAX_KHZ 20
//#define PWM_FREQ_MIN_KHZ 1

//#define ISR_IND _LATF3

//button 0-17
#define SHARE _LATF3        //B0
#define LSTICK_PRESS _LATC13//B1
#define RSTICK_PRESS _LATC14//B2
#define OPTIONS _LATD1      //B3
#define DPAD_UP _LATD2      //B4
#define DPAD_RIGHT _LATD3   //B5
#define DPAD_DOWN _LATD4    //B6
#define DPAD_LEFT _LATD5    //B7
#define LTRIGGER_D _LATD6   //B8
#define RTRIGGER_D _LATD7   //B9
#define LBUMPER _LATF0      //B10
#define RBUMPER _LATF1      //B11
#define TRIANGLE _LATE0     //B12
#define CIRCLE _LATE1       //B13
#define CROSS _LATE2        //B14
#define SQUARE _LATE3       //B15
#define PS_BUTTON _LATE4    //B16
#define TPAD_CLICK _LATB3  //B17 This changed from 1.0 to 2.0 boards  

#define INDICATOR_LED _LATE5

#define L_STICK_X 0
#define L_STICK_Y 1
#define R_STICK_X 2
#define R_STICK_Y 3
#define L_TRIG_A 4
#define R_TRIG_A 5
#define L_TPAD_X 6
#define L_TPAD_Y 7
#define R_TPAD_X 8
#define R_TPAD_Y 9
#define TILT_X 10
#define TILT_Y 11

#define PWM10_OUT _LATB8
#define PWM11_OUT _LATB12
#define PWM12_OUT _LATB13 

//#define PWM11_OUT _LATB9 //This changed from 1.0 to 2.0 boards
//#define PWM12_OUT _LATB3 //This changed from 1.0 to 2.0 boards


////////////////////DATA TYPES//////////////////////////////////////////

//netbeans doesn't parse correctly if enum is buried in a struct.  Workaround by using typedef.

typedef enum {
    FORWARD, REVERSE
} SERVO_DIR;

typedef enum {
    ABSOLUTE, RELATIVE
} STICK_ABSOLUTE_RELATIVE;

typedef enum {
    TPAD, JOYSTICK
} TOUCHPAD_MODE;

typedef enum {
    WHOLE, SPLIT
} TOUCHPAD_SPLIT;

typedef enum {
    OFF, L_STICK, R_STICK
} HOLD_RECALL;

typedef enum {
    NO_REMAP,
    LEFT_RIGHT,
    UP_DOWN,
    SQUARE_CIRCLE,
    TRIANGLE_CROSS,
    LR_BUMPERS,
    JOYSTICK_PUSHBUTTONS,
    SHARE_OPTIONS
} BUTTON_PAIRS;

#define SERVO_SETTINGS_LENGTH 12 //numbert of integers for servo settings
typedef union _SERVO_SETTINGS {
    struct __attribute__((packed)) {
        unsigned char deadband;
        unsigned char sensitivity;
        signed int offset;
        signed int range;
        SERVO_DIR direction;
        STICK_ABSOLUTE_RELATIVE absOrRel;
        TOUCHPAD_MODE touchpadMode;
        TOUCHPAD_SPLIT touchpadSplit;
//        signed int filterStrength;
        signed int trim;
        signed int zeroPosition;
        HOLD_RECALL holdRecall;
        unsigned int servoRecallValue;
        BUTTON_PAIRS buttonRemap;
    };
    unsigned int array[SERVO_SETTINGS_LENGTH];
} SERVO_SETTINGS;

typedef enum {
    PUSHBUTTON, TOGGLE, SINGLE_SHOT, AUTOFIRE, TOGGLE_AUTOFIRE
} BUTTON_OUTPUT_MODE; //netbeans doesn't parse correctly if enum is buried in a struct.  Workaround by using typedef.

typedef union _BUTTON_SETTINGS {

    struct __attribute__((aligned(2))) {
        BOOL invert;
        BUTTON_OUTPUT_MODE outputMode;
        unsigned int pulseWidth;
        unsigned int autofirePeriod;
    };
    unsigned int array[4];
} BUTTON_SETTINGS;

////////////////////FUNCTIONS//////////////////////////////////////////
void InitOutputs(void);
void LoadDefaultSettings(BOOL loadAll, unsigned int index);
void LoadSavedSettings(void);
void SaveSettings(void);
SERVO_SETTINGS* GetServoSettings(void);
BUTTON_SETTINGS* GetButtonSettings(void);
unsigned int GetTimeoutSetting(void);
BOOL ConfigOutput(BOOL configFlag);

void UpdateButtonOutputs(unsigned int configFlag, unsigned int configIndex);
void UpdateServoOutputs(void);
void ResetOutputs(CONTROLLER_IN* ControllerIn);
unsigned int GetLEDBrightness(void);
LED_COLORS GetLEDColorSetting(void);
unsigned int GetSpiBusMode(void);
void PrintIndexLabel(unsigned int index);
void PrintMenu(void);
/////////////PRIVATE FUNCTIONS/////////////////////////
void SaveLoadSubroutine(BOOL save_switch);

#endif

