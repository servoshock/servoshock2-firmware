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

 Change log:
 5/12/18: Fixed bug where right stick L servo SPI bus mapping was wrong.
 *************************************************************************/

#include "outputs.h"
#include "main.h"
#include "HardwareProfile.h"
#include "uart2.h"
#include "DEE Emulation 16-bit/DEE Emulation 16-bit.h"
#include "PS4_controller.h"
#include "PS4_SPI.h"
#include "USB/usb_host_generic_PS4.h"
#include <stdlib.h>

/********************************************
 * Index: Input
 * //////////////SERVOS///////////////
    0:L-Stick X-axis
    1:L-Stick Y-axis
    2:R-Stick X-axis
    3:R-Stick Y-axis
    4:L-Trig analog
    5:R-Trig analog
    6:TouchPad1(Left side when split)-X
    7:TouchPad1(Left side when split)-Y
    8:TouchPad2(Right side when split)-X
    9:TouchPad2(Right side when split)-Y
    10:X-Accelerometer
    11:Z-Accelerometer
 * //buttons
    12:D-Pad Up
    13:D-Pad Right
    14:D-Pad Down
    15:D-Pad Left
    16:Triangle button
    17:Circle button
    18:Cross button
    19:Square button
    20:L-Bumper
    21:R-Bumper
    22:L-Trig button
    23:R-Trig button
    24:L-Stick button
    25:R-Stick button
    26:Share button
    27:Options button
    28:Touchpad click
    29:PS button
 **********************************************/


///////////////////////PWM REGISTERS//////////////////
static unsigned int oc10r, oc10rs, oc10con1, oc11r, oc11rs, oc11con1, oc12r, oc12rs, oc12con1;
static volatile unsigned int* const OCxR_REG_PTRS[12] = {&OC1R, &OC2R, &OC3R, &OC4R, &OC5R, &OC6R, &OC7R, &OC8R, &OC9R, &oc10r, &oc11r, &oc12r};
static volatile unsigned int* const OCxRS_REG_PTRS[12] = {&OC1RS, &OC2RS, &OC3RS, &OC4RS, &OC5RS, &OC6RS, &OC7RS, &OC8RS, &OC9RS, &oc10rs, &oc11rs, &oc12rs};
static volatile unsigned int* const OCxCON1_REG_PTRS[12] = {&OC1CON1, &OC2CON1, &OC3CON1, &OC4CON1, &OC5CON1, &OC6CON1, &OC7CON1, &OC8CON1, &OC9CON1, &oc10con1, &oc11con1, &oc12con1};

static SERVO_SETTINGS ServoSettings[NUM_SERVOS];
static signed long int servoState[NUM_SERVOS] = {300000, 300000, 300000, 300000, 300000, 300000, 300000, 300000, 300000, 300000, 30000, 30000};

///////////////CONFIG SETTINGS/////////////////////////
//see LoadDefaultSettings for default settings.
static BUTTON_SETTINGS ButtonSettings[NUM_BUTTONS];
static BOOL triggerLink;
static BUTTONS buttonToggles;
static unsigned int spiBusMode;
static unsigned int disconnectToggleReset;
static unsigned int idleTimeout;
static BOOL lStickDiffMix;
static BOOL rStickDiffMix;
static unsigned int LEDBrightness;
static LED_COLORS LEDColorSetting;
static unsigned int PPMOutput = 0;


static enum {
    MECANUM_OFF = 0,
    MECANUM_ON_NO_REMAP = 1,
    MECANUM_REMAP_4 = 2,
    MECANUM_REMAP_5 = 3,
    MECANUM_REMAP_6 = 4,
    MECANUM_REMAP_7 = 5,
    MECANUM_REMAP_8 = 6,
    MECANUM_REMAP_9 = 7,
    MECANUM_REMAP_10 = 8,
    MECANUM_REMAP_11 = 9,
} mecanumSteering = MECANUM_OFF;


//////////////PUBLIC FUNCTIONS///////////////////////////

void InitOutputs(void) {
    unsigned int i;

    //Set initial Servo period, time base Timer 2
    T2CON = 0; //clear state
    T2CONbits.TSIDL = 0; //Continue in Idle
    T2CONbits.TGATE = 0; //No gate accumulation
    T2CONbits.TCKPS = 0b01; //Prescale 1:8
    T2CONbits.T32 = 0; //16-bit mode
    T2CONbits.TCS = 0; //Clock Source: Internal Clock (Fosc/2)
    TMR2 = 0;

    //***

    for (i = 0; i < NUM_SERVOS; i++) {
        *OCxCON1_REG_PTRS[i] = 0b110;
        *OCxRS_REG_PTRS[i] = 1; //this is just to keep it rolling over so we can stagger the outputs below
        *OCxR_REG_PTRS[i] = 3000;
    }
    OC3RS = 10;
    OC1CON2 = 0x003F; //use own OCxRS register for sync
    OC2CON2 = 0x003F; //use own OCxRS register for sync
    OC3CON2 = 0x003F; //use own OCxRS register for sync
    OC4CON2 = 0x003F; //use own OCxRS register for sync
    OC5CON2 = 0x003F; //use own OCxRS register for sync
    OC6CON2 = 0x003F; //use own OCxRS register for sync
    OC7CON2 = 0x003F; //use own OCxRS register for sync
    OC8CON2 = 0x003F; //use own OCxRS register for sync
    OC9CON2 = 0x003F; //use own OCxRS register for sync
    
    T2CONbits.TON = 1; //Timer2 On
    
    //stagger the pulses so that they don't all go high at the same time.
    DelayMs(100); //I think we need to wait for TMR2 to roll over
    OC1RS = 40000;
    DelayMs(2);
    OC2RS = 40000;
    DelayMs(2);
    OC3RS = 40000;
    DelayMs(2);
    OC4RS = 40000;
    DelayMs(2);
    OC5RS = 40000;
    DelayMs(2);
    OC6RS = 40000;
    DelayMs(2);
    OC7RS = 40000;
    DelayMs(2);
    OC8RS = 40000;
    DelayMs(2);
    OC9RS = 40000;

    
    //Stagger the servo signal turn-on sequence to reduce the inrush current
    RPOR10bits.RP21R = 18; //Map PWM 1 Output to RP21
    OC1CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR13bits.RP26R = 19; //Map PWM 2 Output to RP26
    OC2CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR9bits.RP19R = 20; //Map PWM 3 Output to RP19
    OC3CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR13bits.RP27R = 21; //Map PWM 4 Output to RP27
    OC4CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR9bits.RP18R = 22; //Map PWM 5 Output to RP18
    OC5CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR14bits.RP28R = 23; //Map PWM 6 Output to RP28
    OC6CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR6bits.RP13R = 24; //Map PWM 7 Output to RP13
    OC7CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR0bits.RP1R = 25; //Map PWM 8 Output to RP1
    OC8CON2bits.OCTRIS = 0;
    DelayMs(100);
    RPOR0bits.RP0R = 35; //Map PWM 9 Output to RP0
    OC9CON2bits.OCTRIS = 0;

        /////////////Use Timer3 for PWM 10
    T3CON = 0; //clear state
    T3CONbits.TSIDL = 0; //Continue in Idle
    T3CONbits.TGATE = 0; //No gate accumulation
    T3CONbits.TCKPS = 0b01; //Prescale 1:8
    T3CONbits.TCS = 0; //Clock Source: Internal Clock (Fosc/2)
    TMR3 = 0;
    T3CONbits.TON = 1;
    //enable timer4 interrupt
    _T3IE = 1;
    _T3IP = 7; //high priority interrupt
    //////////////////////////////////
}

/*******************************************************************************
Function: LoadDefaultSettings(BOOL loadAll, unsigned int index)

Precondition:
 None

Overview:
 load default settings

Input:
 loadAll:load all the default settings
 index: load the default settings for the channel if loadAll is False.

Output:
 None

 *******************************************************************************/
void LoadDefaultSettings(BOOL loadAll, unsigned int index) {
    unsigned char errorCode=0;
    unsigned int i = 0;
    if (loadAll == FALSE) {
        i = index;
        UART2PrintString("Loading default settings for ");
        PrintIndexLabel(i);
    }
    else {
        UART2PrintString("Loading all default settings...");
        triggerLink = FALSE;
        spiBusMode = 0;
        disconnectToggleReset = 0;
        idleTimeout = 60000;//
        lStickDiffMix = FALSE;
        rStickDiffMix = FALSE;
        mecanumSteering = MECANUM_OFF;
        LEDBrightness = 1;
        LEDColorSetting = 11;
        PPMOutput = FALSE;

    }
    do {
        if (i < NUM_SERVOS) { //servo settings
            ServoSettings[i].offset = 0;
            ServoSettings[i].range = 125;
            ServoSettings[i].deadband = 0;
            ServoSettings[i].sensitivity = 10;
            ServoSettings[i].direction = FORWARD;
            ServoSettings[i].absOrRel = ABSOLUTE;
//            ServoSettings[i].filterStrength = 0;
            ServoSettings[i].touchpadMode = TPAD;
            ServoSettings[i].touchpadSplit = SPLIT;
            ServoSettings[i].trim = 0;
            ServoSettings[i].zeroPosition = 0; //used for calibration of stick
            ServoSettings[i].holdRecall = OFF;
            ServoSettings[i].servoRecallValue = 3000;
            ServoSettings[i].buttonRemap = NONE;

            switch (i) //specific cases
            {
                case L_STICK_X:
                case L_STICK_Y:
                case R_STICK_X:
                case R_STICK_Y:
                    ServoSettings[i].zeroPosition = 128;
                    break;
            }  


            errorCode = DataEEWrite(3000, EEPROM_SERVO_HOME + i);
        }
        else { //button settings
            ButtonSettings[i - NUM_SERVOS].invert = FALSE;
            ButtonSettings[i - NUM_SERVOS].outputMode = PUSHBUTTON;
            ButtonSettings[i - NUM_SERVOS].pulseWidth = 5;
            ButtonSettings[i - NUM_SERVOS].autofirePeriod = 20;
        }
        i++;
    } while (i < (NUM_SERVOS + NUM_BUTTONS) && loadAll == TRUE);
    if (errorCode !=0){
        UART2PrintString("EEPROM write error: ");
        UART2PutDec(errorCode);
        UART2PrintString("\n\r");
    }
    UART2PrintString("Done\n\r");
}

/*******************************************************************************
Function: SaveLoadSubroutine( BOOL save)

Precondition:
 None

Overview:
  Save or load settings.

Input:
  if save_switch==1, then save, if save==0, then load

Output:
 None

 *******************************************************************************/

void SaveLoadSubroutine(BOOL save_switch) {
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int memory_index = EEPROM_CONFIG_SETTINGS_START;
    unsigned int error;

    //servo settings
    for (i = 0; i < NUM_SERVOS; i++) {
        for (j = 0; j < SERVO_SETTINGS_LENGTH; j++) {
            if (save_switch){
                error = DataEEWrite(ServoSettings[i].array[j], memory_index);
                if (error != 0) UART2PutDecInt(error);
            }
            else {
                ServoSettings[i].array[j] = DataEERead(memory_index);
            }
#define DEBUG_EEPROM
#ifdef DEBUG_EEPROM
            UART2PutDecInt(memory_index);
            UART2PrintString("S");
            UART2PutDecInt(i);
            UART2PrintString("-");
            UART2PutDecInt(j);
            UART2PrintString(": ");
            UART2PutHexWord(ServoSettings[i].array[j]);
            UART2PrintString("\n\r");
#endif
            memory_index++;
        }
    }

    //button settings
    for (i = 0; i < NUM_BUTTONS; i++) {
        for (j = 0; j < 4; j++) {
            if (save_switch) DataEEWrite(ButtonSettings[i].array[j], memory_index);
            else ButtonSettings[i].array[j] = DataEERead(memory_index);
#ifdef DEBUG_EEPROM
            UART2PutDecInt(memory_index);
            UART2PrintString("B");
            UART2PutDecInt(i);
            UART2PrintString("-");
            UART2PutDecInt(j);
            UART2PrintString(": ");
            UART2PutHexWord(ButtonSettings[i].array[j]);
            UART2PrintString("\n\r");
#endif
            memory_index++;
        }
    }
    
    //universal settings
    if (save_switch) DataEEWrite(triggerLink, memory_index);
    else triggerLink = DataEERead(memory_index);
    /*
    UART2PutDecInt(memory_index);
    UART2PrintString("Trig Link:");
    UART2PutHexWord(triggerLink);
    UART2PrintString("\n\r");
     */
    
    memory_index++;

    
    if (save_switch) DataEEWrite(spiBusMode, memory_index);
    else spiBusMode = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(disconnectToggleReset, memory_index);
    else disconnectToggleReset = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(idleTimeout, memory_index);
    else idleTimeout = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(lStickDiffMix, memory_index);
    else lStickDiffMix = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(rStickDiffMix, memory_index);
    else rStickDiffMix = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(mecanumSteering, memory_index);
    else mecanumSteering = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(LEDBrightness, memory_index);
    else LEDBrightness = DataEERead(memory_index);
    memory_index++;

    if (save_switch) DataEEWrite(LEDColorSetting, memory_index);
    else LEDColorSetting = DataEERead(memory_index);
    memory_index++;
    
    if (save_switch) DataEEWrite(PPMOutput, memory_index);
    else PPMOutput = DataEERead(memory_index);
    memory_index++;
#ifdef DEBUG_EEPROM
    UART2PrintString("PPM:");
    UART2PutHexWord(PPMOutput);
    UART2PrintString("\n\r");
#endif
    
    UART2PutDecInt(memory_index);
    UART2PrintString(" EEPROM words used.\n\r");

    //when adding settings here, don't forget to update the default load settings function above
}

/*******************************************************************************
Function: LoadSavedSettings()

Precondition:
 None

Overview:
 Wrapper for SaveLoadSettings, loads previously saved settings

Input:
  None

Output:
 None

 *******************************************************************************/
void LoadSavedSettings(void) {
    UART2PrintString("Loading saved settings...\n\r");
    SaveLoadSubroutine(0);
    UART2PrintString("Done\n\r");
}

/*******************************************************************************
Function: SaveSettings()

Precondition:
 None

Overview:
 Wrapper for SaveLoadSettings, saves settings

Input:
  None

Output:
 None

 *******************************************************************************/
void SaveSettings(void) {

    UART2PrintString("Saving Settings...");
    SaveLoadSubroutine(1);
    UART2PrintString("Done.\n\r");
}

//Return servo settings

SERVO_SETTINGS* GetServoSettings(void) {
    return ServoSettings;
}

BUTTON_SETTINGS* GetButtonSettings(void) {
    return ButtonSettings;
}

unsigned int GetTimeoutSetting(void) {
    return idleTimeout;
}

/*******************************************************************************
Function: ConfigOutput()

Precondition:
 None

Overview:
    This routine allows the user to modify output settings

Input: 
 configFlag == TRUE will continue to run the routine.
 configFlag == FALSE will exit the routine.

Output:
  0 = save and exited;
  1 = stay in config mode;

 *******************************************************************************/

BOOL ConfigOutput(BOOL configFlag) {
    static BOOL configCmdReleased = FALSE;
    static unsigned int cycleCounter = 0; //increment each time we enter the function
    static unsigned int index = 0; //index to mark which output we're configuring
    static BOOL settingsSaved;

    /*
     //Servos
    0:L-Stick X-axis
    1:L-Stick Y-axis
    2:R-Stick X-axis
    3:R-Stick Y-axis
    4:L-Trig analog
    5:R-Trig analog
    6:TouchPad1(Left side when split)-X
    7:TouchPad1(Left side when split)-Y
    8:TouchPad2(Right side when split)-X
    9:TouchPad2(Right side when split)-Y
    10:X-Accelerometer
    11:Z-Accelerometer
 * //Buttons
    12:Share button 
    13:L-Stick button 
    14:R-Stick button 
    15:Options button 
    16:D-Pad Up 
    17:D-Pad Right 
    18:D-Pad Down 
    19:D-Pad Left 
    20:L-Bumper
    21:R-Bumper
    22:L-Trig button
    23:R-Trig button
    24:Triangle button
    25:Circle button
    26:Cross button
    27:Square button
    28:PS button
    29:Touchpad click
     */

    /////////////////////////////////////////////////////////////////////////////////
    //wait for user to release the "enter config" command buttons so it won't start
    //making changes right away.
    if (configCmdReleased == FALSE && GetPressDuration()->share == 0 && GetPressDuration()->psButton == 0) {
        //initialize config mode flags
        UART2PrintString("===Entering Config Mode:\n\r\n\r*Press 'Esc' to display config guide===\n\r\n\r");
        configCmdReleased = TRUE;
        index = 0;
        cycleCounter = 0;
        PrintIndexLabel(index);
        UpdateButtonOutputs(FALSE, 0); //update the button outputs before we start and reset the "share" and "PS button" outputs.

    }
    else if (configCmdReleased == FALSE) //force user to release buttons so we don't start making changes right away
    {
        UpdateRumbleFeedback(2);
        return TRUE;
    }


    //////////exit when button is released to prevent the Start button output from activating
    //////////immediately upon exit.
    cycleCounter++;

    ////////////////////Commands here apply when configuring either servos or buttons////////////////////////
    /////////////////////IF NEITHER STICK BUTTON IS DEPRESSED. Beware of overlap with other configs commands/////////////////////////////////
    if (!GetButtonPress()->rStickPress && !GetButtonPress()->lStickPress) {
        //update index
        if (GetNewPress()->rBumper) {
            index++;
            index = index % (NUM_SERVOS + NUM_BUTTONS);
            PrintIndexLabel(index);
        }
        if (GetNewPress()->lBumper) {
            if (index == 0) {
                index = (NUM_SERVOS + NUM_BUTTONS) - 1;
            }
            else {
                index--;
            }
            index = index % (NUM_SERVOS + NUM_BUTTONS);
            PrintIndexLabel(index);
        }
        SetLEDColor(index % 10, 10, (index / 10)*20, (index / 10)*10);

        //Reset Defaults
        if (GetPressDuration()->share == 100 && GetButtonPress()->options == 0) //check start button to make sure user isn't trying to disconnect
        {
            LoadDefaultSettings(FALSE, index);
            UpdateRumbleFeedback(5);
        }

        if (GetPressDuration()->share == 300 && GetButtonPress()->options == 0) //check start button to make sure user isn't trying to disconnect
        {
            LoadDefaultSettings(TRUE, 0);
            UpdateRumbleFeedback(10);
        }

        //Save and exit
        if (GetPressDuration()->options == 100 && GetButtonPress()->share == 0) //check select button to make sure user isn't trying to disconnect
        {
            SaveSettings();
            settingsSaved = TRUE;
            UpdateRumbleFeedback(5);
        }
        else if ((GetButtonPress()->options == 0 && settingsSaved == TRUE) || configFlag == FALSE) {
            UART2PrintString("Exiting Config Mode.\n\r");
            //reset flags
            configCmdReleased = FALSE;
            settingsSaved = FALSE;
            return FALSE;
        }
    } /////////////////////IF RIGHT STICK BUTTON IS DEPRESSED  Beware of overlap with other configs commands/////////////////////////////////
    else if (GetButtonPress()->rStickPress && !GetButtonPress()->lStickPress) //if right stick switch is pressed
    {
        //Cycle SPI bus modes;
        if (GetNewPress()->triangle) //if right stick switch is pressed
        {
            spiBusMode++;
            spiBusMode = spiBusMode % 4;
            switch (spiBusMode) {
                case 0:
                    UART2PrintString("SPI Bus Mode 0\n\r");
                    SPI1STATbits.SPIEN = 0;
                    SPI1CON1bits.CKP = 0;
                    SPI1CON1bits.CKE = 1;
                    SPI1STATbits.SPIEN = 1;
                    break;

                case 1:
                    UART2PrintString("SPI Bus Mode 1\n\r");
                    SPI1STATbits.SPIEN = 0;
                    SPI1CON1bits.CKP = 0;
                    SPI1CON1bits.CKE = 0;
                    SPI1STATbits.SPIEN = 1;
                    break;

                case 2:
                    UART2PrintString("SPI Bus Mode 2\n\r");
                    SPI1STATbits.SPIEN = 0;
                    SPI1CON1bits.CKP = 1;
                    SPI1CON1bits.CKE = 1;
                    SPI1STATbits.SPIEN = 1;
                    break;

                case 3:
                    UART2PrintString("SPI Bus Mode 3\n\r");
                    SPI1STATbits.SPIEN = 0;
                    SPI1CON1bits.CKP = 1;
                    SPI1CON1bits.CKE = 0;
                    SPI1STATbits.SPIEN = 1;
                    break;

                default:
                    UART2PrintString("ERROR SPI BUS MODE\n\r");
                    break;
            }
        }
        if (GetNewPress()->cross) {
            if (idleTimeout == 0) {
                idleTimeout = 60000;
                UART2PrintString("Idle auto-off set to 10 min.\n\r");
            }
            else {
                idleTimeout = 0;
                UART2PrintString("Idle auto-off disabled.\n\r");
            }
        }
        if (GetNewPress()->psButton) {
            disconnectToggleReset = !disconnectToggleReset;
            if (disconnectToggleReset) UART2PrintString("Toggled outputs reset on disconnect.\n\r");
            else UART2PrintString("Toggled outputs preserved on disconnect.\n\r");
        }
    }
        /////////////////////IF LEFT STICK BUTTON IS DEPRESSED  These are universal settings, beware with overlap for channel specific settings/////////////////////////////////
    else if (GetButtonPress()->lStickPress && !GetButtonPress()->rStickPress) //if left stick switch is pressed
    {
        if (GetNewPress()->dPadRight || (GetPressDuration()->dPadRight > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
            if (LEDBrightness < 10) {
                LEDBrightness++;
            }
            else {
                UpdateRumbleFeedback(2);
            }
            UART2PrintString("LED Brightness: ");
            UART2PutDec(LEDBrightness);
            UART2PrintString("\n\r");
        }

        //LED Brightness
        if (GetNewPress()->dPadLeft || (GetPressDuration()->dPadLeft > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
            if (LEDBrightness > 0) {
                LEDBrightness--;
            }
            else {
                UpdateRumbleFeedback(2);
            }
            UART2PrintString("LED Brightness: ");
            UART2PutDec(LEDBrightness);
            UART2PrintString("\n\r");
        }
        
        if (GetNewPress()->share) {
            LEDColorSetting++;
            if (LEDColorSetting > BATTERY) 
            {
                LEDColorSetting = 0;
            }
            UART2PrintString("LED Color Setting: ");
            switch (LEDColorSetting) {
                case BLUE: //blue
                    UART2PrintString("Blue");                    
                    break;
                case RED: //red
                    UART2PrintString("Red");  
                    break;
                case ORANGE: //orange
                    UART2PrintString("Orange");  
                    break;
                case YELLOW: //yellow
                    UART2PrintString("Yellow");  
                    break;
                case LIME://lime
                    UART2PrintString("Lime");  
                    break;
                case GREEN: //green
                    UART2PrintString("Green");  
                    break;
                case TURQUOISE: //turquoise
                    UART2PrintString("Turquoise");  
                    break;
                case PURPLE: //purple
                    UART2PrintString("Purple");  
                    break;
                case PINK: //pink
                    UART2PrintString("Pink");  
                    break;
                case WHITE: //white
                    UART2PrintString("White"); 
                    break;
                case COLOR_ADC:
                    UART2PrintString("ADC");
                    break;
                case BATTERY:
                    UART2PrintString("Battery");  
                    break;
                default:
                    UART2PrintString("Error");  
                    break;
                }
            UART2PrintString("\n\r");
        }
        if (GetNewPress()->options) {
            PPMOutput = !PPMOutput;
            UART2PrintString("PPM Output Mode = ");
            UART2PutDecInt(PPMOutput);
            UART2PrintString("\n\r");
        }
    }

    /////////////////SET SERVO SETTINGS///////////////////////
    if (index < NUM_SERVOS) {
        /////////////////////If right stick is depressed while configuring servos/////////////////////////////////
        //if the RStick button is held down, enable different set of options.
        if (GetButtonPress()->rStickPress && !GetButtonPress()->lStickPress) {
            //Increase output trim
            if (GetNewPress()->dPadRight || (GetPressDuration()->dPadRight > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].trim < TRIM_MAX) {
                    ServoSettings[index].trim++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Trim = ");
                UART2PutDecSInt(ServoSettings[index].trim);
                UART2PrintString("\n\r");
            }

            //Decrease output trim
            if (GetNewPress()->dPadLeft || (GetPressDuration()->dPadLeft > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].trim > TRIM_MIN) {
                    ServoSettings[index].trim--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Trim = ");
                UART2PutDecSInt(ServoSettings[index].trim);
                UART2PrintString("\n\r");
            }

            if (index == L_TPAD_X || index == L_TPAD_Y || index == R_TPAD_X || index == R_TPAD_Y) {
                //TPAD/Virtual Joystick
                if (GetNewPress()->dPadUp) {

                    if (ServoSettings[index].touchpadMode == TPAD) {
                        ServoSettings[index].touchpadMode = JOYSTICK;
                        UART2PrintString("Touchpad Mode = Virtual Joystick\n\r");
                    }
                    else {
                        ServoSettings[index].touchpadMode = TPAD;
                        UART2PrintString("Touchpad Mode = Touchpad\n\r");
                    }
                }

                //Split Touchpad
                if (GetNewPress()->dPadDown) {

                    if (ServoSettings[index].touchpadSplit == WHOLE) {
                        ServoSettings[index].touchpadSplit = SPLIT;
                        UART2PrintString("Touchpad Split = On\n\r");
                    }
                    else {
                        ServoSettings[index].touchpadSplit = WHOLE;
                        UART2PrintString("Touchpad Split = Off\n\r");
                    }
                }
            }

            //Set servo forward/reverse
            if (GetNewPress()->lBumper) {
                ServoSettings[index].direction = !ServoSettings[index].direction;
                if (ServoSettings[index].direction == FORWARD) {
                    UART2PrintString("Servo/Motor Forward.\n\r");
                }
                else if (ServoSettings[index].direction == REVERSE) {
                    UART2PrintString("Servo/Motor Reverse.\n\r");
                }
            }

            //Set absolute/relative mode
            if (GetNewPress()->rBumper) {
                ServoSettings[index].absOrRel = !ServoSettings[index].absOrRel;
                if (ServoSettings[index].absOrRel == ABSOLUTE) {
                    UART2PrintString("Position input mode.\n\r");
                }
                else if (ServoSettings[index].absOrRel == RELATIVE) {
                    UART2PrintString("Incremental input mode.\n\r");
                }
            }
            
            if (GetNewPress()->square) //mecanum steering
            {
                ServoSettings[index].buttonRemap++;
                ServoSettings[index].buttonRemap = ServoSettings[index].buttonRemap % 8;
                switch (ServoSettings[index].buttonRemap) {
                    case NO_REMAP:
                        UART2PrintString("Servo not remapped.\n\r");
                        break;
                    case LEFT_RIGHT:
                        UART2PrintString("Servo remapped to D-pad left/right.\n\r");
                        break;
                    case UP_DOWN:
                        UART2PrintString("Servo remapped to D-pad up/down.\n\r");
                        break;
                    case SQUARE_CIRCLE:
                        UART2PrintString("Servo remapped to square/circle.\n\r");
                        break;
                    case TRIANGLE_CROSS:
                        UART2PrintString("Servo remapped to D-pad triangle/cross.\n\r");
                        break;
                    case LR_BUMPERS:
                        UART2PrintString("Servo remapped to D-pad left/right bumpers.\n\r");
                        break;
                    case JOYSTICK_PUSHBUTTONS:
                        UART2PrintString("Servo remapped to left/right joystick pushbuttons.\n\r");
                        break;
                    case SHARE_OPTIONS:
                        UART2PrintString("Servo remapped to share/options.\n\r");
                        break;                    
                    default:
                        UART2PrintString("Error.\n\r");
                        break;
                }
            }

        }//end settings if rstick is held down

            /////////////////////If left stick is depressed while configuring servos/////////////////////////////////
        else if (GetButtonPress()->lStickPress && !GetButtonPress()->rStickPress) {
            //set filter coefficient
            /*
            if (GetNewPress()->dPadUp || (GetPressDuration()->dPadUp > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].filterStrength < 50) {
                    ServoSettings[index].filterStrength++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Smoothing: ");
                UART2PutDec(ServoSettings[index].filterStrength);
                UART2PrintString("\n\r");
            }

            //set filter coefficient
            if (GetNewPress()->dPadDown || (GetPressDuration()->dPadDown > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].filterStrength > 0) {
                    ServoSettings[index].filterStrength--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Smoothing: ");
                UART2PutDec(ServoSettings[index].filterStrength);
                UART2PrintString("\n\r");
            }
             */

            //SET TRIGGER LINK
            /*linking the trigger linkes the 2 triggers so that one move the servo in a
             * positive direction and the other moves the servo in a negative direction.*/
            if ((index == L_TRIG_A || index == R_TRIG_A) && GetNewPress()->cross) {
                triggerLink = !triggerLink;
                if (triggerLink == TRUE) {
                    UART2PrintString("Trigger Link On.\n\r");
                }
                else {
                    UART2PrintString("Trigger Link Off.\n\r");
                }
            }

            //Left Stick only settings
            if (index == L_STICK_X || index == L_STICK_Y)//Left stick settings
            {
                if (GetNewPress()->cross) //Left Stick Differential Mix Enable/Disable
                {
                    lStickDiffMix = !lStickDiffMix;
                    if (lStickDiffMix == TRUE) {
                        UART2PrintString("Left Stick Differential Mix On.\n\r");
                    }
                    else {
                        UART2PrintString("Left Stick Differential Mix Off.\n\r");
                    }
                }
            }
            //Right Stick Settings
            if (index == R_STICK_X || index == R_STICK_Y) {
                if (GetNewPress()->cross) //Right Stick Differential Mix Enable/Disable
                {
                    rStickDiffMix = !rStickDiffMix;
                    if (rStickDiffMix == TRUE) {
                        UART2PrintString("Right Stick Differential Mix On.\n\r");
                    }
                    else {
                        UART2PrintString("Right Stick Differential Mix Off.\n\r");
                    }
                }
            }

            //Configurations valid for both sticks
            if (GetNewPress()->square) //mecanum steering
            {
                mecanumSteering++;
                mecanumSteering = mecanumSteering % 10;

                switch (mecanumSteering) {
                    case MECANUM_OFF:
                        UART2PrintString("Mecanum Steering Mix Off.\n\r");
                        break;
                    case MECANUM_ON_NO_REMAP:
                        UART2PrintString("Mecanum Steering Mix On.\n\r");
                        break;
                    default:
                        UART2PrintString("Mecanum Steering Mix On, remap right stick y-axis to ");
                        PrintIndexLabel(mecanumSteering + 2);
                        break;
                }
            }

            if (GetNewPress()->circle) {
                ServoSettings[index].holdRecall++;
                ServoSettings[index].holdRecall = ServoSettings[index].holdRecall % 3;
                switch (ServoSettings[index].holdRecall) {
                    case OFF:
                        UART2PrintString("Stick Hold/Recall disabled.\n\r");
                        break;
                    case L_STICK:
                        UART2PrintString("Stick Hold/Recall tied to L-Stick Button.\n\r");
                        break;
                    case R_STICK:
                        UART2PrintString("Stick Hold/Recall tied to R-Stick Button.\n\r");
                        break;
                    default:
                        break;
                }
            }

            //Set RC Servo/PWM mode
            if (GetNewPress()->lBumper) {

            }
        }//end settings if L-Stick is held down

            /////////////////////If neither stick is depressed while configuring servos/////////////////////////////////
        else //if LSTick or RStick are not held down
        {
            //Increase offset
            if (GetNewPress()->dPadRight || (GetPressDuration()->dPadRight > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].offset < OFFSET_MAX) {
                    ServoSettings[index].offset++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Range Offset = ");
                UART2PutDecSInt(ServoSettings[index].offset);
                UART2PrintString("\n\r");
            }

            //Decrease offset
            if (GetNewPress()->dPadLeft || (GetPressDuration()->dPadLeft > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].offset > OFFSET_MIN) {
                    ServoSettings[index].offset--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Range Offset = ");
                UART2PutDecSInt(ServoSettings[index].offset);
                UART2PrintString("\n\r");
            }

            //Increase range
            if (GetNewPress()->dPadUp || (GetPressDuration()->dPadUp > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].range < RANGE_MAX) {
                    ServoSettings[index].range++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Range = ");
                UART2PutDecSInt(ServoSettings[index].range);
                UART2PrintString("\n\r");
            }

            //Decrease range
            if (GetNewPress()->dPadDown || (GetPressDuration()->dPadDown > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].range > RANGE_MIN) {
                    ServoSettings[index].range--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Range = ");
                UART2PutDecSInt(ServoSettings[index].range);
                UART2PrintString("\n\r");
            }

            //Increase deadband
            if (GetNewPress()->circle || (GetPressDuration()->circle > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].deadband < DEADBAND_MAX) {
                    ServoSettings[index].deadband++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Deadband = ");
                UART2PutDec(ServoSettings[index].deadband);
                UART2PrintString("\n\r");
            }

            //Decrease deadband
            if (GetNewPress()->square || (GetPressDuration()->square > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].deadband > DEADBAND_MIN) {
                    ServoSettings[index].deadband--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Deadband = ");
                UART2PutDec(ServoSettings[index].deadband);
                UART2PrintString("\n\r");
            }
            //Increase sensitivity
            if (GetNewPress()->triangle || (GetPressDuration()->triangle > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].sensitivity < SENSITIVITY_MAX) {
                    ServoSettings[index].sensitivity++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Sensitivity = ");
                UART2PutDec(ServoSettings[index].sensitivity);
                UART2PrintString("\n\r");
            }

            //Decrease sensitivity
            if (GetNewPress()->cross || (GetPressDuration()->cross > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ServoSettings[index].sensitivity > SENSITIVITY_MIN) {
                    ServoSettings[index].sensitivity--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Sensitivity = ");
                UART2PutDec(ServoSettings[index].sensitivity);
                UART2PrintString("\n\r");
            }

            //Recalibration
            if (GetNewPress()->psButton) {
                ServoSettings[index].zeroPosition = GetAnalogs()->array[index];
                UART2PrintString("Home Position = ");
                UART2PutDecInt(ServoSettings[index].zeroPosition);
                UART2PrintString("\n\r");
            }
            if (GetPressDuration()->psButton == 100) {
                unsigned int i;
                UART2PrintString("Calibrating All Home Positions...");
                for (i = 0; i < NUM_SERVOS; i++) {
                    ServoSettings[i].zeroPosition = GetAnalogs()->array[i];
                }
                UART2PrintString("Done.\n\r");
                UpdateRumbleFeedback(10);
            }
        }//end if rStick is not pressed
    }// if i<NUM_SERVOS

    ////////////////SET BUTTONS///////////////////////
    if (index >= NUM_SERVOS && index < (NUM_SERVOS + NUM_BUTTONS)) {
        unsigned int buttonIndex;
        buttonIndex = index - NUM_SERVOS;

        if (GetButtonPress()->rStickPress && !GetButtonPress()->lStickPress) {
        }//if right stick is held
        else if (GetButtonPress()->lStickPress && !GetButtonPress()->rStickPress) {
        }//if left stick is held
        else //if neither stick is held
        {
            //set invert
            if (GetNewPress()->cross) {
                ButtonSettings[buttonIndex].invert = !ButtonSettings[buttonIndex].invert;
                if (ButtonSettings[buttonIndex].invert == TRUE) {
                    UART2PrintString("Output Inverted.\n\r");
                }
                else {
                    UART2PrintString("Output Normal.\n\r");
                }
            }

            if (GetNewPress()->triangle) {
                //set output style
                switch (ButtonSettings[buttonIndex].outputMode) {
                    case PUSHBUTTON:
                        ButtonSettings[buttonIndex].outputMode = TOGGLE;
                        UART2PrintString("Output Toggle.\n\r");
                        break;

                    case TOGGLE:
                        ButtonSettings[buttonIndex].outputMode = SINGLE_SHOT;
                        UART2PrintString("Output Single Shot.\n\r");
                        break;

                    case SINGLE_SHOT:
                        ButtonSettings[buttonIndex].outputMode = AUTOFIRE;
                        UART2PrintString("Output Autofire.\n\r");
                        break;

                    case AUTOFIRE:
                        ButtonSettings[buttonIndex].outputMode = TOGGLE_AUTOFIRE;
                        UART2PrintString("Output Toggle Autofire.\n\r");
                        break;

                    case TOGGLE_AUTOFIRE:
                        ButtonSettings[buttonIndex].outputMode = PUSHBUTTON;
                        UART2PrintString("Output Pushbutton.\n\r");
                        break;

                    default:
                        UART2PrintString("ERROR OUTPUT MODE.\n\r");
                        break;
                }
            }

            //Increase pulse width
            if ((GetPressDuration()->dPadRight > FAST_AUTOCLICK_DELAY)&&\
                (ButtonSettings[buttonIndex].pulseWidth < (PULSE_WIDTH_MAX - FAST_AUTOCLICK_SIZE))) {
                ButtonSettings[buttonIndex].pulseWidth += FAST_AUTOCLICK_SIZE;
            }
            if (GetNewPress()->dPadRight || (GetPressDuration()->dPadRight > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ButtonSettings[buttonIndex].pulseWidth < PULSE_WIDTH_MAX) {
                    ButtonSettings[buttonIndex].pulseWidth++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("PulseWidth = ");
                UART2PutDecInt(ButtonSettings[buttonIndex].pulseWidth);
                UART2PrintString("0 ms\n\r");
            }

            //Decrease pulse width
            if ((GetPressDuration()->dPadLeft > FAST_AUTOCLICK_DELAY)&&\
                (ButtonSettings[buttonIndex].pulseWidth > (PULSE_WIDTH_MIN + FAST_AUTOCLICK_SIZE))) {
                ButtonSettings[buttonIndex].pulseWidth -= FAST_AUTOCLICK_SIZE;
            }
            if (GetNewPress()->dPadLeft || (GetPressDuration()->dPadLeft > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ButtonSettings[buttonIndex].pulseWidth > PULSE_WIDTH_MIN) {
                    ButtonSettings[buttonIndex].pulseWidth--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("PulseWidth = ");
                UART2PutDecInt(ButtonSettings[buttonIndex].pulseWidth);
                UART2PrintString("0 ms\n\r");
            }

            //Increase Autofire Period
            if ((GetPressDuration()->dPadUp > FAST_AUTOCLICK_DELAY)&&\
                (ButtonSettings[buttonIndex].autofirePeriod < (AUTOFIRE_PERIOD_MAX - FAST_AUTOCLICK_SIZE))) {
                ButtonSettings[buttonIndex].autofirePeriod += FAST_AUTOCLICK_SIZE;
            }
            if (GetNewPress()->dPadUp || (GetPressDuration()->dPadUp > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ButtonSettings[buttonIndex].autofirePeriod < AUTOFIRE_PERIOD_MAX) {
                    ButtonSettings[buttonIndex].autofirePeriod++;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Autofire Period = ");
                UART2PutDecInt(ButtonSettings[buttonIndex].autofirePeriod);
                UART2PrintString("0 ms\n\r");
            }

            //Decrease Autofire Period
            if ((GetPressDuration()->dPadDown > FAST_AUTOCLICK_DELAY)&&\
                (ButtonSettings[buttonIndex].autofirePeriod > (AUTOFIRE_PERIOD_MIN + FAST_AUTOCLICK_SIZE))) {
                ButtonSettings[buttonIndex].autofirePeriod -= FAST_AUTOCLICK_SIZE;
            }
            if (GetNewPress()->dPadDown || (GetPressDuration()->dPadDown > AUTOCLICK_DELAY && !(cycleCounter % AUTOCLICK_TIMER))) {
                if (ButtonSettings[buttonIndex].autofirePeriod > AUTOFIRE_PERIOD_MIN) {
                    ButtonSettings[buttonIndex].autofirePeriod--;
                }
                else {
                    UpdateRumbleFeedback(2);
                }
                UART2PrintString("Autofire Period = ");
                UART2PutDecInt(ButtonSettings[buttonIndex].autofirePeriod);
                UART2PrintString("0 ms\n\r");
            }
        }
        UpdateButtonOutputs(configFlag, buttonIndex);
    }
    return TRUE;
}

/*******************************************************************************
Function: UpdateButtonOutputs(unsigned int configFlag, unsigned int configIndex)

Precondition:
    Must have run all the UpdateNewPress() and UpdatePS4ButtonDuration() routines.

Overview:
    This routine updates the outputs for button presses.

Input: configFlag: informs the routine if configuration mode is active or not.
 * 0: config mode not active
 * 1: config mode active
 * 2: config mode not active, but re-initialize the button toggle states
 *
 If config mode is active, outputs are activated by the right stick
 press button only to avoid conflicts with button functions.

Output: None.

 *******************************************************************************/

void UpdateButtonOutputs(unsigned int configFlag, unsigned int configIndex) {
    //static unsigned int cycleCounter; //increment each time we enter the function
    unsigned int i;
    unsigned int inputIndex;
    unsigned int buttonIndex;
    static BUTTONS buttonOutputs;
    static BUTTONS cycleCounter;
    static BUTTONS buttonOutBuffer;

    //Calculate output high or low
    for (buttonIndex = 0; buttonIndex < NUM_BUTTONS; buttonIndex++) {
        if (configFlag == 1) {
            buttonIndex = NUM_BUTTONS;
            i = configIndex; //only change the output of the button we're configuring
            inputIndex = 2; //use rStickPress for testing the outputs to avoid conflicts with configuration commands;
        }
        else {
            i = buttonIndex;
            inputIndex = buttonIndex;
        }
        if (configFlag == 2) {
            buttonToggles.array[i] = 0; //reset toggle states to power-on configuration.
        }

        switch (ButtonSettings[i].outputMode) {
            case PUSHBUTTON:
                buttonOutputs.array[i] = GetButtonPress()->array[inputIndex];
                break;

            case TOGGLE:
                if (GetNewPress()->array[inputIndex]) {
                    buttonToggles.array[i] = !buttonToggles.array[i];
                }
                buttonOutputs.array[i] = buttonToggles.array[i];
                break;

            case SINGLE_SHOT:
                if (GetNewPress()->array[inputIndex]) {
                    cycleCounter.array[i] = 1;
                    buttonOutputs.array[i] = 1;
                }
                if (cycleCounter.array[i] > 0 && cycleCounter.array[i] <= ButtonSettings[i].pulseWidth) {
                    buttonOutputs.array[i] = 1;
                    cycleCounter.array[i]++;
                }
                else {
                    cycleCounter.array[i] = 0;
                    buttonOutputs.array[i] = 0;
                }
                break;

            case AUTOFIRE:
                if (GetButtonPress()->array[inputIndex]) {
                    if (cycleCounter.array[i] % ButtonSettings[i].autofirePeriod < ButtonSettings[i].pulseWidth) {
                        buttonOutputs.array[i] = 1;
                    }
                    else {
                        buttonOutputs.array[i] = 0;
                    }
                    cycleCounter.array[i]++;
                }
                else {
                    buttonOutputs.array[i] = 0;
                    cycleCounter.array[i] = 0;
                }
                break;

            case TOGGLE_AUTOFIRE:
                if (GetNewPress()->array[inputIndex]) {
                    buttonToggles.array[i] = !buttonToggles.array[i];
                }
                if (buttonToggles.array[i]) {
                    if (cycleCounter.array[i] % ButtonSettings[i].autofirePeriod < ButtonSettings[i].pulseWidth) {
                        buttonOutputs.array[i] = 1;
                    }
                    else {
                        buttonOutputs.array[i] = 0;
                    }
                    cycleCounter.array[i]++;
                }
                else {
                    buttonOutputs.array[i] = 0;
                    cycleCounter.array[i] = 0;
                }
                break;

            default:
                break;
        }//end switch

        //We must explicitly assign a value to the output each time, or else the
        //following code will keep toggling the output if inversion is on.
        if (ButtonSettings[i].invert == TRUE) {
            buttonOutputs.array[i] = !buttonOutputs.array[i];
        }
    }//end for loop

    for (i=0; i<NUM_BUTTONS; i++)
    {
        buttonOutBuffer.array[i] = buttonOutputs.array[i];
    }
    //load SPI packet with values that would have been output if not overridden
    LoadOutputStatePacketButtons( &buttonOutBuffer );
    
    //Update the physical addresses associated with the output pins
    //make sure this matches the order in BUTTONS struct in PS4_controller.h

    if (GetSpiInputPacket()->overrideDPadUp) buttonOutBuffer.dPadUp = GetSpiInputPacket()->dPadUp;
    if (GetSpiInputPacket()->overrideDPadRight) buttonOutBuffer.dPadRight = GetSpiInputPacket()->dPadRight;
    if (GetSpiInputPacket()->overrideDPadDown) buttonOutBuffer.dPadDown = GetSpiInputPacket()->dPadDown;
    if (GetSpiInputPacket()->overrideDPadLeft) buttonOutBuffer.dPadLeft = GetSpiInputPacket()->dPadLeft;
    if (GetSpiInputPacket()->overrideTriangle) buttonOutBuffer.triangle = GetSpiInputPacket()->triangle;
    if (GetSpiInputPacket()->overrideCircle) buttonOutBuffer.circle = GetSpiInputPacket()->circle;
    if (GetSpiInputPacket()->overrideCross) buttonOutBuffer.cross = GetSpiInputPacket()->cross;
    if (GetSpiInputPacket()->overrideSquare) buttonOutBuffer.square = GetSpiInputPacket()->square;
    if (GetSpiInputPacket()->overrideLBumper) buttonOutBuffer.lBumper = GetSpiInputPacket()->lBumper;
    if (GetSpiInputPacket()->overrideRBumper) buttonOutBuffer.rBumper = GetSpiInputPacket()->rBumper;
    if (GetSpiInputPacket()->overrideLTriggerPull) buttonOutBuffer.lTriggerPull = GetSpiInputPacket()->lTriggerPull;
    if (GetSpiInputPacket()->overrideRTriggerPull) buttonOutBuffer.rTriggerPull = GetSpiInputPacket()->rTriggerPull;
    if (GetSpiInputPacket()->overrideLStickPress) buttonOutBuffer.lStickPress = GetSpiInputPacket()->lStickPress;
    if (GetSpiInputPacket()->overrideRStickPress) buttonOutBuffer.rStickPress = GetSpiInputPacket()->rStickPress;
    if (GetSpiInputPacket()->overrideShare) buttonOutBuffer.share = GetSpiInputPacket()->share;
    if (GetSpiInputPacket()->overrideOptions) buttonOutBuffer.options = GetSpiInputPacket()->options;
    if (GetSpiInputPacket()->overrideTpadClick) buttonOutBuffer.tpadClick = GetSpiInputPacket()->tpadClick;
    if (GetSpiInputPacket()->overridePsButton) buttonOutBuffer.psButton = GetSpiInputPacket()->psButton;
            
    //Must use IF statement and assign explicit values to LATx so compiler will use BSET/BCLR atomic operators
    //Otherwise, compiler wants to use OR/AND masks and you get read-modify-write problems if T3 interrupt interrupts it
    //Ternary operator doesn't work either!
    if (buttonOutBuffer.dPadUp) DPAD_UP = 1; else DPAD_UP = 0;
    if (buttonOutBuffer.dPadRight) DPAD_RIGHT = 1; else DPAD_RIGHT = 0;
    if (buttonOutBuffer.dPadDown) DPAD_DOWN  = 1; else DPAD_DOWN = 0;
    if (buttonOutBuffer.dPadLeft) DPAD_LEFT = 1; else DPAD_LEFT = 0;
    if (buttonOutBuffer.triangle) TRIANGLE = 1; else TRIANGLE = 0;
    if (buttonOutBuffer.circle) CIRCLE = 1; else CIRCLE = 0;
    if (buttonOutBuffer.cross) CROSS = 1; else CROSS = 0;
    if (buttonOutBuffer.square) SQUARE = 1; else SQUARE = 0;
    if (buttonOutBuffer.lBumper) LBUMPER = 1; else LBUMPER = 0;
    if (buttonOutBuffer.rBumper) RBUMPER = 1; else RBUMPER = 0;
    if (buttonOutBuffer.lTriggerPull) LTRIGGER_D = 1; else LTRIGGER_D = 0;
    if (buttonOutBuffer.rTriggerPull) RTRIGGER_D = 1; else RTRIGGER_D = 0;
    if (buttonOutBuffer.lStickPress) LSTICK_PRESS = 1; else LSTICK_PRESS = 0;
    if (buttonOutBuffer.rStickPress) RSTICK_PRESS = 1; else RSTICK_PRESS = 0;
    if (buttonOutBuffer.share) SHARE = 1; else SHARE = 0;
    if (buttonOutBuffer.options) OPTIONS = 1; else OPTIONS = 0;
    if (buttonOutBuffer.tpadClick) TPAD_CLICK = 1; else TPAD_CLICK = 0;
    if (buttonOutBuffer.psButton) PS_BUTTON = 1; else PS_BUTTON = 0;
}

void UpdateServoOutputs(void) {
    unsigned int i;
    signed int direction;
    signed long int range; //use long int to prevent overflow during math calculations
    signed long int range2;
    signed int sensorSignal;
    signed long int scaledSignal[NUM_SERVOS];
    signed long int scaledSignalOriginal[NUM_SERVOS]; //Make a copy so we don't overwrite the original when we mix signals
    signed long int output;
    signed int deadband;

//    static unsigned long int y1[NUM_SERVOS] = {0}; //variable for low pass filter
//    static unsigned long int x1[NUM_SERVOS] = {0}; //variable for low pass filter
//    unsigned long int y = 0; //variable for low pass filter
//    unsigned long int x = 0; //variable for low pass filter

    static BOOL stickHoldToggleL = FALSE;
    static BOOL stickHoldToggleR = FALSE;
    static unsigned int inputHoldValue[NUM_SERVOS];
    static unsigned int analogsBuffer[NUM_SERVOS];
    unsigned int OCxR_REG_PTRS_buffer[NUM_SERVOS];
    //unsigned int OCxCON1_REG_PTRS_buffer[NUM_SERVOS];

#define TPAD_INCREMENT_SENSITIVITY 70

    for (i = 0; i < NUM_SERVOS; i++) {
        analogsBuffer[i] = GetAnalogs()->array[i]; //GetAnalogs() returns a volatile array, make shadow copy just in case

        //process touchpad
        if (i == L_TPAD_X || i == L_TPAD_Y || i == R_TPAD_X || i == R_TPAD_Y){
            if (ServoSettings[i].touchpadSplit == WHOLE) {
                if (i == L_TPAD_X || i == R_TPAD_X){
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->wholePad.absoluteX;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->wholePad.incrementX*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->wholePad.displacementX;
                    }
                }
                else if (i == L_TPAD_Y || i == R_TPAD_Y) {
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->wholePad.absoluteY;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->wholePad.incrementY*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->wholePad.displacementY;
                    }
                }
            }
            else if (ServoSettings[i].touchpadSplit == SPLIT) {
                if (i == L_TPAD_X){
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->leftSide.absoluteX;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->leftSide.incrementX*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->leftSide.displacementX;
                    }
                }
                else if (i == L_TPAD_Y) {
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->leftSide.absoluteY;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->leftSide.incrementY*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->leftSide.displacementY;
                    }
                }
                else if (i == R_TPAD_X){
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->rightSide.absoluteX;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->rightSide.incrementX*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->rightSide.displacementX;
                    }
                }
                else if (i == R_TPAD_Y) {
                    if (ServoSettings[i].touchpadMode == TPAD ){
                        if (ServoSettings[i].absOrRel == ABSOLUTE){
                            analogsBuffer[i] = GetTouchpads()->rightSide.absoluteY;
                        }
                        else if (ServoSettings[i].absOrRel == RELATIVE) { //
                            analogsBuffer[i] = GetTouchpads()->rightSide.incrementY*TPAD_INCREMENT_SENSITIVITY;//normalize sensitivity
                        }
                    }
                    else if (ServoSettings[i].touchpadMode == JOYSTICK) { //touchpadMode == JOYSTICK
                        analogsBuffer[i] = GetTouchpads()->rightSide.displacementY;
                    }
                }
            }
        }      
    }

    if (GetNewPress()->lStickPress == TRUE) {
        stickHoldToggleL = !stickHoldToggleL; //toggle servo hold on/off
    }
    if (GetNewPress()->rStickPress == TRUE) {
        stickHoldToggleR = !stickHoldToggleR; //toggle servo hold on/off
    }

    if (mecanumSteering >= MECANUM_REMAP_4) //override output with R_STICK_Y signal if we're remapping
    {
        analogsBuffer[mecanumSteering + 2] = analogsBuffer[R_STICK_Y];
    }

    for (i = 0; i < NUM_SERVOS; i++) {
        if ((ServoSettings[i].holdRecall == L_STICK && GetButtonPress()->psButton && GetNewPress()->lStickPress) ||
                (ServoSettings[i].holdRecall == R_STICK && GetButtonPress()->psButton && GetNewPress()->rStickPress)) {
            ServoSettings[i].servoRecallValue = (unsigned int) (servoState[i] / 100); //store home position, divide by 100 to make unsigned long  fit unsigned int
            DataEEWrite(ServoSettings[i].servoRecallValue, EEPROM_SERVO_HOME + i);
        } //if the hold/recall function is on in relative mode, recall stored value to servostate
        else if ((ServoSettings[i].absOrRel == RELATIVE && ServoSettings[i].holdRecall == L_STICK && GetNewPress()->lStickPress) || //recall home position
                (ServoSettings[i].absOrRel == RELATIVE && ServoSettings[i].holdRecall == R_STICK && GetNewPress()->rStickPress)) {
            ServoSettings[i].servoRecallValue = DataEERead(EEPROM_SERVO_HOME + i);
            servoState[i] = ((signed long) (ServoSettings[i].servoRecallValue))*100; //recall home position, multiply by 100 cancel out divide when stored
        } //if the hold/recall function is on in absolute mode, overwrite the stick value with stored value
        else if ((ServoSettings[i].absOrRel == ABSOLUTE && ServoSettings[i].holdRecall == R_STICK && stickHoldToggleR == TRUE) ||
                (ServoSettings[i].absOrRel == ABSOLUTE && ServoSettings[i].holdRecall == L_STICK && stickHoldToggleL == TRUE)) {
            analogsBuffer[i] = inputHoldValue[i]; //overwrite stick value with hold value
        }
        else {
            inputHoldValue[i] = analogsBuffer[i]; //update hold value with stick value
        }
        //Channel-specific signal conditioning.
        switch (GetConnectedDeviceType())
        {
            case SPACE_MOUSE:
                switch (i) {
                    case L_TRIG_A:
                    case R_TRIG_A:
                        sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition - 128;
                        break;
                    case L_TPAD_X:
                        if (DPAD_LEFT && !DPAD_RIGHT)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_LEFT && DPAD_RIGHT) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case L_TPAD_Y:
                        if (DPAD_UP && !DPAD_DOWN)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_UP && DPAD_DOWN) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case R_TPAD_X:
                        if (SQUARE && !CIRCLE)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!SQUARE && CIRCLE) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case R_TPAD_Y:                      
                        if (TRIANGLE && !CROSS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!TRIANGLE && CROSS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                        
                    case TILT_X:
                        if (LBUMPER && !RBUMPER)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!LBUMPER && RBUMPER) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;

                    case TILT_Y:
                        if (LSTICK_PRESS && !RSTICK_PRESS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!LSTICK_PRESS && RSTICK_PRESS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    default:
                        sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition;
                        break;
                }
                break;
            
            case THRUSTMASTER:
                switch (i) {
                    case L_TRIG_A:
                        sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition - 128;
                        break;
                    case R_TRIG_A:
                        if (LTRIGGER_D && !RTRIGGER_D)  sensorSignal = 255 - ServoSettings[i].zeroPosition-128;
                        else if (!LTRIGGER_D && RTRIGGER_D) sensorSignal = 0 - ServoSettings[i].zeroPosition-128;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case L_TPAD_X:
                        if (DPAD_LEFT && !DPAD_RIGHT)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_LEFT && DPAD_RIGHT) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case L_TPAD_Y:
                        if (DPAD_UP && !DPAD_DOWN)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_UP && DPAD_DOWN) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case R_TPAD_X:
                        if (SQUARE && !CIRCLE)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!SQUARE && CIRCLE) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case R_TPAD_Y:                      
                        if (TRIANGLE && !CROSS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!TRIANGLE && CROSS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                        
                    case TILT_X:
                        if (LBUMPER && !LSTICK_PRESS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!LBUMPER && LSTICK_PRESS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;

                    case TILT_Y:
                        if (RBUMPER && !RSTICK_PRESS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!RBUMPER && RSTICK_PRESS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    default:
                        sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition;
                        break;
                }   
                break;
                
            default: //Dualshock 4
                switch (i) {
                    case L_TRIG_A:
                    case R_TRIG_A:
                        if (triggerLink == TRUE) {
                            sensorSignal = ((signed int) analogsBuffer[R_TRIG_A]-(signed int) analogsBuffer[L_TRIG_A]) / 2;
                        }
                        else {
                            sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition - 128;
                        }
                        break;
                    case L_TPAD_X:
                    case L_TPAD_Y:
                    case R_TPAD_X:
                    case R_TPAD_Y:
                        sensorSignal = (signed int) analogsBuffer[i]/2 - ServoSettings[i].zeroPosition;
                        break;
                    default:
                        sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[i].zeroPosition;
                        break;
                }
                       
                        //button override
                switch (ServoSettings[i].buttonRemap) {
                    case LEFT_RIGHT:
                        if (DPAD_LEFT && !DPAD_RIGHT)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_LEFT && DPAD_RIGHT) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case UP_DOWN:
                        if (DPAD_UP && !DPAD_DOWN)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!DPAD_UP && DPAD_DOWN) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case SQUARE_CIRCLE:
                        if (SQUARE && !CIRCLE)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!SQUARE && CIRCLE) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case TRIANGLE_CROSS:
                        if (TRIANGLE && !CROSS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!TRIANGLE && CROSS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case LR_BUMPERS:
                        if (LBUMPER && !RBUMPER)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!LBUMPER && RBUMPER) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case JOYSTICK_PUSHBUTTONS:
                        if (LSTICK_PRESS && !RSTICK_PRESS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!LSTICK_PRESS && RSTICK_PRESS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    case SHARE_OPTIONS:
                        if (SHARE && !OPTIONS)  sensorSignal = 127 - ServoSettings[i].zeroPosition;
                        else if (!SHARE && OPTIONS) sensorSignal = -128 - ServoSettings[i].zeroPosition;
                        else sensorSignal = ServoSettings[i].zeroPosition;
                        break;
                    default:
                        break;
                }
                break;
        }
        
        // special case for remapping r-stick y-axis when using mecanum mixing remapping
        if (mecanumSteering >= MECANUM_REMAP_4 && i == (mecanumSteering + 2)) {
            sensorSignal = (signed int) analogsBuffer[i] - ServoSettings[R_STICK_Y].zeroPosition; //use the original home position calibration.
        }

        //Apply deadband
        deadband = (signed int) ServoSettings[i].deadband;
        if (sensorSignal > deadband) {
            sensorSignal = sensorSignal - deadband;
        }
        else if (sensorSignal < -deadband) {
            sensorSignal = sensorSignal + deadband;
        }
        else {
            sensorSignal = 0;
        }

        scaledSignal[i] = (signed long) ServoSettings[i].sensitivity * (signed long) sensorSignal + ServoSettings[i].trim * 10; //beware of overflow
        scaledSignalOriginal[i] = scaledSignal[i]; //we'll overwrite some signals if we mix, so make a copy
    }

    //////////////////////////////////////////////////////////
    //Split the loop to finish applying scalings to all the signals first, so that we can mix the outputs later with the scaled signals.

    for (i = 0; i < NUM_SERVOS; i++) {
        //mix signals for Mecanum wheels
        if (mecanumSteering != MECANUM_OFF) {
            switch (i) {
                case L_STICK_X: //left front wheel
                    scaledSignal[L_STICK_X] = scaledSignalOriginal[L_STICK_Y] - scaledSignalOriginal[R_STICK_X] - scaledSignalOriginal[L_STICK_X];
                    break;
                case L_STICK_Y: //left rear wheel
                    scaledSignal[L_STICK_Y] = scaledSignalOriginal[L_STICK_Y] - scaledSignalOriginal[R_STICK_X] + scaledSignalOriginal[L_STICK_X];
                    break;
                case R_STICK_X: //right front wheel
                    scaledSignal[R_STICK_X] = scaledSignalOriginal[L_STICK_Y] + scaledSignalOriginal[R_STICK_X] + scaledSignalOriginal[L_STICK_X];
                    break;
                case R_STICK_Y: //right rear wheel
                    scaledSignal[R_STICK_Y] = scaledSignalOriginal[L_STICK_Y] + scaledSignalOriginal[R_STICK_X] - scaledSignalOriginal[L_STICK_X];
                    break;
                default:
                    break;
            }
        }
        else //if mecanum mixer is off
        {
            if (lStickDiffMix == TRUE) //check for diff steering
            {
                switch (i) {
                    case L_STICK_X:
                        scaledSignal[L_STICK_X] = scaledSignalOriginal[L_STICK_Y] - scaledSignalOriginal[L_STICK_X]; //left wheel channel
                        break;
                    case L_STICK_Y:
                        scaledSignal[L_STICK_Y] = scaledSignalOriginal[L_STICK_Y] + scaledSignalOriginal[L_STICK_X]; //right wheel channel
                        break;
                    default:
                        break;
                }
            }
            if (rStickDiffMix == TRUE) //check for diff steering
            {
                switch (i) {
                    case R_STICK_X:
                        scaledSignal[R_STICK_X] = scaledSignalOriginal[R_STICK_Y] - scaledSignalOriginal[R_STICK_X]; //left wheel channel
                        break;
                    case R_STICK_Y:
                        scaledSignal[R_STICK_Y] = scaledSignalOriginal[R_STICK_Y] + scaledSignalOriginal[R_STICK_X]; //right wheel channel
                        break;
                    default:
                        break;
                }
            }
        }

        //*******************CHECK BOUNDARIES AND WRITE OUTPUTS*****************************//
        //Check direction
        if (ServoSettings[i].direction == FORWARD) {
            direction = 1;
        }
        else {
            direction = -1;
        }

        //////////////SERVO OUTPUT////////////////////////////////////
        //Servo Position Mode
        if (ServoSettings[i].absOrRel == ABSOLUTE) {
            //check range of motion
            range = ServoSettings[i].range * 10;
            if (scaledSignal[i] > range) {
                scaledSignal[i] = range;
            }
            else if (scaledSignal[i] < -range) {
                scaledSignal[i] = -range;
            }
            //check stops
            output = 3000 + ServoSettings[i].offset * 10 + direction * scaledSignal[i]; //output, 0.5uS per count
            if (output > STOP_HIGH) {
                output = STOP_HIGH;
            }
            else if (output < STOP_LOW) {
                output = STOP_LOW;
            }

            //write output
            OCxR_REG_PTRS_buffer[i] = output;
        }
            /////////////////SERVO RELATIVE MODE/////////////////
        else if (ServoSettings[i].absOrRel == RELATIVE) {
            //scale to normalize sensitivity.  Multiply everything by 100, then divide by 100 counter discretization noise when scaling.
 
            servoState[i] = servoState[i] + direction * scaledSignal[i];             
    

            range = 300000 + 1000 * (signed long) (ServoSettings[i].offset + ServoSettings[i].range); //set stops, beware of overflow!
            range2 = 300000 + 1000 * (signed long) (ServoSettings[i].offset - ServoSettings[i].range); //set stops

            if (servoState[i] > range) {
                servoState[i] = range;
            }
            else if (servoState[i] < range2) {
                servoState[i] = range2;
            }
            //write output
            if (servoState[i] > (signed long) STOP_HIGH * 100) {
                servoState[i] = (signed long) STOP_HIGH * 100;
            }
            else if (servoState[i] < (signed long) STOP_LOW * 100) {
                servoState[i] = (signed long) STOP_LOW * 100;
            }
            OCxR_REG_PTRS_buffer[i] = servoState[i] / 100;
        }
    } //end NUM_SERVO for loop
    //Make sure the values don't make T3 overflow
    if (OCxR_REG_PTRS_buffer[10] > 10000) OCxR_REG_PTRS_buffer[10] = 10000;
    if (OCxR_REG_PTRS_buffer[11] > 10000) OCxR_REG_PTRS_buffer[11] = 10000;
    if (OCxR_REG_PTRS_buffer[12] > 10000) OCxR_REG_PTRS_buffer[12] = 10000;
    
    
    //Load packet with values that would have been written if not overridden
    LoadOutputStatePacketServos(OCxR_REG_PTRS_buffer);
    //SPI Override here.  Incoming value is the pulse width in microseconds, every count = 0.5 uS.

    if (GetSpiInputPacket()->overrideLStickX) OCxR_REG_PTRS_buffer[L_STICK_X] = GetSpiInputPacket()->lStickX*2;
    if (GetSpiInputPacket()->overrideLStickY) OCxR_REG_PTRS_buffer[L_STICK_Y] = GetSpiInputPacket()->lStickY*2;
    if (GetSpiInputPacket()->overrideRStickX) OCxR_REG_PTRS_buffer[R_STICK_X] = GetSpiInputPacket()->rStickX*2;
    if (GetSpiInputPacket()->overrideRStickY) OCxR_REG_PTRS_buffer[R_STICK_Y] = GetSpiInputPacket()->rStickY*2;
    if (GetSpiInputPacket()->overrideLTriggerAnalog) OCxR_REG_PTRS_buffer[L_TRIG_A] = GetSpiInputPacket()->lTriggerAnalog*2;
    if (GetSpiInputPacket()->overrideRTriggerAnalog) OCxR_REG_PTRS_buffer[R_TRIG_A] = GetSpiInputPacket()->rTriggerAnalog*2;
    if (GetSpiInputPacket()->overrideLTpadX) OCxR_REG_PTRS_buffer[L_TPAD_X] = GetSpiInputPacket()->lTpadX*2;
    if (GetSpiInputPacket()->overrideLTpadY) OCxR_REG_PTRS_buffer[L_TPAD_Y] = GetSpiInputPacket()->lTpadY*2;
    if (GetSpiInputPacket()->overrideRTpadX) OCxR_REG_PTRS_buffer[R_TPAD_X] = GetSpiInputPacket()->rTpadX*2;
    if (GetSpiInputPacket()->overrideRTpadY) OCxR_REG_PTRS_buffer[R_TPAD_Y] = GetSpiInputPacket()->rTpadY*2;
    if (GetSpiInputPacket()->overrideTiltX) OCxR_REG_PTRS_buffer[TILT_X] = GetSpiInputPacket()->tiltX*2;
    if (GetSpiInputPacket()->overrideTiltY) OCxR_REG_PTRS_buffer[TILT_Y] = GetSpiInputPacket()->tiltY*2;
     
    //low pass filter if needed
#define COEFF_A0 1
    for (i = 0; i < NUM_SERVOS; i++) {
        /*
        x = (signed long)OCxR_REG_PTRS_buffer[i]; 
        y = (x*COEFF_A0 + y1[i]*ServoSettings[i].filterStrength)/(COEFF_A0+ServoSettings[i].filterStrength);
        y1[i] = y;
        x1[i] = x;
         */
        *OCxR_REG_PTRS[i] = OCxR_REG_PTRS_buffer[i];
    }  
    
}

void __attribute__((__interrupt__, auto_psv))_T3Interrupt(void) {
    static unsigned char T3ServoIndex = 0;
    static unsigned int oc10rBuffer; //buffer since these registers can change
    static unsigned int oc11rBuffer;
    static unsigned int oc12rBuffer;
    static unsigned int T3PeriodResidual;
    static unsigned char PPMHigh = 1;
    
    if (PPMOutput == 0)
    {
        switch (T3ServoIndex){
            case 0:
                PWM10_OUT = 1;
                oc10rBuffer = oc10r-1; //manually tune to minimize error
                PR3 = oc10rBuffer;
                T3ServoIndex++;
                break;
            case 1:         
                PWM10_OUT = 0;
                PWM11_OUT = 1;
                oc11rBuffer = oc11r-1;
                PR3 = oc11rBuffer;
                T3ServoIndex++;
                break;
            case 2:
                PWM11_OUT = 0;
                PWM12_OUT = 1;
                oc12rBuffer = oc12r-1;
                PR3 = oc12rBuffer;
                T3ServoIndex++;       
                break;
            case 3:   
                PWM12_OUT = 0;
                T3PeriodResidual = 39996 - (oc10rBuffer + oc11rBuffer + oc12rBuffer); //manually tune for 20ms period
                PR3 = T3PeriodResidual;
                T3ServoIndex = 0;
                break;
            default:
                T3ServoIndex = 0;
                break;
        }
    }
    else
    {
        if (PPMHigh == 1)
        {
            
            PWM10_OUT = 1;
            PPMHigh = 0;
            if (T3ServoIndex < 12)
            {
                PR3 = *OCxR_REG_PTRS[T3ServoIndex]-602; //tune for timing 
                T3ServoIndex++;
            }
            else
            {
                PR3 = 10000;
                T3ServoIndex = 0;
                PWM11_OUT = 0; //scope trigger
            }
        }
        else //PPMHigh == 0
        {
            PWM10_OUT = 0;
            PR3 = 600;       
            PPMHigh = 1;
        }
    }
    _T3IF = 0; //clear interrupt flag   
}



void ResetOutputs(CONTROLLER_IN* ControllerIn) {

    UART2PrintString("***Reset Outputs.***\n\r");
    ControllerIn->triangle = 0;
    ControllerIn->circle = 0;
    ControllerIn->cross = 0;
    ControllerIn->square = 0;
    ControllerIn->dPad = 0b1000;
    ControllerIn->rStickPress = 0;
    ControllerIn->lStickPress = 0;
    ControllerIn->options = 0;
    ControllerIn->share = 0;
    ControllerIn->rTriggerPull = 0;
    ControllerIn->lTriggerPull = 0;
    ControllerIn->lBumper = 0;
    ControllerIn->rBumper = 0;
    ControllerIn->tpadClick = 0;
    ControllerIn->psButton = 0;

    ControllerIn->lStickX = ServoSettings[L_STICK_X].zeroPosition;
    ControllerIn->lStickY = ServoSettings[L_STICK_Y].zeroPosition;
    ControllerIn->rStickX = ServoSettings[R_STICK_X].zeroPosition;
    ControllerIn->rStickY = ServoSettings[R_STICK_Y].zeroPosition;
    ControllerIn->lTriggerAnalog = ServoSettings[L_TRIG_A].zeroPosition;
    ControllerIn->rTriggerAnalog = ServoSettings[R_TRIG_A].zeroPosition;
    ControllerIn->accelX = ServoSettings[TILT_X].zeroPosition;
    ControllerIn->accelZ = ServoSettings[TILT_Y].zeroPosition;
    ControllerIn->tpad[0].finger[0].noFinger = 1;
    ControllerIn->tpad[0].finger[1].noFinger = 1;


    CopyPS4Report(ControllerIn);
    UpdateCurrentStateBuffer((CONTROLLER_IN*)GetPS4Report()); //update controller input fields
    UpdateNewPress();
    if (disconnectToggleReset == 1) {
        UpdateButtonOutputs(2, 0); //2: config mode not active, but re-initialize the button toggle states
    }
    else {
        UpdateButtonOutputs(0, 0); //0: config mode not active
    }
    UpdateServoOutputs();
}

/*******************************************************************************
Function: GetSpiBusMode()

Precondition:
 None

Overview:
    This subroutine returns the SPI bus mode

Input:

Output:
0,1,2,3

 *******************************************************************************/
unsigned int GetSpiBusMode(void) {
    return spiBusMode;
}

unsigned int GetLEDBrightness(void)
{
    return LEDBrightness;
}

unsigned int GetLEDColorSetting(void)
{
    return LEDColorSetting;
}
/*******************************************************************************
Function: PrintIndexLabel(index)

Precondition:
 None

Overview:
    This subroutine prints the controller input associated with the channel 
number.

Input: Output
 * //////////////SERVOS///////////////
    0:L-Stick X-axis
    1:L-Stick Y-axis
    2:R-Stick X-axis
    3:R-Stick Y-axis
    4:L-Trig analog
    5:R-Trig analog
    6:Touchpad Left  X-axis
    7:Touchpad Left  Y-axis
    8:Touchpad Right X-axis
    9:Touchpad Right Y-axis
    10:X-Accelerometer
    11:Z-Accelerometer
 * //Buttons
    12:Share button 
    13:L-Stick button 
    14:R-Stick button 
    15:Options button 
    16:D-Pad Up 
    17:D-Pad Right 
    18:D-Pad Down 
    19:D-Pad Left 
    20:L-Bumper
    21:R-Bumper
    22:L-Trig button
    23:R-Trig button
    24:Triangle button
    25:Circle button
    26:Cross button
    27:Square button
    28:PS button
    29:Touchpad click

 *******************************************************************************/

void PrintIndexLabel(unsigned int index) {
    char* indexLabels[] = {\
    "L-Stick X-axis",\
    "L-Stick Y-axis",\
    "R-Stick X-axis",\
    "R-Stick Y-axis",\
    "L-Trig analog",\
    "R-Trig analog",\
    "Touchpad Left  X-axis",\
    "Touchpad Left  Y-axis",\
    "Touchpad Right X-axis",\
    "Touchpad Right Y-axis",\
    "X-Tilt",\
    "Y-Tilt",\
    "Share button",\
    "L-Stick button",\
    "R-Stick button",\
    "Options button",\
    "D-Pad Up",\
    "D-Pad Right",\
    "D-Pad Down",\
    "D-Pad Left",\
    "L-Trig button",\
    "R-Trig button",\
    "L-Bumper",\
    "R-Bumper",\
    "Triangle button",\
    "Circle button",\
    "Cross button",\
    "Square button",\
    "PS button",\
    "Touchpad button"
    };

    if (index > 29) {
        UART2PrintString("Error: Index out of range. \n\r");
    }
    else {
        UART2PrintString("Channel ");
        UART2PutDecInt(index);
        UART2PutChar(':');
        UART2PutChar(' ');
        UART2PrintString(indexLabels[index]); //Careful, don't try to call a label that doesn't exist!
        UART2PrintString("\n\r");
    }
}

void PrintMenu(void)
{

    UART2PrintString("\n\r===General Settings===\n\r");
    UART2PrintString("Enter config mode: Hold PS Button + Share 3 seconds\n\r");
    UART2PrintString("Save and exit: Hold Options\n\r");
    UART2PrintString("Load defaults: Hold Share 1 second (current channel)/3 seconds (all channels)\n\r");
    UART2PrintString("Cycle to next/previous channel : R/L Bumper\n\r");
    UART2PrintString("Cycle SPI bus modes: R-Stick + Triangle\n\r");
    UART2PrintString("Enable/Disable idle auto-disconnect: R-Stick + Cross\r\n");
    UART2PrintString("Preserve/Reset outputs on controller disconnect: R-Stick + PS Button\n\r");
    UART2PrintString("Change lightbar brightness: L-Stick + D-Pad Left/Right\n\r");
    UART2PrintString("Change lightbar color: L-Stick + Share\n\r");
    UART2PrintString("Toggle RC PPM Mode on ch11: L-Stick + Options\n\r");
    
    UART2PrintString("\n\r===Servo Adjustments===\n\r");
    UART2PrintString("Toggle absolute position/incremental mode: R-Stick + R-Bumper\n\r");
    UART2PrintString("Invert direction: R-Stick + L-Bumper\n\r");
    UART2PrintString("Sensitivity: Triangle/Cross\n\r");
    UART2PrintString("Deadband: Square/Circle\n\r");
    UART2PrintString("Range of motion: D-Pad Up/Down\n\r");
    UART2PrintString("Trim (range of motion limits move with trim): D-Pad Left/Right\n\r");
    UART2PrintString("Center offset trim (range of motion limits do not move): R-Stick + D-Pad Left/Right\n\r");
    UART2PrintString("Joystick zero calibration: Hold PS Button 1 second (current channel)/3 seconds (all channels)\n\r");
    UART2PrintString("Servo hold/recall position mode enable: L-Stick + Circle\n\r");
    UART2PrintString("Trigger link: L-Stick + Cross\n\r");
    UART2PrintString("Touchpad mode (absolute position vs virtual joystick): R-Stick + D-Pad Up\n\r");
    UART2PrintString("Touchpad split vs whole: R-Stick + D-Pad Down\n\r");
    UART2PrintString("Joystick differential drive mixer : L-Stick + Cross\n\r");
    UART2PrintString("Joystick mecanum wheel mixer: L-Stick + Square\n\r");
    UART2PrintString("Remap channel to button pair: R-Stick + Square\n\r");
    
    UART2PrintString("\n\r===Button Adjustments===\n\r");
    UART2PrintString("Test output while configuring: R-Stick\n\r");
    UART2PrintString("Invert Output: Cross\n\r");
    UART2PrintString("Change button mode (pushbutton, toggle, etc.): Triangle\n\r");
    UART2PrintString("Single-Shot/Autofire pulse width: D-Pad Left/Right\n\r");
    UART2PrintString("Adjust autofire period: D-Pad Up/Down\n\r\n\r");
   
}
