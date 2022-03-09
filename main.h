/*
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
 */


//***My Defines
#ifndef __MAIN_H
#define __MAIN_H

#define DATA_PACKET_LENGTH  87

#define LED_SEL_0 _RB12
//#define LED_SEL_1 _RB13 //reassign to PS button output
#define _PGC _RB6
#define _PGD _RB7

#define FIRMWARE_REV "3.10"
#define VERSION_CODE 310
//Flag to load defaults on first startup
#define DEFAULTS_INITIALIZED_FLAG 0

//EEPROM Addresses (1-3): Bluetooth addresses
#define EEPROM_BT_ADDR_0 1
#define EEPROM_BT_ADDR_1 2
#define EEPROM_BT_ADDR_2 3


#define EEPROM_SERVO_HOME 4 //Start of memory location for storing servo home positions. length = NUM_SERVOS

#define EEPROM_CONFIG_SETTINGS_START (EEPROM_SERVO_HOME+NUM_SERVOS) //Start of  memory location for non-volatile configuration settings.



//simulated EEPROM addresses must be less than 510
#endif
