/*
 ********************************************************************************
                                                                                
Software License Agreement                                                      
                                                                                
Copyright ? 2007-2008 Microchip Technology Inc.  All rights reserved.           
                                                                                
Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital signal controller  
that is integrated into your product or third party product (pursuant to the    
sublicense terms in the accompanying license agreement).                        
                                                                                
You should refer to the license agreement accompanying this Software for        
additional information regarding your rights and obligations.                   
                                                                                
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,   
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF        
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.  
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER       
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR    
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES         
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR     
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF        
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES          
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.     
                                                                                
 ********************************************************************************
 */

// Created by the Microchip USBConfig Utility, Version 2.7.1.0, 7/27/2011, 20:42:36

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "USB\usb.h"
#include "USB\usb_host_generic_PS4.h"

// *****************************************************************************
// Client Driver Function Pointer Table for the USB Embedded Host foundation
// *****************************************************************************

CLIENT_DRIVER_TABLE usbClientDrvTable[] ={
    {
        USBHostGenericInitBT,
        USBHostGenericEventHandler,
        0
    }
    ,
    {
        USBHostGenericInitWire,
        USBHostGenericEventHandler,
        0
    }
    ,
    {
        USBHostInitSpaceMouse,
        USBHostGenericEventHandler,
        0
    }
    ,
    {
        USBHostInitThrustmaster,
        USBHostGenericEventHandler,
        0
    }
};

// *****************************************************************************
// USB Embedded Host Targeted Peripheral List (TPL)
// *****************************************************************************

USB_TPL usbTPL[] ={
    { INIT_CL_SC_P(0xE0ul, 0x01ul, 0x01ul), 0, 0, {
            TPL_CLASS_DRV}} // (Class: Wireless Controller/Subclass: RF Controller/Protocol: Bluetooth)
    ,
    { INIT_VID_PID(0x046Dul, 0xC626ul), 0, 2, {
            0}} // Wired Space Navigator
    ,
    { INIT_VID_PID(0x046Dul, 0xC627ul), 0, 2, {
            0}} // Space Explorer
    ,
    { INIT_VID_PID(0x256Ful, 0xC62Eul), 0, 2, {
            0}} // SpaceMouse Wireless (cable)
    ,
    { INIT_VID_PID(0x256Ful, 0xC62Ful), 0, 2, {
            0}} // SpaceMouse Wireless (Dongle)
    ,
    { INIT_VID_PID(0x44Ful, 0xB67Bul), 0, 3, {
            0}} // Thrustmaster HOTAS PS4
    ,
    { INIT_VID_PID(0xFFFFul, 0xFFFFul), 0, 1, {
            0}} // Dualshock4 Controller




};

