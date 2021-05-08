This is the source code for the ServoShock module.  For more information, please go to www.servoshock.com.

This project should be opened with Microchip's MPLAB X and compiled 
with the C30 compiler for 16-bit PIC microcontrollers, which are available
from Microchip's web site.  The compiled .hex file can be found in the 
PS3_Host.X\dist\default\production directory, and this hex file can be loaded
onto the ServoShock using the ds30 bootloader over the serial port.


This project does not include the bootloader.
The bootloader GUI and the microcontroller image can be downloaded from www.servoshock.com.  

 
 ******************************************************************************
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
 * Guillem Viñals Gangolells
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
******************************************************************************