This is the readme file for the Kelvin NTC10KA project. This project is also the base for a VSCP 
implemention on a Microchip 18F PIC device. This is a commercial module that can bought from 
Grodans Paradis ( see http://www.auto.grodansparadis.com/kelvinntc10k/kelvin_ntc10ka.html)

The procesor used is the Microchop PIC18F2580 or PIC18F25K80

MPLAB X is used to build the project. Some files

eprom.h - VSCP root/firmware/pic/common)
eprom.c - (VSCP root/firmware/pic/common)
vscp_firmware.h - (VSCP root/firmware/common)
vscp_firmware.c - (VSCP root/firmware/common)
vscp_class.h - (VSCP root/src/vscp/common)
vscp_type.h - (VSCP root/src/vscp/common)

Where "VSCP Root" is the location where you installed the VSCP source tree.  Go to projectpage for the 
VSCP project (http://www,vscp,org) and download the sorcetree to get acces to the file. 


"_reloc" projects are built for use with the bootloader. The bootloader occupies space between 0x000 and 0x1ff. 
Vectors are relocated to 0x200. This is managed in the linker scripts. Also the C initialization code has 
been changed to start code at the relocated vector. This also means that the bootloader has to be programmed
into the chip before the working with the project. It also means that the environment must be configured 
*not* to erase the chip before programming it.

If no bootloader is required it is OK to use the standard C startup code, a standard linker description. In this 
case the absolute vector address for the low priority interrup in main.c  must be changed back froo 0x218 to 0x18.



Steps you should go through to adopt this file for your own VSCP project.
=========================================================================

1.) The frequency for your system may be different. This will affect the CAN bit rate settings
which are defined in the biginning of the can18f.c file. Here you also find the filter/mask 
defines if you need to change these.

Timer0 is used as a 1 ms timer for the VSCP functionality. You should adopt the timer reload value 
in the main.h file to your system.

2.) The GUID (Glocal Unique ID) is defined in the beginning of vscp.c This number should be 
different for each piece of equiment you produce. Regard it is as the device serial number. It
can be moved to EEPROM if that is found more convinient. In that case the vscp_rcv_readreg 
procedure in vscp.c also has to be changed.

3.) The device URL is defined in the beginning of the vscp.c file. This is an URL without the
initial "http://" that should point to a XML file that give information about your piece of 
equipment.

4.) The manufacturer id is also defined in the beginning of the vscp.c file. You can use this
id in any way you like.

In addition to this you may use different pins for the INIT button (RC0 today) and the 
status LED (RC1) today. Changes affect the main interrupt vector.


Ake Hedman
Grodans Paradis AB
info@grodansparadis.com
http://www.grodansparadis.com

VSCP (Very Simple Control Protocol) 
http://www.vscp.org


