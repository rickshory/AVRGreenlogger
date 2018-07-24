AVRGreenlogger
======

 #### Greenlogger, implemented using AVR microcontroller.
 
 #### Works on
* AVR ATmega1284P

 This is the code to run an embedded system.

 The code is in C.

 The target microcontroller is the AVR ATmega1284P

 #### Status as of 2017-08-24:

 The system logs irradiance levels of infrared and visible light, to be used to calculate Normalized Difference Vegetation Index (NDVI), an index of vegetation greenness. Plants absorb visible light for photosynthesis, but reflect near-infrared because it is no use to them. The ratio of reflected infrared to visible (normalized) indicates the "greenness".

 The system is designed to be mounted about 0.5 meters above vegetation such as meadow or tundra. The system looks down on an area on the order of 1 square meter in size. The system logs the greenness through the growing season. The finished instruments are intended for use in remote locations where they may not be visited or attended for months, or even years. Such sites are often beyond cell phone range. This has been the source of several design decisions. These include making each instrument independent, rather than attempting to network them, and implementing extreme power saving.

 The irradiance sensor is a digital integrated circuit (IC) produced for handheld devices, to detect ambient light levels, to automatically adjust screen brightness. The IC has the capability to detect both visible and infrared in order to distinguish different light sources, such as outdoor daylight from indoor fluorescent lighting. This is repurposed for NDVI. A digital IC works better than discreet photodiodes: Interfacing is easier (I2C bus). The IC manufacturer specifies that the sensor channels are pre-calibrated, unlike raw photodiodes that require careful biasing and calibration. The data is noise-immune as soon as it leaves the IC. The IC costs less than a pair of photodiodes.

 The system logs data to a micro SD card using the FAT file system. The majority of the code is for the SD card file system routines. The SD card serves as a backup in case of failure, in which case you can remove the SD card and copy off the data. However in normal use, you retrieve data wirelessly by Bluetooth without opening the case.

 The system runs on a 1.2 volt AA rechargeable NiMH cell, stepped up to standard 3.3 volts through boost converters. One boost module runs continually to supply general system power, and two others are switched on intermittently to power Bluetooth and a GPS unit. The system has a set of solar cells which keep the power cell charged. Present prototypes, tested through winters at latitudes greater than 45 degrees north, appear to harvest enough solar energy to run indefinitely. Tests are in process through winters in Greenland and Alaska, to test survival through polar night.

 The system saves power by going into sleep mode during darkness, when there is no irradiance to log. In this mode, all functions are shut down except the internal clock, and one line that can input a wake-up signal. The system also goes into micropower mode between readings.

 The system has an accelerometer IC. This is used both for leveling, and for wake-up. For wake-up, the accelerometer detects taps, and wakes the microcontroller. There must be two taps, both above an acceleration threshold, to fully rouse the system. This avoids accidental waking.

 Communication is by RS-232 through two different serial ports on the microcontroller. One port goes to the GPS and the other is for external data exchange. Both of these can be tapped into by hardware ports. In normal operation, however, the GPS is only on intermittently for time checking, and external data goes by Bluetooth.

 When roused by accelerometer taps, the system attempts for two minutes to establish a Bluetooth connection. If successful, the system stays in roused mode and a diagnostic data stream is routed through the Bluetooth module. Using a terminal program on the other end of the Bluetooth link, the user can control the system by various commands, such as for setting the time of the internal clock.

 The system contains a temperature sensor. Under summer conditions, temperatures inside the sealed transparent case have reached 65 degrees Centigrade, but this has evidently not impaired function. Temperature readings allow compensation of other components, such as the irradiance sensor and real time clock (RTC).

 The system contains a magnetic reed switch wired for reset, so the system can be reset without opening the case by holding a strong magnet in the correct position. Although recovery from reset is inherently undefined, the system often retains its time setting because the RTC is a separate chip from the microcontroller.
 
 The system has a GPS unit. This is more important for time than for position. A GPS signal contains accurate timestamp information, and this allows the system to recover from reset or other failure, and establish a valid time base from anywhere in the world, unattended.
 
 The GPS unit is on a separate circuit board, run by a separate microcontroller. The code for that module is the repository **GPS_time_841**. This main module sends a "time request" pulse to the GPS board. The microcontroller there powers up the GPS unit, waits for a valid fix, parses out the relevant information, and sends it back to this main module as a set-time signal. The main module has heuristics to re-try time requests on startup and after long storage.
 
 This instrument generates a large amount of data. A system for managing the data is the repository **NDVI-modules**. This is a cross-platform GUI system written using WX Python.

 Still to do:
 - Implement dead battery recovery. Presently, if the AA cell drains too low, the system dies. When the cell is re-charging from the solar cells, and gets just to the system start threshold, the startup drops the AA cell voltage, and causes a reset. Then the AA cell charges again just to the start threshold, another reset occurs, and so the system is caught in an infinite loop. Need to hold off system start until AA cell is sufficiently charged to carry startup through to low-power modes.
 - Replace magnetic reed switch with solid state magnetic sensor, to avoid reset by mechanical shock of instrument being dropped.
 - Make battery holder solder joints more robust, to avoid breakage from instrument being dropped.
 - Implement watchdog timer
 - Transfer data by wireless file server, rather than Bluetooth serial, if power can be kept low enough.
 - Implement a bootloader, for field upgrade
 - Implement user-settable darkness threshold
 
## Contact
#### Developer/Company
* Project of: http://www.rickshory.com/
* e-mail: rickshory@gmail.com
