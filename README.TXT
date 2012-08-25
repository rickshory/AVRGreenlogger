 This is the code to run an embedded system.

 The code is in C.

 The target microcontroller is the AVR ATmega1284P

 Status as of 2012-08-24:

 The system logs irradiance levels of infrared and visible light, to be used to calculate Normalized Difference Vegetation Index (NDVI), an index of vegetation greenness. Plants absorb visible light for photosynthesis, but reflect near-infrared because it is no use to them. The ratio of reflected infrared to visible (normalized) indicates the "greenness".

 The system is designed to be mounted about 0.5 meters above vegetation such as meadow or tundra. The system looks down on an area on the order of 1 square meter in size. The system logs the greenness through the growing season. The finished instruments are intended for use in remote locations where they may not be visited or attended for months. Such sites are often beyond cell phone range. This has been the source of several design decisions.

 The irradiance sensor is a digital integrated circuit (IC) produced for handheld devices, to detect ambient light levels, to automatically adjust screen brightness. The IC has the capability to detect both visible and infrared in order to distinguish different light sources, such as outdoor daylight from indoor fluorescent lighting. This is repurposed for NDVI. A digital IC works better than discreet photodiodes: Interfacing is easier (I2C bus). The sensor channels are specified as pre-calibrated. The data is noise-immune as soon as it leaves the IC. The IC costs less than a pair of photodiodes.

 The system logs data to a micro SD card using the FAT file system. The majority of the code is for the SD card file system routines. The SD card serves as a backup in case of failure, in which case you can remove the SD card and copy off the data. However in normal use, you retrieve data wirelessly by Bluetooth without opening the case.

 The system has a GPS module. This is more important for time data than for position data. A GPS signal contains accurate time information, and this allows the system to recover from reset or other failure, and establish a valid time base from anywhere in the world, unattended.

 The system runs on a 1.2 volt AA rechargeable NiMH cell, stepped up to standard 3.3 volts through boost converters. One boost module runs continually to supply general system power, and two others are switched on intermittently to power Bluetooth and a GPS unit. The system has a set of solar cells which keep the power cell charged. Present prototypes, tested so far during summer, appear to harvest enough solar energy to run indefinitely.

 The system saves power by going into sleep mode during darkness, when there is no irradiance to log. In this mode, all functions are shut down except the internal clock, and one line that can input a wake-up signal. The system also goes into micropower mode between readings.

 The system has an accelerometer IC. This is used both for leveling, and for wake-up. For wake-up, the accelerometer detects taps, and wakes the microcontroller. There must be two taps, both above an acceleration threshold, to fully rouse the system. This avoids accidental waking.

 Communication is by RS-232 through two different serial ports on the microcontroller. One port goes to the GPS and the other is for external data exchange. Both of these can be tapped into by hardware ports. In normal operation, however, the GPS is only on intermittently for time checking, and external data goes by Bluetooth.

 When roused by accelerometer taps, the system attempts for two minutes to establish a Bluetooth connection. If successful, the system stays in roused mode and a diagnostic data stream is routed through the Bluetooth module. Using a terminal program on the other end of the Bluetooth link, the user can control the system by various commands, such as for setting the time of the internal clock.

 The system contains a temperature sensor. Under summer conditions, temperatures inside the sealed transparent case have reached 65 degrees Centigrade, but this has evidently not impaired function. Temperature readings allow compensation of other components, such as the irradiance sensor and real time clock (RTC).

 The system contains a magnetic reed switch wired for reset, so the system can be reset without opening the case by holding a strong magnet in the correct position. Although recovery from reset is inherently undefined, the system often retains its time setting because the RTC is a separate chip from the microcontroller.

 Still to do:
 - Implement watchdog timer
 - Increase Bluetooth data speed from 9600 to 115,200 baud
 - Make data transfer more foolproof and automatic.
 - Implement a bootloader, for field upgrade
 - Improve leveling
 - Fully implement GPS time acquisition
 - Implement user=settable darkness threshold
