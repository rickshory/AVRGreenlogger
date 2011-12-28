 This is the code to run an embedded system.

 The code is in C.

 The target microcontroller is the AVR ATmega1284P

2011-12-27. First prototype. Increments a counter about once per second and outputs the number through the UART.

 Below is anticipated development:

 The system logs irradiance levels of infrared and visible light, to be used to calculate Normalized Difference Vegetation Index (NDVI), an index of vegetation greenness. Plants absorb visible light for photosynthesis, but reflect near-infrared because it is no use to them. The ratio of reflected infrared to visible (normalized) indicates the "greenness".

 The system is designed to be mounted about 0.5 meters above vegetation such as meadow or tundra. The system looks down on an area on the order of 1 square meter in size. The system logs the greenness through the growing season.

 The irradiance sensor is a digital integrated circuit (IC) produced for handheld devices, to detect ambient light levels, to automatically adjust screen brightness. The IC has the capability to detect both visible and infrared in order to distinguish different light sources, such as outdoor daylight from indoor fluorescent lighting. This is repurposed for NDVI. A digital IC works better than discreet photodiodes: Interfacing is easier (I2C bus). The data is noise-immune as soon as it leaves the IC. The IC costs less than a pair of photodiodes.

 The system logs data to a micro SD card using the FAT file system. The majority of the code is for the SD card file system routines. Presently, to retrieve data, you manually remove the SD card and copy off the data. Future development will be to retrieve data wirelessly, without opening the case.

 The system runs on a 1.2 volt AA rechargeable NiMH cell, stepped up to standard 3.3 volts through a boost converter. The system has a solar cell (presently, nominal 3 V and 50 mA; various options are in prototype), which keeps the cell charged. Present prototypes appear to harvest enough solar energy to run indefinitely.

 The system saves power by going into sleep mode during darkness, when there is no irradiance to log. In this mode, all functions are shut down except the internal clock, and one line that can input a wake-up signal. The system also goes into micropower mode between readings.

 The system has an accelerometer IC. This is used both for leveling, and for wake-up. For wake-up, the accelerometer detects taps, and wakes the microcontroller.

 Communication is presently by RS-232 serial. When woken by an accelerometer tap, the system outputs diagnostics for 30 seconds via Port 1, then shuts down communication again. During these 30 seconds, Port 1 is available to receive various commands, such as for setting the time of the internal clock. On receiving any serial input through Port 1, the system re-starts the 30 second timer, so it is possible to keep the system awake by periodically sending it a dummy character such as "A". If Port 1 input is idle for 30 seconds, the system shuts down Port 1 again till the next wake-up tap.

 Future development will be:
 - Route Port 1 through Bluetooth, to control the system without opening the sealed weatherproof case.
 - Add commands to export stored data out Port 1.
 - Implement Port 2 and connect it to a GPS module, so the system can automatically determine its location and the time.
