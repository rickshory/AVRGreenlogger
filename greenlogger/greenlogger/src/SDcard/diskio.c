//
// MMCv3/SDv1/SDv2 (in SPI mode) control module  (C)ChaN, 2007
//
// platform and app dependent:
//  rcvr_spi()
//  xmit_spi()
//  heartBeat()
//  get_fattime()
//   some macros 

#include <avr/io.h>
#include <inttypes.h>
#include "../greenlogger.h"
#include "ff.h"
#include "diskio.h"
#include "RTC/DS1342.h"
#include "BattMonitor/ADconvert.h"

// 
/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static volatile
DSTATUS Stat = STA_NOINIT;	// Disk status

extern volatile uint8_t stateFlags1;
extern volatile uint8_t stateFlags2;

extern char str[128]; // generic space for strings to be output
extern char strJSON[256]; // string for JSON data
extern char strHdr[64];

extern volatile dateTime dt_CurAlarm;
extern volatile int8_t timeZoneOffset;

extern volatile
BYTE Timer1, Timer2;	// 100Hz decrement timer

extern volatile adcData cellVoltageReading;

static
BYTE CardType;			// Card type flags


/**
 * \brief Writes N characters to the SD card
 *
 *  This function writes N characters from the passed char
 * pointer to the file specified by the year, month, and
 * day of the Real Time Clock.
 *  The containing folder is named 'yy-mm' where 'yy' is
 * the last two digits of the year (for example '12' in
 * the year '2012') and 'mm' is the month (for example '07'
 * for July).
 *  The file is named 'dd.TXT', where 'dd' is the day of 
 * the month, for example '09' for the ninth.
 *  This function automatically creates the folders and files
 * as needed, and appends to files that already exist
 *
 * \note 
 * 
 */

BYTE writeCharsToSDCard (char* St, BYTE n) {
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	char stDir[6], stFile[20];
	BYTE sLen, retVal = sdOK;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely write card
	}

	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
		//  flag error
//		len = sprintf(str, "\n\r f_mount failed: 0x%x\n\r", 0);
//		outputStringToUART0(str);
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
//	flag error.
//	len = sprintf(str, "\n\r disk_initialize failed; driveStatus: 0x%x\n\r", driveStatus);
//	outputStringToUART0(str);
	}
	sLen = sprintf(stDir, "%02d-%02d", dt_CurAlarm.year, dt_CurAlarm.month);
	res = f_mkdir(stDir);
	if (!((res == FR_OK) || (res == FR_EXIST))) {
		retVal = sdMkDirFail;
		goto unmountVolume;
//		len = sprintf(str, "\n\r f_mkdir failed: 0x%x\n\r", 0);
//		outputStringToUART0(str);	
	}
	
	FIL logFile;
	//
	//works
	sLen = sprintf(stFile, "%02d-%02d/%02d.txt", dt_CurAlarm.year, dt_CurAlarm.month, dt_CurAlarm.day);
	if(f_open(&logFile, stFile, FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
//		len = sprintf(str, "\n\r f_open failed: 0x%x\n\r", 0);
//		outputStringToUART0(str);
	//flag error
}

	// Move to end of the file to append data
	if (f_lseek(&logFile, f_size(&logFile)) != FR_OK) {
		retVal = sdFileSeekFail;
		goto closeFile;
	}

	unsigned int bytesWritten, tmpLen;
	// if flagged, insert any JSON messages
	if (stateFlags1 & (1<<writeJSONMsg)){
		tmpLen = strlen(strJSON);
		if (f_write(&logFile, strJSON, tmpLen, &bytesWritten) != FR_OK) {
			retVal = sdFileWriteFail;
			goto closeFile;
		}
		stateFlags1 &= ~(1<<writeJSONMsg); // clear flag, write only once
		strJSON[0] = '\0'; // "erase" the string
		if (bytesWritten < n) { // probably strJSON is corrupted; proceed next time with string and flag cleared
			// at least allow normal logging to resume
			retVal = sdFileWritePartial;
			goto closeFile;
		}			
	}
	// if flagged, insert column headers
	if (stateFlags1 & (1<<writeDataHeaders)){
//		tmpLen = sprintf(str, "\n\rTimestamp\tBBDn\tIRDn\tBBUp\tIRUp\tT(C)\tVbatt(mV)\n\r");
		tmpLen = sprintf(str, strHdr);
		stateFlags1 &= ~(1<<writeDataHeaders); // clear flag, attempt to write only once
		if (f_write(&logFile, str, tmpLen, &bytesWritten) != FR_OK)
			retVal = sdFileWriteFail;
			goto closeFile;
		if (bytesWritten < n) {
			retVal = sdFileWritePartial;
			goto closeFile;
		}
			
	}
	
	if (f_write(&logFile, St, n, &bytesWritten) != FR_OK) {
		retVal = sdFileWriteFail;
		goto closeFile;
	}
	
	if (bytesWritten < n) {
		retVal = sdFileWritePartial;
		goto closeFile;
	}
	
//		len = sprintf(str, "\n\r test file written: 0x%x\n\r", 0);
//		outputStringToUART0(str);
	//Flush the write buffer with f_sync(&logFile);

// retVal = sdOK;

	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}

/**
 * \brief Writes a string to a file on the SD card
 *
 * This function writes a single string to the named
 * file on the SD card. It overwrites the file each
 * time, so it is primarily for storing configuration
 * parameters. This fn is the core of other functions
 * that store specific parameters.
 *
 * The return value is the SD card file system error code.
 * \note 
 * 
 */

BYTE writeStringInFileToSDCard (char* stParam, char* stFile) {
	
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	BYTE sLen, retVal = sdOK;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely write card
	}

	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
	}
	
	FIL logFile;
	if(f_open(&logFile, stFile, FA_WRITE | FA_CREATE_ALWAYS)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
	}

	int bytesWritten;
	bytesWritten =  f_puts(stParam, &logFile);
	
	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}

/**
 * \brief Reads a string from a file on the SD card
 *
 * This function reads as a single string the contents of
 * the named file on the SD card. It is primarily for
 * retrieving configuration parameters. This fn is the
 * core of other functions that fetch specific parameters.
 *
 * The return value is the SD card file system error code.
 * \note 
 * 
 */
BYTE readStringFromFileFromSDCard (char* stParam, char* stFile) {
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	BYTE sLen, retVal = sdOK;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely read card
	}

	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
	}
	FIL logFile;
	if(f_open(&logFile, stFile, FA_READ | FA_OPEN_EXISTING)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
}	
	f_gets(stParam, sizeof stParam, &logFile);
	
	if (f_error(&logFile)) {
		retVal = sdFileReadFail;
		goto closeFile;
	}
		
	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}


/**
 * \brief Writes the global Timezone Offset to the SD card
 *
 * This function writes the globally maintained variable
 * 'timeZoneOffset' to the SD card as a string in a file.
 *  The string is e.g. "-07" where the first character is
 * the sign and the next two are the hours + or - UT, always
 * padded with zero.
 * The file is named "TIMEZONE.TXT" and is in the root
 * folder of the SD card.
 * Using the SD card as persistent storage this way allows
 * recovery of the time zone offset after a power-down
 * reset.
 * Run this function when 'timeZoneOffset' is changed.
 * This function completely re writes the file
 * The return value is the error code.

*
 * \note 
 * 
 */

BYTE writeTimezoneToSDCard (void) {
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	char stTZ[5];
	BYTE sLen, retVal = sdOK;
	uint8_t iTmp;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely write card
	}

	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
	}
	
	FIL logFile;
	if(f_open(&logFile, "TIMEZONE.TXT", FA_WRITE | FA_CREATE_ALWAYS)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
}

	int bytesWritten;
	if (timeZoneOffset < 0) {
		stTZ[0] = '-';
		iTmp = -timeZoneOffset;
	} else {
		stTZ[0] = '+';
		iTmp = timeZoneOffset;
	}
	stTZ[1] = '0' + (iTmp / 10);
	stTZ[2] = '0' + (iTmp % 10);
	stTZ[3] = 0;

//	outputStringToBothUARTs(" Timezone string: ");
//	outputStringToBothUARTs(stTZ);
//	outputStringToBothUARTs("\n\r\n\r");
	
//	bytesWritten = f_printf(&logFile, "%d\n", timeZoneOffset);
	bytesWritten =  f_puts(stTZ, &logFile);
	
//	sLen = sprintf(str, "\n\r timezone characters written to file : %d\n\r", bytesWritten);
//	outputStringToBothUARTs(str);

//Flush the write buffer with f_sync(&logFile);

// retVal = sdOK;

	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}

/**
 * \brief Reads the global Timezone Offset from the SD card
 *
 * This function reads the globally maintained variable
 * 'timeZoneOffset' from the SD card.
 * The function expects a file named "TIMEZONE.TXT"
 * in the root folder of the SD card.
 *  The file contains a single string e.g. "-07" where the
 * first character is the sign and the next two are the
 * hours + or - UT, always padded with zero.
 * Run this function to recover the time zone offset
 * when the time is read from the RTC chip after a 
 * uC reset.
 * The return value is the error code. 
*
 * \note 
 * 
 */

BYTE readTimezoneFromSDCard (void) {
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	char stTZ[10];
	BYTE sLen, retVal = sdOK;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely read card
	}

	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
	}
	FIL logFile;
	if(f_open(&logFile, "TIMEZONE.TXT", FA_READ | FA_OPEN_EXISTING)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
}	
	f_gets(stTZ, 4, &logFile);
	
	if (f_error(&logFile)) {
		retVal = sdFileReadFail;
		goto closeFile;
	}
	
	outputStringToUART0("\n\rTimezone read from file: ");
	outputStringToUART0(stTZ);
	outputStringToUART0("\n\r\n\r");

	timeZoneOffset = atoi(stTZ);
	
	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}


/**
 * \brief Outputs file contents to USART
 *
 * This function reads the strings in a file and outputs
 * them to the USARTs.
 * The function looks for a file on the SD card
 * corresponding to the passed string, which must be
 * a date in strict format, e.g. "2012-05-03". The file in
 * this case would be "12-05\03.TXT".
* The return value is the error code, one of which is 
 * "file not found". 
*
 * \note 
 * 
 */

BYTE outputContentsOfFileForDate (char* stDt) {
	FATFS FileSystemObject;
	FRESULT res;         // FatFs function common result code
	char stFile[20], stLine[128];
	BYTE sLen, retVal = sdOK;
	
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_THRESHOLD_SD_CARD) {
		return sdPowerTooLowForSDCard; // cell voltage is below threshold to safely read card
	}
	
	if (!isValidDate(stDt)) {
//		outputStringToBothUARTs("Invalid date: ");
//		outputStringToBothUARTs(stDt);
//		outputStringToBothUARTs("\n\r\n\r");
		return sdInvalidDate; // not a valid calendar date, e.g. 2011-02-30; or date is malformed
	}
	
	if(f_mount(0, &FileSystemObject)!=FR_OK) {
		return sdMountFail;
	}

	DSTATUS driveStatus = disk_initialize(0);

	if(driveStatus & STA_NOINIT ||
		driveStatus & STA_NODISK ||
		driveStatus & STA_PROTECT) {
			retVal = sdInitFail;
			goto unmountVolume;
	}
	
	strncpy(stFile, stDt + 2, 5); // slice out e.g. "12-05" from "2012-05-03"
	strcat(stFile, "/"); // append the folder delimiter
	strncat(stFile, stDt + 8, 2); // append e.g. "03" from "2012-05-03"
	strcat(stFile, ".TXT"); // complete the filename
	
	FIL logFile;
	
	if(f_open(&logFile, stFile, FA_READ | FA_OPEN_EXISTING)!=FR_OK) {
		retVal = sdFileOpenFail;
		goto unmountVolume;
}	
	while (f_gets(stLine, (sizeof stLine) - 1, &logFile) != NULL) {
		if (f_error(&logFile)) {
			retVal = sdFileReadFail;
			goto closeFile;
		}
		outputStringToBothUARTs(stLine);
	}
	
	//Close and unmount.
	closeFile:
	f_close(&logFile);
	unmountVolume:
	f_mount(0,0);
	return retVal;
}


/**
 * \brief Reports errors writing to the SD card
 *
 *  This function sends a message out both UARTS
 * (if power is on) if there were any error
 * accessing files on the SD card.
 *
 * \note 
 * 
 */

void tellFileError (BYTE err)
{
 switch (err) {
  //case IgnoreCard: {
   //outputStringToUART0("\r\n SD card ignored (\"O\" toggles)\r\n");
   //break;
  //}
  case sdPowerTooLowForSDCard: {
   outputStringToBothUARTs("\r\n power too low, SD write skipped\r\n");
   break;
  }
  //case NoCard: {
   //outputStringToBothUARTs("\r\n SD card not present or not detected\r\n");
   //break;
  //}
  case sdMountFail: {
   outputStringToBothUARTs("\r\n could not mount SD card\r\n");
   break;
  }
 case sdInitFail: {
   outputStringToBothUARTs("\r\n could not initialize SD card\r\n");
   break;
  }
  case sdMkDirFail: {
   outputStringToBothUARTs("\r\n could not create the requested directory\r\n");
   break;
  }
  case sdFileOpenFail: {
   outputStringToBothUARTs("\r\n could not open the requested file\r\n");
   break;
  }
  case sdFileSeekFail: {
   outputStringToBothUARTs("\r\n could not seek as requested\r\n");
   break;
  }
  case sdFileWriteFail: {
   outputStringToBothUARTs("\r\n could not write to the file\r\n");
   break;
  }
  case sdFileWritePartial: {
   outputStringToBothUARTs("\r\n partial write to the file\r\n");
   break;
  }
  case sdFileReadFail: {
   outputStringToBothUARTs("\r\n could not read from the file\r\n");
   break;
  }
  case sdInvalidDate: {
   outputStringToBothUARTs("\r\n invalid date\r\n");
   break;
  }
  //case NoClose: {
   //outputStringToBothUARTs("\r\n could not close file\r\n");
   //break;
  //}
  default: {
   outputStringToBothUARTs("\r\n unknown error\r\n");
   break;
  } 
 } // switch err
} // end of tellFileError

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */
/* This is not required in read-only configuration.        */

DWORD get_fattime ()
{

	// use the timestamp of the current alarm time
	// this will be the time it is when the alarm has triggered, when work such as creating files occurs

	// Pack date and time into a DWORD variable
	return	  ((DWORD)(2000 + dt_CurAlarm.year - 1980) << 25)
			| ((DWORD)dt_CurAlarm.month << 21)
			| ((DWORD)dt_CurAlarm.day << 16)
			| ((DWORD)dt_CurAlarm.hour << 11)
			| ((DWORD)dt_CurAlarm.minute << 5)
			| ((DWORD)dt_CurAlarm.second >> 1);
	
/*
	// return default, winter solstice 2011
	// 2011-12-22 05:30:00 UTC
	return	  ((DWORD)(2011 - 1980) << 25)
			| ((DWORD)12 << 21)
			| ((DWORD)22 << 16)
			| ((DWORD)5 << 11)
			| ((DWORD)30 << 5)
			| ((DWORD)0 >> 1);
*/
}

/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/

#define xmit_spi(dat) 	SPDR=(dat); loop_until_bit_is_set(SPSR,SPIF)



/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/

static
BYTE rcvr_spi (void)
{
	SPDR = 0xFF;
	loop_until_bit_is_set(SPSR, SPIF); // syntax: loop_until_bit_is_set(sfr, bit)
	return SPDR;
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)	SPDR=0xFF; loop_until_bit_is_set(SPSR,SPIF); *(dst)=SPDR



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
BYTE wait_ready (void)
{
	BYTE res;


	Timer2 = 50;	/* Wait for ready in timeout of 500ms */
	rcvr_spi();
	do
		res = rcvr_spi();
	while ((res != 0xFF) && Timer2);

	return res;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void release_spi (void)
{
	DESELECT();
	rcvr_spi();
}



/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

/*
static
void power_on (void)
{
	PORTE &= ~0x80;				// Socket power ON
	for (Timer1 = 3; Timer1; );	// Wait for 30ms
	PORTB = 0b10110101;			// Enable drivers
	DDRB  = 0b11000111;
	SPCR = 0b01010000;			// Initialize SPI port (Mode 0)
	SPSR = 0b00000001;
}
*/

static 
void power_on (void) 
{ 
//#if (defined SD_PWR_BIT | defined SD_PWR_PORT) 
//   SD_PWR_PORT|=(1<<SD_PWR_BIT);   // Drives PWR pin high
   SD_PWR_PORT &= ~(1<<SD_PWR_BIT);   // Drives PWR pin low, inverted through FET 
   SD_PWR_DD |= (1<<SD_PWR_BIT);          // Turns on PWR pin as output 

//#endif 
	PRR0 &= ~(1<<PRSPI); // assure SPI module power is on
	for (Timer1 = 3; Timer1; );	// Wait for 30ms
	SD_CS_DD |= (1<<SD_CS_BIT);          // Turns on CS pin as output 
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS)); 
	SPCR = (1<<SPE)|(1<<MSTR); // Initialize SPI port (Mode 0)
	// enable MISO internal pull-up resistor
	DDR_SPI &= ~(1<<SD_MISO_PU_BIT); // force MISO to be in input
	SD_MISO_PU_PORT |= (1<<SD_MISO_PU_BIT); // output high to the port, enables pull-up on the input
} 

/*
static
void power_off (void)
{
	SELECT();				// Wait for card ready
	wait_ready();
	release_spi();

	SPCR = 0;				// Disable SPI function
	DDRB  = 0b11000000;		// Disable drivers
	PORTB = 0b10110000;
	PORTE |=  0x80;			// Socket power OFF
	Stat |= STA_NOINIT;		// Set STA_NOINIT
}
*/

static 
void power_off (void) 
{
	if (!(PRR0 & (1<<PRSPI))) // is SPI power on (Pwr Save bit clear)?
	{
		if (SPCR & (1<<SPE)) // is SPI enabled?
		{ // shutdown sequence
			SELECT(); // Wait for card ready
			//   CS_LOW();            // Wait for card ready
			wait_ready();
			release_spi();
			//   deselect();
			//   SD_CS_DD |= (1<<SD_CS_BIT);          // Turns on CS pin as output
			// for (Timer1 = 2; Timer1; );	// Wait for 20ms
			// SD_CS_DD |= (1<<SD_CS_BIT);          // Turns on CS pin as output
			// DESELECT(); // this may be redundant, but explicitly deselect the SD card
			SPCR &= ~(1<<SPE); // disable SPI
			
		}
		PRR0 |= (1<<PRSPI); // turn off power to SPI module, stop its clock
	}		
	for (Timer1 = 2; Timer1; );	// Wait for 20ms
    DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_SCK)); // change SPI output lines MOSI and SCK into inputs
	// pins might source current momentarily
	 // set port bits to 0, disable any internal pull-ups; tri-state the pins
	SPI_PORT &= ~((1<<SPI_MOSI_BIT)|(1<<SPI_SCK_BIT)|(1<<SPI_MISO_BIT)); // MISO was already an input
	
	SD_CS_DD &= ~(1<<SD_CS_BIT); // change SS to an input, momentarily sources current through internal pull-up
	SD_CS_PORT &= ~(1<<SD_CS_BIT); // set port bit to zero, tri-state the input

	SD_PWR_DD |= (1<<SD_PWR_BIT);          // Turns on PWR pin as output 
	SD_PWR_PORT |= (1<<SD_PWR_BIT);   // Drive PWR pin high; this will turn FET off
	SD_PWR_DD &= ~(1<<SD_PWR_BIT);          // change PWR pin to an input
	// internal pull-up momentarily pulls high, along with external pull-up
	SD_PWR_PORT &= ~(1<<SD_PWR_BIT);   // tri-state the pin; external pull-up keeps FET off
	
	Stat |= STA_NOINIT;      // Set STA_NOINIT
} 

void turnSDCardPowerOff(void)
{
// public wrapper function
	power_off();
}


/*
static
int chk_power(void)		// Socket power state: 0=off, 1=on
{
	return (PORTE & 0x80) ? 0 : 1;
}
*/
static
int chk_power(void)		/* Socket power state: 0=off, 1=on */
{
//	return (SD_PWR_PORT & (1<<SD_PWR_BIT)) ? 0 : 1;
   return 1; 
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
BOOL rcvr_datablock (
	BYTE *buff,			/* Data buffer to store received data */
	UINT btr			/* Byte count (must be multiple of 4) */
)
{
	BYTE token;


	Timer1 = 20;
	do {							/* Wait for data packet in timeout of 200ms */
		token = rcvr_spi();
	} while ((token == 0xFF) && Timer1);
	if(token != 0xFE) return FALSE;	/* If not valid data token, retutn with error */

	do {							/* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 4);
	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

	return TRUE;					/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
static
BOOL xmit_datablock (
	const BYTE *buff,	/* 512 byte data block to be transmitted */
	BYTE token			/* Data/Stop token */
)
{
	BYTE resp, wc;


	if (wait_ready() != 0xFF) return FALSE;

	xmit_spi(token);					/* Xmit data token */
	if (token != 0xFD) {	/* Is data token */
		wc = 0;
		do {							/* Xmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);
		xmit_spi(0xFF);					/* CRC (Dummy) */
		xmit_spi(0xFF);
		resp = rcvr_spi();				/* Reveive data response */
		if ((resp & 0x1F) != 0x05)		/* If not accepted, return with error */
			return FALSE;
	}

	return TRUE;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* ACMD<n> is the command sequence of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready */
	DESELECT();
	SELECT();
	if (wait_ready() != 0xFF) return 0xFF;

	/* Send command packet */
	xmit_spi(cmd);						/* Start + Command index */
	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive number (0) */
)
{
	BYTE n, cmd, ty, ocr[4];


	if (drv) return STA_NOINIT;			/* Supports only single drive */
	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */

	power_on();							/* Force socket power on */
	FCLK_SLOW();
	for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		Timer1 = 100;						/* Initialization timeout of 1000 msec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDHC */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();		/* Get trailing return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at vdd range of 2.7-3.6V */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30));	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else {							/* SDSC or MMC */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			while (Timer1 && send_cmd(cmd, 0));			/* Wait for leaving idle state */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
	release_spi();

	if (ty) {			/* Initialization succeeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
		FCLK_FAST();
	} else {			/* Initialization failed */
		power_off();
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {	/* Single block read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	release_spi();

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	release_spi();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY == 0 */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL != 0
DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	WORD csize;


	if (drv) return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0:		/* Sub control code == 0 (POWER_OFF) */
			if (chk_power())
				power_off();		/* Power off */
			res = RES_OK;
			break;
		case 1:		/* Sub control code == 1 (POWER_ON) */
			power_on();				/* Power on */
			res = RES_OK;
			break;
		case 2:		/* Sub control code == 2 (POWER_GET) */
			*(ptr+1) = (BYTE)chk_power();
			res = RES_OK;
			break;
		default :
			res = RES_PARERR;
		}
	}
	else {
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might not left updated. */
			SELECT();
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
					*(DWORD*)buff = (DWORD)csize << 10;
				} else {					/* SDC ver 1.XX or MMC*/
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(DWORD*)buff = (DWORD)csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
			*(WORD*)buff = 512;
			res = RES_OK;
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
			if (CardType & CT_SD2) {	/* SDC ver 2.00 */
				if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
					rcvr_spi();
					if (rcvr_datablock(csd, 16)) {				/* Read partial block */
						for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
						*(DWORD*)buff = 16UL << (csd[10] >> 4);
						res = RES_OK;
					}
				}
			} else {					/* SDC ver 1.XX or MMC */
				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
					if (CardType & CT_SD1) {	/* SDC ver 1.XX */
						*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					} else {					/* MMC */
						*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					}
					res = RES_OK;
				}
			}
			break;

		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
			*ptr = CardType;
			res = RES_OK;
			break;

		case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
			if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
			if (send_cmd(CMD10, 0) == 0		/* READ_CID */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 4; n; n--) *ptr++ = rcvr_spi();
				res = RES_OK;
			}
			break;

		case MMC_GET_SDSTAT :	/* Receive SD statsu as a data block (64 bytes) */
			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
				rcvr_spi();
				if (rcvr_datablock(ptr, 64))
					res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
		}

		release_spi();
	}

	return res;
}
#endif /* _USE_IOCTL != 0 */




