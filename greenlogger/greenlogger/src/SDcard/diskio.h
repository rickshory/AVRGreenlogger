/*-----------------------------------------------------------------------
/  Low level disk interface module include file  R0.07   (C)ChaN, 2009
/-----------------------------------------------------------------------*/

// defines for diskio.c (added by Rick Shory)

enum fileWriteResults
{
 sdOK = 0, // return value if it happened with no issues
 sdNoCard, // SD card not present or not detected
 sdMountFail, // could not mount file system object
 sdInitFail, // could not initialize file system
 sdMkDirFail, // could not create the requested directory
 sdChDirFail, // could not change to the requested directory
 sdFileOpenFail, // could not open the requested file
 sdFileWriteFail, // could not write to the file
 sdFileWritePartial, // did not write all bytes to file
 sdFileSeekFail, // could not seek as requested
 sdFileReadFail, // could not read as requested
 sdCloseFail, // could not close file
 sdPowerTooLowForSDCard, // cell voltage is below threshold to safely write card
 sdInvalidDate, // if seeking for file by date, passed date is not valid
 sdIgnoreCard // flag is set to ignore SD card
};

#ifndef _DISKIO

/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define	ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/*SPI configuration*/
#define SPI_PORT PORTB
#define SPI_MOSI_BIT 5
#define SPI_MISO_BIT 6
#define SPI_SCK_BIT 7
#define DD_MOSI   DDB5 
#define DD_SCK   DDB7 
#define DDR_SPI   DDRB 
#define DD_SS   4 
#define DD_MISO DDB6 

/* Defines for SD card SPI access */ 
#define SD_CS_BIT   4
#define SD_CS_PORT   PORTB
#define SD_CS_DD DDRB
#define SD_PWR_BIT   3
#define SD_PWR_PORT   PORTB
#define SD_PWR_DD DDRB

// enable internal pull-up resistor on MISO, instead of a physical one on the board
#define SD_MISO_PU_PORT PORTB
#define SD_MISO_PU_BIT 6

// #define SELECT()      SD_CS_PORT &= ~(1<<SD_CS_BIT)      // MMC CS = L
// #define DESELECT()   SD_CS_PORT |=  (1<<SD_CS_BIT)      // MMC CS = H

/* Port Controls  (Platform dependent) */
//#define SPI_PORT |= (1<<DD_MISO); 
#define CS_LOW()	SD_CS_PORT &= ~(1<<SD_CS_BIT)			// MMC CS = L
#define	CS_HIGH()	SD_CS_PORT |=  (1<<SD_CS_BIT)			// MMC CS = H

// original below, from "C:\Users\rshory\Documents\Current work\DataLogger\SD card\ff9sample\avr\mmc.c"
// #define SOCKWP		(PINB & 0x20)		/* Write protected. yes:true, no:false, default:false */
// #define SOCKINS		(!(PINB & 0x10))	/* Card detected.   yes:true, no:false, default:true */

#define	FCLK_SLOW()	SPCR = 0x52		/* Set slow clock (100k-400k) */
#define	FCLK_FAST()	SPCR = 0x50		/* Set fast clock (depends on the CSD) */

// Port Controls  (Platform dependent) 

#define SELECT()	SD_CS_PORT &= ~(1<<SD_CS_BIT)		// MMC CS = L
#define	DESELECT()	SD_CS_PORT |=  (1<<SD_CS_BIT)		// MMC CS = H

#define SOCKPORT	PINB			// Socket contact port
#define SOCKWP		1			// Write protect switch (PB1)
#define SOCKINS		2			// Card detect switch (PB2)
// following were from original example file
// "C:\Users\rshory\Documents\Current work\DataLogger\SD card\ff007sample\avr\mmc.c"
/*
#define SELECT()	PORTB &= ~1		// MMC CS = L
#define	DESELECT()	PORTB |= 1		// MMC CS = H

#define SOCKPORT	PINB			// Socket contact port
#define SOCKWP		0x20			// Write protect switch (PB5)
#define SOCKINS		0x10			// Card detect switch (PB4)

#define	FCLK_SLOW()					// Set slow clock (100k-400k)
#define	FCLK_FAST()					// Set fast clock (depends on the CSD)
*/

#define _READONLY	0	/* 1: Read-only mode */
#define _USE_IOCTL	1

#include "integer.h"

// diagnostics, for blinker
#define TOGGLE_INTERVAL 100

/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;

// wrapper function
void turnSDCardPowerOff(void);

/*---------------------------------------*/
/* Prototypes for disk control functions */

BYTE writeCharsToSDCard (char* St, BYTE n);
BYTE writeStringInFileToSDCard (char* stParam, char* stFile);
BYTE readStringFromFileFromSDCard (char* stParam, char* stFile);

BYTE writeTimezoneToSDCard (void);
BYTE readTimezoneFromSDCard (void);
BYTE outputContentsOfFileForDate (char* stDt);

void tellFileError (BYTE err);
BOOL assign_drives (int argc, char *argv[]);
DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);
DRESULT disk_read (BYTE, BYTE*, DWORD, BYTE);
#if	_READONLY == 0
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
#endif
DRESULT disk_ioctl (BYTE, BYTE, void*);

/* Card type flags (CardType) */

#define CT_MMC 0x01
#define CT_SD1 0x02
#define CT_SD2 0x04
#define CT_SDC (CT_SD1|CT_SD2)
#define CT_BLOCK 0x08

/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/* Command code for disk_ioctrl() */

/* Generic command */
#define CTRL_SYNC			0	/* Mandatory for write functions */
#define GET_SECTOR_COUNT	1	/* Mandatory for only f_mkfs() */
#define GET_SECTOR_SIZE		2
#define GET_BLOCK_SIZE		3	/* Mandatory for only f_mkfs() */
#define CTRL_POWER			4
#define CTRL_LOCK			5
#define CTRL_EJECT			6
/* MMC/SDC command */
#define MMC_GET_TYPE		10
#define MMC_GET_CSD			11
#define MMC_GET_CID			12
#define MMC_GET_OCR			13
#define MMC_GET_SDSTAT		14
/* ATA/CF command */
#define ATA_GET_REV			20
#define ATA_GET_MODEL		21
#define ATA_GET_SN			22


#define _DISKIO
#endif
