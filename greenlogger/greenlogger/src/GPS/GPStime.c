/*
 * GPStime.c
 *
 * Created: 4/10/2014 8:59:10 PM
 *  Author: rshory
 */ 

#include <inttypes.h>
#include <string.h>
#include "../GPS/GPStime.h"
#include "../Bluetooth/RN42.h"
#include "../I2C/I2C.h"
#include "../greenlogger.h"
#include "../RTC/DS1342.h"
#include "../Accelerometer/ADXL345.h"
#include "../TemperatureSensor/TCN75A.h"
#include "../SDcard/diskio.h"
#include "../BattMonitor/ADconvert.h"
#include "../LtSensor/TSL2561.h"
#include <util/twi.h>
#include <math.h>

// generic control of GPS subsystem

extern volatile adcData cellVoltageReading;
extern volatile gFlags gpsFlags;
extern volatile iFlags initFlags;
extern volatile gpsLocation curLocation, prevLocation;
extern volatile dateTime dt_LatestGPS;
extern volatile uint16_t gpsTimeReqCountdown;
extern char commandBuffer[COMMAND_BUFFER_LENGTH];
extern char strJSONloc[256];
extern volatile uint8_t dailyTryAtAutoTimeSet;

/**
 * \brief send GPS time-request signal
 *
 * Send low-going reset pulse to external uC, to
 * initiate the time-request sequence in the GPS subsystem.
 * Subsystem attempts to power cycle the GPS and
 * parse a time signal from the GPS NMEA
 * data. If successful, returns a set-time command on this uC's
 * UART. This uC should stay roused for up to 3 minutes, to 
 * receive that command.
 */
void GPS_initTimeRequest(void) {
	if (gpsFlags.gpsTimeRequested) {
#ifdef VERBOSE_DIAGNOSTICS
		outputStringToBothUARTs("\r\n 'gpsTimeRequested' already set, 'GPS_initTimeRequest' skipped \r\n");
#endif
		return; // don't duplicate request while one is pending
	} 
	// check if enough power to get time from GPS
	uint8_t intTmp1 = readCellVoltage(&cellVoltageReading);
	if (cellVoltageReading.adcWholeWord < CELL_VOLTAGE_OK_FOR_GPS) {
		outputStringToBothUARTs("\r\n power too low for GPS get-time request \r\n");
		return;
	}
	// count how many tries
	if ((gpsFlags.gpsReqTest == 0) // an automatically generated request, not manual
			&& (initFlags.gpsTimeAutoInit) // past the auto-init tries, when device first powered up
			&& (1)) { // other criteria here
		if (++dailyTryAtAutoTimeSet > MAX_DAILY_TRIES_FOR_GPS_TIME) {
#ifdef VERBOSE_DIAGNOSTICS
			int l;
			char s[64];
			l = sprintf(s, "\n\r tried %d times in a row to get GPS time today, try %d ignored",
				 MAX_DAILY_TRIES_FOR_GPS_TIME, dailyTryAtAutoTimeSet);
			outputStringToBothUARTs(s);
#endif
			return;
		}
	}	
	outputStringToBothUARTs("\r\n sending get-time request to GPS subsystem \r\n");
	stayRoused(18000); // stay awake for up to 3 minutes to receive any reply
	gpsFlags.gpsTimeRequested = 1; // for now, use this to distinguish any
	// time-set command that comes back as being from the GPS

	//dailyTryAtAutoTimeSet
	cli(); // temporarily disable interrupts to prevent Timer3 from
	// changing the count partway through
	gpsTimeReqCountdown = 18000; // set a timeout of 3 minutes;
	// after that, gpsFlags.gpsTimeRequested will be cleared
	sei();
	PORTB &= ~(1<<GPS_SUBSYSTEM_CTRL); // set low
	// uC in GPS subsystem, at Vcc 3V, needs a 700ns low-going pulse for definite reset
	// each clock cycle of this uC, at 8MHz, is 125ns
	// so 6 clock cycles of this uC would be 750ns and ought to do it
	for (uint8_t i=6; i; i--){ 
		// loop opcodes will make the time more than double, 
		// but still brief enough to not interfere with overall
		// program flow
		asm volatile ("nop");
	}
	PORTB |= (1<<GPS_SUBSYSTEM_CTRL); // set high
	// 'erase' the command buffer, prep to receiving the set-time signal there
	commandBuffer[0] = '\0';
}

uint16_t getAverageMinute (chargeInfo *startOfArrayOfReadings) {
	chargeInfo *curReading;
	int16_t minutesFromTime;
	uint8_t i;
	double minuteRadians, sumSine = 0, sumCosine = 0;
	for (i=0; i<DAYS_FOR_MOVING_AVERAGE; i++) {
		curReading = startOfArrayOfReadings + i;
		if ((curReading->timeStamp.day) > 0) { // day=0 flags that this is not a filled-in item
			// get minutes
			// adjust by hour offset to always treat as if Universal Time
			// range zero to 1440, can be negative after offset by time zone
			minutesFromTime = (((curReading->timeStamp.hour) - (curReading->timeStamp.houroffset)) 
					* 60) + (curReading->timeStamp.minute);
			// map to 2Pi radians
			double minuteRadians = 2 * M_PI * minutesFromTime / 1440;
			sumSine += sin(minuteRadians);
			sumCosine += cos(minuteRadians);
		} // don't even need to count how many
	}
	// we have summed all the valid items and are now ready to calc the average
	if ((sumSine == 0.0) && (sumCosine == 0.0)) { // did not get any valid readings
		return 0; // return a usable default, time check will occur at Universal Time midnight
	} else { // atan2 supposedly returns something for (0,0), but above is attempt to make fail safe
		minuteRadians = atan2(sumSine, sumCosine); // fn output range is -pi to +pi
		if (minuteRadians < 0) minuteRadians += (2 * M_PI); // should now range 0 to 2pi
		return (uint16_t) (1440 * minuteRadians / (2 * M_PI)); // convert back to a positive minutes count
	}
	
}

void chargeInfo_getString(char* ciStr, chargeInfo *cip) {
	// get charge level into string
	// chargeInfo.level is the 32 bit reading stored direct from the ADC
	// V(measured) = adcResult * 2.5 (units are millivolts, so as to get whole numbers)
	int iLen;
	iLen = sprintf(ciStr, "%lumV\t", (unsigned long)(2.5 * (unsigned long)(cip->level)));
	// if not a valid date/time, will show as "2000-00-00 00:00:00 +00"
	datetime_getstring(ciStr + iLen, &(cip->timeStamp));
	strcat(ciStr, "\r\n");
}

void saveGPSLocation(char* locStr) {
	// this will typically receive a string that is tagged on the 
	// end of a set-time command
	// the whole string would be e.g.
	//"t2016-03-19 20:30:01 -08 4523.896300N12204.334200W\n\r\n\r\0";
	// the received pointer would point to position 24, just after
	// the timezone, and thus would see
	//" 4523.896300N12204.334200W\n\r\n\r\0";
	// if the pointer hits any space characters, the fn ignores them and moves on
	// if the pointer hits a null terminator, the fn returns without doing anything
	// in the rest of the string, the fn expects fixed-width fields packed from NMEA data
	// 2 numeric characters that are the whole degrees of latitude (padded with leading zeros)
	// 2 numeric characters that are the whole minutes of latitude (padded with leading zeros)
	// decimal point character
	// 6 decimal characters that are the fractional degrees latitude (always padded out to 6 digits with trailing zeros)
	// 1 character that is either 'N' for north latitude or 'S' for south latitude
	// 3 numeric characters that are the whole degrees of longitude (padded with leading zeros)
	// 2 numeric characters that are the whole minutes of longitude (padded with leading zeros)
	// decimal point character
	// 6 decimal characters that are the fractional degrees longitude (always padded out to 6 digits with trailing zeros)
	// 1 character that is either 'E' for east longitude or 'W' for west longitude
	
	// if fn gets a valid location, sets the flags "gpsGotLocation" and "gpsNewLocation"
	// if so, works on the globals "curLocation" and "prevLocation"
	// if "curLocation" was previously initialized, copies that to "prevLocation" (allows one level of undo)
	// puts parsed location into "curLocation"
	// uses time from global "dt_LatestGPS"
	// sets up the string "strJSONloc" for writing to SD card e.g.
	// {"Locations":[{"Priority":"Latest", "Latitude":"45.489230", "Longitude":"-122.094380", "TimeAcquired":"2017-02-10 10:47:20 +00"}, {"Priority":"Previous", "Latitude":"45.489140", "Longitude":"-122.093040", "TimeAcquired":"2017-01-31 14:12:10 +00"}]}
	// datum is always WGS84
	char *p;
	char tmpStr[26];
	double lat, lon, tmpD;
	int stLen;
	p = locStr;
	if (*p == '\0') return;
	while (*p == ' ') {
		p++;
		if (*p == '\0') return;
	}
	if (strlen(p) < 25) {
		outputStringToBothUARTs("\r\n");
		outputStringToBothUARTs(locStr);
		stLen = sprintf(tmpStr, "\r\n Length: %d \r\n", strlen(p));
		outputStringToBothUARTs("\r\n GPS string not long enough to parse \r\n");
		outputStringToBothUARTs(tmpStr);
		return; // not long enough to be valid data
	}
	strncpy(tmpStr, p, 2); // get whole degrees of latitude
	tmpStr[2] = '\0';
	lat = strtod(tmpStr, '\0');
	if (lat > 90.0) {
		outputStringToBothUARTs("\r\n GPS latitude >90 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // latitude out of range
	} 
	p += 2;
	strncpy(tmpStr, p, 9); // get minutes of latitude
	tmpStr[9] = '\0';
	tmpD = strtod(tmpStr, '\0');
	if (tmpD > 60.0) {
		outputStringToBothUARTs("\r\n GPS latitude minutes >60 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // latitude minutes out of range
	} 
	lat += (tmpD / 60.0); // complete the numeric latitude
	if (lat > 90.0) {
		outputStringToBothUARTs("\r\n GPS latitude final check >90 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // check latitude one more time
	} 
	p += 9;
	if (!((*p == 'N') | (*p == 'S'))) {
		outputStringToBothUARTs("\r\n GPS latitude letter not 'N' or 'S' \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // must be 'N' or 'S', for north/south latitude
	} 
	if (*p == 'S') lat *= -1; // south latitude is negative
	p++;
	strncpy(tmpStr, p, 3); // get whole degrees of longitude
	tmpStr[3] = '\0';
	lon = strtod(tmpStr, '\0');
	if (lon > 180.0) {
		outputStringToBothUARTs("\r\n GPS longitude >180 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // longitude out of range
	} 
	p += 3;
	strncpy(tmpStr, p, 9); // get minutes of longitude
	tmpStr[9] = '\0';
	tmpD = strtod(tmpStr, '\0');
	if (tmpD > 60.0) {
		outputStringToBothUARTs("\r\n GPS longitude minutes >60 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // longitude minutes out of range
	} 
	lon += (tmpD / 60.0); // complete the numeric longitude
	if (lon > 180.0) {
		outputStringToBothUARTs("\r\n GPS longitude final check >180 \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // check longitude one more time
	} 
	p += 9;
	if (!((*p == 'E') | (*p == 'W'))) {
		outputStringToBothUARTs("\r\n GPS longitude letter not 'E' or 'W' \r\n");
		outputStringToBothUARTs(locStr);
		outputStringToBothUARTs("\r\n");
		return; // must be 'E' or 'W', for east/west longitude
	}
	if (*p == 'W') lon *= -1; // west longitude is negative
	// ignore any characters after valid fields
	
	// valid lat/lon; store
	if (curLocation.timeStamp.month != 0) { // "curLocation" was previously initialized
		// copy to "prevLocation"
		// if we ever do this more than once, make it a separate fn
		prevLocation.latVal = curLocation.latVal;
		prevLocation.lonVal = curLocation.lonVal;
		strcpy(prevLocation.latStr, curLocation.latStr);
		strcpy(prevLocation.lonStr, curLocation.lonStr);
		// datetime_copy is to/from
		datetime_copy(&(prevLocation.timeStamp), &(curLocation.timeStamp) );
	}
	
	// to make floating point numbers appear correctly in Studio 7:
	// Project > Properties > Toolchain > AVR/GNU Linker > General
	//  X Use vprintf library(-Wl,-u,vfprintf)
	// Project > Properties > Toolchain > AVR/GNU Linker > Miscellaneous
	//  Other Linker Flags: -lprintf_flt
	
	// store lat/lon in "curLocation"
	curLocation.latVal = lat;
	curLocation.lonVal = lon;
	stLen = sprintf(curLocation.latStr, "%.6f", lat);
	stLen = sprintf(curLocation.lonStr, "%.6f", lon);
	// datetime_copy is to/from
	datetime_copy(&(curLocation.timeStamp), &dt_LatestGPS);
	
	// generate the JSON string
	// {"Locations":[{"Priority":"Latest", "Latitude":"45.489230", "Longitude":"-122.094380", "TimeAcquired":"2017-02-10 10:47:20 +00"}, {"Priority":"Previous", "Latitude":"45.489140", "Longitude":"-122.093040", "TimeAcquired":"2017-01-31 14:12:10 +00"}]}
	strcpy(strJSONloc, "{\"Locations\":[{\"Priority\":\"Latest\", \"Latitude\":\"");
	strcat(strJSONloc, curLocation.latStr);
	strcat(strJSONloc, "\", \"Longitude\":\"");
	strcat(strJSONloc, curLocation.lonStr);
	strcat(strJSONloc, "\", \"TimeAcquired\":\"");
	datetime_getstring(tmpStr, &(curLocation.timeStamp));
	strcat(strJSONloc, tmpStr);
	strcat(strJSONloc, "\"}");
	if (prevLocation.timeStamp.month != 0) { // "prevLocation" has a value
		strcat(strJSONloc, ", {\"Priority\":\"Previous\", \"Latitude\":\"");
		strcat(strJSONloc, prevLocation.latStr);
		strcat(strJSONloc, "\", \"Longitude\":\"");
		strcat(strJSONloc, prevLocation.lonStr);
		strcat(strJSONloc, "\", \"TimeAcquired\":\"");
		datetime_getstring(tmpStr, &(prevLocation.timeStamp));
		strcat(strJSONloc, tmpStr);
		strcat(strJSONloc, "\"}");
	}
	strcat(strJSONloc, "]}\r\n\r\n");

	// set the flags
	gpsFlags.gpsGotLocation = 1;
	gpsFlags.gpsNewLocation = 1;
	
}