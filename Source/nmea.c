/* ========================================
 * NMEA_IF.c -NMEA Decode functions
 *
 * Copyright (C) 2021 - G2 AIRBORNE SYSTEMS
 * All Rights Reserved
 *
 *  Created on: Sep 9, 2021
 *      Author: Guillermo
 * ========================================*/

#include "common.h"

/*------------------------------------------------------------------------------
 * Public variables
 *------------------------------------------------------------------------------*/

bool IsValidGpsTime = false;
GpsDateTime GpsTime = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

Position GpsPosition =  { "","","","","","" };

bool IsValidNMEA = false;
int MsgTypeID = 0;      	// valid only while NMEA_IDDLE after reception of NMEA_MSG_COMPLETE


/*------------------------------------------------------------------------------
 * read_1e6us() - custom function to read decimals aligned to the microsecond
 *------------------------------------------------------------------------------*/
int read_1e6us( char *cp)
{
    int e=0;
    int val=0;

	while ((*cp != 0) && (e < 6))
	{
		val = val * 10 + dec2bin(*cp++);
		e++;
	}
    while (e++ < 6)     // adjust to 6 digits
    {
        val *= 10;
    }
    return val;
}

/*--------------------------------------------------------------------------
 * NMEA records decoding:
 *
 * Sample NOVATEL records:
 *  $GPGGA,193522.987,4929.33117,N,09641.64750,W,1,19,0.6,247.4,M,-27.7,M,0.0,1023*47
 *  $GPVTG,15.7,T,,M,0.0,N,0.0,K*53
 *
 * Sample APPLANIX POSAV-6 records:
 *  $INGGA,225104.100,3955.50232,N,10507.04254,W,1,19,0.6,1654.00,M,,,,*0C
 *  $INZDA,225104.1083,06,07,2018,,*76
 *
 * 	ZDA message duration: 3.5 MSec @ 115200 bauds
 * 	GGA message duration: 7.2 MSec @ 115200 bauds
 * 	@ 10 Hz ~ 10% dutty cycle
 *
 *--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
 * NMEA GGA and ZDA state machinne constants
 *--------------------------------------------------------------------------*/
	enum _states	// GPGGA Fields coma separated
	{
		IDDLE,	 	// 0 Awaiting for start character
		HDR,		// 1 Header decode

		ZDA,		// 1 Date and time
		UTCZ,		// 2 UTC time in ZDA message hhmmss.ss
		DAY,		// 3 Day xx 01 to 31
		MONTH,		// 4 month xx 01 to 12
		YEAR,		// 5 year xxxx
		HTZOFF,		// 6 Local time offset Hours NO OPERATION for last 3 empty fields
		MTZOFF,		// 7 Local time zone offset minutes NO OPERATION

        //------------ GPGGA message duration = 7.2 ms @ 115200 Bauds
		GGA,		// 1 Time and Position Fix Data
		UTCG,		// 2 UTC time HHMMSS.SS (9 chars)
		LAT,		// 3 Latitude DDMM.mm (7 chars)
		NS,			// 4 N or S (1 char)
		LON,		// 5 Longitue DDDMM.mm (8 chars)
		EW,			// 6 E or W (1 char)
		QUAL,		// 7 GPS quality (ignored)
		SATS,		// 8 # of Satelites (ignored)
		DOP,		// 9 Dilution of precision (ignored)
		ALT,		// 10 Altitude float
		AU,			// 11 Units for altitude = M 1 char
		UND,		// 12 Undulation of WGS84 elipsoid float (ignored)
		UU,			// 13 Units for undulation M 1 char (ignored)
		AGE,		// 14 Age of correction data 2 chars (ignored)
		SID,		// 15 Differential station ID 4 chars (ignored)
		CSM0,		// 16 Checksum *XX followed by CR,LF in serial transmissions
        CSM1
	};

	typedef struct
	{
		char* type;
		int	  state;
        int   returnID;     // Identify the completed message to the caller
	} MsgID;

#define MSGIDCNT 4

	MsgID msgIDlist[MSGIDCNT+1] =	// list of supported messages
	{
		{ "GPZDA", ZDA, NMEA_ZDA },			// Novatel ZDA
		{ "INZDA", ZDA, NMEA_ZDA },			// Appanix ZDA
		{ "GPGGA", GGA, NMEA_GGA },			// Novatel GGA
		{ "INGGA", GGA, NMEA_GGA },			// Applanix GGA
		{ "UNKNOWN", IDDLE - 1, NMEA_UNKNOWN },	// FALL THROUGH CASE
	};


#define BUFFER_SIZE 80

int nmea_state_machine_consume(char ch)
{
	static int state = IDDLE;
	static int chksm = 0;
	static int vchk = 0;

	static char buffer[BUFFER_SIZE];
	static int bufcnt = 0;			// char counter for the field

    if ( state == IDDLE )
    {
    	if (ch == '$')					// start of new record?
    	{
    		state = HDR;
    		bufcnt = 0;
    		vchk = chksm = 0;
    		IsValidNMEA = false;
    		return NMEA_REC_START;
    	}
        return NMEA_IDDLE;              // do nothing until we receive the start record
    }
	else if (ch == '*')					// checksum does not includes the star
	{
		bufcnt = 0;
		state = CSM0;
		return NMEA_REC_INCOMPLETE;
	}
	else if ( state == CSM0 )			//recover checksum
	{
			vchk = hex2bin(ch);
            state++;
    		return NMEA_REC_INCOMPLETE;
	}
	else if ( state == CSM1 )
	{
		vchk = (vchk << 4) | hex2bin(ch);
		IsValidNMEA = (chksm == vchk);
		state = IDDLE;

		if (MsgTypeID == NMEA_ZDA )
			IsValidGpsTime = true;

        return IsValidNMEA? NMEA_MSG_COMPLETE : NMEA_REC_ERROR;
	}

	else if ( bufcnt == BUFFER_SIZE || ch == 0 )	// avoid buffer overflow exception
	{
        state = IDDLE;
		return NMEA_REC_ERROR;
	}

	chksm ^= ch;						// add to the checksum

	if (ch != ',')						// end of field?
	{
		buffer[bufcnt++] = ch;			// store and return
		return NMEA_REC_INCOMPLETE;
	}

	buffer[bufcnt] = 0;					// terminate string and fall through decoder

	switch (state)
	{
		case HDR:						// find Msg ID in our list of known IDs
		{
			int i = 0;
			for (; i < MSGIDCNT; i++)
			{
				if (strcmp(buffer, msgIDlist[i].type) == 0 )	// If record ID found skip
				{
					break;
				}
			}
			state = msgIDlist[i].state;			// advance to next state

            MsgTypeID = msgIDlist[i].returnID;  // to identify message when complete

			break;
		}
		case UTCZ:
		case UTCG:								// Recover UTC Time
		{
			if (bufcnt >= 6)
			{
				uint32_t tt = (dec2bin(buffer[0]) * 10) + dec2bin(buffer[1]);

				if (tt <= 24 )
					GpsTime.Hour = tt;

				tt = (dec2bin(buffer[2]) * 10) + dec2bin(buffer[3]);

				if ( tt < 60 )
					GpsTime.Min = tt;
				tt = (dec2bin(buffer[4]) * 10) + dec2bin(buffer[5]);

				if ( tt < 60 )
					GpsTime.Sec = tt;
			}
			if ( bufcnt >= 8)
			{
                /* Read fractional seconds aligned to the microsecond */
				GpsTime.Usec = read_1e6us( buffer + 7);
			}
			else
			{
				GpsTime.Usec = 0;
			}
			break;
		}
		case DAY:
		{
			if (bufcnt == 2)
			{
				uint32_t dd = atoi( buffer ); // (dec2bin(buffer[0]) * 10) + dec2bin(buffer[1]);
				if (dd > 0 && dd <= 31)
					GpsTime.Day = dd;
			}
			break;
		}
		case MONTH:
		{
			if (bufcnt == 2)
			{
				GpsTime.Month = atoi( buffer ); // (dec2bin(buffer[0]) * 10) + dec2bin(buffer[1]);
			}
			break;
		}
		case YEAR:
		{
			if (bufcnt == 4)
			{
				GpsTime.Year = atoi( buffer ); // (dec2bin(buffer[0]) * 1000) + (dec2bin(buffer[1]) * 100) + (dec2bin(buffer[2]) * 10) + dec2bin(buffer[3]);
			}
			break;
		}
		case LAT:
		{
		    strncpy(GpsPosition.Lat, buffer,LATLON_SIZE);
			break;
		}
		case NS:
		{
   			strncpy(GpsPosition.NS, buffer, EWNS_SIZE);
			break;
		}
		case LON:
		{
			strncpy(GpsPosition.Lon, buffer, LATLON_SIZE);
			break;
		}
		case EW:
		{
           	strncpy(GpsPosition.EW, buffer, EWNS_SIZE);
			break;
		}
		case ALT:
		{
		    strncpy(GpsPosition.Alt,buffer,LATLON_SIZE);
			break;
		}
		case AU:
		{
   			strncpy(GpsPosition.Unit, buffer,EWNS_SIZE);
			break;
		}
	}
	/* move to next state and reset the buffer count*/
	state++;
	bufcnt = 0;

    return NMEA_REC_INCOMPLETE;
}

bool new_gps_msg = false;
bool PPSTimeValid = false;
GpsDateTime PPSTime;

double *pSyncSOW = NULL;		// Pointer to sync Seconds of the week;

_Notify_GPS_Time ptime_notify = NULL;


void Read_NMEA_Message(char *buf, uint32_t cnt)
{
/*
	for (int i = 0; i < cnt; i++ ) // Read all data in buffer;
	{
		if ( nmea_state_machine_consume( buf[i] ) == NMEA_MSG_COMPLETE )
		{
			if (MsgTypeID == NMEA_ZDA || (MsgTypeID == NMEA_GGA && IsValidGpsTime) )
			{
				// if same block do not repeat PPS report
				if (PPSTime.Sec < GpsTime.Sec )
				{
					memcpy( (void*)&PPSTime, (void*)&GpsTime, sizeof(PPSTime) );
					PPSTime.Usec = 0;
					GetGpsWeekSeconds(&PPSTime,0);
					PPSTimeValid = true;

					if (pSyncSOW != NULL)
					{
						*pSyncSOW = PPSTime.WeekSec;
					}

					if (ptime_notify != NULL)
					{
						(*ptime_notify)(&PPSTime);
					}
				}
			}

			new_gps_msg = true;
		}
	}

	*/
}




void RegisterGPSTimeNotify(_Notify_GPS_Time pnotify)
{
	ptime_notify = pnotify;
}

void SyncSOW( double *psync)
{
	pSyncSOW = psync;
}

//==============================================================================
//
// GPS Week:Seconds calculation
//
//==============================================================================
#define GPS_YEAR_ZERO			1980
#define GPS_DAY_ZERO			6		// Jan 6, 1980 00h:00m:00s
#define START_YEAR_NOW      	2020    // Skip calculation of previous leap years
#define LEAP_YEARS_BEFORE_2020	10      // Leap years since 1980 as of 2021
#define TOTAL_WEEK_SECONDS		604800	// Total seconds in a week

/*==============================================================================
 * GpsLeapSeconds
 *
 * The Leap seconds can be retrieved from GPS with the TIME log of NovAtel receivers
 *
 * If it can't be obtained from the GPS the value must be stored in non-volatile memory
 * and routines must be provided to retrieve and update its value.
 *
 * Note that the time reported in NMEA records is GMT time.
 *
 *------------------------------------------------------------------------------*/
uint32_t  GpsLeapSeconds = 18;
Time_Type GpsTimeType;
/*==============================================================================*/


uint32_t leapYearTable[] =
{
    /* 1980, 1984, 1988, 1992, 1996, 2000, 2004, 2008, 2012, 2016,*/
       2020, 2024, 2028, 2032, 2036, 2040, 2044, 2048, 2052, 2056,
       2060, 2064, 2068, 2072, 2076, 2080, 2084, 2088, 2092, 2096,
};

int DaysIntoTheYear[13] =
{
	/*12 / 31 / 2017*/	0,
	/* 1 / 31 / 2018*/	31,
	/* 2 / 28 / 2018*/	59,
	/* 3 / 31 / 2018*/	90,
	/* 4 / 30 / 2018*/	120,
	/* 5 / 31 / 2018*/	151,
	/* 6 / 30 / 2018*/	181,
	/* 7 / 31 / 2018*/	212,
	/* 8 / 31 / 2018*/	243,
	/* 9 / 30 / 2018*/	273,
	/*10 / 31 / 2018*/	304,
	/*11 / 30 / 2018*/	334,
	/*12 / 31 / 2018*/	365
};

/*----------------------------------------------------------------------
 * Analytical Leap Year calculator
 *
int OLDIsLeapYear(int year)
{
	if ((year % 4) == 0)				// if divisible by 4 ...
	{
		if ((year % 100) != 0)			// ... but not by 100...
			return 1;

		if ((year % 400) == 0)			// ... unless it is ALSO  divisible by 400
			return 1;
	}
	return 0;
}

int LeapYears[] =
{
        1980, 1984, 1988, 1992, 1996, 2000, 2004, 2008, 2012, 2016,
        2020, 2024, 2028, 2032, 2036, 2040, 2044, 2048, 2052, 2056,
        2060, 2064, 2068, 2072, 2076, 2080, 2084, 2088, 2092, 2096,
};

*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------
 * Table based Leap Year calculator
 *----------------------------------------------------------------------*/
uint32_t GetLeapYears(uint32_t year)
{
    uint32_t lyears = LEAP_YEARS_BEFORE_2020;

    for (int i = 0; i < ARRAY_SIZE(leapYearTable); i++)
    {
        if (year < leapYearTable[i])
            break;

        lyears++;
    }
    return lyears;
}

int GetGpsWeekSeconds(GpsDateTime* DT, uint32_t usoffset)
{
    uint32_t Year = DT->Year;
    uint32_t Month = DT->Month;
    uint32_t Day = DT->Day;
    uint32_t Hour = DT->Hour;
    uint32_t Min = DT->Min;
    uint32_t Sec = DT->Sec;

    if (Year < START_YEAR_NOW)                  // Won't work with past dates
        return 0;

    if (Month < 1 || Month > 12)		        // check ranges
        return 0;

    if (Day < 1 || Day > 31)
        return 0;

    if ( Hour > 23 || Min > 59 || Sec > 59 )
        return 0;

    uint32_t yeardays = (Year - GPS_YEAR_ZERO) * 365;	// calculate complete years

    yeardays += GetLeapYears(Year);				        // Add extra day of leap years

    yeardays += DaysIntoTheYear[Month - 1];		        // Add complete month days

    /* because GPS started on January 6 */
    yeardays += Day - GPS_DAY_ZERO;                     // add complete days from GPS start day
                                                        // This will get the # of weeks right
    Sec += Hour * 3600 + Min * 60;                      //

    div_t W = div(yeardays, 7);				            // get weeks and remaining days of the week
    uint32_t week = W.quot;
    uint32_t seconds = W.rem * 86400 + Sec;             // convert to total seconds into the week
    seconds += GpsLeapSeconds;  					    // Add GPS leap seconds to UTC time.

    // Now add the microseconds offset

    div_t Q = div(usoffset, 1000000);
    seconds += Q.quot;                  		        // add rollover seconds
    DT->Usec = Q.rem;                   		        // update the milliseconds

    if (seconds >= TOTAL_WEEK_SECONDS)              	// handle rollover of week on midnight of Sunday
    {
        week++;
        seconds -= TOTAL_WEEK_SECONDS;
    }

    DT->GPSWeek = week;
    DT->WeekSec = seconds;             // Substract GPS leap seconds to convert UTC time.

    return 1;
}


#ifdef USE_PAPLEVT
//==============================================================================
//
// ENCODE MESSAGES
//
//==============================================================================

/*
*	Applanix PAPLEVT encoder
*
*	Header   , event time  , , event #, latitude ,X, Longitude ,X, elev  ,M,xxx.xx, ,xx.xx,x.xx,xxx.xx, checksum
*	$PAPLEVT1,513255.928482,G,00000001,3955.50083,N,10507.04488,W,1654.18,M,-16.40,M,-2.53,2.29,359.98,*08
*	$PAPLEVT1,513409.694419,G,00000002,3955.50262,N,10507.04318,W,1654.20,M,-16.40,M,-2.39,2.20,243.89,*09
*
*/

int Encode_PEAPLEVT(GpsDateTime *GpsTime, uint16 event, Position *P, char* buffer, int bsize)
{
    if (bsize < 200 )
        return 0;

	int cnt = sprintf(buffer, "$PAPLEVT1,%ld.%.6ld,G,%.8u,",GpsTime->WeekSec, GpsTime->Usec,event);
	cnt += sprintf(buffer + cnt, "%s,%s,%s,%s,%s,%s,", P->Lat, P->NS, P->Lon, P->EW,P->Alt,P->Unit);
	cnt += sprintf(buffer + cnt, /*"-16.40,M,-2.38,2.20,243.76,"*/ "00.00,M,0.00,0.00,000.00,");

	// compute checksum
	char *cp = buffer + 1;
	char cs = *cp++;

	while (*cp != 0)
	{
		cs ^= *cp++;
	}
	cnt += sprintf(buffer + cnt, "*%.2X\r\n", cs);

	return cnt;
}
#endif

/* [] END OF FILE */



