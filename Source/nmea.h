/*
 * NMEA_IF.h - NMEA interface
 *
 * Copyright (C) 2021 - G2 AIRBORNE SYSTEMS
 * All Rights Reserved
 *
 *  Created on: Sep 9, 2021
 *      Author: G.Gallo
 */

#ifndef NMEA_H_
#define NMEA_H_

#include "globaldefs.h"

/*========================================
 *  NMEA - Decode functions
 *
 *  structures, macros, inline functions
 *========================================*/
	typedef struct
	{
		int Day;
		int Month;
		int Year;

		int Hour;
		int Min;
		int Sec;
		uint32_t Usec;		// microseconds are the same for UTC or GPS time

		uint32_t GPSWeek;
		uint32_t WeekSec;	// Seconds into the week

	} GpsDateTime;

#define LATLON_SIZE		16
#define EWNS_SIZE		4

typedef struct
{
	char Lat[LATLON_SIZE];
	char NS[EWNS_SIZE];
	char Lon[LATLON_SIZE];
	char EW[EWNS_SIZE];
	char Alt[LATLON_SIZE];
	char Unit[EWNS_SIZE];

} Position;

typedef enum tagNMEA_MsgID		// recognized NMEA messages
{
	NMEA_UNKNOWN,
	NMEA_ZDA,
	NMEA_GGA,

} NMEA_MsgID;


typedef enum tagNMEA_Return_Values
{
    NMEA_IDDLE,         	// Awaiting for record start char $
    NMEA_REC_START,     	// Record start character $ received
    NMEA_REC_INCOMPLETE,	// Message reception in pgrogess
    NMEA_REC_ERROR,     	// Check sum error or invalid record data (goes back to iddle)
    NMEA_MSG_COMPLETE,  	// Message reception is complete

} NMEA_DECODE_STATE;



/************************************************************************
 *  Read_NMEA_Message(char *buf, uint32_t cnt)
 *
 *  Can be called recursively one byte at a time or with the full message
 *  it sets the flag IsValidNMEA and IsValidGpsTime
 *-----------------------------------------------------------------------*/

void Read_NMEA_Message(char *buf, uint32_t cnt);

extern bool IsValidNMEA;
extern bool IsValidGpsTime;
extern GpsDateTime GpsTime;
extern Position GpsPosition;
extern int MsgTypeID;      		// valid only while NMEA_IDDLE after reception of NMEA_MSG_COMPLETE

extern bool new_gps_msg;
extern bool PPSTimeValid;
extern GpsDateTime PPSTime;


/************************************************************************
 *  Official GPS start time is Monday Jan 6,1980 00:00:00.000
 *  Note that 1980 was a Leap Year!
 *
 *  GPS time runs ahead of UTC by 18 leap seconds as of Sep. 2021
 *
 ************************************************************************/
typedef enum tTime_Type
{
	TIME_TYPE_INVALID,
	TIME_TYPE_LOCAL,
	TIME_TYPE_GMT,
	TIME_TYPE_GPS
}Time_Type;


typedef struct tGPS_Time
{
	Time_Type	TimeType;
	uint16_t	GpsWeek;
	double		WeeekSeconds;

} GPS_Time;



/*-------------------------------------------------------------*/
extern uint32_t  GpsLeapSeconds;
extern Time_Type GpsTimeType;		// Local, GMT or GPS
/*-------------------------------------------------------------*/


int GetGpsWeekSeconds(GpsDateTime *DT, uint32_t offset);

void SyncSOW( double *psync);		/* Sync the Seconds of the Week */


typedef void (*_Notify_GPS_Time)(const GpsDateTime *pDT);

void RegisterGPSTimeNotify(_Notify_GPS_Time pnotify);


#ifdef USE_PAPLEVT
int Encode_PEAPLEVT(GpsDateTime *GpsTime, uint16 event, Position *P, char* buffer, int bsize);
#endif



#endif /* NMEA_H_ */

/* [] END OF FILE */


