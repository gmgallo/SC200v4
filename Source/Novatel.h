/*
 * Novatel.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Guillermo
 */
#ifndef NOVATEL_H_
#define NOVATEL_H_

#include "globaldefs.h"

#pragma pack(push,1)


/*---------------------------- EXTRACTED FROM MESSAGE HEADERS ------------*/
extern volatile uint16_t GPS_Week;
extern volatile uint32_t GPS_WeekSeconds;
extern volatile bool     Enable_GNSS_Logging;

/*------------------------------------------------------------------------*/

typedef enum
{
	TIME_STATUS_UNKNOWN 			= 20, // Time validity is unknown
	TIME_STATUS_APPROXIMATE 		= 60, // Time is set approximately
	TIME_STATUS_COARSEADJUSTING 	= 80, // Time is approaching coarse precision
	TIME_STATUS_COARSE 				= 100, // This time is valid to coarse precision
	TIME_STATUS_COARSESTEERING 		= 120, // Time is coarse set and is being steered
	TIME_STATUS_FREEWHEELING 	   	= 130, // Position is lost and the range bias cannot be calculated
	TIME_STATUS_FINEADJUSTING 	   	= 140, // Time is adjusting to fine precision
	TIME_STATUS_FINE 			   	= 160, // Time has fine precision
	TIME_STATUS_FINEBACKUPSTEERING 	= 170, //Time is fine set and is being steered by the backup system
	TIME_STATUS_FINESTEERING		= 180, // Time is fine set and is being steered
	TIME_STATUS_SATTIME 			= 200  // ignore this one

}gps_time_status;


typedef enum
{
	Status_OK,
	CRC_Error,
	HW_Error,

} health_t;

typedef struct
{
	uint16_t Health;
	uint16_t ReciverStatus;
	uint16_t TimeStatus;
	uint16_t TimeSync;			// true if TimeSync was received from the receiver
	uint32_t CRCErrors;

} GNSS_Status;

extern volatile GNSS_Status nStatus;

void ClearNovatelStatus();
size_t GetNovatelStatus(char* buffer, size_t cnt, bool short_status);
//size_t GetNovatelStatusStr(char* buffer, size_t cnt);
//size_t GetNovatelStatusShort(char* buffer, size_t cnt);
void  PurgeNovatelBuffer();

/************************************************************************
 *  Read_Novatel_Message(char *buf, uint32_t cnt)
 *
 *  Reads message stream in Novatel binanry format and pushes it into the
 *  message queue.
 *
 *  it updates GPSWeek and GPSMsec
 *
 *  returns: # of bytes expected to complete the message. Or 0 if not known yet.
 *
 *-----------------------------------------------------------------------*/
#define LARGEST_NOVATEL_MESSAGE		2048U
//#define NOVATEL_RECORD_SIZE		256U  // for flex ring buffer (not working!)
#define NUMBER_OF_GNSS_RECORDS	 32U


void Read_Novatel_Message( uint8_t *buf, size_t cnt);

/*--------------------------------------------------------------
 *
 * NOVATEL MESSAGE HEADERS
 *
 *--------------------------------------------------------------*/

enum _header_sync_chars		// Sync character values for binary logs
{
	NOVATEL_SYNC1 		= 0xAA,
	NOVATEL_SYNC2 		= 0x44,
	NOVATEL_SYNC3_LONG  = 0x12,			// 3rd ID for LONG binary headers
	NOVATEL_SYNC3_SHORT = 0x13			// 3rd ID for SHORT binary Header
};

enum asc_log_answer_chars
{
	BEGUIN_CMD_ANSWER	= '<',
	END_CMD_ANSWER		= ']'
};

typedef struct _short_header
{
	uint8_t		sync1;			// NOVATEL_S1
	uint8_t		sync2;			// NOVATEL_S2
	uint8_t		sync3;			// NOVATEL_S3_SHORT
	uint8_t		msglen;			// data length excluding header and CRC
	uint16_t	msgid;
	uint16_t	gpsweek;
	uint32_t	gpsmsec;		// seconds of the week in milliseconds

} span_short_hdr;				// 12 bytes


//---------------------------------------------------------------- LONG binary header
typedef struct _long_binhdr
{
	uint8_t		sync1;		// 1
	uint8_t		sync2;		// 2
	uint8_t		sync3;		// 3
	uint8_t		hdrlen;		// 4
	uint16_t	msgid;		// 6
	uint8_t		msgtype;	// 7
	uint8_t		portaddr;	// 8
	uint16_t  	msglen;		// 10
	uint16_t  	sequence;	// 12
	uint8_t	    iddletime;	// 13
	uint8_t		timestatus; // 14
	uint16_t	gpsweek;	// 16
	uint32_t	gpsmsec;	// 20
	uint32_t	recstatus;	// 24
	uint16_t	_reserved_;	// 26
	uint16_t	swversion;  // 28

} span_long_hdr;



//---------------------------------------------------------------- Use this union to decode header data.
typedef union _span_hdr
{
	span_short_hdr S;
	span_long_hdr  L;

} span_hdr;


inline bool IsShortHeaderTest(const uint8_t* buf)
{
	return ( (buf[0] == NOVATEL_SYNC1) && (buf[1] == NOVATEL_SYNC2) && (buf[2] == NOVATEL_SYNC3_SHORT));
}

inline bool IsLongHeaderTest(const uint8_t* buf)
{
	return (buf[0] ==NOVATEL_SYNC1 && buf[1] == NOVATEL_SYNC2 && buf[2] == NOVATEL_SYNC3_LONG);
}

inline bool IsShortHeader(const uint8_t* hdr) { return ((span_hdr*)hdr)->S.sync3 ==	NOVATEL_SYNC3_SHORT; }
inline bool IsLongHeader(const uint8_t* hdr)  { return ((span_hdr*)hdr)->L.sync3 ==	NOVATEL_SYNC3_LONG; }

inline uint16_t GetMsgId(const uint8_t* hdr)
{
	if (IsShortHeader(hdr))
		return ((span_hdr*)hdr)->S.msgid;

    if (IsLongHeader(hdr))
		return ((span_hdr*)hdr)->L.msgid;

	return 0;
}

inline uint16_t GetRecordLength(const uint8_t* hdr)
{
	if (IsShortHeader(hdr))
	{
		return((uint16_t)((span_hdr*)hdr)->S.msglen ) + sizeof(span_short_hdr) + sizeof(uint32_t);
	}
	if (IsLongHeader(hdr))
	{
		return ((span_hdr*)hdr)->L.msglen  + sizeof(span_long_hdr) + sizeof(uint32_t);
	}

	return 0;
}

/*--------------------------------------------------------------------------------------
 * MESSAGE_IDS
 *--------------------------------------------------------------------------------------*/

typedef enum
{
	VERSION_ID	= 37,		// Include at the start of each log
	TIME_ID		= 101,		// OEM 7 Pag. 927
	TIMESYNC_ID	= 492,		// OEM 7 Pag. 930
	BESTPOS_ID	= 42,		// long header!
	BESTUTM_ID	= 726,		// long header!

	MARK1TIME_ID = 1130,	// MARKxTIME log (OEM7 pag. 663)
	MARK2TIME_ID = 616,
	MARK3TIME_ID = 1075,
	MARK4TIME_ID = 1076,

	MARKPOS_ID	 = 181,		// MARKPOS logs https://docs.novatel.com/OEM7/Content/Logs/MARKxPOS.htm?tocpath=Commands%20%2526%20Logs%7CLogs%7CGNSS%20Logs%7C_____97
	MARK2POS_ID	 = 615,
	MARK3POS_ID	 = 1738,
	MARK4POS_ID	 = 1739,

	//SPAN logs
	INSATTS_ID	 = 319,		// Attitude short header
	INSPOSS_ID   = 321,		// short header
	RAWIMUS_ID	 = 325,		// short header
	RAWIMUSX_ID	 = 1462,	// short header
	INSCONFIG_ID = 1945,
	INSPVAX_ID	 = 1465,
	SETINSTRANSLATION_ID = 1920, // This is a command, reports the current status

} Message_ID_t;

/*--------------------------------------------------------------
 *
 * NOVATEL SPAN IMU REOCRD
 *
 *--------------------------------------------------------------*/

typedef enum t_imu_type
{
	UNKNOWN			= 0 , 	// Unknown IMU type(default)
	HG1700_AG11		= 1 , 	// Honeywell HG1700 AG11
	HG1700_AG17		= 4 , 	// Honeywell HG1700 AG17
	HG1900_CA29		= 5 , 	// Honeywell HG1900 CA29
	LN200			= 8 , 	// Northrop Grumman LN200 / LN200C
	HG1700_AG58		= 11, 	// Honeywell HG1700 AG58
	HG1700_AG62		= 12, 	// Honeywell HG1700 AG62
	IMAR_FSAS		= 13, 	// iMAR iIMU - FSAS
	KVH_COTS		= 16, 	// KVH CPT IMU
	HG1930_AA99		= 20, 	// Honeywell HG1930 AA99
	ISA100C			= 26, 	// Northrop Grumman Litef ISA - 100C
	HG1900_CA50		= 27, 	// Honeywell HG1900 CA50
	HG1930_CA50		= 28, 	// Honeywell HG1930 CA50
	ADIS16488		= 31, 	// Analog Devices ADIS16488
	STIM300			= 32, 	// Sensonor STIM300
	KVH_1750		= 33, 	// KVH1750 IMU
	EPSON_G320		= 41, 	// Epson G320N
	LITEF_MICROIMU	= 52, 	// Northrop Grumman Litef μIMU - IC
	STIM300D		= 56, 	// Safran STIM300, Direct Connection
	HG4930_AN01		= 58, 	// Honeywell HG4930 AN01
	EPSON_G370		= 61, 	// Epson G370N
	EPSON_G320_200HZ= 62, 	// Epson G320N – 200 Hz

}Imu_Type;


typedef struct _raw_imu_short_extended // ID 1462 (OEM7 pag. 1123)
{
	span_short_hdr	Hdr;			// Short header

	uint8_t			imu_info;		// from IMU info enum above
	uint8_t			imu_type;		// from IMU_Type enum above
	uint16_t		gnss_week;
	double			week_seconds;	// This is more precise than milliseconds from header
	uint32_t		imu_status;
	int32_t			z_accel;
	int32_t			_y_accel;		// Minus accel Y
	int32_t			x_accel;
	int32_t			z_gyro;			// change in angle conts right hand
	int32_t			_y_gyro;		// Minus change in angle around Y axis
	int32_t			x_gyro;

	uint32_t		crc;

} RAWIMUSX_t; // size 56 bytes

#define RAWIMUSX_MSG_LENGTH (sizeof(RAWIMUSX_t)-sizeof(span_short_hdr)-sizeof(uint32_t)) /* data length excluding header and CRC */
#define RAWIMUSX_CRC_OFFSET (sizeof(RAWIMUSX_t)-sizeof(uint32_t)) /* Include header for CRC calculation */


// IMU status when handled by SPAN enabled
typedef struct _span_status
{
	uint8_t		ImuType;
	uint16_t	Status;
	bool		SpanEnabled;

} SPAN_STATUS_t, * PSPAN_STATUS_t;

extern volatile SPAN_STATUS_t SpanStatus; // Updated by INSATTS or INSPVAX recordS decoded in novatel.c

int GetSpanStatus(char *bufffer, int size);

/*--------------------------------------------------------------------------------------
 *  TIMESYNC message
 *--------------------------------------------------------------------------------------*/
typedef struct _st_timesinc
{
	span_long_hdr	hdr;			// long header
	uint32_t		week;
	uint32_t		msow;			// milliseconds of the week
	uint32_t		time_status;
	uint32_t		crc;

} TIMESYNCB_t ;

/*------------------------------------------------- 42 - BESTPOS log
 *
 * On systems with SPAN enabled, this log contains the best
 * available combined GNSS and Inertial Navigation System
 *  (if available) position computed by the receiver.
 */
typedef enum __table88_Solution_status
{
	SOL_VALID	= 0,
	SOL_COMPUTED = 0,			//  Solution computed
	INSUFFICIENT_OBS = 1,		//  Insufficient observations
	NO_CONVERGENCE = 2,			//  No convergence
	SINGULARITY = 3,			//  Singularity at parameters matrix
	COV_TRACE = 4,				//  Covariance trace exceeds maximum (trace > 1000 m)
	TEST_DIST = 5,				//  Test distance exceeded (maximum of 3 rejections if distance >10 km)
	COLD_START = 6,				//  Not yet converged from cold start
	V_H_LIMIT = 7,				//  Height or velocity limits exceeded (in accordance with export licensingrestrictions)
	VARIANCE = 8,				//  Variance exceeds limits
	RESIDUALS = 9,				//  Residuals are too large
	INTEGRITY_WARNING = 13,		//  Large residuals make position unreliable
	PENDING = 18,				//  not enough satellites tracking yet (start of receiver)
	INVALID_FIX = 19,			//  The fixed position, entered using the FIX POSITION command, is not
	UNAUTHORIZED = 20			//  Position type is unauthorized - HP or XP on a receiver not authorized for

} SOL_STATUS;



typedef enum __table89_Vel_Tpe
{
	NONE = 0,					//  No solution
	FIXEDPOS = 1,				//  Position has been fixed by the FIX POSITION command
	FIXEDHEIGHT = 2,			//  Position has been fixed by the FIX HEIGHT/AUTO command
	DOPPLER_VELOCITY = 8,		//  Velocity computed using instantaneous Doppler
	SINGLE = 16,				//  Single point position
	PSRDIFF = 17,				//  Pseudorange differential solution
	WAAS = 18,					//  Solution calculated using corrections from an WAAS
	PROPAGATED = 19,			//  Propagated by a Kalman filter without new observations
	OMNISTAR = 20,				//   In addition to a NovAtel receiver with L-Band capability, a subscription for OmniSTAR or use of a DGPS service is required.
	L1_FLOAT = 32,				//  Floating L1 ambiguity solution
	IONOFREE_FLOAT = 33,		//  Floating ionospheric-free ambiguity solution
	NARROW_FLOAT = 34,			//  Floating narrow-lane ambiguity solution
	L1_INT = 48,				//  Integer L1 ambiguity solution
	NARROW_INT = 50,			//  Integer narrow-lane ambiguity solution
	OMNISTAR_HP = 64,			//  a OmniSTAR HP position
	OMNISTAR_XP = 65,			//  a OmniSTAR XP or G2 position
	PPP_CONVERGING = 68,		//  NovAtel CORRECT™ with PPP requires access to a suitable correction stream, delivered either through L-Band orthe Internet.
	PPP = 69,					//   Converged TerraStar-C solution
	OPERATIONAL = 70,			//  Solution accuracy is within UAL operational limit
	WARNING = 71,				//  Solution accuracy is outside UAL operational limit but within warning limit
	OUT_OF_BOUNDS = 72,			//  Solution accuracy is outside UAL limits
	PPP_BASIC_CONVERGING = 77,	//  b Converging TerraStar-L solution
	PPP_BASIC = 78				//  b Converged TerraStar-L solution

}VEL_TYPE;

#define VEL_VALID Vel_Type != 0


typedef struct _st_BestPos
{
	span_long_hdr	H;			// long header

	int32_t			SolStatus;	// Solution status (table 88)
	int32_t			PosType;	// Position type (table 89)
	double			Lat;		// Latitude in degress
	double			Lon;		// longitud in degrees
	double			Hgt;		// elevation in ASL [m]
	float			undulation; // undulation between geoid and elipsoid [m]
	uint32_t		datum_id;		// datum ID (table25)
	float			sd_lat;		// Standard deviations for lat,lon, and hgt:
	float			sd_lon;
	float			sd_hgt;
	char			stnid[4];	// Station ID
	float			dif_age;	// Diferential age [sec]
	float			sol_age;	// solution age [sec]
	uint8_t			nSVs;		// number of statellites tracked
	uint8_t			nsolSVs;	// # of sats used in solution
	uint8_t			nsolL1;		// # of sats in solution L1 etc
	uint8_t			nsolMult;	// # of  sats used in multi freq. solution
	uint8_t			_reserved_;
	uint8_t			xt_sol_stat; // extended solution status
	uint8_t			Ga_Be_mask;	 // Galileo and BeiDou signal mask
	uint8_t			GP_GL_mask;	 // GPS and GLonas signal mask

	uint32_t		crc;

} BESTPOS_t;

//------------------------------------------------- 726 - BESTUTM log
typedef enum
{
	WGS84	= 61,
	USER	= 63,
} Datum_ID;


typedef struct _st_BestUTM
{
	span_long_hdr	H;			// short header

	int32_t			SolStatus;	// Solution status (table 88)
	int32_t			PosType;	// Position type (table 89)
	uint32_t		ZoneNr;		// Longitude Zone
	uint32_t		ZoneLetter;	// Latitud zone Letter (N/S)
	double			Northing;	// Y coordinate [m]
	double			Easting;	// X coordinate	[m]
	double	 		Hgt;		// Height coordinate [m]
	float			Undulation;	// Undulation of the geoid [m]
	uint32_t		DatumID;	// from Datum_ID enum
	float			Nstdev;		// North std deviation
	float			Estdev;		// East "
	float			Hstdev;		// Height "
	char			StationID[4]; // Base station ID (cast to 4x chars)
	float			DifAge;		// Differential age [sec]
	float			SolAge;		// solution age [sec]
	uint8_t			SVs;
	uint8_t			SolSV;
	uint8_t			ggL1;
	uint8_t			solMultiSV;
	uint8_t			reserved;
	uint8_t			extSolStat;	// Extended solution status (bit fiedl)
	uint8_t			GalBeidStat;
	uint8_t			GPSGloMask;

	uint32_t		crc;

} BESTUTM_t;


/*------------------------------------------------- 42 - INSPOS log
 * NOTE:
 * On systems with SPAN enabled, BESTPOS log contains the best available
 * combined GNSS and INS (If available) position computed by the receiver.
 */


typedef enum _ins_status
{
	INS_INACTIVE		  	= 0,   // IMU logs are present, but the alignment routine has not started; INS is inactive.
	INS_ALIGNING		  	= 1,   // INS is in alignment mode.
	INS_HIGH_VARIANCE		= 2,   // The INS solution uncertainty contains outliers and the solution may be outside specifications.1 The solution is still valid but you should monitor the solution uncertainty in the INSSTDEV log. It may be encountered during times when GNSS is absent or poor.
	INS_SOLUTION_GOOD		= 3,   // The INS filter is in navigation mode and the INS solution is good.
	INS_SOLUTION_FREE		= 6,   // The INS Filter is in navigation mode and the GNSS solution is suspected to be in error. The inertial filter will report this status when there are no available updates (GNSS or other) being accepted and used.
	INS_ALIGNMENT_COMPLETE	= 7,   // The INS filter is in navigation mode, but not enough vehicle dynamics have been experienced for the system to be within specifications.
	DETERMINING_ORIENTATION	= 8,   // INS is determining the IMU axis aligned with gravity.
	WAITING_INITIALPOS		= 9,   // The INS filter has determined the IMU orientation and is awaiting an initial position estimate to begin the alignment process.
	WAITING_AZIMUTH		  	= 10,  // The INS filer has orientation, initial biases, initial position and valid roll/pitch estimated. Will not proceed until initial azimuth is entered.
	INITIALIZING_BIASES		= 11,  // The INS filter is estimating initial biases during the first 10 seconds of stationary data.
	MOTION_DETECT		   	= 12,  // The INS filter has not completely aligned, but has detected motion.
	WAITING_ALIGNMENTORIENTATION = 14, // The INS filter is waiting to start alignment until the current Vehicle Frame

} INS_STATUS;

typedef struct _st_insposs
{
	span_short_hdr	H;			// short header

	uint32_t		gps_week;	// GPS week number
	double			gps_sow;	// GPS Seconds of the Week

	double			Lat;		// Latitude in degress
	double			Lon;		// longitud in degrees
	double			Hgt;		// elevation in ASL [m]
	uint32_t		InsStatus; 	// from INSSTATUS_t

	uint32_t		crc;

} INSPOSS_t;

typedef struct _st_insatts		// INS Attitude log
{
	span_short_hdr	H;			// short header

	uint32_t		gps_week;	// GPS week number
	double			gps_sow;	// GPS Seconds of the Week

	double			Roll;		// Right handed rotation of Y
	double			Pitch;
	double			Azimuth;	// LEFT handed rotation around Z
	uint32_t		InsStatus; 	// from INSSTATUS_t

	uint32_t		crc;

} INSATTS_t;


typedef struct _st_inspvax		// INS Attitude log
{
	span_long_hdr	H;			// 28 short header

	uint32_t		InsStatus; 	// 4 from INSSTATUS_t
	uint32_t		PosType;	// 4 Position or Velocity
	double			Lat;		// 8
	double 			Lon;		// 8
	double			Hgt;		// 8
	float			Undulation;	// 4
	double			NorthVel;	// 8
	double 			EastVel;	// 8
	double			UpVel;		// 8
	double			Roll;		// 8 Right handed rotation of Y
	double			Pitch;		// 8
	double			Azimuth;	// 8 LEFT handed rotation around Z
	float			sigmaLat;	// 4
	float			sigmaLon;	// 4
	float			sigmaHgt;	// 4
	float			sigmaNvel;	// 4
	float			sigmaEvel;	// 4
	float			sigmaUpVel;	// 4
	float			sigmaRoll;	// 4
	float			sigmaPitch;	// 4
	float			sigmaAzimuth; // 4
	uint32_t		ExtSolStatus; // 4
	uint16_t		ElapsedTime;  // 2 Seconds since last ZUPT

	uint32_t		crc;		 // 4

} INSPVAX_t; // 158 Bytes



/*--------------------------------------------------------------------------------------
 * MARKxTIME and MARKPOS messages
 *--------------------------------------------------------------------------------------*/

typedef enum
{
	CM_VALID = 0,
	CM_CONVERGING = 1,
	CM_ITERATING = 2,
	CM_INVALID = 3

} clock_model_status;


typedef struct _st_mark_time
{
	span_long_hdr	H;				// long header

	uint32_t		gps_week;		// GPS week number
	double			seconds;		// GPS Seconds of the Week of the event
	double			offset;			// Receiver clock offset. GPS system time = GPS_Ref_Time(from header) - offset;
	double			offset_std;
	double			utc_offset;		// UTC = GPS_Ref_Time(from header) - offset + UTC_offset;
	uint32_t		cm_status;		// clock model status  0 = valid, 1 = converging, 2 = iterating. 3 = invalid

	uint32_t		crc;

} MARKTIME_t;

/* MARKPOS
 * The position at the mark input pulse is extrapolated using the last valid position and velocities.
 * The latched time of mark impulse is in GPS reference weeks and seconds into the week.
 * The resolution of the latched time is 10 ns.
 */

typedef struct _st_mark_POS
{
	span_long_hdr	H;				// long header

	uint32_t		SolStatus;
	uint32_t		PosType;
	double			Lat;
	double 			Lon;
	double			Hgt;
	float			Undulation;
	uint32_t		DatumID;
	float			sigmaLat;
	float			sigmaLon;
	float			sigmaHgt;
	char			StationID[4];
	float			DifAge;
	float			SolAge;
	uint8_t			NSats;
	uint8_t			NSatsSol;
	uint8_t			NggL1;
	uint8_t			NSatsMulti;
	uint8_t			_reserved_;
	uint8_t			ExtSolStat;
	uint8_t			GalBeiMask;
	uint8_t			GpsGloMask;

	uint32_t		crc;

} MARKPOS_t;


/*--------------------------------------------------------------------------------------
 *  NOVATEL IMR (IMU neutral) message format
 *--------------------------------------------------------------------------------------*/

#ifndef INS_t_DEFINED
#define INS_t_DEFINED
	typedef struct tINS_t
	{
		double Time; 			// 8  - GPS time frame – seconds of the week
		int32_t dAng[3];		// 12 - delta theta or angular rate depending on flag in the header
		int32_t dVel[3];		// 12 - delta Velocity or acceleration depending on the the flag in the header.

	} INS_t;					// 32 bytes
#endif

/*------------------------------------------------------------------------------------
 * Novatel CRC functions
 *------------------------------------------------------------------------------------*/
enum _CRC_Constants
{
	CRC32_SIZE	=	sizeof(uint32_t),
	CRC32_INIT	= 0,

};

uint32_t CalculateBlockCRC32(uint32_t ulCount, const uint8_t* ucBuffer);
uint32_t CalculateCharacterCRC32(uint8_t ucChar_, uint32_t ulCRC);

/*------------------------------------------------------------------------
 * WaitOEM7700SignIn() - waits for OEM7700 wakeup (sends [COMx] to the ports
 *------------------------------------------------------------------------*/
bool WaitOEM7700SignIn(uint32_t max_msec);

/*------------------------------------------------------------------------
 * InitializeOEM7700() - sends required initialization strings to OEM7700
 *------------------------------------------------------------------------*/
bool InitializeOEM7700();

/*-----------------------------------------------------------------------------
 * ComposeMark1OutCommand() - Creates an EVENTOUTn (MARKn) command used to trigger IMU reads
 *
 * mark
 * freq = frequency in Hertz
 * uswdt = pulse width
 * polarty : true = Positive, false = Negative
 *
 *----------------------------------------------------------------------------*/
int ComposeMarkOutCommand(char* pbuf, size_t size, uint16_t mark, uint32_t freq, uint32_t uswdt, bool polarity);

/*-----------------------------------------------------------------------------
 * ComposeMark1Disable() - creates an EVENT_OUT_1 (MARK1) disable command
 *----------------------------------------------------------------------------*/
int ComposeMarkOutDisable(char* pbuf, size_t size, uint16_t mark);

typedef void(*pnotify_time)(uint16_t week, uint32_t sow);

void RegisterNovatelTimeNotify(pnotify_time fn);

/************************************************************************************
 *
 * SIDEBAND REPORTING
 *
 ***********************************************************************************/
#define MIN_POSITION_REPORT_PERIOD_MSEC	100 /* 10Hz is the max position reportig */

typedef enum
{
	POSTYPE_INVALID,
	POSTYPE_GEO,
	POSTYPE_UTM,

}PosType_t;


typedef struct _st_position		// internal position record for sideband reporting
{
	uint16_t GpsWeek;
	double   WeekSeconds;
	double	 Lat;
	double   Lon;
	double   Hgt;
	PosType_t PosValid;

} position_t, * pposition_t;

extern volatile position_t LastPosition;

void InterlockPosCopy(pposition_t dest, pposition_t source);

size_t PrintGeoCoordinates(char* buffer, size_t cnt, pposition_t pos );


typedef struct _st_attitude
{
	double		NorthVel;
	double 		EastVel;
	double		UpVel;
	double		Roll;
	double 		Pitch;
	double		Azimuth;

	uint16_t	InsStatus;

} attitude_t, * pattitude_t;

extern volatile attitude_t LastAttitude;

void InterlockCopyAzimuth(pattitude_t dest, pattitude_t source);

/*------------------------------------------------------------------------------
 *  RegisterMarkTimeNotify() - Sends MARTIMEx (x[1-4]) command and reports to the callback
 *------------------------------------------------------------------------------*/

typedef struct 	_st_marktime_rep	// REPORT STRUCT, NOT A NOVATEL RECORD
{
	uint16_t	Mark;				// indicator of Marktime 1, 2, 3, etc
	uint16_t 	EventNr;			// event counter
	uint16_t 	Week;
	double 		EventTime;			// week seconds of the event

} marktime_rep_t, * pmarktime_rep_t;


typedef void(*pnotify_marktime)(marktime_rep_t mark);

// MARKCONTROL MARK1 ENABLE NEGATIVE 0 25 // for events up to 40Hz
void RegisterMarkTimeNotify(uint16_t mark_nr, bool polarity, pnotify_marktime callback );

void CancelMarkTimeNotify(uint16_t mark_nr);

int FormatMarkTimeRecord(char*buffer, size_t cnt, pmarktime_rep_t prec);


/*------------------------------------------------------------------------------
 *  RegisterMarkPoseNotify() - Sends MARKPOSx (x[2-4]) command and reports to the callback
 *------------------------------------------------------------------------------*/

typedef struct _st_markpos_rep
{
	uint16_t	Mark;			// indicator of Marktime 1, 2, 3, etc
	uint16_t 	EventNr;		// event counter
	uint16_t 	Week;
	double 		EventTime;		// week seconds of the event (comes from the header of MARKPOS)
	double 		Lat;
	double 		Lon;
	double		Hgt;

} markpos_rep_t, * pmarkpos_rep_t;

typedef void(*pnotify_markpos)(markpos_rep_t mark);

void RegisterMarkPosNotify(uint16_t mark_nr, bool polarity, pnotify_markpos callback );
void CancelMarkPosNotify(uint16_t mark_nr);
int  FormatMarkPosRecord(char*buffer, size_t cnt, pmarkpos_rep_t prec);

void SendStartLoggingMessages();
//bool SendStoptLoggingMessages();


bool MapLadyBugLogs(const char* port); 	// returns true on error
bool RequestGpsLog(const char* log);		// returns true on error

#pragma pack(pop)

#endif /* NOVATEL_H_ */
