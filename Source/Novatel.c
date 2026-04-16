/*
 * Novatel.c
 *
 *  Created on: Nov 4, 2021
 *      Author: Guillermo
 *
 * Rev: 3.1 - Oct. 2024
 */

#include "common.h"


//#define DEBUG_OEM7700_ISR
//#define DEBUG_OEM7700_TP		_TP1


/* GPS_Week and GPS_WeekSeconds are updated by
 * TimeSyc with the PPS pulse.
 */
volatile uint16_t GPS_Week = 0;
volatile uint32_t GPS_WeekSeconds = 0;

volatile position_t LastPosition =
{
	.GpsWeek =0,
	.WeekSeconds = 0,
	.Lat = 0,
	.Lon = 0,
	.Hgt = 0,
	.PosValid = POSTYPE_INVALID,
};


volatile bool poslock = false;

void InterlockPosCopy(pposition_t dest, pposition_t source)
{
	while (poslock) ; // sync reentrance

	poslock = true;
	memcpy(dest, source, sizeof(position_t));
	poslock = false;
}

/*
volatile attitude_t LastAttitude =
{
	.NorthVel = 0,
	.EastVel = 0,
	.UpVel = 0,
	.Roll = 0,
	.Pitch = 0,
	.Azimuth = 0,

	.InsStatus = INS_INACTIVE
};
*/

volatile SPAN_STATUS_t SpanStatus =
{
	.ImuType = IMUType_INVALID,
	.Status = INS_INACTIVE,
	.SpanEnabled = false
};

static char * ImuTypeName[] =
{
	"UNKNOWN",
	"FSAS",
	"STIM-300",
	"KVH-1750",
};


volatile bool lockAtt = false;

void InterlockCopyAttitude(pattitude_t dest, pattitude_t source)
{
	while(lockAtt);	// hold until previous call exits
	lockAtt = true;
	memcpy(dest, source, sizeof(attitude_t));
	lockAtt = false;
}



const char* const OEM7700_Basic[] =
{
	//"SERIALCONFIG COM2 230400 N 8 1 N OFF",  				// COM2 must be externally configured for this to work
	//"UNLOGALL COM1 TRUE",									// TRUE also removes RXSTATUSEVENTA.
	//"UNLOGALL COM3 TRUE",
	//"UNLOGALL USB1 TRUE",
	//"UNLOGALL USB2 TRUE",
	//"UNLOGALL USB3 TRUE",

	"UNLOG COM1 RXSTATUSEVENTA",							// Remove unwanted logs from NMEA for  LiDAR
	//"UNLOG COM2 RXSTATUSEVENTA",							// Passthrough to logs
	"UNLOG COM4 RXSTATUSEVENTA",							// Not wanted for Phase One
	"PPSCONTROL ENABLE_FINETIME POSITIVE",					// PPS enabled positive (1ms pulse)
	"MAGVAR AUTO",											// Bearing Magnetic variation correction https://docs.novatel.com/OEM7/Content/Commands/MAGVAR.htm?Highlight=MAGVAR

	"SERIALCONFIG COM1 115200 N 8 1 N OFF",				// RS232 FOR NMEA output
	"LOG COM1 GPGGA ONTIME 1 HOLD",						// PPS NMEA
	"LOG COM1 GPZDA ONTIME 1 HOLD",						// PPS NMEA
	"LOG COM1 GPVTG ONTIME 1 HOLD",						// PPS NMEA

	"LOG COM2 TIMESYNCB ONTIME 1",							// we need this one to sync PPS time seconds counter
	"LOG COM2 INSATTS ONTIME 1"								// Get SPAN status and attitude angles (if available)
};

/* With non SPAN enabled receivers:
 * The COM2 logs are the sent by PSOC to the CPU
 * along with RAWIMUSXB logs from FSAS IMU processed
 * by PSOC.
 *
 * With SPAN enabled receivers the IMU records are
 * processed by the receiver and sent along with
 * the GNSSS records. PSOC is a pass through intermediary.
 * COM3 is the IMU port for OEM7700 receivers with SPAN.
 */

const char* const NovAtel_Logs[] =
{
	// COM2 must be externally configured for this to work
	// "SERIALCONFIG COM2 230400 N 8 1 N OFF",

	//"UNLOGALL USB1 TRUE",
	//"UNLOGALL USB2 TRUE",

	"UNLOG COM1 RXSTATUSEVENTA",							// Remove unwanted
	"UNLOG COM2 RXSTATUSEVENTA",							// We have to respect user logs set in this
	"UNLOG COM4 RXSTATUSEVENTA",							// We have to respect user logs set in this

	"PPSCONTROL ENABLE_FINETIME POSITIVE",					// PPS enabled positive (1ms pulse)
	"MAGVAR AUTO",											// Bearing Magnetic variation correction https://docs.novatel.com/OEM7/Content/Commands/MAGVAR.htm?Highlight=MAGVAR

	"LOG COM2 TIMESYNCB ONTIME 1",
	"LOG COM2 TIMEB ONTIME 1",
	"LOG COM2 RXSTATUSB ONCHANGED",
	"LOG COM2 RAWEPHEMB ONNEW",
	"LOG COM2 GPSEPHEMB ONNEW",
	"LOG COM2 BDSEPHEMERISB ONNEW",
	"LOG COM2 GLOEPHEMERISB ONNEW",
	"LOG COM2 GALFNAVEPHEMERISB ONNEW",
	"LOG COM2 GALINAVEPHEMERISB ONNEW",
	"LOG COM2 HEADING2B ONNEW",
	"LOG COM2 RANGECMP4B ONTIME 1",
	"LOG COM2 BESTPOSB ONTIME 0.1",		// Request BestPos @ 10Hz for real time monitor
};

/* for SPAN enabled receivers */
const char* const NovAtel_SPAN_Logs[] =
{
	"LOG COM2 INSCONFIGB ONCHANGED",			// IMU configuration
	"LOG COM2 INSUPDATESTATUSB ONCHANGED",		// IMU configuration
	"LOG COM2 INSPVAXB ONTIME 1",				// Position Velocity and attitude
	"LOG COM2 RAWIMUSXB ONNEW",					// Log all IMU records
	//	"LOG COM2 INSATTSB ONTIME 1",				// N
};


const char* const LadyBug_Logs[] =
{

	"UNLOGALL %s TRUE",
	"LOG %s GPGGA ONTIME 1",
	"LOG %s GPZDA ONTIME 1",
	"LOG %s GPVTG ONTIME 1",
};

typedef struct _storeComand
{
	uint16_t cmdID;
	const char* coomand;
} StoreComand;



StoreComand NovAtel_SPAN_LogOnce[] =	// Logs sent at each start logging session
{
	{ VERSION_ID, 			"LOG COM2 VERSIONB ONCE"},
	{ INSCONFIG_ID, 		"LOG COM2 INSCONFIGB ONCE"},
	{ SETINSTRANSLATION_ID, "LOG COM2 SETINSTRANSLATIONB ONCE"},
};


StoreComand NovAtel_LogOnce[] =	// Logs sent at each start logging session
{
	{ VERSION_ID, 			"LOG COM2 VERSIONB ONCE"},
};


typedef enum OEM7700_Answer
{
	NO_ANSWER,
	OEM7700_OK,
	OEM7700_ERROR,
	OEM7700_TIMEOUT,
	OEM7700_SIGNIN,
	OEM7700_UNKNOWN,

} OEM7700_Answer;

const char pingString[]  = "ECHO OFF\r";
const char strAnsOK[]    = "<OK";
const char strAnsERROR[] = "<ERROR";
const char cmdTerminator[] = "\r";


/****************************************************************************
 *
 * LogNovatelMessage()	- Stores the record IF logging is active
 *
 ****************************************************************************/
uint32_t GNSS_Rec_Count = 0;
uint16_t gnss_seq = 0;

volatile bool Enable_GNSS_Logging = false;

/*---------------------------------------------------------------------
 * LogNovatelMessage() - Called from ISR or message processing functions
 */
void LogNovatelMessage(uint8_t* buf, uint32_t size)
{
	if (LoggingPort != 0 )
	{
		#ifdef _DEBUG
			gnss_seq++;

			/* for debug purposes only
			 * THE CRC MUST BE REGENERATED!
			 */
		   if (IsLongHeader(buf))
			   ((span_hdr*)buf)->L.sequence = gnss_seq;

		   uint32_t lc = size-4;
		   uint32_t crc = CalculateBlockCRC32(lc, buf);
		   *((uint32_t*)(buf + lc )) = crc;

		   if ( CalculateBlockCRC32(size, buf) != 0)
		   {
			   printf("CRC Error!");
				return;
		   }
		#endif

	   if (Enable_GNSS_Logging)
		   Send_GNSS_Record(buf,size);
	}
}


/*-------------------------------------------------------------------------
 *  ProcessCommandAnswer()
 *
 * Interprets OEM7700 response
 *-------------------------------------------------------------------------*/
OEM7700_Answer oem7700_answer = NO_ANSWER;

#define OEM_BUF_SIZE 256

uint8_t MsgBuffer[OEM_BUF_SIZE];

bool process_answer = false;

void ProcessCommandAnswer(uint8_t* buf, size_t size )
{
	 process_answer = false;

	if ( size > 0 )
	{
		size_t cnt = size < OEM_BUF_SIZE? size: OEM_BUF_SIZE;

		memcpy(MsgBuffer, buf, cnt);

		if ( strstr((char*)buf, strAnsOK) != NULL ) // exclude null terminator
		{
			oem7700_answer = OEM7700_OK;
		}
		else if ( strstr((char*)buf, strAnsERROR) != NULL )
		{
			oem7700_answer = OEM7700_ERROR;
		}
		else
		{
			oem7700_answer = OEM7700_UNKNOWN;
		}
	}
}

int delaycnt = 0;

int WaitOemAnswer(uint32_t max_msec)
{
	delaycnt = 0;

	while (oem7700_answer == NO_ANSWER)
	{
		cyhal_system_delay_ms(10);
		delaycnt += 10;

		if (delaycnt > max_msec)
			return OEM7700_TIMEOUT;
	}

	return oem7700_answer;
}


extern uint16_t LastStoredRecord; // forward reference

int WaitForRecord(uint16_t msg_id)
{
	delaycnt = 0;

	do {
		cyhal_system_delay_ms(10);
		delaycnt += 10;

		if (LastStoredRecord == msg_id)
			return OEM7700_OK;

	} while (delaycnt < 500 );

	return OEM7700_TIMEOUT;
}


OEM7700_Answer SendOEM7700CommandWithRetry(const char* str, int retries)
{
	int result = OEM7700_UNKNOWN;
	int retry = 0;

	char tmpbuf[120];

	do {
		printf("%40s", str);

		size_t len = snprintf(tmpbuf,ARRAY_SIZE(tmpbuf),"%s%s",str,cmdTerminator);

		if (len < 0 || len >= ARRAY_SIZE(tmpbuf) )
		{
			printf(" - String TOO LONG!\n" );
			return NO_ANSWER;
		}

		oem7700_answer = NO_ANSWER;
		process_answer = true;
		OEM7700_SendString(tmpbuf);
		result = WaitOemAnswer(500);

		switch (result)
		{
			case OEM7700_OK:
				printf(" - OK in %d ms\n", delaycnt );
				return result;
				break;

			case OEM7700_ERROR:
				printf(" - OEM7700_ERROR in  %d ms\n{%s}\n",  delaycnt, MsgBuffer );
				return result;
				break;

			case OEM7700_TIMEOUT:
				printf(" - OEM7700_TIMEOUT in %d ms\n", delaycnt );
				break;

			case OEM7700_UNKNOWN:
				printf(" - UNKNOWN answer in %d ms\n{%s}\n", delaycnt, MsgBuffer );
				break;
		}

		cyhal_system_delay_ms(250);		// delay 1/4 seg and retry
		retry++;
		printf("\nRetry %d - %s\n", retry, str );

	} while (retry < retries);

	return result;
}

/* Semd comands and ignore result */
void Send_Command_List_with_retry(const char* const* list, int count)
{
	for (int i = 0; i < count; i++)
	{
		SendOEM7700CommandWithRetry( list[i],3);
	}
}

/* Semd comands and test result ends at first failed command */
bool Send_Command_List_with_test(const char* const* list, int count)
{
	for (int i = 0; i < count; i++)
	{
		if ( SendOEM7700CommandWithRetry( list[i],3) != OEM7700_OK )
			return false;
	}
	return true;
}


/*------------------------------------------------------------------------
 * WaitOEM7700SignIn() - waits for OEM7700 wakeup (sends [COMx] to the ports
 *------------------------------------------------------------------------*/
bool OEM7700_SignIn = false;

bool WaitOEM7700SignIn(uint32_t max_msec)
{
	uint32_t msecs=0;
	char tmpbuf[100];

	SetOEM7700dMsgProcessor(Read_Novatel_Message);

	OEM7700_SignIn = false;
	oem7700_answer = NO_ANSWER;
	process_answer = true;

	do {
		OEM7700_SendString( pingString );

		if ( WaitOemAnswer(1000) != OEM7700_TIMEOUT )  // wait for any answer
		{
			msecs += delaycnt;
			OEM7700_SignIn = true;
			break;
		}
		msecs += delaycnt;

	} while (msecs < max_msec);

	if( OEM7700_SignIn == true )
	{
		snprintf(tmpbuf,ARRAY_SIZE(tmpbuf),"OEM7700 Signed IN - in %lu ms\n",msecs);
		PrintWithTime(tmpbuf);
	}
	else
	{
		PrintWithTime("OEM7700 not detected\n===> Set SERIALCONFIG COM2 230400 N 8 1 N OFF <===\n");

	}
	return OEM7700_SignIn;
}


/*------------------------------------------------------------------------
 * InitializeOEM7700() - sends required initialization strings to OEM7700
 *------------------------------------------------------------------------*/
bool InitializeOEM7700()
{
	bool retval = false;

	if ( !OEM7700_SignIn )
	{
		PrintWithTime("OEM7700 is Offline. Skipping Initialization.\n");
		return false;
	}
	SetOEM7700dMsgProcessor(Read_Novatel_Message);

	if (SysConfig.no_com2_logs_init == false)  // FOR DEBUG
	{
		PrintWithTime("Initializing NovAtel full config with logs...\n");
		Send_Command_List_with_retry(NovAtel_Logs,  ARRAY_SIZE(NovAtel_Logs));

		if(Send_Command_List_with_test(NovAtel_SPAN_Logs,   ARRAY_SIZE(NovAtel_SPAN_Logs))== true)
		{
			SysConfig.enable_ins = true;
			SpanStatus.SpanEnabled = true;
			PrintWithTime("SPAN Enabled receiver\n");
		}
		else
		{
			SysConfig.enable_ins = false;
			SpanStatus.SpanEnabled = false;
			PrintWithTime("SPAN not available\n");
		}
	}
	else
	{
		PrintWithTime("Initializing NovAtel basic config...\n");
		Send_Command_List_with_retry( OEM7700_Basic,  ARRAY_SIZE(OEM7700_Basic));
	}
	/* no need to handle INS logs, BESTPOS will use it if available
	*/
	return retval;
}


OEM7700_Answer SendOEM7700CommandToStore(const StoreComand* SC)
{
	int result = OEM7700_UNKNOWN;

	LastStoredRecord = 0;
	result = SendOEM7700CommandWithRetry(SC->coomand,3);

	if (result == OEM7700_OK )
	{
		result = WaitForRecord( SC->cmdID );

		switch ( result )
		{
		case OEM7700_OK:
			{
				printf("%40s\n", "RECORD STORED OK" );
				break;
			}
			default:
				printf("%40s\n", "ERROR: RECORD NOT NOT RECEIVED" );
				break;
		}
	}

	return result;
}


/*-------------------------------------------------------------
 * MapLadyBugLogs() - Sends the list of LadyBug logs
 * port - String with valid NovAtel port name
 *
 * returns: true on error
 */
bool MapLadyBugLogs(const char* port)
{
	char buffer[100];

	int result;

	for (int i=0; i < ARRAY_SIZE(LadyBug_Logs); i++)
	{
		snprintf(buffer, sizeof(buffer), LadyBug_Logs[i], port);
		result = SendOEM7700CommandWithRetry(buffer,3);
		if (result != OEM7700_OK)
			return true;
	}
	return false;
}

/*--------------------------------------------------------------
 * RequestLog() - sends a log comand to NovAael.
 * log - should not include CR or LF
 *
 * returns true on error;
 */
bool RequestGpsLog(const char* log)
{
	return SendOEM7700CommandWithRetry(log,3) != OEM7700_OK;
}



/*-------------------------------------------------------------------------------
 * SendStartLoggingMessages()
 * Logs to start at the beginning of each log session.
 * These commands will be stored ahead in the send ring buffer;
 *------------------------------------------------------------------------------*/
void SendStartLoggingMessages()
{
	Purge_GNSS_Buffer();

	gnss_seq = 0;

	if (SysConfig.enable_ins == true)
	{
		for (int i = 0; i < ARRAY_SIZE(NovAtel_SPAN_LogOnce); i++)
		{
			SendOEM7700CommandToStore(NovAtel_SPAN_LogOnce + i );
			cyhal_system_delay_ms(100);
		}
	} else {

		for (int i = 0; i < ARRAY_SIZE(NovAtel_LogOnce); i++)
		{
			SendOEM7700CommandToStore(NovAtel_LogOnce + i );
			cyhal_system_delay_ms(100);
		}
	}

}


/*-----------------------------------------------------------------------------
 * ComposeMarkOutCommand() - Creates an EVENTOUT (MARK1 to MARK4) command
 *
 * mark - 1 to 4 for MARK1 to MARK4
 * freq = frequency in Hertz
 * uswdt = pulse width in microseconds
 * polarty : true = Positive, false = Negative pulse
 * 
 *----------------------------------------------------------------------------*/
int ComposeMarkOutCommand(char* pbuf, size_t size, uint16_t mark, uint32_t freq, uint32_t uswdt, bool polarity)
{
	uint32_t period = 1000000000UL / freq;
	uint32_t pulse_wdt = uswdt * 1000UL;

	return snprintf(pbuf, size, "EVENTOUTCONTROL MARK%d ENABLE %s %lu %lu\r", mark,
			(polarity? "POSITIVE": "NEGATIVE"), pulse_wdt, period - pulse_wdt);
}


/*-----------------------------------------------------------------------------
 * ComposeMark1Disable() - creates an EVENT_OUT_1 (MARK1) disable command
 *----------------------------------------------------------------------------*/
int ComposeMarkOutDisable(char* pbuf, size_t size, uint16_t mark)
{
	return snprintf(pbuf, size, "EVENTOUTCONTROL MARK%d DISABLE\r", mark);
}


/****************************************************************************
 * GetNovatelStatusStr()
 ****************************************************************************/

volatile GNSS_Status nStatus =
{
	.Health = Status_OK,
	.ReciverStatus = 0,
	.TimeStatus = TIME_STATUS_UNKNOWN,
	.TimeSync = false,
	.CRCErrors = 0
};

static dictionary_t TimeStatusDictionary[] =
{
	{ TIME_STATUS_FINESTEERING, "FINESTEERING" },
	{ TIME_STATUS_SATTIME, "SATTIME" },
	{ TIME_STATUS_FINEBACKUPSTEERING ,"FINEBACKUPSTEERING" },
	{ TIME_STATUS_FINE ,  "FINE" },
	{ TIME_STATUS_FINEADJUSTING , "FINEADJUSTING"  },
	{ TIME_STATUS_FREEWHEELING ,   "FREEWHEELING"},
	{TIME_STATUS_COARSESTEERING  , "COARSESTEERING" },
	{ TIME_STATUS_COARSE , "COARSE" },
	{ TIME_STATUS_COARSEADJUSTING , "COARSEADJUSTING" },
	{TIME_STATUS_APPROXIMATE  , "APPROXIMATE" },
};

//ARRAY_SIZE(TimeStatusDictionary)

static dictionary_t InsStatusDictionary[] =
{
	{ INS_INACTIVE, "INS_INACTIVE" },
	{ INS_ALIGNING, "INS_ALIGNING" },
	{ INS_HIGH_VARIANCE, "INS_HIGH_VARIANCE" },
	{ INS_SOLUTION_GOOD, "INS_SOLUTION_GOOD" },
	{ INS_SOLUTION_FREE, "INS_SOLUTION_FREE" },
	{ INS_ALIGNMENT_COMPLETE, "INS_ALIGNMENT_COMPLETE" },
	{ DETERMINING_ORIENTATION, "DETERMINING_ORIENTATION" },
	{ WAITING_INITIALPOS, "WAITING_INITIALPOS" },
	{ WAITING_AZIMUTH, "WAITING_AZIMUTH" },
	{ INITIALIZING_BIASES, "INITIALIZING_BIASES" },
	{ MOTION_DETECT, "MOTION_DETECT" },
	{ WAITING_ALIGNMENTORIENTATION, "WAITING_ALIGNMENT" },
};


int GetSpanStatus(char *buffer, int size)
{
	return snprintf(buffer, size, "IMU Type: %s, SPAN Status: %s\n",
		ImuTypeName[SpanStatus.ImuType],
		 SearchDictionary(InsStatusDictionary, ARRAY_SIZE(InsStatusDictionary), SpanStatus.Status, "UNKNOWN")
		);
}


size_t GetNovatelStatus(char* buffer, size_t cnt, bool short_status)
{
	GNSS_Status S;

	memcpy( &S, (GNSS_Status*) &nStatus, sizeof(GNSS_Status) );

	if (short_status == true)
	{
		char stat_ch = 'K';

		switch(S.ReciverStatus)
		{
			case CRC_Error:
				stat_ch = 'C';
				break;
			case HW_Error:
				stat_ch = 'H';
				break;
		}

		return snprintf(buffer, cnt,"%c,%.8Xh,%d,%c,%d,%ld,%ld,%lu,%d",
				stat_ch,
				S.ReciverStatus,
				S.TimeStatus,
				( S.TimeSync== true? 'S' : 'N'),
				GPS_Week,
				GPS_WeekSeconds,
				GNSS_Rec_Count,
				S.CRCErrors,
				SpanStatus.SpanEnabled
				);
	}else {

		char* stat_str = "";

		switch(S.ReciverStatus)
		{
			case CRC_Error:
				stat_str = "CRC Error";
				break;
			case HW_Error:
				stat_str = "HW Error";
				break;
			default:
				stat_str = "OK";
				 break;
		}

		const char *tsstr = SearchDictionary(TimeStatusDictionary, ARRAY_SIZE(TimeStatusDictionary), S.TimeStatus, "UNKNOWN");
		const char *instr = SearchDictionary(InsStatusDictionary, ARRAY_SIZE(InsStatusDictionary), SpanStatus.Status, "UNKNOWN");

		return  snprintf(buffer, cnt,
			"GNSS: %s,Status: %.8Xh,%s,%s,Records: %lu,CRC: %lu\n"
			"Week:Seconds: [%d:%ld]\n INS: %s\n",
			stat_str,
			S.ReciverStatus,
			tsstr,
			(S.TimeSync != 0? "TimeSync": "NoTimeSync"),
			GNSS_Rec_Count,
			S.CRCErrors,
			GPS_Week,
			GPS_WeekSeconds,
			instr
		);
	}
}

void ClearNovatelStatus()
{
	nStatus.CRCErrors = 0;
	nStatus.Health = Status_OK;
}

/********************************************************************************************
 * PrintCoordinates() - for Position reports
*********************************************************************************************/

size_t PrintGeoCoordinates(char* buffer, size_t cnt, pposition_t pos )
{
	// grab a copy of the last known position

	char pvalid = pos->PosValid == POSTYPE_INVALID? '?' : 'G';

	// prefix #POS = Position report
	return snprintf(buffer, cnt,"#POS,%c,%.8lf,%.8lf,%.3lf,%d,%.6lf\n",
			pvalid,
			pos->Lat,
			pos->Lon,
			pos->Hgt,
			pos->GpsWeek,
			pos->WeekSeconds
			);
}

/*=========================================================================
 * MARKTIME event reports
 *=========================================================================*/

pnotify_marktime _pmarktime_cb[4] =
{
	NULL,NULL,NULL,NULL
};

uint16_t marktime_counts[4] =
{
	0,0,0,0
};


void RegisterMarkTimeNotify(uint16_t mark_nr, bool polarity, pnotify_marktime callback )
{
	char buf[100];

	if (mark_nr > 0 && mark_nr <5)
	{
		int mark_ndx = mark_nr -1;

		_pmarktime_cb[mark_ndx] = callback;

		// MARKCONTROL MARK1 ENABLE NEGATIVE 0 25 // 25 for events up to 40Hz
		snprintf(buf, 100, "MARKCONTROL MARK%d ENABLE %s 0 25", mark_nr, (polarity == 0? "NEGATIVE":"POSITIVE"));

		if ( SendOEM7700CommandWithRetry(buf, 3) )
		{
			marktime_counts[mark_ndx] = 0;
			_pmarktime_cb[mark_ndx] = callback;

			snprintf(buf, 100, "LOG MARK%dTIMEB ONNEW", mark_nr );

			SendOEM7700CommandWithRetry(buf, 3);
		}
	}
}



void CancelMarkTimeNotify(uint16_t mark_nr)
{
	char buf[100];

	snprintf(buf,100, "UNLOG MARK%dTIMEB\r", mark_nr);
	SendOEM7700CommandWithRetry(buf, 3);
	_pmarktime_cb[mark_nr-1] = NULL;
}


void NotifyMarkTime(MARKTIME_t *T, int mark_ndx)
{
	if (T->cm_status != CM_VALID ) // do not report bad times
		return;

	pnotify_marktime _notify = _pmarktime_cb[mark_ndx];

	if ( _notify == NULL )
		return;

	marktime_counts[mark_ndx] += 1;

	marktime_rep_t R;

	R.Mark = mark_ndx +1;
	R.EventTime = T->seconds - T->offset;
	R.EventNr = marktime_counts[mark_ndx];
	R.Week = T->gps_week;

	_notify(R);
}

int FormatMarkTimeRecord(char*buffer, size_t cnt, pmarktime_rep_t prec)
{
	// prefix #MARKTIME = 	MARK#TIME record
	return snprintf(buffer, cnt,"#MARKTIME,%d,%d,%d,%.6lf\n",
			prec->Mark,
			prec->EventNr,
			prec->Week,
			prec->EventTime);
}

/*=========================================================================
 * MARKPOS event reports
 *=========================================================================*/
pnotify_markpos _pmarkpos_cb[4] =
{
	NULL,NULL,NULL,NULL
};

uint16_t markpos_counts[4] =
{
	0,0,0,0
};

void RegisterMarkPosNotify(uint16_t mark_nr, bool polarity, pnotify_markpos callback )
{
	char buf[100];

	if (mark_nr > 0 && mark_nr <5)
	{
		int mark_ndx = mark_nr -1;

		// MARKCONTROL MARK1 ENABLE NEGATIVE 0 25 // 25 for events up to 40Hz
		snprintf(buf, 100, "MARKCONTROL MARK%d ENABLE %s 0 25", mark_nr, (polarity == 0? "NEGATIVE":"POSITIVE"));

		if ( SendOEM7700CommandWithRetry(buf, 3) == OEM7700_OK )
		{
			markpos_counts[mark_ndx] = 0;
			_pmarkpos_cb[mark_ndx] = callback;

			if (mark_nr == 1)
				snprintf(buf, 100, "LOG MARKPOSB ONNEW" );
			else
				snprintf(buf, 100, "LOG MARK%dPOSB ONNEW", mark_nr );

			SendOEM7700CommandWithRetry(buf, 3);
		}
	}
}

void CancelMarkPosNotify(uint16_t mark_nr)
{
	char buf[100];

	if (mark_nr < 1 || mark_nr > 4)
		return;

	if (mark_nr == 1)
		snprintf(buf, 100, "UNLOG MARKPOSB" );
	else
		snprintf(buf, 100, "UNLOG MARK%dPOSB", mark_nr );

	SendOEM7700CommandWithRetry(buf, 3);
	_pmarkpos_cb[mark_nr-1] = NULL;
}

int FormatMarkPosRecord(char*buffer, size_t cnt, pmarkpos_rep_t prec)
{
	// prefix #MARKPOS = 	MARK#POS record
	return snprintf(buffer, cnt,"#MARKPOS,%d,%d,%d,%.6lf,%.8lf,%.8lf,%.3lf\n",
			prec->Mark,
			prec->EventNr,
			prec->Week,
			prec->EventTime,
			prec->Lat,
			prec->Lon,
			prec->Hgt
			);
}

void NotifyMarkPos(MARKPOS_t *T, int mark_ndx)
{
	if (T->SolStatus != SOL_COMPUTED ) // do not report bad times
		return;

	pnotify_markpos _notify = _pmarkpos_cb[mark_ndx];

	if ( _notify == NULL )
		return;

	markpos_counts[mark_ndx] += 1;

	markpos_rep_t R;

	R.Mark = mark_ndx +1;
	R.EventNr = markpos_counts[mark_ndx];
	R.EventTime = ((double) T->H.gpsmsec)/1000.0;
	R.Week = T->H.gpsweek;
	R.Lat = T->Lat;
	R.Lon = T->Lon;
	R.Hgt = T->Hgt;

	_notify(R);
}


/*------------------------------------------------------------------
 * Notify Time
 *------------------------------------------------------------------*/
pnotify_time _notify_time = NULL;

void RegisterNovatelTimeNotify(pnotify_time fn)
{
	_notify_time = fn;
}


void UpdateGPTimeSync(void* vp)
{
	if (((TIMESYNCB_t*)vp)->time_status == TIME_STATUS_FINESTEERING )
	{
		GPS_WeekSeconds = ((TIMESYNCB_t*)vp)->msow / 1000;
		GPS_Week = ((TIMESYNCB_t*)vp)->week;
		nStatus.TimeSync = true;

		if (_notify_time != NULL )
		{
			_notify_time( GPS_Week, GPS_WeekSeconds );
		}
	}
}

/*--------------------------------------------------------------------------
 * Position Update - requested at 10Hz
 *
 *  BESTPOS and INSPOS reports because
 *  The monitoring is set to 10 Hz
 *  */
#define POS_DECIMATOR 10

uint32_t poscounter = 0;

void UpdateBestPosition(void* vp)
{
	position_t lastpos;

	lastpos.GpsWeek = ((BESTPOS_t*)vp)->H.gpsweek;
	lastpos.WeekSeconds = (double)(((BESTPOS_t*)vp)->H.gpsmsec)/1000.0F;  // ms from the header convert to seconds;
	lastpos.Lat = ((BESTPOS_t*)vp)->Lat;
	lastpos.Lon = ((BESTPOS_t*)vp)->Lon;
	lastpos.Hgt = ((BESTPOS_t*)vp)->Hgt;
	lastpos.PosValid = ((BESTPOS_t*)vp)->SolStatus == SOL_VALID? POSTYPE_GEO: POSTYPE_INVALID;

	InterlockPosCopy((pposition_t) &LastPosition, (pposition_t) &lastpos );

	if ((++poscounter % POS_DECIMATOR) == 0)
		LogNovatelMessage((uint8_t*)vp, sizeof(BESTPOS_t));
}


void UpdateInsAttitude(void* vp)
{
//	attitude_t la;

	if (GetMsgId(vp) == INSPVAX_ID)
	{
		/*============================================== strange error.
		la.NorthVel	=((INSPVAX_t)vp)->NorthVel;
		la.EastVel = ((INSPVAX_t)vp)->EastVel;
		la.UpVel = ((INSPVAX_t)vp)->UpVel;
		la.Roll = ((INSPVAX_t)vp)->Roll;
		la.Pitch = ((INSPVAX_t*)vp)->Pitch;
		la.Azimuth = ((INSPVAX_t*)vp)->Azimuth;
		la.InsStatus = ((INSPVAX_t*)vp)->InsStatus;
*/
		SpanStatus.Status = ((INSPVAX_t*)vp)->InsStatus;
		LogNovatelMessage((uint8_t*)vp,  GetRecordLength(vp));

//		InterlockCopyAttitude((pattitude_t) &LastAttitude,(pattitude_t)  &la);
	}

}

/********************************************************************************************
 * Read_Novatel_Message() - Reads and stores Novatel Records received by the Uart
 * 						 May contain IMU records if the receiver is SPAN enabled.
*********************************************************************************************/
#define  NOVATEL_BUFFER_SIZE	4096
union	tagRecod	// NovAtel binary record
{
	uint8_t 		Buf[NOVATEL_BUFFER_SIZE];
	span_short_hdr	Short_hdr;
	span_long_hdr	Long_hdr;

} nRec;

#define MAX_DATA_LEN 	(NOVATEL_BUFFER_SIZE - sizeof(span_long_hdr) - CRC32_SIZE)

/*--------------------------------------------------------
 * Read Once record storage
 *--------------------------------------------------------*/
uint16_t LastStoredRecord = 0;

void StoreOnceMessage(void* vp)
{
	LastStoredRecord = GetMsgId(vp);
	LogNovatelMessage(vp, GetRecordLength(vp));
}

void SendRawIMUSX(void *vp)
{
	LogNovatelMessage(vp, GetRecordLength(vp));
}

/*------------------------------------------------------------------
 *  Special messages Processing
 *------------------------------------------------------------------*/
uint16_t idList[] =
{
	RAWIMUSX_ID,
	TIMESYNC_ID,
	BESTPOS_ID,
	INSPVAX_ID,			// Updates INS status
	VERSION_ID,
	INSCONFIG_ID,
	SETINSTRANSLATION_ID,
};

typedef void (*msg_process)(void*);

msg_process process_fn[] =
{
	SendRawIMUSX,
	UpdateGPTimeSync,
	UpdateBestPosition,
	UpdateInsAttitude,
	StoreOnceMessage,
	StoreOnceMessage,
	StoreOnceMessage,
};


void ProcessMessages(uint16_t msgid, int cnt )
{
	for(int i=0; i < ARRAY_SIZE(idList); i++ )
	{
		if (msgid == idList[i])
		{
			process_fn[i](nRec.Buf);
			return; 			// processed
		}
	}

	/* MARKPOS and MARKTIME special handling */
	if (msgid == MARK1TIME_ID)
		NotifyMarkTime((MARKTIME_t*) nRec.Buf, 0);

	else if (msgid == MARK2TIME_ID)
		NotifyMarkTime((MARKTIME_t*) nRec.Buf, 1);

	else if (msgid == MARK3TIME_ID)
		NotifyMarkTime((MARKTIME_t*) nRec.Buf, 2);

	else if (msgid == MARK4TIME_ID)
		NotifyMarkTime((MARKTIME_t*) nRec.Buf, 3);

	else if (msgid == MARKPOS_ID)
		NotifyMarkPos((MARKPOS_t*) nRec.Buf, 0);

	else if (msgid == MARK2POS_ID)
		NotifyMarkPos((MARKPOS_t*) nRec.Buf, 1);

	else if (msgid == MARK3POS_ID)
		NotifyMarkPos((MARKPOS_t*) nRec.Buf, 2);

	else if (msgid == MARK4POS_ID)
		NotifyMarkPos((MARKPOS_t*) nRec.Buf, 3);

	LogNovatelMessage(nRec.Buf, cnt); // sends message to data logger
}


/*========================================================================================
 * Read_Novatel_message() - NovAtel record decoder and dispatcher
 *=======================================================================================*/

enum tagDecodeStateMachine
{
	IDDLE		= 0,
	HEADER1		= 1,
	HEADER2		= 2,
	HEADER_SHORT= 3,
	HEADER_LONG	= 4,
	PAYLOAD		= 5,
	CMD_ANSWER	= 6,
};

uint32_t DecodeState = IDDLE;

void Read_Novatel_Message( uint8_t* buf, size_t cnt)
{
	int pos = 0;
	static int nextsync;		// ensure that sync bytes are consecutive
	static int hdrlen;
	static int record_size;		// Full record size, including header and CRC
	static int nRecBufCnt;			// # of bytes in nRec.Buffer
	static uint16_t msgid;

	static uint32_t _crc;

	while( pos < cnt)
	{
		uint8_t b = buf[pos++];

		switch (DecodeState)
		{
			case IDDLE:
			{
				if( b == NOVATEL_SYNC1)		// Start of record
				{
					nRec.Buf[0] = b;
					nRecBufCnt = 1;
					DecodeState = HEADER1;
					nextsync = pos < cnt? pos+1: 1;
					_crc = CalculateCharacterCRC32(b, CRC32_INIT);
				}
				// receive command answer
				else if( process_answer )
				{
					if (b == BEGUIN_CMD_ANSWER)
					{
						nRec.Buf[0] = b;
						nRecBufCnt = 1;
						DecodeState = CMD_ANSWER;
					}
				}
				break;
			}

			case CMD_ANSWER:
			{
				nRec.Buf[nRecBufCnt++] = b;

				if (b == END_CMD_ANSWER || nRecBufCnt == MAX_DATA_LEN )
				{
					ProcessCommandAnswer(nRec.Buf, nRecBufCnt);
					DecodeState = IDDLE;
				}
				break;
			}
			case HEADER1:
			{
				if (b == NOVATEL_SYNC2 && pos == nextsync )
				{
					nRec.Buf[1] = b;
					nRecBufCnt = 2;
					DecodeState = HEADER2;
					nextsync = pos < cnt? pos+1: 1;

					_crc = CalculateCharacterCRC32(b, _crc);
				}
				else
				{
					DecodeState = IDDLE;
				}
				break;
			}
			case HEADER2:
			{
				nRec.Buf[2] = b;
				nRecBufCnt = 3;

				_crc = CalculateCharacterCRC32(b, _crc);

				if( b == NOVATEL_SYNC3_SHORT  && pos == nextsync )		// Is short OR long header ?
				{
					DecodeState = HEADER_SHORT;
					hdrlen = sizeof(span_short_hdr);
				}
				else if( b == NOVATEL_SYNC3_LONG  && pos == nextsync )		// Is short OR long header ?
				{
					DecodeState = HEADER_LONG;
					hdrlen = sizeof(span_long_hdr);
				}
				else
				{
					DecodeState = IDDLE;		// failed
				}
				break;
			}

			case HEADER_SHORT:		// store the rest of the header
			{
				nRec.Buf[nRecBufCnt++] = b;

				_crc = CalculateCharacterCRC32(b, _crc);

				if (hdrlen == nRecBufCnt)
				{
					msgid = nRec.Short_hdr.msgid;
					record_size = nRecBufCnt + nRec.Short_hdr.msglen + CRC32_SIZE;

					/* GUARD for buffer overrun */
					if (record_size <= MAX_DATA_LEN)
						DecodeState = PAYLOAD;
					else
						DecodeState = IDDLE;
				}
				break;
			}

			case HEADER_LONG:
			{
				nRec.Buf[nRecBufCnt++] = b;

				_crc = CalculateCharacterCRC32(b, _crc);

				if (nRecBufCnt == 4)		// recover the current header length
					hdrlen = b;

				if ( hdrlen == nRecBufCnt )
				{
					msgid = nRec.Long_hdr.msgid;
					record_size = hdrlen + nRec.Long_hdr.msglen + CRC32_SIZE;

					nStatus.TimeStatus = nRec.Long_hdr.timestatus;
					nStatus.ReciverStatus = nRec.Long_hdr.recstatus;

					if (record_size <= MAX_DATA_LEN)
						DecodeState = PAYLOAD;
					else
						DecodeState = IDDLE;
				}
				break;
			}

			case PAYLOAD: // includes the CRC
			{
				nRec.Buf[nRecBufCnt++] = b;
				_crc = CalculateCharacterCRC32(b, _crc);

				if( record_size == nRecBufCnt  )
				{
					if( _crc == 0 )
					{
						ProcessMessages( msgid, nRecBufCnt);
					}
					else // Bad CRC
					{
						nStatus.CRCErrors++;
						nStatus.Health = CRC_Error;
					}
					DecodeState = IDDLE;
				} 	// Full record read
			} 		// case PAYLOAD
		}
	}
}


/********************************************************************************************
 * 	CRC32 functions
 *********************************************************************************************/
const int BINARY_CRC_SIZE = sizeof(uint32_t);    // ** Change this line if the CRC changes type!!

	const uint32_t ulCRCTable[256] =
	{
	0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
	0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
	0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL, 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
	0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
	0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
	0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
	0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
	0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L, 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
	0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
	0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
	0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
	0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
	0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L, 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
	0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
	0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
	0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
	0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
	0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L, 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
	0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
	0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
	0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
	0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
	0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L, 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
	0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
	0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
	0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
	0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
	0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL, 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
	0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
	0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
	0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
	0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
	};


	enum
	{
		CRC32_POLYNOMIAL = 0xEDB88320L /* Used to generate the table above */
	};

	// --------------------------------------------------------------------------
	// Calculates the CRC-32 of a block of data all at once
	// --------------------------------------------------------------------------
	uint32_t CalculateBlockCRC32(uint32_t ulCount, const uint8_t *ucBuffer)
	{
		uint32_t ulCRC = CRC32_INIT;

		while (ulCount-- != 0)
		{
			uint32_t ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
			uint32_t ulTemp2 = ulCRCTable[((int)ulCRC ^ *ucBuffer++) & 0xff];
			ulCRC = ulTemp1 ^ ulTemp2;
		}
		return(ulCRC);
	}

	// --------------------------------------------------------------------------
	// Calculates the CRC-32 of a block of data one character for each call
	// --------------------------------------------------------------------------
	uint32_t CalculateCharacterCRC32(uint8_t ucChar_, uint32_t ulCRC_ )
	{
		uint32_t ulTemp1;
		uint32_t ulTemp2;

		ulTemp1 = (ulCRC_ >> 8) & 0x00FFFFFFL;
		ulTemp2 = ulCRCTable[((int)ulCRC_ ^ ucChar_) & 0xff];
		return(ulTemp1 ^ ulTemp2);
	}

