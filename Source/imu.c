/*
 * imu.c
 *
 *  Created on: Sep 11, 2021
 *      Author: Guillermo
 *
 *  Rev.3.1: Oct. 2024
 */

/******************************************************************************************************************
 * iIMU-FSAS-HP
 * ICD and User Manual - Pag. 14
 *
 * 4.2.1.3.2 Data Output Protocol of the Version -SN
 *
 * This IMU provides an UART output on RS422 level. The data is sent at up to 400 Hz according to the external
 * trigger via DAS signal:
 *
 * 2 byte CRC-CCITT (related to start value 0x0000, calculation over all bytes incl. sync; see Figure 8)
 * 1 byte package counter (unsigned integer) 3 x 4 byte angular increments (32 bit integer for 0,1,2; unit = 0.1/16 arcsec/LSB)
 * 3 x 3 byte velocity increments (24 bit integer for 0,1,2); unit = 0.05/16/215 m/s/LSB)
 * 4 byte odometer velocity (4 byte float); unit = counts / sample)
 * 4 byte odometer counter2 (32 bit long integer)
 * 4 byte status (failure Status and IMU Status, see Table 7 and Table 8)
 * 2 byte sync 0x7E 0x7E
 *
 * Record size: 38 bytes = 380 bit / sample
 *		at 100 Hz = 38 kBit/s ==> Baud rate >= 57.6 kBd
 *		at 200 Hz the data rate is 76 kBit/s ==> Baud rate >= 115200 Bd
 *		at 400 Hz the data rate is 152 kBit/s ==> Baud rate >= 230400 Bd
 *********************************************************************************************************************/
#include "common.h"

//#define DEBUG_IMU_ISR
//#define DEBUG_IMU_TP	TP7
//#define USE_IMU_SEQUENCE

/****************************************************************************
*IMU GLOBALS
*****************************************************************************/
uint32_t imuFrequency = DEFAULT_IMU_FREQUENCY;

volatile bool IMU_online = false;
volatile bool IMU_init = false;
volatile bool IMU_NoGo = false;

volatile bool Enable_IMU_Logging = false;

volatile double IMU_Clock = 200;	// Default to 200Hz used when PSOC handles the IMU

bool DisableImuLogs = false;

volatile double IMU_WeekSeconds;

FSAS_Status volatile IMUStatus =
{
		.Crc_Errors = 0,
		.HW_Error = IMU_NO_HW_ERROR,
		.HW_ErrorCount = 0,
		.IMU_Status = 0,
		.Records_Received = 0,
		.Records_Skipped = 0
};

const tkeywrd_t ImuTypeList[] =
{
	{ "FSAS", IMUType_FSAS },
	{ "STIM", IMUType_STIM300 },
	{ "KVH", IMUType_KVH },
};

const size_t ImuTypeCount = DICTIONARY_SIZE(ImuTypeList);


const tkeywrd_t ImuComTargets[] =
{
	{ "PSOC", Target_PSOC },
	{ "NOVATEL", Target_NovAtel },
};

const size_t ImuComTargetsCount = DICTIONARY_SIZE(ImuComTargets);


const tkeywrd_t ImuFormatsDictionary[] =
{
	{"FSAS", fmtFSAS_NATIVE},		// used for debugging only
	{"NRAW", fmtNOVATEL_RAW},		// default to be consistent with other logs form OEM7700 receiver
	{"NIMR", fmtNOVATEL_IMR},		// IMR format not used
	{"STIM", fmt_STIM300},			// For debugging only
	{"KVH", fmt_KVH},				// For debugging only
};

const size_t ImuFormatsCount = DICTIONARY_SIZE(ImuFormatsDictionary);

/*----------------------------------------------------------------------------*/
/****************************************************************************
* FSAS IMU NOGO Signal Detect
*****************************************************************************/
#define PPS_DISABLE_DELAY	(10000U)		// waiting time to disable PPS indicator
volatile GPT_HANDLE thandle;

void timer_callbback(void* arg)
{
	PrintWithTime("NOGO Signal time out.\n");
	SetSPPSMonintorFrequency(SLOW_PPS_MONITOR_FREQ, SHORT_PPS_MONITOR_PULSE);
}

void Imu_NoGo_detect_cb(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;

    if(event & CYHAL_GPIO_IRQ_RISE)
    {
    	StopGPTimer(thandle);
        IMU_NoGo = false;
    	SetSPPSMonintorFrequency(SLOW_PPS_MONITOR_FREQ, LONG_PPS_MONITOR_PULSE);
    	ClearImuErrors();

		#ifdef DEBUG
   		PrintWithTime("<--- IMU NOGO INACTIVE --->\n");
		#endif
    }
    else
	{
		IMU_NoGo = true;
 	    StartGPTimer(thandle);

		#ifdef DEBUG
 	   	PrintWithTime("<--- IMU NOGO ASERTED ---->\n");
		#endif
	}
}


cyhal_gpio_callback_data_t Imu_NoGo_detect_cb_data =
{
		Imu_NoGo_detect_cb,
		NULL, 0, 0,
};


/****************************************************************************
*  Test_IMU_NoGo() - True = NOGO | False = GO
*****************************************************************************/
bool Test_IMU_NoGo()
{
	IMU_NoGo = !cyhal_gpio_read(IMU_NOGO_TOV);

	if (IMU_NoGo)
	{
		SetSPPSMonintorFrequency(SLOW_PPS_MONITOR_FREQ, SHORT_PPS_MONITOR_PULSE);
	}
	else
	{
		SetSPPSMonintorFrequency(SLOW_PPS_MONITOR_FREQ, LONG_PPS_MONITOR_PULSE);
	}
	return IMU_NoGo;
}

/****************************************************************************
* Init_IMU_NOGO_Detect() - for FSAS IMU
*****************************************************************************/
void Init_IMU_NOGO_Detect()
{
	/* Initialize the NOGO Detect Pin P3.2 */
	cyhal_gpio_init(IMU_NOGO_TOV, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	cyhal_gpio_register_callback(IMU_NOGO_TOV, &Imu_NoGo_detect_cb_data);

	/* create a delay timer to disable logging if NOGO goes active for more than 5 seconds */
    thandle = CreateGPTimer(timer_callbback, NULL, PPS_DISABLE_DELAY, true);

    cyhal_gpio_enable_event(IMU_NOGO_TOV, CYHAL_GPIO_IRQ_BOTH, CYHAL_ISR_PRIORITY_DEFAULT, true);

    if ( Test_IMU_NoGo() )
	    StartGPTimer(thandle);
}


/****************************************************************************
 * Set_FSAS_Trigger_Frequency()
 *
 *  - adjust the TDAS (strobe) frequency
 *  - also sets the MARK1 control in the NovAtel receiver.
 *
 ****************************************************************************/
void Set_FSAS_Trigger_Frequency(uint32_t _frequency)
{
	char buf[100];

	/* prevent out of valid range settings */

	if (_frequency > MAX_IMU_FREQUENCY )
		_frequency = 200;
	else if (_frequency < 1 )
		_frequency = 1;

	IMU_Clock = 200.0;

	int j = ComposeMarkOutCommand(buf, ARRAY_SIZE(buf), 1, _frequency, 10, false);
	Uart_Oem7700_Send((uint8_t*)buf,j);

	imuFrequency = _frequency;
}

void Stop_FSAS_Trigger_Frequency()
{
	char buf[100];
	int j = ComposeMarkOutDisable(buf, ARRAY_SIZE(buf), 1);
	Uart_Oem7700_Send((uint8_t*)buf,j);

	imuFrequency = 0;
}


/****************************************************************************
* Init_IMU_Interface()
*****************************************************************************/
bool Init_IMU_Interface(imu_type_t Type, imu_target_t Target)
{
	char buf[100];

	PrintWithTime("Initializing IMU interface ...");

	Set_IMU_COM_Target(Target);								// Novatel (w. SPAN) or PSOC (no SPAN)

	if (Type == IMUType_FSAS )
	{
		Stop_FSAS_Trigger_Frequency();						// In case we have a warm restart

		SpanStatus.ImuType = IMUType_FSAS;
		IMU_Clock = 200.0;

		Init_USec_Timer();									// TDAS event timing for IMU data timestamp
		SetImuDataFormat(fmtNOVATEL_RAW);					// IMU data format reporting
		SetImuMsgProcessor(Decode_FSAS_IMU_Data);			// Switch to IMU data decoding

		Init_IMU_NOGO_Detect();								// GO/NOGO signal monitoring

		snprintf(buf,100,"IMU Type FSAS - Port: %s\n", Target == Target_PSOC? "PSOC": "NOVATEL");
		PrintWithTime( buf );

		Init_Uart_IMU( B115200 );

		Set_FSAS_Trigger_Frequency(DEFAULT_IMU_FREQUENCY);			// 200hz start IMU trigger

		IMU_init = true;
	}
	else if (Type == IMUType_STIM300)			// STIM300 is handled by OEM7700 with SPAN firmware
	{
		snprintf(buf,100,"IMU Type STIM300 - Port: %s", Target == Target_PSOC? "PSOC": "NOVATEL\n");
		PrintWithTime( buf );

		Init_STIM_TOV_Detect( Target );

		SpanStatus.ImuType = IMUType_STIM300;

		IMU_Clock = 125.0;					// 125Hz

		if(Target == Target_NovAtel )		// do STIM300 initialization
		{
			IMU_init = true;
		}
		else // PSOC handles STIM input (must reproduce RAWIMUSX records)
		{
			Init_Uart_IMU(B460800);

			// to be implemented
			PrintWithTime( "STIM300 handled by PSOC\n" );
			SetImuMsgProcessor( Receive_STIM300_Datagram );		// Switch to IMU data decoding
			Init_STIM_Framing = true;

			IMU_init = true;
		}
	}
	else if(Type == IMUType_KVH)
	{
		snprintf(buf,100,"IMU Type KVH - Target: %s", Target == Target_PSOC? "PSOC": "NOVATEL\n");
		PrintWithTime( buf );
		
		SpanStatus.ImuType = IMUType_KVH;

		Init_Uart_IMU(B460800);

		SetImuMsgProcessor( Receive_KVH_Datagram );		// Switch to IMU data decoding

		IMU_Clock = 200.0;					// 200Hz

		IMU_init = true;
	}
	else
	{
		IMU_init = false;
	}

	return !IMU_init;
}

/****************************************************************************
 * Query IMU type and connection targe. 
 */
void PrintImuType()
{
	char buf[100];

	const char* type = GetImuTypeName(SysConfig.imu_type);

	if (type == NULL)
		type = "INVALID";
	
	const char* Target = GetImuConnectName(SysConfig.imu_connect);

	if (Target == NULL)
		Target = "INVALID";

	snprintf(buf,100,"IMU Type %s - Port: %s", type, Target);
	PrintWithTime( buf );
}

void PrintValidImuTypes()
{
	PrintWithTime("Valid IMU Types:\n");
	for (size_t i=0; i<ImuTypeCount; i++)
	{
		printf( "%s\n", ImuTypeList[i].name );
	}	
}

const char* GetImuTypeName( imu_type_t type)
{
	return find_KeywordName(ImuTypeList, ImuTypeCount, type);
}

const char* GetImuConnectName(imu_target_t target)
{
	return find_KeywordName(ImuComTargets, ImuComTargetsCount, target);
}

/****************************************************************************
* Set_IMU_COM_Target()
*
* Connects the IMU to PSOC6 or OEM7700 COM2 for IMUs supported by NovAtel
*
*****************************************************************************/
void Set_IMU_COM_Target(imu_target_t Target)
{
	cyhal_gpio_write(IMU_COM_SELECT, Target == Target_NovAtel? true: false );
	SysConfig.imu_connect = Target;
}

/****************************************************************************
* Store_IMU_COM_Target()
*
* Connects the IMU to PSOC6 or OEM7700 COM2 for IMUs supported by NovAtel
*
*****************************************************************************/
void Store_IMU_COM_Target(imu_target_t target)
{
	SysConfig.imu_connect = target;
	SaveConfig(&SysConfig);
}

/****************************************************************************
* Set_IMU_Type()
*
* Selects the IMU type attached to the controller.
*
*****************************************************************************/
void Set_IMU_Type(imu_type_t Type, bool save)
{
	SpanStatus.ImuType = Type;
	SysConfig.imu_type = Type;
	if (save)
		SaveConfig(&SysConfig);
}


/****************************************************************************
 * GetIMUStatusStr()
 *
 *  - Returns the status as a string.
 *
 ****************************************************************************/
volatile uint16_t Last_HW_Error = 0;

extern double ClockDif;
extern double CountAdjust;

size_t GetIMUStatusStr(char*buffer, size_t size)
{
	size_t cnt;
	if (SysConfig.imu_type == IMUType_STIM300)
	{
		cnt = GetSpanStatus(buffer, size);
	}
	else // FSAS IMU
	{
		FSAS_Status S;

		memcpy(&S, (FSAS_Status*)&IMUStatus, sizeof(FSAS_Status));

		cnt = snprintf(buffer, size, "IMU records: %.6ld, CRC Errors: %ld, Records skipped: %ldf\n"\
									"IMU Status: %.4Xh, HW Errors: %ld, HW Error: %.4Xh Last HW Error: %.4Xh\n"\
									"ClockDif: %.1lf ClockAdj: %.3lf\n",
				S.Records_Received,
				S.Crc_Errors,
				S.Records_Skipped,
				S.IMU_Status,
				S.HW_ErrorCount,
				S.HW_Error,
				Last_HW_Error,
				ClockDif,
				CountAdjust
				);

		if (IMUStatus.HW_Error != 0)
			Last_HW_Error = IMUStatus.HW_Error;

		IMUStatus.IMU_Status = 0;
		IMUStatus.HW_Error = 0;
	}

	return cnt;
}

/****************************************************************************
* GetIMUStatusShort()
*****************************************************************************/
size_t GetIMUStatusShort(char*buffer, size_t size)
{
	size_t cnt;

	if (SysConfig.imu_type == IMUType_STIM300)
	{
		cnt = snprintf(buffer, size,"%d,%d,0,0,0,0,0,G,",
				SpanStatus.ImuType, SpanStatus.Status);
	}
	else // FSAS IMU
	{
		FSAS_Status S;

		memcpy(&S, (FSAS_Status*)&IMUStatus, sizeof(FSAS_Status));

		cnt = snprintf(buffer, size,"%d,%.4Xh,%.4Xh,%ld,%ld,%ld,%lu,%c,",
				SpanStatus.ImuType,
				S.IMU_Status,
				S.HW_Error,
				S.HW_ErrorCount,
				S.Crc_Errors,
				S.Records_Skipped,
				S.Records_Received,
				(int)(IMU_NoGo? 'N' : 'G')
				);

		IMUStatus.IMU_Status = 0;
		IMUStatus.HW_Error = 0;
	}
	return cnt;
}


/****************************************************************************
* LogImuErrors()
*****************************************************************************/
void LogImuErrors()
{
	Test_IMU_NoGo();
	printf( "[%s] - IMU Status: %.4Xh - IMU Errors: %.4Xh, NOGO: %s\n",
			GetUpTimeStr(),
			IMUStatus.IMU_Status,  IMUStatus.HW_Error, IMU_NoGo? "NOGO" : "GO" );

	ClearImuErrors();
}

/*============================================================================
 * ClearImuErrors()
 *============================================================================*/
void ClearImuErrors()
{
	IMUStatus.IMU_Status = 0;
	IMUStatus.Crc_Errors = 0;
	IMUStatus.HW_Error = 0;
	IMUStatus.HW_ErrorCount = 0;
}

void ClearImuStatus()
{
	IMUStatus.HW_ErrorCount=0;
	IMUStatus.Records_Received = 0;
	IMUStatus.Records_Skipped = 0;
	IMUStatus.IMU_Status = 0;
	IMUStatus.Crc_Errors = 0;
	IMUStatus.HW_Error = 0;
}

/****************************************************************************
 * SetIMUScalling()
 *
 *  - adjust the scale factors according to the TDAS (strobe) frequency
 *
 *
 ****************************************************************************/
double GyroScale = 1;		// deg/seg
double AccelScale = 1;		// m/seg2

void SetIMUScalling(double frequency)
{
	if (frequency >= 1.0)
	{
		GyroScale = (double)(1 / 36000.0 * frequency);		// deg/seg
		AccelScale = (double)(0.05 /32768.0 * frequency);	// m/seg2
	}
}


/****************************************************************************
 * SetImuDataFormat()
 *
 *  - select FSAS native or NOVATEL RAW report data format
 *
 ****************************************************************************/
typedef void (*fnImuFormatter)(PFSAS_SN_t);

void Format_FSAS_SN_to_NovatelRawSX(PFSAS_SN_t);
void Format_FSAS_SN_to_txINS(PFSAS_SN_t);
void Format_FSAS_SN_Native(PFSAS_SN_t);

fnImuFormatter _FormatImuRecord = Format_FSAS_SN_to_NovatelRawSX;

void SetImuDataFormat(imu_format_t format)
{
	switch(format)
	{
	case INVALID_FORMAT:
	case fmtNOVATEL_RAW: /* DEFAULT Format compatible with OEM7700 receiver logs */
		_FormatImuRecord = Format_FSAS_SN_to_NovatelRawSX;
		log_gnss_records = true;
		PrintWithTime("IMU data format set to NOVATEL_RAW.\n");
		break;

	case fmtFSAS_NATIVE: /* format incompatible with OEM7700 receiver logs */
		_FormatImuRecord = Format_FSAS_SN_Native;
		log_gnss_records = false;
		PrintWithTime("IMU data format set to FSAS NATIVE.\n");
		break;

	case fmtNOVATEL_IMR: /* format incompatible with OEM7700 receiver logs */
		_FormatImuRecord = Format_FSAS_SN_to_txINS;
		log_gnss_records = false;
		PrintWithTime("IMU data format set to NOVATEL IMR.\n");
		break;

	case fmt_STIM300:
		{
			/* TODO change to this format */
			PrintWithTime("STIM300 data format not implemented.\n");
			break;
		}
	case fmt_KVH:
		{
			/* TODO change to this format */
			PrintWithTime("KVH data format not implemented.\n");
			break;
		}
	}

	ClearImuErrors();
}

/****************************************************************************
 * Cast_3Byte_To_Int32() - Gyros reported with 3 byte integers (!!)
 ****************************************************************************/
inline int32_t Cast_3Byte_To_Int32(int8_t *p)
{
#ifdef CAST_SAFE_VERSION
	int32_t h = 0;
	memcpy((void*)&h, p, 3);
#else
	int32_t h = *((int*)p);	// we can do this because the V2 is not the last byte of the structure
#endif

	if (h & 0x800000)		// sign extend (arithmetic shift right would be more efficient)
		h |= 0xFF000000;
	else
		h &= 0x00FFFFFF;

	return h;
}

/****************************************************************************
 * Format_FSAS_SN_to_NovatelRawSX()
 *
 *  - Checks record integrity and posts it as tXINS record.
 *
 ****************************************************************************/
void Format_FSAS_SN_to_NovatelRawSX( PFSAS_SN_t psn )
{
	RAWIMUSX_t _RawImuSX =		// defined in Novatel.h
	{
		.Hdr = // short header
		{
			.sync1  = NOVATEL_SYNC1,
			.sync2  = NOVATEL_SYNC2,
			.sync3  = NOVATEL_SYNC3_SHORT,
			.msglen = RAWIMUSX_MSG_LENGTH,
			.msgid  = RAWIMUSX_ID
		},
	};

	_RawImuSX.Hdr.gpsweek = UsecEventTime.GPSWeek;
	_RawImuSX.Hdr.gpsmsec = UsecEventTime.WeekMilliSeconds;

	_RawImuSX.imu_info   = psn->Status != 0? 1: 0;	// Biy 0 gl
	_RawImuSX.imu_type   = IMAR_FSAS;

#ifdef USE_IMU_SEQUENCE					// For debugging purposes only!
	static uint32_t sequence=0;

	if(psn->Counter == 0)
		sequence++;
	else
	{
		sequence &= 0xffffff00;
		sequence += psn->Counter;
	}
 	_RawImuSX.imu_status = sequence; // psn->Status for testing purposes

#else
	_RawImuSX.imu_status = psn->Status;
#endif

	IMU_WeekSeconds = UsecEventTime.WeekSeconds;			// for status monitoring only

	_RawImuSX.gnss_week    = UsecEventTime.GPSWeek;
	_RawImuSX.week_seconds = UsecEventTime.WeekSeconds;		// a double in full seconds

	_RawImuSX.z_accel  = Cast_3Byte_To_Int32(psn->V2);
	_RawImuSX._y_accel = Cast_3Byte_To_Int32( psn->V1);
	_RawImuSX.x_accel  = Cast_3Byte_To_Int32( psn->V0);

	_RawImuSX.z_gyro  = psn->A2;
	_RawImuSX._y_gyro = psn->A1;
	_RawImuSX.x_gyro  = psn->A0;

	_RawImuSX.crc = CalculateBlockCRC32( RAWIMUSX_CRC_OFFSET, (void*)&_RawImuSX );

	if ( Enable_IMU_Logging)
		Send_IMU_Record( (void*)&_RawImuSX, sizeof(RAWIMUSX_t));
}


/****************************************************************************
 * Format_FSAS_SN_to_txINS()
 *
 *  - Checks record integrity and posts it as tXINS record.
 *
 ****************************************************************************/
void Format_FSAS_SN_to_txINS( PFSAS_SN_t psn)
{
	 /* to check the CRC in this weird upside down format we have to
	  * save the CRC and replace it with the FSAS_SYNC_WORD and check
	  * the CRC excluding the SYNC word at the end.
	  */

	TxINS_t _ins;

	// The event time was already registered
	_ins.INS.Time =  UsecEventTime.WeekSeconds;

	_ins.SyncID   = (uint16_t)(FSAS_SYNC_WORD);
	_ins.Sequence = psn->Counter;
	_ins.Status   = IMUStatus.IMU_Status;

	_ins.INS.dAng[0] = psn->A0;
	_ins.INS.dAng[1] = psn->A1;
	_ins.INS.dAng[2] = psn->A2;

	_ins.INS.dVel[0] =  Cast_3Byte_To_Int32( psn->V0);
	_ins.INS.dVel[1] =  Cast_3Byte_To_Int32( psn->V1);
	_ins.INS.dVel[2] =  Cast_3Byte_To_Int32( psn->V2);
	_ins.CRC = crc16_ccitt((uint8_t*)&_ins, sizeof(TxINS_t)-sizeof(uint16_t));

	 Send_IMU_Record( (void*)&_ins, sizeof(TxINS_t));
}

/****************************************************************************
 * Format_FSAS_SN_Native()
 *
 *  - Sends the original IMU record received.
 *
 ****************************************************************************/
void Format_FSAS_SN_Native( PFSAS_SN_t psn )
{
  Send_IMU_Record((void*)psn, sizeof(FSAS_SN_t));
}


/****************************************************************************
 * Decode_FSAS_IMU_Data()
 *
 *   - Receives a raw IMU record and pass it to the format converter.
 *   - ISR called once per fix record of 56 bytes.
 *
 * DECODE DURATION
 *  Decode to Novatel RAW IMU < 30 usec.
 *  Time between records: 5 ms @ 200Hz
 ****************************************************************************/
#define	CRC_START_VALUE		(0)

#define SKEEP_SYNC_SEED 0x0B40F

volatile bool     sqquence_init = 0;
volatile uint8_t  sequence_counter = 0;
volatile uint16_t sync_word = 0;

void Decode_FSAS_IMU_Data(uint8_t *buf, size_t cnt)
{
#ifdef DEBUG_IMU_ISR
	SET_DEBUG_TP(DEBUG_IMU_TP);		// start with full record received Use TP3 to monitor ISR duration
#endif

	if (cnt == 1) // in sync mode
	{
		sync_word = (sync_word << 8) | buf[0];

		if (sync_word == FSAS_SYNC_WORD)
			Cy_SCB_SetRxFifoLevel(UART_IMU_HW, FSAS_SN_RECORD_SIZE -1);

		return;  //sync_word is the last word of the record.
	}

	/* This version expects complete records in sync */
	else if ( cnt != FSAS_SN_RECORD_SIZE )
	{
		sync_word = 0;
		Cy_SCB_SetRxFifoLevel(UART_IMU_HW, 0);
		return;
	}

	FSAS_SN_t *psn = (PFSAS_SN_t) buf;

	if ( psn->SyncWord != FSAS_SYNC_WORD) // uart out of sync with record.
	{
		sync_word = 0;
		Cy_SCB_SetRxFifoLevel(UART_IMU_HW, 0);
		return;
	}

	/* to check the CRC in this weird upside down format we have to
	 * save the CRC and replace it with the FSAS_SYNC_WORD and check
	 * the CRC excluding the SYNC word at the end.
	 */
	uint16_t crc = psn->CRC;

	psn->CRC = (uint16_t)(FSAS_SYNC_WORD);

	if( crc != crc16_ccitt((uint8_t*)psn,(sizeof(FSAS_SN_t)-FSAS_CRC_SIZE))  )
	{
		 IMUStatus.Crc_Errors++;
	}

	psn->CRC = crc;		// restore CRC

	if (psn->Counter != sequence_counter )
	{
		if (sqquence_init == true)
			IMUStatus.Records_Skipped++;		// time mark will correct

		sequence_counter = psn->Counter;		// non fatal error
		sqquence_init = true;
	}
	sequence_counter++; // uint8_t rolls over by itself = (sequence_counter+1)& 0xFF;

	IMUStatus.IMU_Status = psn->Status;
	IMUStatus.HW_Error = psn->Error;

	if (psn->Error != 0 )
	{
		Last_HW_Error = psn->Error;
		IMUStatus.HW_ErrorCount++;
	}

	/* format and report new record */
	if (_FormatImuRecord != NULL)
	{
		_FormatImuRecord( psn );				// call record formatter
	}

	IMUStatus.Records_Received++;

#ifdef DEBUG_IMU_ISR
	CLEAR_DEBUG_TP(DEBUG_IMU_TP);		// end of conversion
#endif
}



/*=======================================================================================================
 * CRC16 FUNCTIONS
 ********************************************************************************************************/

/* CRC16 implementation according to CCITT standards */

static const uint16_t crc16tab[256] =
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};



uint16_t crc16_ccitt(uint8_t* buf, size_t len)
{
	unsigned short crc = CRC_START_VALUE;

	for (int i = 0; i < len; i++)
	{
		crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ buf[i]) & 0x00FF];
	}

	return crc;
}


/***************** CRC with User selected SEED *******************/

uint16_t crc16wseed(uint8_t* buf, size_t len, unsigned short seed)
{
	unsigned short crc = seed;

	for (int i = 0; i < len; i++)
	{
		crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ buf[i]) & 0x00FF];
	}
	return crc;
}


/*
 * uint16_t update_crc_ccitt( uint16_t crc, unsigned char c );
 *
 * The function update_crc_ccitt() calculates a new CRC-CCITT value based on
 * the previous value of the CRC and the next byte of the data to be checked.



uint16_t update_crc_ccitt(uint16_t crc, uint16_t c)
{
	int16_t tmp;
	int16_t short_c = 0x00FF & c;

	tmp = (crc >> 8) ^ short_c;
	crc = (crc << 8) ^ crc16tab[tmp];

	return crc;

}   update_crc_ccitt */


/* crc32 - Table based implementation of CRC for STIM300
	Input:
		data = address of data
		len = length of data (number of bytes)
	Output:
		CRC of data

	Author: Sverre Normann
	Modifyed by: Ola Fremming
*/


