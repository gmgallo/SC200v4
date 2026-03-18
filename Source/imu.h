/*
 * imu.h
 *
 *	COMON FUNCTIONS TO ALL IMUS
 *
 *  Created on: Sep 11, 2021
 *      Author: Guillermo
 *
 * Rev.3.1: Oct. 2024
 */

#ifndef IMU_H_
#define IMU_H_

#include "globaldefs.h"
#include "Novatel.h"

#pragma pack(push,1)


 /* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 *  NOTE: THE IMU TRIGGER LOGIC NEEDS TO BE REWORKED FOR SC200.
 *  IMU TRIGGER NOW GOES DIRECTLY FROM OEM7700 EVENT OUT 1 TO IMU DAS SIGNAL
 *  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


/****************************************************************************
* IMU flags
*****************************************************************************/
extern volatile bool IMU_online;
extern volatile bool IMU_init;
extern volatile bool IMU_NoGo;
extern volatile bool Enable_IMU_Logging;

typedef enum
{
	IMUType_INVALID = 0,
	IMUType_FSAS 	= 1,
	IMUType_STIM300 = 2,
	IMUType_KVH		= 3,

} imu_type_t;

typedef enum
{
	Target_INVALID  = 0,
	Target_PSOC 	= 1,
	Target_NovAtel 	= 2,

} imu_target_t;

#define DEFAULT_IMU_TYPE	IMUType_FSAS	// can be changed and stored in NV EEPROM
#define DEFAULT_IMU_TARGET	Target_PSOC		// For receivers without SPAN firmware

/****************************************************************************
* Init_IMU_Interface()
*****************************************************************************/
bool Init_IMU_Interface(imu_type_t Type, imu_target_t Target);
//bool Test_IMU_NoGo();
void LogImuErrors();


/****************************************************************************
* Set_IMU_Type()
*
* Selects the IMU type attached to the controller, saves the config if true
*
*****************************************************************************/
void Set_IMU_Type(imu_type_t Type, bool save);		// Select where to connect the IMU data


/****************************************************************************
* Set_IMU_COM_Target()
*
* Connects the IMU to PSOC6 or OEM7700 COM2 for IMUs supported by NovAtel
*
*****************************************************************************/
void Store_IMU_COM_Target(imu_target_t target);

/****************************************************************************
* Set_IMU_COM_Target()
*
* Connects the IMU to PSOC6 or OEM7700 COM2 for IMUs supported by NovAtel
*
*****************************************************************************/
void Set_IMU_COM_Target(imu_target_t Target);		// Select where to connect the IMU data

/****************************************************************************
 * Set_FSAS_Trigger_Frequency() - FSAS ONLY
 *
 * SC200 - Uses OEM7700 Mark1 out to supply TDAS trigger.
 *
 ****************************************************************************/
void Set_FSAS_Trigger_Frequency(uint32_t _frequency);
void Stop_FSAS_Trigger_Frequency();


#define MIN_IMU_FREQUENCY	  (100U)	// 100 Hertz min frequency without IMU errors
#define MAX_IMU_FREQUENCY	  (200U)	// 200 Hertz (5 ms period @115.2k baud)
#define DEFAULT_IMU_FREQUENCY (200U)	// For startup

typedef enum
{
	INVALID_FORMAT = 0,
	fmtFSAS_NATIVE = 1,
	fmtNOVATEL_RAW = 2,		// == RAWIMUSX Default logging format for Inertial Explorer processing
	fmtNOVATEL_IMR = 3,		// Novatel neutral format
	fmt_STIM300	   = 4,
	fmt_KVH		   = 5,

	} imu_format_t;				// This is the IMU format to report to the logger app

/****************************************************************************
 * SetImuDataFormat()
 *
 *  - selects the report data format
 *  - sets the reporting structure pointer.
 *
 ****************************************************************************/
void SetImuDataFormat(imu_format_t format);  // select FSAS native or NOVATEL RAW report data format
//void PurgeImuBuffer();

/****************************************************************************
 * IMU Status and Error flags
 *
 ****************************************************************************/

enum FSAS_HW_Error
{
	IMU_NO_HW_ERROR		= 0,
	GIRO_FAILED		    = 0X40,
	ACCEL_FAILED	    = 0x00040000,
	POWER_UP_BIT_FAILED = 0x100,		// Power up Built In Test failed
	NOT_OPERATIONAL     = 0x20000000,	// Failed if bit set
};

typedef struct
{
	volatile	uint16_t 	IMU_Status;
	volatile	uint16_t	HW_Error;
	volatile	uint32_t 	HW_ErrorCount;
	volatile	uint32_t 	Crc_Errors;
	volatile	uint32_t 	Records_Skipped;
	volatile	uint32_t	Records_Received;

} FSAS_Status;

extern volatile FSAS_Status  IMUStatus;


/****************************************************************************
 * GetIMUStatusStr()
 *
 *  - Returns the status as a string.
 *
 ****************************************************************************/
size_t GetIMUStatusStr(char*buffer, size_t size);
size_t GetIMUStatusShort(char*buffer, size_t size);
void ClearImuStatus();
void ClearImuErrors();

/****************************************************************************
 * Decode_IMU_Data()
 *
 *  - Collects raw IMU records and pass it to the format converter.
 *
 ****************************************************************************/
void Decode_FSAS_IMU_Data(uint8_t *buf, size_t cnt);
uint16_t crc16_ccitt(uint8_t* buf, size_t len);

/*-----------------------------------------------------------
 * FSAS native IMU SN data format (Novatel)
 *-----------------------------------------------------------*/
typedef struct tagFSAS_SN
{
	uint16_t	CRC;		// 2
	uint8_t		Counter;	// 3 round robin record counter
	int32_t		A2;			// 7
	int32_t		A1;			// 11
	int32_t		A0;			// 15
	int8_t		V2[3];		// 18
	int8_t		V1[3];		// 21
	int8_t		V0[3];		// 24
	int32_t		Odo[2];		// 32 ignored
	uint16_t	Status;		// 34
	uint16_t	Error;		// 36
	uint16_t	SyncWord;	// 38

} FSAS_SN_t, * PFSAS_SN_t;

#define FSAS_SYNC_WORD		(0X7E7E)
#define FSAS_CRC_SIZE		(sizeof(uint16_t))
#define FSAS_SN_RECORD_SIZE	(sizeof(FSAS_SN_t))
#define FSAS_SN_SYNC_OFFSET (sizeof(FSAS_SN_t)-sizeof(uint16_t))

#ifndef INS_t_DEFINED
#define INS_t_DEFINED
/* IMU record of NovAtel IMR file format */
typedef struct tINS_t
{
	double Time; 			// 8  - GPS time frame – seconds of the week
	int32_t dAng[3];		// 12 - delta theta or angular rate depending on flag in the header
	int32_t dVel[3];		// 12 - delta Velocity or acceleration depending on the the flag in the header.

} INS_t;					// 32 bytes
#endif

/* INS transmit record - includes start ID, status and CRC */

typedef struct tTxINS
{
	uint16_t	SyncID;		// 2  - same as FSAS sync word
	uint8_t		Sequence;	// 1  - Sequence counter
	uint8_t 	Status;		// 1  - FSAS_Status
	INS_t		INS;		// 32 - Defined in NOVATEL.H
	uint16_t	CRC;		// 2  - CRC checksum

} TxINS_t, * PTxINS_t;		// 38 Bytes


#define IMU_RECORD_SIZE  			(MAX(MAX(sizeof(RAWIMUSX_t),sizeof(FSAS_SN_t)),sizeof(TxINS_t)))
#define IMU_RECORD_STORE_COUNT 300	/* 1.5 second of roundrobin IMU records storage @ 200Hz ~16k */


#pragma pack(pop)

#endif /* IMU_H_ */
