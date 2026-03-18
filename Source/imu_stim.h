/*
 * imu_stim.h
 *
 * STIM300 IMU Data format and functions
 *
 *  Created on: Oct 27, 2023
 *      Author: Guillermo
 *
 * Rev. 3.1: Oct 2024
 */

#ifndef SOURCE_IMU_STIM_H_
#define SOURCE_IMU_STIM_H_


#include "globaldefs.h"
#include "Novatel.h"

#pragma pack(push,1)


/*-----------------------------------------------------------------------------------------------------------------
 *  STIM300 IMU - Diagnostics
 */
void Decode_STIM300_Datagram();
void Receive_STIM300_Datagram(uint8_t* buffer, size_t cnt);

/*-------------------------------------------------------------
 * Safran STIM300 - Native A5 record (Rate, Acceleration, Temperature)
 *
 * Note: No <CR><LF> Line terminator included
 *-----------------------------------------------------------*/
typedef struct _st_stim_as
{
	uint8_t		ID;			// 1
	uint8_t		Gx[3];		// 4  Gyro X, Y, Z
	uint8_t		Gy[3];		// 7
	uint8_t		Gz[3];		// 10
	uint8_t		SG;			// 11 Gyro status

	uint8_t		Ax[3];		// 14 Accel X, Y Z,
	uint8_t		Ay[3];		// 17
	uint8_t		Az[3];		// 20
	uint8_t		SA;			// 21 Accel status

	uint16_t	TGx;		// 23 Temp Gyros X, Y, Z
	uint16_t	TGy;		// 25
	uint16_t	TGz;		// 27
	uint8_t		STG;		// 28  Temp Gyros status

	uint16_t	TAx;		// 30 Temp Accel X, Y, Z
	uint16_t	TAy;		// 32
	uint16_t	TAz;		// 34
	uint8_t		STA;		// 35  Temp Gyros status

	uint8_t		Counter;	// 36 round robin record counter
	uint16_t	Latency;	// 38
	uint32_t	CRC;		// 42

} STIM_A5_t, * PSTIM_A5_t;


#define STIM_RECORD_SIZE sizeof(STIM_A5_t)
#define STIM_A5_DG_ID	0xA5

#pragma pack(pop)

typedef enum
{
	x_channel 		= 0x01,
	y_channel 		= 0x02,
	z_channel 		= 0x04,
	meas_error 		= 0x08,
	overload  		= 0x10,
	outofrange 		= 0x20,		//  voltage < 4.5V
	startup  		= 0x40,
	integrity_error	= 0x80

} stim_record_status_t;


/*-----------------------------------------------------------
 *Init_STIM_TOV_Detect - Forwards the STIM TOV (NOGO line)
 * to OEM7700 EVENT_IN2 input.
 *-----------------------------------------------------------*/
void Init_STIM_TOV_Detect( imu_target_t _target);

/* ---------------------------------------- DEBUG data */
extern bool Init_STIM_Framing;
void Stim_Log_To_Port(_ports_t _port);
void Stim_Stop_Logging();
int GetStimStatus(char* buf, int len);

void InitBaudsMonitor(); // for debug

#endif /* SOURCE_IMU_STIM_H_ */
