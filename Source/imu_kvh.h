/*
 * KVH.h - KVH IMU definitions and functions
 *
 * Copyright (C) 2024 - 2026 - G2 Airborne Systems.
 * Version 1.00 - Mar. 2026 - Initial development of KVH IMU support.
 */

#ifndef KVH_H
#define KVH_H
 
#include "common.h"

#pragma pack(push,1)


/*--------------------------------------
 *KVH Data format
 *--------------------------------------*/

typedef enum _Tag_Kvh 
{
	//BE_KVH_HEADER725 = 0xFE81FF55,	// 4th byte is the record ID, 0x55 for IMU model 1725 IN BIG ENDIAN format.
	LE_KVH_HEADER = 0x55FF81FE,			// Little Endian header as readed from the unmodified UART stream. 
	KVH_EXBIT2HDR = 0xFE8100AB,
	KVH_MSG_BYTE0 = 0xFE,				// BIG ENDIAN BYTE ORDER
	KVH_MSG_BYTE1 = 0x81,
	KVH_MSG_BYTE2 = 0xFF,
	KVH_MSG_BYTE3 = 0x55,

} KVH_HEADERS;


typedef struct
{
 uint8_t Xg :1;	// 1 bit per flag, 0 means invalid, 1 means good
 uint8_t Yg :1;
 uint8_t Zg :1;
 uint8_t :1;
 uint8_t Xa :1;
 uint8_t Ya :1;
 uint8_t Za :1;
 uint8_t :1;
} kvh_status_t;

#define KVH_STATUS_XG (1u << 0)
#define KVH_STATUS_YG (1u << 1)
#define KVH_STATUS_ZG (1u << 2)
#define KVH_STATUS_XA (1u << 4)
#define KVH_STATUS_YA (1u << 5)
#define KVH_STATUS_ZA (1u << 6)

typedef struct// ALL DATA IS RECEIVED IN BIG ENDIAN FORMAT
{
	uint32_t	header;		// 4 bytes
	float		Xgyro;		// 4 bytes
	float		Ygyro;	// 4 bytes
	float		Zgyro;		// 4 bytes
	float		Xaccel;		// 4 bytes
	float		Yaccel;		// 4 bytes
	float		Zaccel;		// 4 bytes
	uint8_t 	Status;		// 1 byte, bit 0-2 gyro status, bit 4-6 accel status
	uint8_t		Sequence;	// 1 byte, incremented with each record, roll over to 0 after 255	
	int16_t 	Temp;		// 2 bytes, signed integer in 0.1 deg C, so 250 means 25.0 deg C
	uint32_t	crc;		// 4 bytes, CRC32 of all previous bytes in the record, including the header

} kvh_msg_t, * PKVH_MSG;

#define KVH_RECORD_SIZE sizeof(kvh_msg_t)

bool Init_KVH_IMU();

size_t GetKVHStatus(char* buffer, size_t size);
size_t GetKVHStatusShort(char* buffer, size_t size);

void KVH_EnterConfigMode(_ports_t port);
void KVH_ExitConfigMode();
void Receive_KVH_Datagram(uint8_t* buffer, size_t cnt);

#pragma pack(pop)

#endif /* KVH_H */    