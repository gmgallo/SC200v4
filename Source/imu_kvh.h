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
	LE_KVH_HEADER725 = 0x55FF81FE,		// Little Endian format. MSB is the last byte of the record, 0x55 for IMU model 1725. 
	//BE_KVH_HEADER775 = 0xFE81FF56,	// 4th byte is the record ID, 0x56 for IMU model 1775 IN BIG ENDIAN format.
	LE_KVH_HEADER775 = 0x56FF81FE,		// Little Endian format. MSB is the last byte of the record, 0x56 for IMU model 1775. 
	KVH_EXTBITHDR = 0xFE8100AA,
	KVH_EXBIT2HDR = 0xFE8100AB,
	KVH_MSG_BYTE0 = 0xFE,				// BIG ENDIAN BYTE ORDER
	KVH_MSG_BYTE1 = 0x81,
	KVH_MSG_BYTE2 = 0xFF,
	KVH_MSG_BYTE3 = 0x55,

} KVH_HEADERS;

typedef struct// ALL DATA IS RECEIVED IN BIG ENDIAN FORMAT
{
	uint32_t	header;		// 4 bytes
	float		Xgyro;		// 4 bytes
	float		Ygyro;
	float		Zgyro;
	float		Xaccel;
	float		Yaccel;
	float		Zaccel;
	uint8_t 	Status;
	uint8_t		Sequence;
	int16_t 	Temp;
	uint32_t	crc;

} kvh_msg_t, * PKVH_MSG;

#define KVH_RECORD_SIZE sizeof(kvh_msg_t)



void KVH_EnterConfigMode(_ports_t port);
void KVH_ExitConfigMode();
void Receive_KVH_Datagram(uint8_t* buffer, size_t cnt);

#pragma pack(pop)

#endif /* KVH_H */    