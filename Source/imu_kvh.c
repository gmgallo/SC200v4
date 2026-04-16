/*
 * KVH.h - KVH IMU definitions and functions
 *
 * Copyright (C) 2024 - 2026 - G2 Airborne Systems.
 * Version 1.00 - Mar. 2026 - Initial development of KVH IMU support.
 */
#include "common.h"


uint32_t kvh_crc(const void* data, size_t data_len);

char* kvh_test_cmd = "=BIT\n";
char* kvh_enter_config_cmd = "=CONFIG,1\n=MSYNC,EXT\n=BAUD,460800\n=CONFIG,0\n";
char* kvh_signin_cmd = "?WS\n";
char* kvh_exit_config_cmd  = "=CONFIG,0\n";

char* kvh_bauds_115200 = "=BAUD,115200\n";
char* kvh_bauds_460800 = "=BAUD,460800\n";
char* kvh_bauds_576000 = "=BAUD,576000\n";
char* kvh_bauds_921600 = "=BAUD,921600\n";

_ports_t console_port;
fnMessageProcessor prev_msg_processor;

char kvh_console_buf[256];
uint32_t kvh_console_buf_cnt = 0;

void kvh_console_msg_processor(uint8_t* buf, size_t cnt)
{
    while (cnt-- > 0)
    {
        char c = *buf++;

        if (c == '\n' || c == '\r' )
        {
            if (kvh_console_buf_cnt > 0)
            {	// ignore empty lines
                kvh_console_buf[kvh_console_buf_cnt++] = '\n';
                kvh_console_buf[kvh_console_buf_cnt] = 0;
                PostAssyncMessage((uint8_t*)kvh_console_buf, kvh_console_buf_cnt, console_port);
            }
            kvh_console_buf_cnt = 0;
        }
        else if (kvh_console_buf_cnt < sizeof(kvh_console_buf) - 2)
        {
            kvh_console_buf[kvh_console_buf_cnt++] = c;
        }
    }
}

void KVH_EnterConfigMode(_ports_t port)
{
    console_port = port;
  //  Disable_Uart_IMU_RX_Interrupt();
    prev_msg_processor =SetImuMsgProcessor(NULL);

    Reconfig_Uart_IMU(0, true);  // One interrupt per character
    Uart_IMU_SendString((char_t*)kvh_enter_config_cmd);
    Cy_SysLib_Delay(100);
    SetImuMsgProcessor(kvh_console_msg_processor);
  //  Cy_SysLib_Delay(100);
  //  Uart_IMU_SendString((char_t*)kvh_signin_cmd);
}

void KVH_ExitConfigMode()
{
    Reconfig_Uart_IMU(KVH_RECORD_SIZE-1, true);
    SetImuMsgProcessor(prev_msg_processor);
    Uart_IMU_SendString((char_t*)kvh_exit_config_cmd);
}   



/****************************************************************************
 * Format_FSAS_SN_to_NovatelRawSX()
 *
 *  - Checks record integrity and posts it as tXINS record.
 *
 ****************************************************************************/
void Format_KVH_to_NovatelRawSX( PKVH_MSG pmsg )
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

	_RawImuSX.imu_info   = pmsg->Status != 0? 1: 0;	// Biy 0 gl
	_RawImuSX.imu_type   = KVH_1750;
	_RawImuSX.imu_status = pmsg->Status;

	_RawImuSX.gnss_week    = UsecEventTime.GPSWeek;
	_RawImuSX.week_seconds = UsecEventTime.WeekSeconds;		// a double in full seconds

	_RawImuSX.z_accel  =(int32_t) pmsg->Zaccel;	
    _RawImuSX._y_accel = (int32_t) pmsg->Yaccel;
	_RawImuSX.x_accel  = (int32_t) pmsg->Xaccel;

	_RawImuSX.z_gyro  = (int32_t) pmsg->Zgyro;	
    _RawImuSX._y_gyro = (int32_t) pmsg->Ygyro;
	_RawImuSX.x_gyro  = (int32_t) pmsg->Xgyro;

	_RawImuSX.crc = CalculateBlockCRC32( RAWIMUSX_CRC_OFFSET, (void*)&_RawImuSX );

	if ( Enable_IMU_Logging)
		Send_IMU_Record( (void*)&_RawImuSX, sizeof(RAWIMUSX_t));
}

/*----------------------------------------------------------------------
* KVH IMU decodidng and formatting functions
*----------------------------------------------------------------------*/

uint32_t kvh_uart_error = 0;
uint32_t kvh_crc_error = 0;

uint32_t kvh_msg_count = 0;
uint8_t kvh_msg_buf[KVH_RECORD_SIZE+2];
uint8_t kvh_status;

bool Init_KVH_IMU()
{
    Stop_IMU_Trigger_Frequency();						// In case we have a warm restart
    Init_Uart_IMU(B460800);
    SetImuMsgProcessor( Receive_KVH_Datagram );		    // Switch to IMU data decoding
    Sync_Uart_IMU(KVH_RECORD_SIZE);
    Set_IMU_Trigger_Frequency(DEFAULT_IMU_FREQUENCY);	// 200hz start IMU trigger
//    Init_IMU_Trigger_Monitor(DEFAULT_IMU_FREQUENCY);	// event timing for IMU data timestamp
    Init_USec_Timer();
    
    return true;
}

size_t GetKVHStatusShort(char* buffer, size_t size)
{
    bool gyro_error = (kvh_status & (KVH_STATUS_XG | KVH_STATUS_YG | KVH_STATUS_ZG))
                != (KVH_STATUS_XG | KVH_STATUS_YG | KVH_STATUS_ZG);

    bool accel_error = (kvh_status & (KVH_STATUS_XA | KVH_STATUS_YA | KVH_STATUS_ZA))
                    != (KVH_STATUS_XA | KVH_STATUS_YA | KVH_STATUS_ZA);
   
	return snprintf(buffer, size,"%d,%.2Xh,0,0,0,0,0,%c,",
			(int)KVH_1750, 
            (int)kvh_status,
            (int)((gyro_error || accel_error)? 'N' : 'G'));

        
}

size_t GetKVHStatus(char* buffer, size_t size)
{
    bool gyro_error = (kvh_status & (KVH_STATUS_XG | KVH_STATUS_YG | KVH_STATUS_ZG))
                != (KVH_STATUS_XG | KVH_STATUS_YG | KVH_STATUS_ZG);

    bool accel_error = (kvh_status & (KVH_STATUS_XA | KVH_STATUS_YA | KVH_STATUS_ZA))
                    != (KVH_STATUS_XA | KVH_STATUS_YA | KVH_STATUS_ZA);
   
    size_t cnt = snprintf(buffer, size,"IMU Type: %s, Status: %.2Xh, Gyro error: %c, Accel error: %c\n"
        "CRC Errors: %ld, UART Errors: %ld, Records: %ld\nClockDif: %.1lf ClockAdj: %.3lf\n",
        "KVH-1725",
        (int)kvh_status,
        (int)(gyro_error? 'Y' : 'N'),
        (int)(accel_error? 'Y' : 'N'),
        (long)(kvh_crc_error),
        (long)(kvh_uart_error),
        (long)(kvh_msg_count),
        ClockDif,
        CountAdjust);

    kvh_crc_error = 0;
    kvh_uart_error = 0;    

    return cnt;
}

void Decode_KVH_Datagram()
{
    uint32_t* p = (uint32_t*)kvh_msg_buf;

    for (int i=0; i<7; i++)
    {
       *p = __builtin_bswap32(*p);
       p++;
    }
    
    PKVH_MSG pmsg = (PKVH_MSG)kvh_msg_buf;
    pmsg->Temp = __builtin_bswap16(pmsg->Temp);
    kvh_status = pmsg->Status;
    kvh_msg_count++;

    Format_KVH_to_NovatelRawSX(pmsg);
}

void Receive_KVH_Datagram(uint8_t* buffer, size_t cnt)
{
	if (Uart_IMU_Error != 0)
	{
		kvh_uart_error |= Uart_IMU_Error;
		Uart_IMU_Error = 0;
	}

    if (cnt < KVH_RECORD_SIZE)
    {
        // not a full record, ignore
        return;
    }

    PKVH_MSG pmsg = (PKVH_MSG)buffer;

    if (pmsg->header == LE_KVH_HEADER )
    {
        memcpy(kvh_msg_buf, buffer, KVH_RECORD_SIZE);

        if (kvh_crc(kvh_msg_buf, KVH_RECORD_SIZE) == 0)
        {
            Decode_KVH_Datagram();
        }
        else
        {
            kvh_crc_error++;
        }
    }
    else
    {
        Sync_Uart_IMU(KVH_RECORD_SIZE);
    }


#ifdef DEBUG_KVH
	// end of conversion
//	CLEAR_DEBUG_TP(TP8);
#endif

}


/**
 * Static table used for the table_driven implementation of CRC. 
 */
static const uint32_t crc_table[256] =
{
    0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
    0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
    0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
    0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
    0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
    0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
    0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
    0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
    0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
    0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
    0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
    0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
    0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
    0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
    0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
    0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
    0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
    0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
    0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
    0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
    0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
    0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
    0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
    0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
    0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
    0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
    0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
    0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
    0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
    0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
    0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
    0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
    0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
    0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
    0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
    0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
    0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
    0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
    0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
    0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
    0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
    0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
    0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

#define	CRC_START_32 0xFFFFFFFFul

uint32_t kvh_crc(const void* data, size_t data_len)
{
    uint32_t crc = CRC_START_32;

    const unsigned char* d = (const unsigned char*)data;
    unsigned int tbl_idx;

    while (data_len--) {
        tbl_idx = ((crc >> 24) ^ *d) & 0xff;
        crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xffffffff;
        d++;
    }
    return crc & CRC_START_32;
}