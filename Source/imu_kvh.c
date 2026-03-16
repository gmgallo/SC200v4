/*
 * KVH.h - KVH IMU definitions and functions
 *
 * Copyright (C) 2024 - 2026 - G2 Airborne Systems.
 * Version 1.00 - Mar. 2026 - Initial development of KVH IMU support.
 */
#include "common.h"
#include "imu_kvh.h"

char* kvh_test_cmd = "=BIT\n";
char* kvh_enter_config_cmd = "=CONFIG,1\n";
char* kvh_exit_config_cmd  = "=CONFIG,0\n";

char* kvh_bauds_115200 = "=BAUD,115200\n";
char* kvh_bauds_460800 = "=BAUD,460800\n";
char* kvh_bauds_576000 = "=BAUD,576000\n";
char* kvh_bauds_921600 = "=BAUD,921600\n";

_ports_t console_port;
fnMessageProcessor prev_msg_processor;

void kvh_console_msg_processor(uint8_t* buf, size_t cnt)
{
    SendToPort(console_port, buf, cnt);
}
void KVH_EnterConfigMode(_ports_t port)
{
    console_port = port;
     Uart_IMU_Send((uint8_t*)kvh_enter_config_cmd, strlen(kvh_enter_config_cmd));
     prev_msg_processor = SetImuMsgProcessor(kvh_console_msg_processor);
}

void KVH_ExitConfigMode()
{
    Uart_IMU_Send((uint8_t*)kvh_exit_config_cmd, strlen(kvh_exit_config_cmd));
    SetImuMsgProcessor(prev_msg_processor);
}   

uint32_t kvh_uart_error = 0;
int kvh_msg_count = 0;
uint8_t kvh_msg_buf[KVH_MSG_SIZE+2];

void Decode_KVH_Datagram()
{
    // fill in and convert to NOVATEL_RAW FORMAT

}

void Receive_KVH_Datagram(uint8_t* buffer, size_t cnt)
{
	if (Uart_IMU_Error != 0)
	{
		kvh_uart_error |= Uart_IMU_Error;
		Uart_IMU_Error = 0;
	}

	while(cnt--)
	{
		uint8_t ch = *buffer++;

		if (kvh_msg_count == 0 && ch != KVH_MSG_BYTE0) 	/* wait for record ID */
        {
			continue;
		}
        else if (kvh_msg_count == 1 && ch != KVH_MSG_BYTE1)
        {
            kvh_msg_count = 0;		// not a valid record, wait for next ID
            continue;
        }
        else if (kvh_msg_count == 2 && ch != KVH_MSG_BYTE2)
        {
            kvh_msg_count = 0;		// not a valid record, wait for next ID
            continue;
        }
        else if (kvh_msg_count == 3 && ch != KVH_MSG_BYTE3)
        {
            kvh_msg_count = 0;		// not a valid record, wait for next ID
            continue;
        }   
        kvh_msg_buf[kvh_msg_count++] = ch;

		if (kvh_msg_count == KVH_MSG_SIZE )
        {
            Decode_KVH_Datagram();
            kvh_msg_count = 0;
        }

#ifdef DEBUG_KVH
CLEAR_DEBUG_TP(STIM_ISR_TP);		// end of record
#endif
	}

#ifdef DEBUG_KVH
	// end of conversion
//	CLEAR_DEBUG_TP(TP8);
#endif

}

