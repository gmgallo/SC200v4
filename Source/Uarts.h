/*
 * Uart.h
 *
 *  Created on: Aug 29, 2021
 *      Author: Guillermo
 */

#ifndef UART_H_
#define UART_H_

#include "globaldefs.h"

typedef void (*fnMessageProcessor)(uint8_t* txBuffer, size_t count);

typedef uint32_t (*fnMessageProcessor_2)(uint8_t* txBuffer, size_t count);

typedef enum // Predefined UART baud rates
{
	B115200 = 0,
	B230400 = 1,
	B460800 = 2,
	B921600 = 3,
} bauds_t;


typedef enum _Ports 	// port ID bit fields
{
	INVALID_PORT = 0X00,
	UART_COM1    = 0x01,
	USB_COM1 	 = 0x02,
	USB_COM2	 = 0x04,
	UART_OEM7700 = 0x08,
	UART_CONSOLE = 0x10,
	UART_IMU	 = 0x20,
	UART_J6		 = 0x40,

}_ports_t;


const char* GetPortName(_ports_t port);		/* defined in cmdprocessor.c */


int GetUartErrorStr(char* buf, int cnt, uint32_t err);

/*
 * GPS UART - (not used)
 */
int Init_Uart_GPS();
void Disable_Uart_GPS();
void Enable_Uart_GPS();
void SetGpdMsgProcessor(fnMessageProcessor fn );

size_t Uart_GPS_Receive(uint8_t *txBuffer, size_t count);
void   Uart_GPS_Send(uint8_t *txBuffer, size_t count);

/*
 * IMU UAART
 * Uart_IMU_Send() is not used in FSAS IMU (cable connection missing!)
 */
extern uint32_t Uart_IMU_Error;

int  Init_Uart_IMU(bauds_t bauds);
void Disable_Uart_IMU();
void Enable_Uart_IMU();
uint32_t Reconfig_Uart_IMU(uint32_t level, bool enable);
void Disable_Uart_IMU_RX_Interrupt();
void Clear_Uart_IMU_Buffers(bool enable);

fnMessageProcessor SetImuMsgProcessor( fnMessageProcessor fn);
void Uart_IMU_Send(uint8_t *txBuffer, size_t count);
void Uart_IMU_SendString(const char_t*string);
//void Uart_IMU_WaitForTxComplete();

void Set_Uart_IMU_Baudrate(bauds_t bauds);
bool _cycfg_Uart_IMU_clock(uint32_t div, uint32_t frac, uint32_t oversample );
void Sync_Uart_IMU(uint32_t record_length);
void Disable_Uart_IMU_Watchdog();

/*------------------------------------------------------------------
 * COM UART
 */
extern bool New_COM1_Message;

int    Init_Uart_COM_Port();
void   Uart_COM_Send(uint8_t *txBuffer, size_t count);
size_t Uart_COM_Read(uint8_t *txBuffer, size_t count);
void   Uart_COM_Putc(uint8_t c);

/*
 * OEM7700 UART
 */
int  Init_Uart_Oem7700();
void Uart_Oem7700_Send(uint8_t *txBuffer, size_t count);
void SetOEM7700dMsgProcessor( fnMessageProcessor fn );
void Reconfig_Uart_Oem7700(uint32_t level, bauds_t bauds);
void Check_OEM7700_UartErrors();
void OEM7700_SendString(const char_t* string);

extern uint32_t Uart_IMU_error;
extern uint32_t Uart_GPS_error;
extern uint32_t Uart_OEM7700_error;
extern uint32_t Uart_LOGGER_error;
extern uint32_t Uart_J6_error;

extern bool COM1_Connected;

/* debug helpers */
#ifdef DEBUG
extern char gpsBuf[];
extern size_t gps_cnt;
#endif

/*------------------------------------------------------------------
 * UART J6
 */
int Init_Uart_J6();
void Uart_J6_Send(uint8_t *txBuffer, size_t count);
void Disable_Uart_J6();
void Enable_Uart_J6();
fnMessageProcessor Set_UART_J6_MsgProcessor(fnMessageProcessor fn);

#endif /* UART_H_ */
