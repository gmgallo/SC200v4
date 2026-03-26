/*
 * Uart.c
 *
 *  Created on: Aug 29, 2021
 *      Author: Guillermo
 */

#include "common.h"
#include "dmacfg.h"

//#define DEBUG_GPS_ISR
//#define DEBUG_OEM7700_TX_ISR
//#define DEBUG_OEM7700_RX_ISR

#define DEBUG_IMU_ISR
#define UART_IMU_TP	(TP8)


/*************************************************************************
 * PDL (Peripheral Driver Library) is the hardware interface
 *
 * HAL (Hardware Abstraction Library) is a layer on top of PDL
 *
 *------------------------------------------------------------------------
 *
 * TX uses PDL - High Level functions in all serial ports
 *
 * ISR TX start - 7 usec at the beginning of the transmission
 * 		TX end  - 2 usec at the end of the transmission
 *
 *************************************************************************/

#define TMP_BUF_SIZE	(256U)

#define UART_IMU_IRQ_PRIORITY 		(4U) // IMU Logs
#define UART_OEM7700_IRQ_PRIORITY	(6U) // GNSS Logs
#define UART_COM_IRQ_PRIORITY		(7U) // SERIAL PORT LOGGER
#define UART_GPS_IRQ_PRIORITY		(7U) // PPS MSG
#define UART_J6_IRQ_PRIORITY		(7U) // PPS MSG

/*-----------------------------------------------------------------------
 * SetUartBaudRate()  	NEEDS REWORK
 *
 * uartscbclk - not defined in chcfg_peripherals must be manually identified from Device Configurator
 *-----------------------------------------------------------------------*/
int SetUartBaudRate(en_clk_dst_t uartscbclk, bauds_t bauds)
{
	/*
	uint32_t result = Cy_SysClk_PeriphAssignDivider(uartscbclk, CY_SYSCLK_DIV_16_5_BIT, divderNum[bauds]);
	if (result != CY_SYSCLK_SUCCESS )
	{
#ifdef DEBUG
		printf("Cy_SysClk_PeriphAssignDivider() error: %lX\n", result);
#endif
		return true;
	}
	*/
	return false;
}



int GetUartErrorStr(char* buf, int cnt, uint32_t err)
{
	memset(buf,0, cnt);
	int j = 0;

	if ( (err & CY_SCB_RX_INTR_OVERFLOW) != 0 )
		j += snprintf(buf,cnt, "%s", " RX_OVERFLOW");

	if ( (err & CY_SCB_RX_INTR_UART_FRAME_ERROR) != 0 )
		j += snprintf(buf+j,cnt-j, "%s", " RX_FRAME_ERROR");

	if ( (err & CY_SCB_RX_INTR_UART_PARITY_ERROR) != 0 )
		j += snprintf(buf+j,cnt-j, "%s", " RX_PARITY_ERROR");
	if (j == 0)
		j += snprintf(buf,cnt, "NO RX Errors");

	return j;
}

/*---------------------------------------------------------------------
 *  UART_OEM7700 - SERIAL PORT RECEIVER CONTROL
 *----------------------------------------------------------------
 *  UART_OEM7700       | 230400,N,8,1  | SCB1 - RX/TX P10.0/P10.1 |
 *----------------------------------------------------------------
 *---------------------------------------------------------------------*/
int32_t Uart_OEM7700_Error = 0;
#define UART_OEM7700_TMP_BUF_SIZE 	256

/* Allocate context for UART operation */
cy_stc_scb_uart_context_t oem_uart_contex;

uint8_t uartOem7700TmpBuf[UART_OEM7700_TMP_BUF_SIZE];


fnMessageProcessor _processOEM7700sMsg = Read_Novatel_Message;

void SetOEM7700dMsgProcessor( fnMessageProcessor fn )
{
	_processOEM7700sMsg = fn;
}


void OEM7700_SendString(const char_t*string)
{
    Cy_SCB_UART_PutString(UART_OEM7700_HW, string);
}


void Isr_Uart_Oem7700()
{
#ifdef DEBUG_OEM7700_RX_ISR
   	SET_DEBUG_TP(_TP2);
#endif

    if (0UL != (CY_SCB_RX_INTR & Cy_SCB_GetInterruptCause(UART_OEM7700_HW)))
    {
 		uint32_t srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(UART_OEM7700_HW);
		uint32_t srcErr = (srcInterrupt & CY_SCB_UART_RECEIVE_ERR);

		 /* Handle the error conditions */
		if (0UL != srcErr )
		{
			Uart_OEM7700_Error |= srcErr;
		}

		/* service receive interrupts */
		if (srcInterrupt & (CY_SCB_UART_RX_NOT_EMPTY|CY_SCB_RX_INTR_LEVEL) )
		{
			/* we assume that the receiving buffer is large enough to ingest all the bytes in the UART
			 * internal buffer (63 bytes) in a single call as a safety precaution.
			 * But, RX Fifo Not Empty interrupt must be enabled to receive byte by byte.
			 */
			uint32_t numCopied = Cy_SCB_UART_GetArray(UART_OEM7700_HW, uartOem7700TmpBuf, UART_OEM7700_TMP_BUF_SIZE );

			/*uint32_t trigLevel =*/ _processOEM7700sMsg(uartOem7700TmpBuf, numCopied );

			// SCB_TX_FIFO_CTRL(UART_OEM7700_HW) = _VAL2FLD(SCB_TX_FIFO_CTRL_TRIGGER_LEVEL, (trigLevel <= 127? trigLevel : 127) );
		}

		/* clear RX_interrupts */
		Cy_SCB_ClearRxInterrupt(UART_OEM7700_HW, srcInterrupt);
    }

#ifdef DEBUG_OEM7700_RX_ISR
   	CLEAR_DEBUG_TP(_TP2);
#endif

}

/*-----------------------------------------------------------------------------------
 * Init_Uart_Oem7700()
 *------------------------------------------------------------------------------------*/
#ifdef UART_OEM7700_HW
#define UART_OEM7700_CLK PCLK_SCB1_CLOCK
#endif

int Init_Uart_Oem7700()
{
  	cy_en_scb_uart_status_t result =
  			Cy_SCB_UART_Init(UART_OEM7700_HW, &UART_OEM7700_config, &oem_uart_contex);

	if ( result == CY_SCB_UART_SUCCESS )
	{
		 ConfigureInterrupt(UART_OEM7700_IRQ,UART_OEM7700_IRQ_PRIORITY,Isr_Uart_Oem7700);
		 Cy_SCB_SetRxInterrupt(UART_OEM7700_HW,CY_SCB_UART_RX_INTR);
		 Cy_SCB_SetRxInterruptMask(UART_OEM7700_HW, CY_SCB_UART_RX_INTR);  // enables ALL RX interrupts
		 Cy_SCB_UART_Enable(UART_OEM7700_HW);
	}

    return result;
}


void Uart_Oem7700_Send(uint8_t *txBuffer, size_t count)
{
	//Cy_SCB_UART_Transmit(UART_OEM7700_HW, txBuffer, count, &oem_uart_contex);

	Cy_SCB_UART_PutArrayBlocking(UART_OEM7700_HW, txBuffer, count);
}

/*-----------------------------------------------------------------------------------
 * Reconfig_Uart_Oem7700()
 *------------------------------------------------------------------------------------*/
void Reconfig_Uart_Oem7700(uint32_t level, bauds_t bauds)
{
	 cy_stc_scb_uart_config_t config;

	 Cy_SCB_UART_Disable(UART_OEM7700_HW, &oem_uart_contex);
	 Cy_SCB_UART_DeInit(UART_OEM7700_HW);

	 memcpy(&config, &UART_OEM7700_config, sizeof(cy_stc_scb_uart_config_t));

	 config.rxFifoTriggerLevel = level;

	 Cy_SCB_UART_Init(UART_OEM7700_HW, &config, &oem_uart_contex);
	// Cy_SCB_SetRxInterrupt(UART_OEM7700_HW,CY_SCB_UART_RX_INTR);

	// SetUartBaudRate(UART_OEM7700_CLK, bauds);

	 uint32_t mask = level == 0UL? CY_SCB_UART_RX_INTR : CY_SCB_UART_RX_INTR &(~CY_SCB_UART_RX_NOT_EMPTY);
	  Cy_SCB_SetRxInterruptMask(UART_OEM7700_HW, mask);

	 Cy_SCB_UART_Enable(UART_OEM7700_HW);
}


/*---------------------------------------------------------------------
 *  UART_COM- SERIAL PORT FOR DATA LOGGER
 *----------------------------------------------------------------
 *  UART_COM        | 115200,N,8,1  | SCB2 - RX/TX P9.0/P9.1   |
 *----------------------------------------------------------------
 *---------------------------------------------------------------------*/
int32_t Uart_COM_Error = 0;
bool    New_COM1_Message = false;
bool    COM1_TX_Complete = true;

/* Allocate context for UART operation */
cy_stc_scb_uart_context_t uart_COM_Context;

uint8_t tmpComBuffer[TMP_BUF_SIZE];

uint8_t comReadBuffer[TMP_BUF_SIZE];
uint8_t *pComReadBuf = comReadBuffer;  // needed for partial reads of input
size_t  comDataCount=0;


void Process_COM_Message(uint8_t *buf, size_t cnt )
{
	while ( cnt-- > 0  )
	{
		uint8_t ch = *buf++;

		if (comDataCount < ARRAY_SIZE(comReadBuffer) )
			pComReadBuf[comDataCount++] = ch;

		if ( IS_LINE_TERMINATOR(ch) && comDataCount > 0 ) // drop trailing characters if any
		{
			if (comDataCount < ARRAY_SIZE(comReadBuffer) ) // null terminate the string
				pComReadBuf[comDataCount] = 0;

			/* this logic assumes that the line will be read
			 * fast enough before the next character comes in
			 */
			pComReadBuf = comReadBuffer;
			New_COM1_Message = true;
		}
	};
}


void Isr_Uart_COM()
{
	uint32_t cause = Cy_SCB_GetInterruptCause(UART_COM_HW);

    if (0UL != (CY_SCB_RX_INTR & cause))
    {
		uint32_t srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(UART_COM_HW);
		uint32_t srcErr = (srcInterrupt & CY_SCB_UART_RECEIVE_ERR);

		 /* Handle the error conditions */
		if (0UL != srcErr )
		{
			Uart_COM_Error |= srcErr;
		}

		/* service receive interrupts */

		 srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(UART_COM_HW);

		if (srcInterrupt & (CY_SCB_UART_RX_NOT_EMPTY|CY_SCB_RX_INTR_LEVEL|CY_SCB_RX_INTR_FULL) )
		{
			/* we assume that the receiving buffer is large enough to read all the bytes in the UART
			 * internal buffer (63 bytes) in a single call as a safety precaution.
			 * But, RX Fifo Not Empty interrupt must be enabled to receive byte by byte.
			 */
			uint32_t  numCopied = Cy_SCB_UART_GetArray(UART_COM_HW, tmpComBuffer, ARRAY_SIZE(tmpComBuffer) );

			Process_COM_Message(tmpComBuffer, numCopied );
		}

		/* clear only RX interrupts */
		Cy_SCB_ClearRxInterrupt(UART_COM_HW, srcInterrupt );
    }
	if (cause  & CY_SCB_TX_INTR)
	{
		uint32_t txInterrupt = Cy_SCB_GetTxInterruptStatusMasked(UART_COM_HW);

		if (txInterrupt & CY_SCB_UART_TX_EMPTY)
		{
			COM1_TX_Complete = true;
		}
	}
	Cy_SCB_UART_Interrupt(UART_COM_HW, &uart_COM_Context);
}

void _uart_event_cb(uint32_t event)
{
	if (event & CY_SCB_UART_TRANSMIT_DONE_EVENT )
	{
		COM1_TX_Complete = true;
	}
}

/****************************************************************************
* PSOC_COM DCD Detect
*
* Detects a port open DCD detect pin activity P9.3.
* RTS or DTR must be connected to this pin.
*****************************************************************************/

bool COM1_Connected = false;

void dcd_detect_cb(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;

    if(event & CYHAL_GPIO_IRQ_RISE)
    {
    	int pin = TEST_HAL_PIN( PSOC_DCD);

    	printf("COM1 Disconnected (RISE), PIN= %d Event= %d\n", pin, event);

    	if (pin != 0)
    		COM1_Connected = false;
    }
    else
   	{
    	int pin = TEST_HAL_PIN( PSOC_DCD);

    	printf("COM1 Connected (FALL), PIN= %d Event= %d\n", pin, event);
    	Cy_SysLib_Delay(50);
    	 Process_COM_Message((uint8_t*)"PORTID\n",7);
		COM1_Connected = true;
    }

}

cyhal_gpio_callback_data_t dcd_detect_cb_data =
{
		dcd_detect_cb,
		NULL, NULL, 0
};

void Init_COM1_DCD_Detect()
{
	/* Initialize the PSOC COM1 Detect Pin P9.3 */
	Init_Input_Pin(PSOC_DCD, &dcd_detect_cb_data );
}


int Init_Uart_COM_Port()
{
	 cy_en_scb_uart_status_t result =
		Cy_SCB_UART_Init(UART_COM_HW, &UART_COM_config, &uart_COM_Context);

	if ( result == CY_SCB_UART_SUCCESS )
	{
		 ConfigureInterrupt(UART_COM_IRQ,UART_COM_IRQ_PRIORITY,Isr_Uart_COM);
		 Cy_SCB_UART_Enable(UART_COM_HW);
	}

	Cy_SCB_UART_RegisterCallback(UART_COM_HW, _uart_event_cb, &uart_COM_Context);
	Init_COM1_DCD_Detect();

	return result;
}


void Uart_COM_Send(uint8_t *txBuffer, size_t count)
{
	COM1_TX_Complete = false;

	Cy_SCB_UART_Transmit(UART_COM_HW, txBuffer, count, &uart_COM_Context);

	while (COM1_TX_Complete == false)
	;

	/* Blocking wait for transmission completion */
	// while (0UL != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(UART_COM_HW, &uart_COM_Context)))
	// {
	// }
}

void Uart_COM_Putc(uint8_t c)
{
	uint32_t count = 0;
	while (count == 0)
	{
		count = Cy_SCB_UART_Put(UART_COM_HW,  c);
	}
}

size_t Uart_COM_Read(uint8_t *rxBuffer, size_t count)
{
	size_t send = (comDataCount <= count )? comDataCount: count;
	memcpy(rxBuffer,pComReadBuf, send);
	comDataCount -= send;

	if (comDataCount > 0)	// data remaining
	{
		pComReadBuf += send;
		New_COM1_Message = true;
	}

	return send;
}


/*---------------------------------------------------------------------
 * UART_GPS - GPS MESSAGES
 *
 * Connectd to OEM7700 COM1 - PPS NMEA output - RX ONLY
 *----------------------------------------------------------------
 * UART_GPS NMEA MSG  | 115200,N,8,1  | SCB3 - RX P6.0           |
 *----------------------------------------------------------------
 *---------------------------------------------------------------------*/

#ifdef DEBUG
char gpsBuf[TMP_BUF_SIZE];
size_t gps_cnt=0;

void Process_GPS_Message(uint8_t *buf, size_t cnt )
{
	memset(gpsBuf, 0, sizeof(gpsBuf));
	memcpy(gpsBuf, buf, cnt);
	gps_cnt = cnt;
	new_gps_msg = true;
}

#endif


/* Allocate context for UART operation */
cy_stc_scb_uart_context_t uart_GPS_Context;

uint32_t Uart_GPS_Error = 0;

char uartGpsTmpBuf[TMP_BUF_SIZE];

fnMessageProcessor _processGpsMsg = Process_GPS_Message;

void SetGpdMsgProcessor( fnMessageProcessor fn )
{
	_processGpsMsg = fn;
}


/*-------------------------------------------------------------------------
 *  Isr_Uart_GPS()
 *
 *  ISR time duration Min / Max = 2.5 / 6 usec.
 *
 *  ZDA message 3.25 ms @ 115200 bauds
 *  GPGGA message 6.5 ms    @ 115200 bauds
 *
 *  ISR duty cycle < 1.4% @ 115200 bauds
 *-------------------------------------------------------------------------*/

void Isr_Uart_GPS()
{
#ifdef DEBUG_GPS_ISR
   	SET_DEBUG_TP(_TP2);
#endif

    if (0UL != (CY_SCB_RX_INTR & Cy_SCB_GetInterruptCause(UART_GPS_HW)))
    {
		uint32_t srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(UART_GPS_HW);
		uint32_t srcErr = (srcInterrupt & CY_SCB_UART_RECEIVE_ERR);

		 /* Handle the error conditions */
		if (0UL != srcErr )
		{
			Uart_GPS_Error |= srcErr;
		}

		/* service receive interrupts */
//		srcInterrupt = Cy_SCB_GetRxInterruptStatusMasked(UART_GPS_HW);

		if ( srcInterrupt & (CY_SCB_UART_RX_NOT_EMPTY|CY_SCB_RX_INTR_LEVEL|CY_SCB_RX_INTR_FULL) )
		{
			/* we assume that the receiving buffer is large enough to receive all the bytes in the UART
			 * internal buffer (63 bytes) in a single call as a safety precaution.
			 * But, RX Fifo Not Empty interrupt must be enabled to receive byte by byte.
			 */
			uint32_t  numCopied = Cy_SCB_UART_GetArray(UART_GPS_HW, uartGpsTmpBuf, ARRAY_SIZE(uartGpsTmpBuf) );

			 _processGpsMsg((uint8_t*)uartGpsTmpBuf, numCopied );
		}

		Cy_SCB_ClearRxInterrupt(UART_GPS_HW, srcInterrupt);
    }

    /* service transmit interrupts */
    Cy_SCB_UART_Interrupt(UART_GPS_HW, &uart_GPS_Context);

#ifdef DEBUG_GPS_ISR
 	CLEAR_DEBUG_TP(_TP2);
#endif
}

int Init_Uart_GPS()
{
  	cy_en_scb_uart_status_t result =
		Cy_SCB_UART_Init(UART_GPS_HW, &UART_GPS_config, &uart_GPS_Context);

	if ( result != CY_SCB_UART_SUCCESS )
		return 1;

	 Cy_SCB_UART_Enable(UART_GPS_HW);
	 ConfigureInterrupt(UART_GPS_IRQ,UART_GPS_IRQ_PRIORITY,Isr_Uart_GPS);
	 Cy_SCB_SetRxInterrupt (UART_GPS_HW,CY_SCB_UART_RX_INTR);
     Cy_SCB_SetRxInterruptMask(UART_GPS_HW, CY_SCB_UART_RX_INTR);

	 return 0;
}


void Disable_Uart_GPS()
{
	 Cy_SCB_UART_Disable(UART_GPS_HW, &uart_GPS_Context);
}

void Enable_Uart_GPS()
{
	Cy_SCB_UART_ClearRxFifo(UART_GPS_HW);
	Cy_SCB_UART_Enable(UART_GPS_HW);
}

void Uart_GPS_Send(uint8_t *txBuffer, size_t count)
{
	/* Blocking wait for previous transmission completion */
	while (0UL != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(UART_GPS_HW, &uart_GPS_Context)))
		;

	Cy_SCB_UART_Transmit(UART_GPS_HW, txBuffer, count, &uart_GPS_Context);
}


/*---------------------------------------------------------------------
 * UART IMU - IMU DATA IN
 *-----------------------------------------------------------------
 * UART_IMU           | 115200,N,8,1  |  SCB4 - RX/TX P8.0/P8.1   |
 *-----------------------------------------------------------------
 *---------------------------------------------------------------------*/
/* Allocate context for UART operation */
cy_stc_scb_uart_context_t uart_IMU_Context;

cy_stc_scb_uart_config_t _uart_IMU_Config;

uint32_t Discard_Chars	= 0;
uint32_t Uart_IMU_Error = 0;

#define IMU_TMP_BUFF  128
uint8_t uartImuTmpBuf[IMU_TMP_BUFF];


static inline void Uart_IMU_WaitForTxComplete()
{
	while (0UL != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(UART_IMU_HW, &uart_IMU_Context)))
		;
}


fnMessageProcessor _process_uart_imu_msg = NULL;

fnMessageProcessor SetImuMsgProcessor(fnMessageProcessor fn )
{
	fnMessageProcessor current = _process_uart_imu_msg;
	_process_uart_imu_msg = fn;
	return current;
}

uint32_t  num_purged;

volatile bool break_detect = false;

void Isr_Uart_IMU(void)
{

    /* Get RX interrupt sources */
	uint32_t InterruptCause = Cy_SCB_GetInterruptCause(UART_IMU_HW);

    if (0UL != (CY_SCB_RX_INTR & InterruptCause))
    {
#ifdef DEBUG_IMU_ISR
   	SET_DEBUG_TP(UART_IMU_TP);		// Break detect
#endif
     	uint32_t srcInterrupt =  Cy_SCB_GetRxInterruptStatusMasked(UART_IMU_HW);

    	if ((srcInterrupt & CY_SCB_RX_INTR_UART_BREAK_DETECT) != 0UL )
    	{
  			// discard any bytes in the RX buffer
   			num_purged = Cy_SCB_UART_GetArray(UART_IMU_HW, uartImuTmpBuf, ARRAY_SIZE(uartImuTmpBuf) );
   			Uart_IMU_Error = 0;

   			break_detect = true;
     	}
    	else
    	{
    		if ( break_detect )
    		{
    			break_detect = false;
    		}
        	Uart_IMU_Error |= ( srcInterrupt & CY_SCB_UART_RECEIVE_ERR);

    		if (0UL != ( srcInterrupt & (CY_SCB_UART_RX_NOT_EMPTY|CY_SCB_RX_INTR_LEVEL|CY_SCB_RX_INTR_FULL|CY_SCB_RX_INTR_OVERFLOW)) )
			{
				uint32_t  numCopied = Cy_SCB_UART_GetArray(UART_IMU_HW, uartImuTmpBuf, ARRAY_SIZE(uartImuTmpBuf) );

				fnMessageProcessor fn = _process_uart_imu_msg;

				/* process received data */
				if(fn != NULL )
					fn( uartImuTmpBuf, numCopied );
			}

#ifdef DEBUG_IMU_ISR
   	CLEAR_DEBUG_TP(UART_IMU_TP);		// Clear at the end of  read (1 or n bytes)
#endif
    	}

		Cy_SCB_ClearRxInterrupt(UART_IMU_HW, srcInterrupt);
    }

    /* service transmit interrupts */
   	Cy_SCB_UART_Interrupt(UART_IMU_HW, &uart_IMU_Context);
}

/*---------------------------------------------------------------------------
 *  Change UART Baud Rate
 *  Uses a preassigned clock source
 *---------------------------------------------------------------------------*/
typedef struct
{
	uint32_t Integer;
	uint32_t Fractional;
	uint32_t Oversample;
}_clk_divider;


_clk_divider Oversample_ClkDividers[] = // dividers for 16.5 clocks
{
	{ 72, 16, 12 },		// 115200
	{ 54,  8,  8 },		// 230400
//	{ 15, 16, 14 },		// 460800
	{ 14, 10, 14 },		// 460800 <<================ for debug only!
	{  9,  1, 12 },		// 921600
};


/*-----------------------------------------------------------------------
 *  _cycfg_Uart_IMU_clock()
 *
 * uartscbclk - Is not defined in chcfg_peripherals.
 * It must be manually named in Device Configurator
 -----------------------------------------------------------------------*/
bool _cycfg_Uart_IMU_clock(uint32_t div, uint32_t frac )
{
    Cy_SysClk_PeriphDisableDivider(CLK_IMU_UART_HW, CLK_IMU_UART_NUM);
    Cy_SysClk_PeriphSetFracDivider(CLK_IMU_UART_HW, CLK_IMU_UART_NUM, div, frac);
    Cy_SysClk_PeriphEnableDivider(CLK_IMU_UART_HW, CLK_IMU_UART_NUM	);

    uint32_t _d,_f;

    Cy_SysClk_PeriphGetFracDivider(CLK_IMU_UART_HW, CLK_IMU_UART_NUM, &_d, &_f);

    return (div != _d || frac != _f);
}

void Set_Uart_IMU_Baudrate(bauds_t bauds)
{
	if (bauds >= B115200 && bauds <= B921600) 
	{
		_cycfg_Uart_IMU_clock(Oversample_ClkDividers[bauds].Integer, Oversample_ClkDividers[bauds].Fractional);
		_uart_IMU_Config.oversample = Oversample_ClkDividers[bauds].Oversample;
	}
}


int Init_Uart_IMU(bauds_t bauds)
{
    /* Initialize, configure baud rate and enable UART interrupt
     * The UART interrupt sources are set in _uart_IMU_Config  in the Device Configurator */

	 memcpy(&_uart_IMU_Config, &UART_IMU_config, sizeof(cy_stc_scb_uart_config_t));

	 Set_Uart_IMU_Baudrate(bauds);

 	cy_en_scb_uart_status_t result =
		Cy_SCB_UART_Init(UART_IMU_HW, &_uart_IMU_Config, &uart_IMU_Context );

	if ( result == CY_SCB_UART_SUCCESS )
	{
		 ConfigureInterrupt(UART_IMU_IRQ, UART_IMU_IRQ_PRIORITY,Isr_Uart_IMU );
		 Cy_SCB_SetRxInterrupt(UART_IMU_HW,CY_SCB_UART_RX_INTR);
		 Cy_SCB_SetRxInterruptMask(UART_IMU_HW, CY_SCB_UART_RX_INTR);

		 Cy_SCB_UART_Enable(UART_IMU_HW);
	 }
	 return result;
}

/*-----------------------------------------------------------------------------------------------
 * Reconfig_Uart_IMU(uint32_t record_length, bool enable)
 *
 * Changes the number of bytes in RX_FIFO that triggers an interrupt.
 * Returns: the previous level set.
 * record_length actual size -1. and must be <= 63 (RX FIFO size)
 *----------------------------------------------------------------------------------------------*/
#ifdef UART_IMU_HW
#define UART_IMU_CLK	PCLK_SCB4_CLOCK  // <---- MANUAL UPDATE NEEDED IF UART_IMU CHANGED TO ANOTHER SCB!!
#endif

uint32_t Reconfig_Uart_IMU(uint32_t record_length, bool enable)
{
	 Cy_SCB_UART_Disable(UART_IMU_HW, &uart_IMU_Context);
	 Cy_SCB_UART_DeInit(UART_IMU_HW);

	 uint32_t old_level = _uart_IMU_Config.rxFifoTriggerLevel;
	 _uart_IMU_Config.rxFifoTriggerLevel = record_length;

	 Cy_SCB_UART_Init(UART_IMU_HW, &_uart_IMU_Config, &uart_IMU_Context);
	 Cy_SCB_SetRxInterrupt(UART_IMU_HW,CY_SCB_UART_RX_INTR);

	 if ( enable )
		 Cy_SCB_UART_Enable(UART_IMU_HW);

	 return old_level;
}

void Clear_Uart_IMU_Buffers(bool enable)
{
	 Cy_SCB_UART_Disable(UART_IMU_HW, &uart_IMU_Context);

	 if ( enable )
		 Cy_SCB_UART_Enable(UART_IMU_HW);
}

void Disable_Uart_IMU_RX_Interrupt()
{
	 Cy_SCB_UART_Disable(UART_IMU_HW, &uart_IMU_Context);
	 Cy_SCB_SetRxInterrupt(UART_IMU_HW,0);
	 Cy_SCB_UART_Enable(UART_IMU_HW);
}


void Disable_Uart_IMU()
{
	 Cy_SCB_UART_Disable(UART_IMU_HW, &uart_IMU_Context);
}


void Enable_Uart_IMU()
{
	Cy_SCB_UART_ClearRxFifo(UART_IMU_HW);
	Cy_SCB_UART_ClearTxFifo(UART_IMU_HW);

	Cy_SCB_UART_Enable(UART_IMU_HW);
}

void Uart_IMU_Send(uint8_t *txBuffer, size_t count)
{
	Cy_SCB_UART_Transmit(UART_IMU_HW, txBuffer, count, &uart_IMU_Context);
	Uart_IMU_WaitForTxComplete();
}

void Uart_IMU_SendString(const char_t*string)
{
    Cy_SCB_UART_PutString(UART_IMU_HW, string);
	Uart_IMU_WaitForTxComplete();
}

/*---------------------------------------------------------------------------------*
 * UART J6 - Aux data over MB J6 connector                                         |
 *---------------------------------------------------------------------------------|
 * UART_J6  N,8,1 | 16 div  | Bauds: Same as UART_IMU   SCB6 - RX/TX P13.0/P13.1   |
 *---------------------------------------------------------------------------------*/
/* Allocate context for UART operation */
cy_stc_scb_uart_context_t uart_J6_Context;

cy_stc_scb_uart_config_t _uart_J6_Config;


void Uart_J6_Send(uint8_t *txBuffer, size_t count)
{
	/* Blocking wait for previous transmission completion */
	while (0UL != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(UART_J6_HW, &uart_J6_Context)))
		;

	Cy_SCB_UART_Transmit(UART_J6_HW, txBuffer, count, &uart_J6_Context);
}


void Disable_Uart_J6()
{
	 Cy_SCB_UART_Disable(UART_J6_HW, &uart_J6_Context);
}

void Enable_Uart_J6()
{
	Cy_SCB_UART_ClearRxFifo(UART_J6_HW);
	Cy_SCB_UART_ClearTxFifo(UART_J6_HW);

	Cy_SCB_UART_Enable(UART_J6_HW);
}


/*--------------------------------------------------------------------- ISR */
uint32_t J6_Discard_Chars	= 0;
uint32_t Uart_J6_Error 		= 0;

#define J6_TMP_BUFF  128
uint8_t uartJ6TmpBuf[J6_TMP_BUFF];

fnMessageProcessor _process_uart_J6_msg = Uart_J6_Send;  // default to echo

void Isr_Uart_J6(void)
{
#ifdef DEBUG_IJ6_ISR
   	SET_DEBUG_TP(UART_J6_TP);		// Use TP1 to monitor ISR duration
#endif

    /* Get RX interrupt sources */
    if (0UL != (CY_SCB_RX_INTR & Cy_SCB_GetInterruptCause(UART_J6_HW) ))
    {
    	uint32_t srcInterrupt =  Cy_SCB_GetRxInterruptStatusMasked(UART_J6_HW);
    	uint32_t srcErr = ( srcInterrupt & CY_SCB_UART_RECEIVE_ERR);

		//* Handle the error conditions
		if (0UL != srcErr )
		{
			Uart_J6_Error |= srcErr;
		}

		if (0UL != ( srcInterrupt & (CY_SCB_UART_RX_NOT_EMPTY|CY_SCB_RX_INTR_LEVEL|CY_SCB_RX_INTR_FULL)) )
		{
			uint32_t  numCopied = Cy_SCB_UART_GetArray(UART_J6_HW, uartJ6TmpBuf, ARRAY_SIZE(uartJ6TmpBuf) );

			/* process received data */
			_process_uart_J6_msg( uartJ6TmpBuf, numCopied );
		}

		Cy_SCB_ClearRxInterrupt(UART_J6_HW, srcInterrupt);
    }

    /* service transmit interrupts */
     Cy_SCB_UART_Interrupt(UART_J6_HW, &uart_J6_Context);

#ifdef DEBUG_J6_ISR
   	CLEAR_DEBUG_TP(UART_J6_TP);
#endif
}

fnMessageProcessor Set_UART_J6_MsgProcessor(fnMessageProcessor fn )
{
	fnMessageProcessor current = _process_uart_J6_msg;
	_process_uart_J6_msg = fn;
	return current;
}

int Init_Uart_J6()
{
    /* Initializeand enable UART interrupt - The baudrate is the same as UART_IMU
     * The UART interrupt sources are set in _uart_IMU_Config  in the Device Configurator */

	 memcpy(&_uart_J6_Config, &UART_J6_config, sizeof(cy_stc_scb_uart_config_t));

 	cy_en_scb_uart_status_t result =
		Cy_SCB_UART_Init(UART_J6_HW, &_uart_J6_Config, &uart_J6_Context );

	if ( result == CY_SCB_UART_SUCCESS )
	{
		 ConfigureInterrupt(UART_J6_IRQ, UART_J6_IRQ_PRIORITY,Isr_Uart_J6 );
		 Cy_SCB_SetRxInterrupt(UART_J6_HW,CY_SCB_UART_RX_INTR);
		 Cy_SCB_SetRxInterruptMask(UART_J6_HW, CY_SCB_UART_RX_INTR);

		 Cy_SCB_UART_Enable(UART_J6_HW);
	 }
	 return result;
}

