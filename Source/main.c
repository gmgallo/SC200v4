/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 of G2-SC200
*
*              This application interfaces the FSAS, STIM300 or KVH IMUs with 
*              NovAtel OEM7700 GNSS receivers and an external data logger cpu.
*              It feeds time-tagged IMU and GNSS records to the data logger.
*
* 	Copyright (C) 2024 - 2026 - G2 Airborne Systems.
*
*   ## USB Device Specific Instructions

		The user must ensure that the parameters selected in the USB Device personality
		are aligned with the descriptor configuration in the USB Configurator, because
		there is no connection between the USB Device personality in the Device
		Configurator and USB Configurator.

		Specifically, parameter "Endpoints Mask" in the USB personality must be aligned
		with the endpoints selected in the USB Configurator. If DMA Automatic mode is
		selected, parameter  "Endpoint Buffer Size" must be aligned with the total size
		of the endpoint buffers allocated in the USB Configurator.
*
* Version 3.0 - Feb. 2024 - Released to RUDAZ-CH
* Version 3.2 - June 2025 - Released to STOFFEL-CA
* 		Major code revisions for FSAS IMU, FlexFifo
* 		New GPIO for Scanner power detector, requires scanner interface PCB added to NMEA ouutput
* Version 3.21 - Aug. 2025 Revised STIM IMU code for ESTEIO
* Version 3.22 - Sep. 2025 Corrected FSAS IMU time mark in RAWIMUSX records.
* Version 3.23 - Sep. 2025 Added clock correction for IMU timing based on PPS
*			   - Corrected IMU time gap at 1 seccond of logging.
* Version 4.00 - Mar. 2026 - Added support for KVH IMU.
********************************************************************************/
#include "common.h"

/* __TIMESTAMP__ is a gcc defined, fix length string of the type: Thu Jun 02 19:13: 2022 */

const char VersionString[] = "G2-SC200 - Firmware Version 4.0 - " __TIMESTAMP__;

const char _consoleHeader[] ="\r\n"
		"*******************************************************************************\r\n"
	    "*         - G2-SC200 - System Controller with built-in GNSS/INS -             *\r\n"
		"*                Copyright (C) 2024 - 2026, G2 AIRBORNE SYSTEMS               *\r\n"
		"*                                                                             *\r\n"
		"*       THIS FIRMWARE VERSION SUPPORTS IMAR FSAS, STIM300, and KVH IMUs       *\r\n"
		"*-----------------------------------------------------------------------------*\r\n"
		"*       %s            *\r\n" /* This line to be filled by  IDString */
		"*******************************************************************************\r\n\n";

sys_config_t SysConfig;


/********************************************************************************************
*                                                                                           *
*   PINS AND PERIPHERALS - CY8CPROTO-062-4343W BOARD  | AP1050 INTERFACE BOARD              *
*                                                                                           *
*--------------------------------------------------------------------------------------------
* DEVICE / FUNCTION  | PARAMETER     | CY8CPROTO-062-4343W      | G2-SC200 IF BOARD         |
*--------------------------------------------------------------------------------------------
* UART_OEM7700 CTL   | 115200,N,8,1  | SCB6 - RX/TX P12.0/P12.1 |  SCB1 - RX/TX P10.0/P10.1 |
*--------------------------------------------------------------------------------------------
* UART_GPS NMEA MSG  | 115200,N,8,1  | SCB3 - RX/TX P6.0        |  SCB3 - RX P6.0 (no TX)   | (not used)
*--------------------------------------------------------------------------------------------
* UART_IMU           | 115200,N,8,1  | SCB4 - RX/TX P8.0/P8.1   |  SCB4 - RX/TX P8.0/P8.1   |
*--------------------------------------------------------------------------------------------
* UART_LOGGER (PSOC) | 115200,N,8,1  | SCB2 - RX/TX P9.0/P9.1   |  SCB2 - RX/TX P9.0/P9.1   | renamed COM
*--------------------------------------------------------------------------------------------
* UART_AUX1 (DEBUG)  | 115200,N,8,1  | SCB5 - RX/TX P5.0/P5.1   |  SCB5 - RX/TX P5.0/P5.1   | (1)
*--------------------------------------------------------------------------------------------
* TDAS STROBE        |    50us       | TCPMW[0].1 P6.2          |   TCPMW[0].1 P6.2         |
*--------------------------------------------------------------------------------------------
* PPS_IN             |    IN         | P6.5                     |    P6.5 (from OEM7700)    |
*--------------------------------------------------------------------------------------------
* PPS_MONITOR        |    OUT        |      N/A                 |    P3.1 (PPS indicator)   |
*--------------------------------------------------------------------------------------------
* NOGO               |    IN         |   P1.0 (PROTO_NOGO)      |    P3.2 (FROM IMU)        |
*--------------------------------------------------------------------------------------------
* INIT_BIT           |    OUT        |      N/A                 |    P3.0 (To IMU)          |
*--------------------------------------------------------------------------------------------
* AUX1_RX (1)        |    IN         |      P5.0                |    P5.0                   |
*--------------------------------------------------------------------------------------------
* AUX1_TX (1)        |    OUT        |      P5.1                |    P5.1                   |
*--------------------------------------------------------------------------------------------
* AUX1_RTS (1)       |    OUT        |      P5.2                |    P5.2                   |
*--------------------------------------------------------------------------------------------
* AUX1_CTS  (1)      |    IN         |      P5.3                |    P5.3                   |
*--------------------------------------------------------------------------------------------
* EVENT_OUT_1        |    IN         |      N/A                 |    P4.1 (From OEM770)     |
*--------------------------------------------------------------------------------------------
* USB_V_SENS         |    IN         |     P5.7 (USB_V_SENS2)   |    P3.3                   |
*--------------------------------------------------------------------------------------------
* TP1                |    OUT        | P9.4                     |   P3.5	(_TP1)	  		|
*--------------------------------------------------------------------------------------------
* TP2                |    OUT        | P9.5                     |   P3.4    (SI_TP2)        |
*--------------------------------------------------------------------------------------------
* TP3                |    OUT        | P8.7                     |  	P7_3    (SI_TP3)		|
*--------------------------------------------------------------------------------------------
* TP4                |    OUT        | P8.3                     |                           |
*--------------------------------------------------------------------------------------------
* TP5                |    OUT        | P8.4                     |                           |
*--------------------------------------------------------------------------------------------
* TP6                |    OUT        | P8.2                     |                           |
*--------------------------------------------------------------------------------------------
* CAPTURE_EVENT      |    OUT        | P9.3                     |  P7.3  (TP3 in schematic) |
*--------------------------------------------------------------------------------------------
* (1) Peripherals used with HAL should not be configured by Device Configurator
*-------------------------------------------------------------------------------------------*/

/****************************************************************************
* GLOBALS
*****************************************************************************/
#define WDT_TIME_OUT_MS  	4900		/* _CYHAL_WDT_MAX_TIMEOUT_MS = 4915 Seconds watchdog timer timeout */
#define WARMUP_WAIT_TIME_MS 10000		/* 10 seconds cold start warmup */

_ports_t LoggingPort = 0;				/* Flags indicating where to send IMU / GNSS reports */
_ports_t SidebandPort = 0;				/* port for side band reports and control */
bool log_gnss_records = true;

char mainBuffer[1024] = "";

/****************************************************************************
* PrintWithTime - Outputs message with up time stamp to log terminal
*****************************************************************************/
void PrintWithTime(const char *msg)
{
	printf( "\n[%s] %s", GetUpTimeStr(), msg );
}

/*--------------------------------------------------------------------------------*/
cy_rslt_t LogResetCause()
{
	cy_rslt_t cause = cyhal_system_get_reset_reason();

	cyhal_system_clear_reset_reason();

	switch (cause)
	{
	case CYHAL_SYSTEM_RESET_NONE:				// power on or hardware reset
		printf("CYHAL_SYSTEM_RESET_NONE\n" );
		break;

	case CYHAL_SYSTEM_RESET_WDT:
		printf("CYHAL_SYSTEM_RESET_WDT\n");
		break;

	case  CYHAL_SYSTEM_RESET_ACTIVE_FAULT:
		printf("CYHAL_SYSTEM_RESET_ACTIVE_FAULT\n");
		break;

	case CYHAL_SYSTEM_RESET_DEEPSLEEP_FAULT:
		printf("CYHAL_SYSTEM_RESET_DEEPSLEEP_FAULT\n");
		break;

	case CYHAL_SYSTEM_RESET_SOFT:					// Debugger or software reset
		printf("CYHAL_SYSTEM_RESET_SOFT\n");
		break;

	case CYHAL_SYSTEM_RESET_HIB_WAKEUP:
		printf("CYHAL_SYSTEM_RESET_HIB_WAKEUP\n");
		break;

	case  CYHAL_SYSTEM_RESET_WCO_ERR:
		printf("CYHAL_SYSTEM_RESET_WCO_ERR\n");
		break;

	case CYHAL_SYSTEM_RESET_SYS_CLK_ERR  :
		printf("CYHAL_SYSTEM_RESET_SYS_CLK_ERR\n");
		break;

	case  CYHAL_SYSTEM_RESET_PROTECTION:
		printf("CYHAL_SYSTEM_RESET_PROTECTION\n");
		break;
	}

	return cause;
}

/****************************************************************************
* TrapImuHeader() - FSAS header not send by some versions of firmware
*****************************************************************************
void TrapImuHeader(uint8_t *buf, size_t cnt)
{
	static int nbuf=0;
	static const char token[] = "live mode";

	if(IMU_online == true) // ignore until uart cmd processor reconfigured
		return;

	while(cnt--)
	{
		uint8_t ch = *buf++;

		if ( IS_LINE_TERMINATOR(ch) && nbuf > 0 ) // drop trailing characters if any
		{
				mainBuffer[nbuf] = '\n';
				mainBuffer[nbuf+1] = 0;
				printf(mainBuffer);

				if (strstr( mainBuffer,token) != NULL)
				{
					IMU_online = true;
					PrintWithTime ("<-- IMU online -->\n");
				}

				nbuf = 0;
		}
		else
			mainBuffer[nbuf++] = ch;
	}
}
*/

/****************************************************************************
* Command routers
*****************************************************************************/
#define COMMAND_BUFFER_SIZE 128
#define MAX_COMMAND_LEN  (COMMAND_BUFFER_SIZE-2)

uint8_t usbCmd[COMMAND_BUFFER_SIZE]; // we expect only short commands from logger port
uint8_t comCmd[COMMAND_BUFFER_SIZE];
uint8_t monCmd[COMMAND_BUFFER_SIZE];

void Process_USB_Data_Received( uint32_t usb_port, _ports_t logic_port )
{
	memset(usbCmd, 0, ARRAY_SIZE(usbCmd));

	uint32_t count = Read_USB_CDC_Data(usb_port, usbCmd, ARRAY_SIZE(usbCmd));

    char* retval = ScanCommandLine(usbCmd,count, logic_port);

    if (  retval != NULL && TEST_BITS(Cmd_Echo_On,true))
    	Send_USB_CDC_Data( usb_port, (uint8_t*)retval, strlen(retval) );
}

void Monitor_COM_Ports()
{
	if (!New_COM1_Message)
 		return;

    New_COM1_Message = false;
    
	memset(comCmd, 0, ARRAY_SIZE(comCmd));

	size_t count = Uart_COM_Read(comCmd, ARRAY_SIZE(comCmd));
	char* retval = ScanCommandLine(comCmd,count, UART_COM1);
	size_t retval_len = strlen(retval);

	if ( retval != NULL && retval_len > 0 && TEST_BITS(Cmd_Echo_On,true))
	{
		Uart_COM_Send((uint8_t*)retval, retval_len);
	}
}

void MonitorConsoleInput()
{
	static int monIndex = 0;

	while (GetConsoleInputBufferCount() != 0 )
	{
		char ch = console_getchar();

		if ( IS_LINE_TERMINATOR( ch ) && monIndex > 0 ) // We have a line, ignore empty lines
		{
			monCmd[monIndex++] = (uint8_t)ch;

			char * retval = ScanCommandLine(monCmd, monIndex, UART_CONSOLE );

			if ( retval != NULL && TEST_BITS(Cmd_Echo_On,true))
			{
				printf(retval);
			}
			monIndex = 0;
		}
		else if ( monIndex < MAX_COMMAND_LEN)
		{
			monCmd[monIndex++] =  ch;
		}
	}
}


/****************************************************************************
* GLOBALS for main()
*****************************************************************************/

#define WARMUP_TIME		10		// 10 seconds

uint32_t secCnt = WARMUP_TIME;

void HBAction(uint32_t cnt)
{
	if ( --secCnt == 0 )
		SetHartBitAction(NULL);
}


/****************************************************************************
* Log buffers
*****************************************************************************/

/*--------------------------------------------------------------------- GNSS Buffer-*/
uint8_t _novatel_buffer[FLEX_RING_BUF_MEM(LARGEST_NOVATEL_MESSAGE, NUMBER_OF_GNSS_RECORDS)];
RDBUF_HANDLE HandleGNSSRecords = NULL;

static inline void CreateNovatelBuffer()
{
	HandleGNSSRecords = CreateFlexRingBuffer(_novatel_buffer, sizeof(_novatel_buffer));
}

void Send_GNSS_Record( void* pdata, size_t data_size)
{
	PushToFlexRingBuffer(HandleGNSSRecords,pdata, data_size);
}

void Purge_GNSS_Buffer()
{
	FlushFlexRingBuffer(HandleGNSSRecords);
}

/*--------------------------------------------------------------------- IMU Buffer-*/
uint8_t _imu_buffer[FLEX_RING_BUF_MEM(IMU_RECORD_SIZE, IMU_RECORD_STORE_COUNT)];
RDBUF_HANDLE HandleImuRecords  = NULL;

static inline void CreateImudBuffer()
{
	HandleImuRecords = CreateFlexRingBuffer( _imu_buffer, sizeof(_imu_buffer) );
}

void Send_IMU_Record( void* pdata, size_t data_size)
{
	if ( HandleImuRecords != NULL )
		PushToFlexRingBuffer( HandleImuRecords, pdata, data_size );
}



/*--------------------------------------------------------------------- Report Buffer-*/
#define NUMBER_OF_REPORTS	50U

typedef struct
{
	_ports_t Port;
	dbuf_t Dbuf;

} repbuf_t;

DBUF_HANDLE HandleReportRecords = NULL;
uint8_t _reoprt_buffer[RING_BUFFER_SPACE( sizeof(repbuf_t), NUMBER_OF_REPORTS)];

static inline void CreateReportBuffer()
{
	HandleReportRecords = CreateRingBuffer(sizeof(repbuf_t), NUMBER_OF_REPORTS, _reoprt_buffer, sizeof(_reoprt_buffer));
}

volatile bool newReportRecords = false;

void StoreReportRecord( dbuf_t *buf, _ports_t port)
{
	if ( HandleReportRecords != NULL)
	{
		repbuf_t rbuf;
		rbuf.Port = port;

		memcpy( &rbuf.Dbuf, buf, sizeof(dbuf_t));

		if (PushRingBufferData(HandleReportRecords, &rbuf, sizeof(repbuf_t)))
			newReportRecords = true;
	}
}


/*---------------------------------------------------------- Create Log Buffers-*/
void CreateLogBuffers()
{
	/* IMU ring buffer */
	CreateImudBuffer();
	/* GNSS  buffer */
	CreateNovatelBuffer();
	/* secondary report buffer */
	CreateReportBuffer();
}

/*---------------------------------------------------------------------------------*/
void PurgeBuffers()
{
//	PurgeImuBuffer();
	FlushFlexRingBuffer(HandleImuRecords);
	FlushFlexRingBuffer(HandleGNSSRecords);
	PurgeRingBuffer(HandleReportRecords);
}

/*------------------------------------------------------------------------ watch dog */
cyhal_wdt_t wdt_obj;

void Init_WDT()
{
    /* Initialize the WDT */
    cy_rslt_t result = cyhal_wdt_init(&wdt_obj, WDT_TIME_OUT_MS);

    /* WDT initialization failed. Stop program execution */
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    cyhal_wdt_start(&wdt_obj);
}

//#define MONITOR_IMU_LOG
//#define MONITOR_GNSS_LOG


/*===============================================================  Main() */
int main(void)
{
    /* Initialize the device and board peripherals defined
	 * in Device Configurator (excludes HAL managed device)
	 * */
	cy_rslt_t result = cybsp_init();
	CY_ASSERT (result == CY_RSLT_SUCCESS);

	__enable_irq();

	if ( ReadConfig(&SysConfig) ) /* true on error */
	{
		SysConfig.soft_reset = false;
		SysConfig.no_com2_logs_init = false;
		SysConfig.imu_type = DEFAULT_IMU_TYPE;
		SysConfig.imu_connect = DEFAULT_IMU_TARGET;
		SaveConfig(&SysConfig);
	}

	if (SysConfig.imu_type == IMUType_INVALID || SysConfig.imu_connect == Target_INVALID )
	{
		SysConfig.imu_type = DEFAULT_IMU_TYPE;
		SysConfig.imu_connect = DEFAULT_IMU_TARGET;
		SaveConfig(&SysConfig);
	}

	/* Initialize retarget console to use the debug UART port
	 *
	 * WARNING: DO NOT CONFIGURE UART IN DEVICE CONFIGURATOR.
	 * THE DEVICE WILL BE RESERVED AND HAL WILL NOT BE ABLE TO USE IT.
	 */
	result = retarget_console_init(AUX1_TX, AUX1_RX);

	/* Do it early to trap IMU startup message.*
	SetImuMsgProcessor(TrapImuHeader);
	Init_Uart_IMU();
	 */

	Cy_SysLib_Delay(2000);

	Init_HARTBIT_PWM(HBAction);			// 1sec system hart bit used for low accuracy timing
	Init_GPTIMER1_PWM();				// 10khz timer drive

	Init_Uart_Oem7700();				// Main UART for log reports
	Init_Uart_COM_Port();
	Init_Uart_J6();

	result = LogResetCause();

	printf( _consoleHeader, VersionString );

	/* setup warming up blinking indicator*/
	Init_PPS_MONITOR_PWM(false);		// Used for warmup blinking and PPS pulse
	SetSPPSMonintorFrequency(FAST_PPS_MONITOR_FREQ, SHORT_PPS_MONITOR_PULSE);

	if (result == CYHAL_SYSTEM_RESET_NONE)		// Power ON or hardware reset
	{
	//	SetImuMsgProcessor(TrapImuHeader);

		PrintWithTime("Cold start. Warming up 10 seconds...\n");

		cyhal_system_delay_ms(WARMUP_WAIT_TIME_MS);
	}
	else if (result == CYHAL_SYSTEM_RESET_WDT)
	{
		PrintWithTime("Reset by WDT\n");
	}

	ChangePPSMonitorRunMode(true);												// run now it will be triggered by the PPS pulse
	SetSPPSMonintorFrequency( SLOW_PPS_MONITOR_FREQ, DEFAULT_PPS_ON_TIME );		// 2 hertz

	if ( WaitOEM7700SignIn(3000 ) )
	{
		if (InitializeOEM7700())
		{
			PrintWithTime("OEM7700 Initialization error\n");
		}
	}

	if (Init_IMU_Interface(SysConfig.imu_type, SysConfig.imu_connect ) )
	{
		PrintWithTime("Error: IMU Initialization Failed\n");
	}

	Init_PPS_Counter();

	PrintWithTime("Initializing USB CDC...\n");

	if ( !USB_CDC_Init() )
	{
		PrintWithTime("USB NOT connected.\n");
	}

	CreateLogBuffers();
	ClearImuStatus();
	ClearNovatelStatus();

	PrintWithTime( (USB_Active == true)? "USB ports enabled\r\n": "USB ports disabled\r\n");
	PrintStatusLong(mainBuffer, ARRAY_SIZE(mainBuffer));
	PrintWithTime(mainBuffer);

	PrintWithTime("Entering main loop\n");

	//Init_WDT();

	for (;;)
	{
    	/*--------------------------------------- monitor USB CDC ports */
        if ( USB_CDC_DataReady(USBUART_COM1) != false )
        {
        	Process_USB_Data_Received(USBUART_COM1, USB_COM1);
        }

        if ( USB_CDC_DataReady(USBUART_COM2) != false )
        {
			Process_USB_Data_Received(USBUART_COM2, USB_COM2);
	    }

        /*------------------------------------------------- SCB UART COM1 */
		Monitor_COM_Ports();	// monitor COM ports for flow control and other status changes

		 /*---------------------------------------------- SCB UART Console */
         MonitorConsoleInput();

        /*---------------------------------------------- AsyncMessageProcessor */
		ProcessAsyncMessages();	// process messages from ISRs and other sources outside of main loop context

        /*-------------------------------------------- process LOG reports */

        if (LoggingPort != 0 )
        {
			while (FlexRingBufferItems( HandleImuRecords ) != 0 )
			{
#ifdef MONITOR_IMU_LOG
				SET_DEBUG_TP(_TP3);
#endif
				ring_data_t Rec;
				PopFromFlexRingBuffer(HandleImuRecords, &Rec);

        		if (TEST_BITS(LoggingPort, USB_COM1))				// Transfer of IMU record to USB 10 - 20 us.
				{
					Send_USB_CDC_Data( USBUART_COM1, Rec.Data, Rec.Size );
				}

				if ( TEST_BITS(LoggingPort, USB_COM2) )
				{
					Send_USB_CDC_Data( USBUART_COM2,  Rec.Data, Rec.Size );
				}

				if ( TEST_BITS(LoggingPort, UART_COM1) )
				{
					Uart_COM_Send(  Rec.Data, Rec.Size );
				}

#ifdef MONITOR_IMU_LOG
				CLEAR_DEBUG_TP(_TP3);
#endif
			}

			if (FlexRingBufferItems( HandleGNSSRecords ) != 0 )
			{
#ifdef MONITOR_GNSS_LOG
				SET_DEBUG_TP(_TP3);
#endif
				ring_data_t Rec;
				PopFromFlexRingBuffer(HandleGNSSRecords, &Rec);

				if (TEST_BITS(LoggingPort, USB_COM1))				// Transfer of IMU record to USB 10 - 20 us.
				{
					Send_USB_CDC_Data( USBUART_COM1, Rec.Data, Rec.Size );
				}

				if ( TEST_BITS(LoggingPort, USB_COM2) )
				{
					Send_USB_CDC_Data( USBUART_COM2, Rec.Data, Rec.Size );
				}

				if ( TEST_BITS(LoggingPort, UART_COM1) )
				{
					Uart_COM_Send(  Rec.Data, Rec.Size );
				}
#ifdef MONITOR_GNSS_LOG
				CLEAR_DEBUG_TP(_TP3);
#endif
			}
		}

        /*------------------------------------ Periodic reports -*/
        static uint32_t send_cnt = 0;

        if (newReportRecords == true)
        {
        	newReportRecords = false;

        	while(GetRingBufferCount(HandleReportRecords ) > 0 )
        	{
        		send_cnt++;

        		repbuf_t* R = PopRigngBufferData(HandleReportRecords);

          		CY_ASSERT(R->Dbuf.Buffer < (uint8_t*) 0x0804FFFFu);

            	if( R->Port == UART_COM1 )
        		{
					Uart_COM_Send(R->Dbuf.Buffer, R->Dbuf.Count);
        		}
        		else if (R->Port == USB_COM1)
        		{
        			Send_USB_CDC_Data(USBUART_COM1, R->Dbuf.Buffer, R->Dbuf.Count);
        		}
           		else if (R->Port == USB_COM2)
				{
					Send_USB_CDC_Data(USBUART_COM2, R->Dbuf.Buffer, R->Dbuf.Count);
				}
           		else if (R->Port == UART_CONSOLE )
				{
					printf((char*) R->Dbuf.Buffer );
				}
        	}
        }
        Monitor_USB_CDC_Status();	// can't reconnect from an ISR

	    //cyhal_wdt_kick(&wdt_obj);	// keep watchdog slipping
//	    cyhal_syspm_sleep();		// will be wake up by interrupts

    } /* end of endless loop */
} /* end of main */

/* [] END OF FILE */
