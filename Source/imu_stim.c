/*
 * imu_stim.c
 *
 * STIM300 IMU Interface.
 *
 * The TOV signal is wired through the NOGO circuitry to PSOC P[3].2
 *
 * TOV is detected by a PIN ISR. The raising edge signals the end of
 * a bloc transmission.
 * The GPIO ISR reflects the TOV level to TOV_STROBE pin (P[6].2) wired to OEM7700
 * event input 2.
 *
 * Lack of MUX during the design don't permit to wire TOV_STROBE directly
 * to EVENT_IN2.

 *  Created on: Oct 27, 2023
 *      Author: G. Gallo
 *
 * Rev. 3.2: Aug 2025 - Removed unnecessary PWM strobe.
 */

#include "common.h"

#define DEBUG_STIM

#define DEBUG_STIM_TOV
//#define DEBUG_STIM_BAUDSMON

#ifdef DEBUG_STIM
#ifdef DEBUG_STIM_TOV
#define STIM_TOV_TP TP6
#endif

#define STIM_TOV_TP TP6

#endif


#define STIM_ISR_TP TP7

#define STIM_BUFFER_SIZE	50
uint8_t  _stim_buffer[STIM_BUFFER_SIZE];
size_t   _stim_rec_count=0;

uint8_t _stim_temp[STIM_BUFFER_SIZE];
size_t  _stim_temp_count=0;

/****************************************************************************
* STIM IMU TOV Signal Detect
*
* for STIM300 imu we mirror the state of NOGO signal carrying STIM TOV signal
*
* STIM300 TOV -> NOGO line -> P3.2 -> GPIO interrupt -> P6.2 -> OEM7700 EVENT_IN2
*
*****************************************************************************/
imu_target_t TOV_Target;
bool first_run = false;

bool Init_STIM_Framing = false;

void STIM_TOV_detect_cb(void *arg, cyhal_gpio_event_t event)
{
	if(event & CYHAL_GPIO_IRQ_RISE)  // End of data frame
	{
#ifdef DEBUG_STIM_TOV
		SET_DEBUG_TP(STIM_TOV_TP);
#endif
		if( TOV_Target == Target_NovAtel ) // End of record flag
		{
			SET_GPIO_PIN(TOV_STROBE);		// This is P6.2 wired to NovAtel Event Input 2
		}

		if (Init_STIM_Framing)
		{
			Reconfig_Uart_IMU( STIM_RECORD_SIZE -1, false );
		}

	}
	else // start of record
	{
		if( TOV_Target == Target_NovAtel ) // PSOC handles this IMU
			CLEAR_GPIO_PIN(TOV_STROBE);

#ifdef DEBUG_STIM_TOV
		CLEAR_DEBUG_TP(STIM_TOV_TP);
#endif

		if (Init_STIM_Framing)
		{
			Init_STIM_Framing = false;
		}

#ifdef DEBUG_STIM_BAUDSMON
		SET_DEBUG_TP(BAUDSMON_TP);
	    Cy_TCPWM_TriggerStart_Single(BAUDSMON_HW, BAUDSMON_NUM);
#endif
	}
}

cyhal_gpio_callback_data_t STIM_TOV_detect_cb_data =
{
		STIM_TOV_detect_cb,
		NULL, 0, 0,
};

/****************************************************************************
* Init_STIM_TOV_Detect() - for STIM300 IMU
*
* Uses the same IMU_NOGO_TOV pin for TOV (Time of Validity) signal. The ISR
* forwards the state of the signal to OEM7700 EVENT_IN2 via TOV_STROBE (P6.2)
*
*****************************************************************************/
void Init_STIM_TOV_Detect( imu_target_t _target)
{
	 TOV_Target = _target;
	 first_run = false;

	cyhal_gpio_init(IMU_NOGO_TOV, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	cyhal_gpio_register_callback(IMU_NOGO_TOV, &STIM_TOV_detect_cb_data);
    cyhal_gpio_enable_event(IMU_NOGO_TOV, CYHAL_GPIO_IRQ_BOTH, CYHAL_ISR_PRIORITY_DEFAULT, true);
}


/*******************************************************************************************************
 *
 * STIM300 IMU DECODING
 *
 *
 ********************************************************************************************************/
bool Check_STIM_CRC(uint8_t* buffer, size_t cnt );

int skip_bytes = 0;
uint8_t counter = 0xFF;

_ports_t port = INVALID_PORT;

void Stim_Log_To_Port(_ports_t _port) { port = _port; }
void Stim_Stop_Logging() { port = INVALID_PORT; }


/****************************************************************************
* Print_STIM_Status() -- For STIM Diagnostics
*****************************************************************************/
#define STATUS_STR_BUF  150
char stim_status_str[STATUS_STR_BUF];

char* status_str[] =
{
	"X channel",
	"Y Channel ",
	"Z Channel ",
	"Meas Error, ",
	"Overload, ",
	"Outside Op Cond, ",
	"Startup, ",
	"Integrity Error, ",
};

char* Print_STIM_Status(uint8_t st)
{
	if( st == 0)
		return "OK";

	memset(stim_status_str, 0, STATUS_STR_BUF );
	int i = 0x80;

	for ( int j=7; j >= 0; j--)
	{
		if ( (st & i) != 0)
		{
			strcat( stim_status_str,status_str[j]);
		}
		i >>= 1;
	}
	return stim_status_str;
}

/****************************************************************************
* STIM Status -- for diagnostic
*****************************************************************************/
uint32_t stim_crc_err=0;
uint32_t uart_error=0;

uint8_t SG=0, SA=0, STG=0, STA=0;

int GetStimStatus(char* buf, int len)
{
	int cnt = snprintf(buf,len, "STIM STATUS: Framming Error %lX | CRC Errors: %ld\n"
								"  STIM Gyro status  %s\n  STIM Accel status %s\n"
			 	 	 	 	 	"  STIM Gyro Temp status %s\n  STIM Accel Temp status %s\n",
									uart_error, stim_crc_err,
			Print_STIM_Status(SG), Print_STIM_Status(SA),
			Print_STIM_Status(STG), Print_STIM_Status(STA));

	SG=SA=STG=STA=0;
	uart_error =0;

	return cnt;
}

/****************************************************************************
* Decode_STIM300_Datagram() -- for diagnostic
*****************************************************************************/
void Decode_STIM300_Datagram()
{
	if (_stim_rec_count == STIM_RECORD_SIZE )
	{
		if ( Check_STIM_CRC(_stim_buffer, _stim_rec_count ) ) // add padding
		{
			stim_crc_err++;

			first_run = false;
		}

		if (port != INVALID_PORT )
		{
			SendToPort(port, _stim_buffer, _stim_rec_count );
		}

		PSTIM_A5_t rec = (PSTIM_A5_t) _stim_buffer;

		if (rec->SG != 0)
			SG |= rec->SG;		//printf("STIM Gyro status %s\n", Print_STIM_Status(rec->SG));

		if (rec->SA != 0)
			SA |= rec->SA; 		//printf("STIM Accel status %s\n", Print_STIM_Status(rec->SA));

		if (rec->STG != 0)
			STG |= rec-> STG; 	// printf("STIM Gyro Temp status %s\n",  Print_STIM_Status(rec->STG));

		if (rec->STA != 0)
			STA |= rec->STA; 	// printf("STIM Accel Temp status %s\n",  Print_STIM_Status(rec->STA));

		//uint8_t rc = rec->Counter;

	}
}


/****************************************************************************
* Receive_STIM300_Datagram() -- For STIM handled by PSOC
*****************************************************************************/

void Receive_STIM300_Datagram(uint8_t* buffer, size_t cnt)
{
	if (Uart_IMU_Error != 0)
	{
		uart_error |= Uart_IMU_Error;
		Uart_IMU_Error = 0;
	}

	while(cnt--)
	{
		uint8_t ch = *buffer++;

		if (_stim_temp_count == 0 ) 	/* wait for record ID */
		{
			if ( ch != STIM_A5_DG_ID)
				continue;

	#ifdef DEBUG_STIM
		SET_DEBUG_TP(STIM_ISR_TP);		// start of record
	#endif
		}

		if (_stim_temp_count < STIM_BUFFER_SIZE )
		{
			_stim_temp[_stim_temp_count++] = ch;

			if ( _stim_temp_count == STIM_RECORD_SIZE  ) // end of Datagram
			{
				memset(_stim_buffer,0,STIM_BUFFER_SIZE );
				memcpy(_stim_buffer, _stim_temp, _stim_temp_count );
				_stim_rec_count = _stim_temp_count;
				_stim_temp_count = 0;

				Decode_STIM300_Datagram();
#ifdef DEBUG_STIM
CLEAR_DEBUG_TP(STIM_ISR_TP);		// end of record
#endif
			}
			break;
		}
	}

#ifdef DEBUG_STIM
	// end of conversion
//	CLEAR_DEBUG_TP(TP8);
#endif

}



/*=======================================================================================================
 * CRC32 FUNCTIONS
 ********************************************************************************************************/

static const uint32_t crc_table[] =
{
	 0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
	 0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75, 0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
	 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
	 0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
	 0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072, 0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
	 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
	 0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
	 0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2, 0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
	 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
	 0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
	 0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B, 0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
	 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
	 0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
	 0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC, 0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
	 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
	 0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

 #define CRC32_SEED 0xFFFFFFFF


/*=================================================================================================
 *  CalcCRC32() - Retuns true if byte count != n x 4 or CRC error
 *=================================================================================================*/

bool CalcCRC32(uint8_t *data, int ByteCount, uint32_t crcin )
{
	/* NUMBER OF BYTES MUST BE A MULTIPLY OF 4 TO BE USED FOR STIM DATA PACKETS */
	if (ByteCount % 4 != 0)
	{
		return true;
	}
	uint32_t CRC = CRC32_SEED;

	while (ByteCount--)
	{
		CRC = crc_table[((CRC >> 24) ^ *data++) & 0xFFL] ^ (CRC << 8);
	}

	return  (CRC != crcin);
}


/*=================================================================================================
 *  bool Check_STIM_CRC() - Checks the CRC of a STIM datagram
 *
 *  Expects: byte count of full datagram, including CRC and <CR><LF> at the end
 *
 *  returns: True on error
 *=================================================================================================*/

bool Check_STIM_CRC(uint8_t* buffer, size_t cnt )
{
	/* recover and convert the CRC to Little-Endian order  */
	uint32_t* pcrc = (uint32_t*)(buffer + cnt - 4);
	uint32_t in_crc = *pcrc;

	/* clear the CRC to make room for the padding */
	*pcrc = 0UL;

	/* this format requires a padding of 2 bytes */
	bool result = CalcCRC32(buffer, cnt -2, _bswap32(in_crc));
	*pcrc = in_crc;

	return result;
}

