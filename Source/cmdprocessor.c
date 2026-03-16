/*
 * cmdprocessor.c
 *
 * Console  Command processor.
 *
 *  Created on: Jun 23, 2022
 *      Author: Guillermo
 */
#include "common.h"
#include "gpio.h"
#include "cmdprocessor.h"


/*--------------------------------------------------------------- PORTS*/
typedef struct
{
	const char* name;
	_ports_t		id;

}port_ids_t;


const port_ids_t PortNames[] =
{
		{"COM1", 	UART_COM1},
		{"USB1",	USB_COM1},
		{"USB2",	USB_COM2},
		{"CONSOLE",	UART_CONSOLE},
		{"OEM7700",	UART_OEM7700},
		{"UARTIMU", UART_IMU},
		{"UARTJ6",  UART_J6},
};

_ports_t FindPortID( char* token)
{
	for (int i=0; i < ARRAY_SIZE(PortNames);i++)
	{
		if (strcmp(token, PortNames[i].name)==0)
			return PortNames[i].id;
	}
	return INVALID_PORT;
}

const char* GetPortName(_ports_t port)
{

	for (int i=0; i < ARRAY_SIZE(PortNames);i++)
	{
		if ( PortNames[i].id == port)
			return PortNames[i].name;
	}

	return "INVALID PORT ID";
}


/*---------------------------------------------------------------- SendToPort */
void SendToPort(_ports_t port, uint8_t* buffer, size_t count)
{
	switch(port)
	{
	case USB_COM1:
		Send_USB_CDC_Data( USBUART_COM1, buffer, count );
		break;

	case USB_COM2:
		Send_USB_CDC_Data( USBUART_COM2, buffer, count );
		break;

	case UART_COM1:
		Uart_COM_Send( buffer, count );
		break;

	case UART_CONSOLE:
		printf((char*) buffer);
		break;

	case UART_J6:
		Uart_J6_Send( buffer, count );
		break;

	case UART_IMU:
		Uart_IMU_Send( buffer, count );
		break;

	case UART_OEM7700:
		Uart_Oem7700_Send(buffer, count );
		break;

	case INVALID_PORT:
		break;
	}
}

void SendStringToPort(_ports_t port, char* str)
{
	int len = strlen(str);
	SendToPort(port,(uint8_t*) str, len);
}


/*---------------------------------------------------------------------------- COMMANDs */
/* COMMAND FORMATS
 *
 * Comand		Target	Switch	  Intarg1,2,3
 *---------------------------------------------------------------------------------------------------
 * LOG 			[PORT] 	 START|STOP				- Starts|Stops logging to port COM or USB
 * VERSIONINFO 	[PORT]							- Sends version info to port
 * STATUS 		[PORT]	[START|STOP] [msecs]	- Sends status report to port ON every x milliseconds
 * SHORTSTATUS 	[PORT]	ON|OFF  	 [msecs]	- Sends status in Binary Format
 * ECHO					ON|OFF					- Turns command answers off on sender port
 * IMUFORMAT			FSAS | NRAW 			- Sets IMU format as native FSAS SN or Novatel RAW (default)
 * IMUFREQ							 Hertz		- Changes the IMU frequency
 * IMUTYPE				type		[SAVE]		- Selects the IMU to be used. Optionally SAVE this setting to EPROM
 * IMUCONNECT			target		[SAVE]		- Selects IMU COM connection OEM7700 or PSOC, Optional SAVE to EPROM
 * IMUREDIR		[PORT]							- Redirects raw IMU data to port for debug
 * IMUBAUDS		baudrate						- Change the IMU port baud rate
 * STANDBY										- Stops all reporting
 * SYSCONFIG									- Print the system config stored in eprom
 * MARKTIME event_nr [polarity] [port] [action] - Request mark time reports
 * POSREPORT 	[PORT]	[START|STOP] [GEO|UTM] [msecs]	- Send position report [FORMAT] x milliseconds
 * MAPLBREPORT  [PORT]							- Maps LADYBUG GPS reports to PORT
 * COPYCONSOLE  [PORT]							- Copy console output to PORT
 * ------------------------------------------------------------------------------------------------
 * GPIO Read/Write and Pulse commands - (Firmware 3.2 and up)
 * Currently only the pins of connector J6 on (P13.1 - P13.4 are available)
 *
 * SETPIN		JX.n	HIGH/LOW				- Sets the pin 'n' of port 'X' High or low
 * WRITEPIN		JX.n	HIGH/LOW				- Sets the pin 'n' of port 'X' High or low
 * READPIN		JX.n							- Reads the pin 'n' of port 'X'
 * PULSEPIN		JX.n				[msecs]		- Default is 1 second, min is 1ms
 * MONITORPIN	JX.n							- Enables an ISR that reports pin level changes. The pin is programmed as input
 *
 * ------------------------------------------------------------------------------------------------

 * BENCHMARK	ID	PARAM1 PARM2 PARAM3 ...		- # Of parameters depends on the benchmark ID
 *
 * PRESCALER	reload							- Value to reload at PPS Sync
 */

typedef char* (*fn_command_t)(char**, int, _ports_t);

char *log_cmd(char**, int, _ports_t);
char *version_cmd(char**, int, _ports_t);
char *status_cmd(char**, int, _ports_t);
char *echo_cmd(char**, int, _ports_t);
char *imufreq_cmd(char**, int, _ports_t);
char *imuformat_cmd(char**, int, _ports_t);
char *imutype_cmd(char**, int, _ports_t);
char *imuconnect_cmd(char**, int, _ports_t);
char *stim_status(char**, int, _ports_t);

char *imuredir_cmd(char**, int, _ports_t);	// debug command
char *imubauds_cmd(char**, int, _ports_t);	// debug command
char *imu_record_size_cmd(char** , int , _ports_t );
char *imu_console_cmd(char**, int, _ports_t);	// debug command

char *system_reset_cmd(char**, int, _ports_t);
char *sysconfig_cmd(char**, int, _ports_t);
char *port_id_cmd(char**,int,_ports_t);
char *help_id_cmd(char**,int,_ports_t);
char *copy_console_cmd(char**,int, _ports_t);
char *benchmark_cmd(char**,int,_ports_t);

char *standby_cmd(char**,int,_ports_t);
char *gnss_log_cmd(char**, int, _ports_t);
char *pos_report_cmd(char**, int, _ports_t);
char *marktime_report_cmd(char**, int, _ports_t);
char *markpos_report_cmd(char** tokens, int cnt, _ports_t port);
char *map_ladybug_report_cmd(char** tokens, int cnt, _ports_t port);
char *request_gps_log_cmd(char** tokens, int cnt, _ports_t port);

char *set_pin_cmd(char** tokens, int cnt, _ports_t port);
char *pulse_pin_cmd(char** tokens, int cnt, _ports_t port);
char *read_pin_cmd(char** tokens, int cnt, _ports_t port);
char *monitor_pin_cmd(char** tokens, int cnt, _ports_t port);
char* cancel_pin_monitor_cmd(char** tokens, int cnt, _ports_t port);
char* genoutput_cmd(char** tokens, int cnt, _ports_t port);
char *echo2_cmd(char**, int, _ports_t);

char *test_cmd(char**,int,_ports_t);
char *disableIMULog_cmd(char**,int,_ports_t);

char *verbose_cmd(char**,int,_ports_t);

char* release_timer_cmd(char** tokens, int cnt, _ports_t port);
char* create_timer_cmd(char** tokens, int cnt, _ports_t port);

/*--------------------------------------------------------------------------- command dictionary */
typedef struct
{
	const char* 	cmd;
	fn_command_t	fn;

}fn_dictionary_t;

const fn_dictionary_t CmdDictionary[] =
{
		{"LOG", 		log_cmd},
		{"VERSIONINFO", version_cmd},
		{"STATUS",		status_cmd},
		{"SHORTSTATUS",	status_cmd},		// sends short status <STATUS,a,b,c,d...x>\r
		{"ECHO",		echo_cmd},
		{"IMUFREQ",		imufreq_cmd},
		{"IMUFORMAT",	imuformat_cmd},
		{"IMUTYPE",		imutype_cmd},
		{"IMUCONNECT",	imuconnect_cmd},
		{"SYSTEMRESET", system_reset_cmd},
		{"SYSCONFIG", 	sysconfig_cmd},
		{"STANDBY",		standby_cmd},
		{"PORTID",		port_id_cmd},
		{"?",			help_id_cmd},
		{"COPYCONSOLE",	copy_console_cmd},
		{"POSREPORT", 	pos_report_cmd},			// Send periodic position reports
		{"MARKTIMEREPORT", marktime_report_cmd},	// Send  reports
		{"MARKPOSREPORT",  markpos_report_cmd},		// Send  reports
		{"MAPLBREPORT",    map_ladybug_report_cmd},	// Send Ladybug GPS logs to PORT#
		{"REQUESTGPSLOG",  request_gps_log_cmd},	// Send Ladybug GPS logs to PORT#

		{"SETPIN",  		set_pin_cmd},			// SET port pin high or low
		{"WRITEPIN",  		set_pin_cmd},			// SET port pin high or low
		{"PULSEPIN",  		pulse_pin_cmd},			// Pulse port pin
		{"READPIN", 		read_pin_cmd},			// read port pin
		{"MONITORPIN", 		monitor_pin_cmd},			// read port pin
		{"CANCELPINMONITOR",cancel_pin_monitor_cmd},

		/* debug commands */
		{"VERBOSE",  	verbose_cmd},			// Echo all commands and answer to the console.
		{"GENOUTPUT",   genoutput_cmd },			// generates a n output of n bytes with crc16
		{"ECHO2",       echo2_cmd	},					// Echoes back what's received by the port
		{"GNSSLOG",     gnss_log_cmd}, 				// enables (default) / disables GNSS logs to be sent with IMU logs
		{"BENCHMARK",	benchmark_cmd},  		// executes several benchmarks
		{"TEST",		test_cmd },				// executes several test commands
		{"DISABLEIMULOG", disableIMULog_cmd},	// To receive GNSS logs only
		{"CREATETIMER", create_timer_cmd},
		{"RELEASETIMER", release_timer_cmd},
		{"IMUREDIR",     imuredir_cmd},
		{"IMUBAUDS",    imubauds_cmd},
		{"STIMSTATUS",  stim_status},
		{"IMURECSIZE",  imu_record_size_cmd},
		{"IMUCONSOLE",  imu_console_cmd},
};

/*----------------------------------------------------------------------------
 * SearchDictionary()
 * input
 * 		pdic - pointer to dictionary
 * 		count - number of entries use ARRAY_SIZE(dictionary)
 * 		key  - search item
 * 	Returns
 * 		string matching key or _default (default) if not found
 *--------------------------------------------------------------------------*/
const char* SearchDictionary(dictionary_t *pd, size_t count, const int key, const char* _deft)
{
	for (int i = 0; i < count; i++)
	{
		if( pd[i].key == key )
			return pd[i].str;
	}
	return _deft;
}


fn_command_t find_Comand(char* token)
{
	for(int i = 0; i < ARRAY_SIZE(CmdDictionary); i++)
	{
		if (strcmp(token, CmdDictionary[i].cmd) == 0)
		{
			return CmdDictionary[i].fn;
		}
	}
	return NULL;
}


/*---------------------------------------------- Actions */
typedef enum
{
	ACTION_NONE	= 0,
	ACTION_START = 1,
	ACTION_STOP = 2,
	ACTION_ENABLE = 3,
	ACTION_DISABLE = 4,
	ACTION_ON = 5,
	ACTION_OFF = 6,

}action_t;

typedef struct
{
	const char* name;
	action_t	action;
}action_name_t;

const action_name_t ActionDictionary[] =
{
		{"", 		ACTION_NONE },
		{"START", 	ACTION_START },
		{"STOP", 	ACTION_STOP },
		{"ENABLE", 	ACTION_ENABLE },
		{"DISABLE",	ACTION_DISABLE },
		{"ON",  	ACTION_ON },
		{"OFF",  	ACTION_OFF },
		{"HIGH",  	ACTION_ON },
		{"LOW",  	ACTION_OFF }
};


action_t find_Action(const char *action_name)
{
	for (int i = 1 ; i < ARRAY_SIZE(ActionDictionary); i++)
	{
		if (strcmp(action_name, ActionDictionary[i].name ) == 0)
			return ActionDictionary[i].action;
	}
	return ACTION_NONE;
}


/*---------------------------------------------------- ON / OFF - YES / NO **/

enum
{
	IS_OFF = 0,
	IS_ON  = 1,
	IS_NEITHER = -1,
	NOT_FOUND = -2,
};

typedef struct
{
	char *key;
	bool value;

} bin_keyval_t;

const bin_keyval_t binaryParams[] =
	{
		{"", IS_NEITHER },		// default parameter missing
		{"OFF", IS_OFF },
		{"ON", IS_ON },
		{"LOW", IS_OFF },
		{"HIGH", IS_ON },
		{"0", IS_OFF },
		{"1", IS_ON },
		{"FALSE", IS_OFF },
		{"TRUE", IS_ON },
		{"NEGATIVE", IS_OFF },
		{"POSITIVE", IS_ON },
	};



int find_binaryParam(char* token)
{
	for (int i=0; i < ARRAY_SIZE(binaryParams);i++)
	{
		if (strcmp(token, binaryParams[i].key)==0)
			return binaryParams[i].value;
	}

	return IS_NEITHER;	// not an ON/OFF token
}

/*------------------------------------------------------------------- Find keyword Constant **/
int find_KeywordConstant ( const keyword_t* array, size_t size, const char* word)
{
	for (int i=0; i < size;i++)
	{
		if (strcmp(word, array[i].name) == 0)
			return array[i].key;
	}

	return NOT_FOUND;	// no key found
}


const char* find_KeywordName ( const keyword_t* list, size_t size, const int key)
{
	for(int i = 0; i < size; i++ )
	{
		if( list[i].key == key )
			return list[i].name;
	}
	return "UNKNOWN";
}


/*----------------------------------------------------------------- IMU Commands arguments */

const keyword_t ImuFormatDictionary[] =
{
	{"FSAS", fmtFSAS_NATIVE},		// used for debugging only
	{"NRAW", fmtNOVATEL_RAW},		// default to be consistent with other logs form OEM7700 receiver
	{"NIMR", fmtNOVATEL_IMR},		// IMR format not used
	{"STIM", fmt_STIM300},			// For debugging only
	{"KVH", fmt_KVH},				// For debugging only
};


const keyword_t ImuTypeList[] =
{
	{ "FSAS", IMUType_FSAS },
	{ "STIM", IMUType_STIM300 },
	{ "KVH", IMUType_KVH },
};


const keyword_t ImuComTargetList[] =
{
	{ "PSOC", Target_PSOC },
	{ "NOVATEL", Target_NovAtel },
};

const char* GetImuTypeName( imu_type_t type)
{
	return find_KeywordName(ImuTypeList, ARRAY_SIZE(ImuTypeList), type);
}

const char* ImuConnectName(imu_target_t target)
{
	return find_KeywordName(ImuComTargetList, ARRAY_SIZE(ImuComTargetList), target);
}


int sprintfconfig(char* buffer, size_t size)
{
	sys_config_t config;

	ReadConfig( &config);
	int cnt = 0;

	cnt += snprintf(buffer, size, "Soft reset: %s\n", (config.soft_reset != 0? "Yes": "No"));
	cnt += snprintf(buffer+cnt, size-cnt,"No COM2 Log Init: %s\n",config.no_com2_logs_init != 0?  "True": "False");
	cnt += snprintf(buffer+cnt, size-cnt,"IMU Type: %s\n", GetImuTypeName(config.imu_type));
	cnt += snprintf(buffer+cnt, size-cnt,"IMU connect to: %s\n", ImuConnectName(config.imu_connect));

	return cnt;
}


/*---------------------------------------------------------------- Value Converters */
int ToInt32(char* token, int32_t *pvalue)
{
	 return sscanf(token, "%ld", pvalue);
}

int ToUint32(char* token, uint32_t *pvalue)
{
	 return sscanf(token, "%lu", pvalue);
}


/****************************************************************************
* Command decoder
*****************************************************************************/
#define SIZEOF_CMD_BUFFER 400
char CmdAnswer[SIZEOF_CMD_BUFFER];
/*
void str_to_upper(char* string)
{
    const char OFFSET = 'a' - 'A';

    while ( *string )
    {
       if (*string >= 'a' && *string <= 'z')
    	   *string -= OFFSET;

        string++;
    }
}
*/
#define MAX_CMD_TOKENS	10

bool VerboseCommands = false;

char* verbose_cmd(char** tokens, int cnt, _ports_t sender)
{
	if(cnt > 1)
	{
	  int res =	find_binaryParam(tokens[1]);
	  if (res != IS_NEITHER)
		  VerboseCommands = res;
	}
	snprintf(CmdAnswer,SIZEOF_CMD_BUFFER,"VERBOSE is %s\n", (VerboseCommands? "ON": "OFF"));
	return CmdAnswer;
}

/*---------------------------------------------------------------- Console Ggrab */
typedef char* (*grab_fn)(uint8_t*, _ports_t);
typedef void (*_gab_disconnect)();

grab_fn grab_input = NULL;
_gab_disconnect GrabDisconnect = NULL;

char* ProcessCommand(char* buffer, _ports_t sender)
{
	if (grab_input != NULL)
		return grab_input((uint8_t*)buffer, sender);

	const char delims[] = " \r\n";
	char *tokens[MAX_CMD_TOKENS];

	int i = 0;

	/* first split the string into tokens */
	char *ch = strtok(buffer, delims);

	while (ch != NULL && i < MAX_CMD_TOKENS)
	{
		tokens[i++] = ch;
		ch = strtok(NULL, delims);
	}

	/* if at least one token found the first one is the command
	 * find the command function and call it
	 */

	if (i > 0 )
	{
		memset(CmdAnswer, 0, SIZEOF_CMD_BUFFER);

		fn_command_t fcommand = find_Comand(tokens[0]);

		if (fcommand != NULL)
			return fcommand(tokens,i, sender);

		snprintf(CmdAnswer,SIZEOF_CMD_BUFFER,"{%s} <ERROR> - Unknown command: %s\n", GetPortName(sender), tokens[0]);
		printf(CmdAnswer);

		return CmdAnswer;
	}
	return "UNKNOWN COMMAND"; /* Something went wrong */
}



/****************************************************************************
* Process command received from serial interface (COM or USB)
*****************************************************************************/
volatile bool Cmd_Echo_On = true;		// command echo ON by default
volatile bool clearEventCntr = false;

char CmdBuffer[SIZEOF_CMD_BUFFER];
int cmdIndex = 0;

typedef char* (*fn_cmd_process)(uint8_t*,int,_ports_t);

char* ScanCommandLine(uint8_t *buf, int cnt, _ports_t sender )
{
	char *retval = "";

	if(VerboseCommands == true)
		printf("CMD: %s", (char*)buf);

	cmdIndex = 0; // cleanup input buffer of incomplete commands.

	while(cnt-- > 0 && cmdIndex < SIZEOF_CMD_BUFFER)
	{
		uint8_t ch = *buf++;

		if ( !IS_LINE_TERMINATOR( ch ) )
		{
			CmdBuffer[cmdIndex++] = toupper( ch );
		}
		else if ( cmdIndex > 0 )
		{
			CmdBuffer[cmdIndex] =  0;
			retval = ProcessCommand(CmdBuffer,sender);
			cmdIndex=0;

			// If received more than one command,
			// only the answer of the last one will be returned to caller
		}
	};

	if(VerboseCommands == true)
		printf("ANS: %s", retval);

	return retval;
}


/****************************************************************************
* Command processors
*****************************************************************************/

typedef struct				// Callback arguments structure
{
	char* 		cbuf;		// Buffer for callback use
	size_t		size;		// buffer size
	_ports_t   	cbport;		// port for callback

} CBARGS_t;


char* PrintCmdOK()
{
	snprintf( CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] <OK>\n", GetUpTimeStr());
	return CmdAnswer;
}

char* PrintCmdError( char *string )
{
	snprintf( CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] Error: %s\n", GetUpTimeStr(), string);
	return CmdAnswer;
}


/*---------------------------------------------------------------------------------- MARKPOS REPORTS */
/* Sends GNSS MARK#POS REPORT
 *
 * 	MARKPOSREPORT event_nr [polarity] [port] [action]
 *
 * 	Parameters:
 * 		event_nr	- Event number of MARKPOS to MARK4POS mandatory (1 to 4 valid values)
 * 		polarity	- [ NEGATIVE(default) | POSITIVE]
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
*----------------------------------------------------------------------------------------------------*/
CBARGS_t pmarpos_args;
char markpos_buf[200];
#define MARKPOS_CNT 4

_ports_t markpos_ports[MARKPOS_CNT] = // target ports
{
	INVALID_PORT,
	INVALID_PORT,
	INVALID_PORT,
	INVALID_PORT
};

typedef enum // Task index
{
	MARK1POS_NDX = 0,
	MARK2POS_NDX = 1,
	MARK3POS_NDX = 2,
	MARK4POS_NDX = 3

} markpos_ndx_t;


char markposbuf[120];

void markpos_callback(markpos_rep_t mark)
{
	if (mark.Mark > 0 && mark.Mark < 4)
	{
		_ports_t port = markpos_ports[mark.Mark -1];

		if (port != INVALID_PORT )
		{
			dbuf_t buf;

			buf.Count = FormatMarkPosRecord(markposbuf, ARRAY_SIZE(markposbuf), &mark);
			buf.Buffer = (uint8_t*) markposbuf;
			StoreReportRecord(&buf, port);
		}
	}
}


void Cancel_Markpos_Report(uint16_t mark_nr)
{
	markpos_ndx_t index = mark_nr -1;

	if( mark_nr < MARKPOS_CNT &&  markpos_ports[index] != INVALID_PORT )
	{
		CancelMarkPosNotify(mark_nr);
		markpos_ports[index] = INVALID_PORT;
	}
}

void Cancel_All_Markpos_Reports()
{
	for(uint32_t i=1; i < 5; i++)
		Cancel_Markpos_Report(i);
}


char* markpos_report_cmd(char** tokens, int cnt, _ports_t port)
{
	_ports_t  target = port;		// default to calling port;
	 action_t action = ACTION_START;
	 uint16_t  marknr = 0;
	 uint16_t  polarity = 0;

	if ( cnt > 1 ) /* if no arguments use defaults above */
	{
			/* scan arguments */
		for (int i = 1; i < cnt; i++ )
		{
			_ports_t _p = FindPortID(tokens[i]);

			if (_p != INVALID_PORT)
			{
				target = _p;
				continue;
			}

			action_t _a = find_Action(tokens[i]);
			if (_a != ACTION_NONE)
			{
				action = _a;
				continue;
			}

			int _u = find_binaryParam(tokens[i]);
			if (_u != IS_NEITHER)
				polarity = (int16_t) _u;

			int32_t _v;

			if ( ToInt32(tokens[i], &_v) != 0 )
			{
				if (_v > 0 && _v < 5) /* validate the event number */
					marknr =(uint32_t) _v;
			}
		}
	}

	if (marknr == 0)
	{
		printf("{%s} %s missing mandatory argument mark number [1-4]\n", GetPortName(port), tokens[0]);
		return  PrintCmdError("ERROR: Missing mandatory mark number\n");
	}

	if (action == ACTION_STOP )
	{
		Cancel_Markpos_Report(marknr);
		printf("{%s} %s Stop to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
	}
	else
	{
		markpos_ports[marknr-1] = target;
		RegisterMarkPosNotify(marknr, polarity, markpos_callback );
		printf("{%s} %s Start to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
	}

	return PrintCmdOK();
}
/*---------------------------------------------------------------------------------- MARTIMEx REPORTS */
/* Sends GNSS MARKTIME REPORT
 *
 * 	MARKTIMEREPORT event_nr [polarity] [port] [action]
 *
 * 	Parameters:
 * 		event_nr	- Event number of MARK1TIME to MARK4TIME mandatory (1 to 4 valid values)
 * 		polarity	- [ NEGATIVE(default) | POSITIVE]
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
*----------------------------------------------------------------------------------------------------*/
CBARGS_t pmarktime_args;
char marktime_buf[200];
#define MARKTIME_CNT 4

_ports_t marktime_ports[MARKTIME_CNT] =
{
	INVALID_PORT,
	INVALID_PORT,
	INVALID_PORT,
	INVALID_PORT
};

typedef enum // Task index
{
	MARKTIME1_NDX = 0,
	MAKRTIME2_NDX = 1,
	MARKTIME3_NDX = 2,
	MARKTIME4_NDX = 3

} marktime_ndx_t;


/* defined in novatel.h
 *
typedef struct
{
	uint16_t	Signal;			// indicator of Marktime 1, 2, 3, etc
	uint16_t 	EventNr;		// event counter
	double 		EventTime;		// week seconds of the event
	uint16_t 	Week;

} marktime_rep_t;
*/

char marktimekbuf[100];

void marktime_callback(marktime_rep_t mark)
{

	if (mark.Mark > 0 && mark.Mark < 4)
	{
		_ports_t port = marktime_ports[mark.Mark -1];

		if (port != INVALID_PORT )
		{
			dbuf_t buf;

			buf.Count = FormatMarkTimeRecord(marktimekbuf, ARRAY_SIZE(marktimekbuf), &mark);
			buf.Buffer = (uint8_t*) marktimekbuf;
			StoreReportRecord(&buf, port);
		}
	}
}


void Cancel_Marktime_Report(uint16_t mark_nr)
{
	marktime_ndx_t index = mark_nr -1;

	if( mark_nr < MARKTIME_CNT &&  marktime_ports[index] != INVALID_PORT )
	{
		CancelMarkTimeNotify(mark_nr);
		marktime_ports[index] = INVALID_PORT;
	}
}

void Cancel_All_Marktime_Reports()
{
	for(uint32_t i=1; i < 5; i++)
		Cancel_Marktime_Report(i);
}

char* marktime_report_cmd(char** tokens, int cnt, _ports_t port)
{
	_ports_t  target = port;		// default to calling port;
	 action_t action = ACTION_START;
	 uint16_t  marknr = 0;
	 uint16_t  polarity = 0;

	if ( cnt > 1 ) /* if no arguments use defaults above */
	{
			/* scan arguments */
		for (int i = 1; i < cnt; i++ )
		{
			_ports_t _p = FindPortID(tokens[i]);

			if (_p != INVALID_PORT)
			{
				target = _p;
				continue;
			}

			action_t _a = find_Action(tokens[i]);
			if (_a != ACTION_NONE)
			{
				action = _a;
				continue;
			}

			int _u = find_binaryParam(tokens[i]);
			if (_u != IS_NEITHER)
				polarity = (int16_t) _u;

			int32_t _v;

			if ( ToInt32(tokens[i], &_v) != 0 )
			{
				if (_v > 0 && _v < 5) /* validate the event number */
					marknr =(uint32_t) _v;
			}
		}
	}

	if (marknr == 0)
	{
		printf("{%s} %s missing mandatory argument mark number [1-4]\n", GetPortName(port), tokens[0]);
		return  PrintCmdError("ERROR: Missing mandatory mark number\n");
	}

	if (action == ACTION_STOP )
	{
		Cancel_Marktime_Report(marknr);
		printf("{%s} %s Stop to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
	}
	else
	{
		marktime_ports[marknr-1] = target;
		RegisterMarkTimeNotify(marknr, polarity, marktime_callback );
		printf("{%s} %s Start to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
	}

	return PrintCmdOK();
}

/*----------------------------------------------------------------------------------------- POSREPORT */
/* Sends position reports
 *
 * 	POSREPORT  [port] [action] [period]
 *
 * 	Parameters:
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
 * 		period		- repetition period in milliseconds. Default 1000 (1 second)
 *----------------------------------------------------------------------------------------------------*/

CBARGS_t posreport_args;
char posreport_buf[200];
GPT_TASK_t posreport_task =
{
	.thandle = INVALID_GPTIMER_HANDLE
};


/*
typedef enum
{
	GEO, UTM

} posformat_t;


void utmreport_callback(void* cbarg, cyhal_timer_event_t event )
{
	dbuf_t buf;
	CBARGS_t* dat = (CBARGS_t*)cbarg;
	buf.Buffer = (uint8_t*) dat->cbuf;

	buf.Count = PrintCoordinates(dat->cbuf, dat->size, true);
	StoreReportRecord(&buf, dat->cbport);
}
*/


//void georeport_callback(void* cbarg, cyhal_timer_event_t event )
void georeport_callback(void* cbarg )
{
	dbuf_t buf;
	CBARGS_t* dat = (CBARGS_t*)cbarg;
	buf.Buffer = (uint8_t*) dat->cbuf;

	position_t pos;
	InterlockPosCopy((pposition_t) &pos, (pposition_t) &LastPosition);

	buf.Count = PrintGeoCoordinates(dat->cbuf, dat->size, &pos);
	StoreReportRecord(&buf, dat->cbport);
}


char *pos_report_cmd(char** tokens, int cnt, _ports_t port)
{
//	posformat_t format = GEO;
	_ports_t  target = port;		// default to calling port;
	 action_t action = ACTION_START;
	 int32_t  period = 1000;


	if ( cnt > 1 ) /* if no arguments use defaults above */
	{
			/* scan arguments */
		for (int i = 1; i < cnt; i++ )
		{
			_ports_t _p = FindPortID(tokens[i]);

			if (_p != INVALID_PORT)
			{
				target = _p;
				continue;
			}

			action_t _a = find_Action(tokens[i]);
			if (_a != ACTION_NONE)
			{
				action = _a;
				continue;
			}

			int32_t _v;

			if ( ToInt32(tokens[i], &_v) != 0 )
			{
				if(_v < MIN_POSITION_REPORT_PERIOD_MSEC)
					period =  MIN_POSITION_REPORT_PERIOD_MSEC;
				else if (_v > 0 )
					period = _v;
			}
//			if ( strcmp(tokens[i], "UTM" ) == 0)
//				format = UTM;
		}
	}

	if (action == ACTION_STOP)
	{
		if ( TEST_BITS( posreport_args.cbport, target) )
		{
			printf("{%s} Stopping %s to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
			StopPeriodicTask(&posreport_task);
		}
		else
		{
			return PrintCmdError( "Invalid target port" );
		}
	}
	else
	{
		/* In this version this function is not reentrant.
		 * If a status report is running it must be cancelled
		 *  and replaced with the new request.
		 */
		if (GPTaskRunning(&posreport_task) )
		{
			StopPeriodicTask(&posreport_task);
			printf("{%s} STOPPING %s to %s\n", GetPortName(port), tokens[0],  GetPortName(posreport_args.cbport));
		}
		printf("{%s} Send %s to %s each %ld ms\n",  GetPortName(port), tokens[0], GetPortName(target), period );

		// setup callback arguments
		posreport_args.cbuf = posreport_buf;
		posreport_args.size = ARRAY_SIZE(posreport_buf);
		posreport_args.cbport = target;

		//setup periodic task
		posreport_task.period   = period;
		posreport_task.oneshot  = false;
		posreport_task.cbarg    = &posreport_args;
//		posreport_task.callback = format == UTM? utmreport_callback : georeport_callback;
		posreport_task.callback = georeport_callback;
		posreport_task.thandle = INVALID_GPTIMER_HANDLE;

		if ( StartPeriodicTask(&posreport_task) != SC_RESULT_OK )
			return PrintCmdError("Failed to create POSREPORT task.\n");
	}
	return CmdAnswer;
}

void Cancel_Pos_Reports()
{
	if (GPTaskRunning(&posreport_task) )
	{
		StopPeriodicTask(&posreport_task);
	}
}

/*---------------------------------------------------------------------------- STATUS | SHORTSTATUS */
/* command formats for both versions:
 *
 * 	STATUS  [port] [action] [period]
 *
 * 	Parameters:
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
 * 		period		- repetition period in milliseconds. Default 1000 (1 second)
 *----------------------------------------------------------------------------------------------------*/

CBARGS_t 	status_report_args;
GPT_TASK_t status_report_task =
{
	.thandle = INVALID_GPTIMER_HANDLE
};

char status_buffer[200];

/****************************************************************************
* PrintStatusLong()
*****************************************************************************/
int PrintStatusLong(char* buffer, size_t size)
{
	int cnt = GetIMUStatusStr(buffer, size);

	if ( cnt < size )
		cnt += GetNovatelStatus(buffer + cnt, size - cnt, false );

	if (cnt > 0 && cnt < size)
	{
		if (LoggingPort == 0 )
			cnt += snprintf(buffer+cnt, size-cnt, "Logging status: IDDLE\n" );
		else
			cnt += snprintf(buffer+cnt, size-cnt, "Logging status: LOGGING on port %s\n", GetPortName(LoggingPort) );
	}

	if (cnt > 0 && cnt < size)
	{
		up_time_t T = GetUpTime();
		cnt += snprintf(buffer+cnt, size-cnt, "Up Time: %d:%d:%d\n", T.Hours, T.Minutes, T.Seconds );
	}

	return cnt;
}


void long_status_callback(void* cbarg )
{
	dbuf_t buf;
	CBARGS_t* dat = (CBARGS_t*)cbarg;

	buf.Count = PrintStatusLong(dat->cbuf, dat->size);
	buf.Buffer = (uint8_t*) dat->cbuf;
	StoreReportRecord(&buf, dat->cbport);
}


/****************************************************************************
* PrintStatusShort()
*****************************************************************************/
int PrintStatusShort(char* buffer, size_t size)
{
	int cnt = snprintf(buffer, size, "#STATUS,%c,",((LoggingPort == 0 )? 'I':'L') );  // 'I' = IDDLE, 'L' = LOGGING

	if (cnt < size )
		cnt += GetIMUStatusShort(buffer + cnt, size - cnt);

	if ( cnt < size )
		cnt += GetNovatelStatus(buffer + cnt, size - cnt, true );

	if ( cnt < size )
	{
		up_time_t T = GetUpTime();
		cnt += snprintf(buffer + cnt, size - cnt, ",%.2d:%.2d:%.2d\n", T.Hours, T.Minutes, T.Seconds );
	}

	return cnt;
}


void short_status_callback(void* cbarg )
{
	dbuf_t buf;
	CBARGS_t* dat = (CBARGS_t*)cbarg;

	buf.Count = PrintStatusShort(dat->cbuf, dat->size);
	buf.Buffer = (uint8_t*) dat->cbuf;
	StoreReportRecord(&buf, dat->cbport);
}


/*----------------------------------------------------------------------- STATUS Command processor -*/

char *status_cmd(char**tokens, int cnt, _ports_t port )
{
	_ports_t target = port;		// default to calling port;
	 action_t action = ACTION_START;
	 int32_t period = 1000;
	 bool short_status = false;

	 if (strcmp(tokens[0], "SHORTSTATUS") == 0)
	 {
		 short_status = true;
	 }

	if (short_status)
		PrintStatusShort(CmdAnswer, ARRAY_SIZE(CmdAnswer));
	else
		PrintStatusLong(CmdAnswer, ARRAY_SIZE(CmdAnswer));

	if ( cnt == 1 ) // if no arguments => send one status report to caller
	{
		return CmdAnswer;
	}

	/* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _p = FindPortID(tokens[i]);

		if (_p != INVALID_PORT)
		{
			target = _p;
			continue;
		}

		action_t _a = find_Action(tokens[i]);
		if (_a != ACTION_NONE)
		{
			action = _a;
			continue;
		}

		int32_t _v;

		if ( ToInt32(tokens[i], &_v) != 0 )
			period = _v;
	}

	/* turn status report off if it is running and correct port requested */
	if ( action == ACTION_STOP )
	{
		if ( TEST_BITS( status_report_args.cbport, target) )
		{
			printf("{%s} Stopping %s to %s\n", GetPortName(port), tokens[0], GetPortName(target) );
			StopPeriodicTask(&status_report_task);
		}
		else
		{
			return PrintCmdError( "Invalid target port" );
		}
	}
	else if ( period > 0 ) // start a periodic timer
	{
		/* In this version this function is not reentrant.
		 * If a status report is running it must be cancelled
		 *  and replaced with the new request.
		 */
		if (GPTaskRunning(&status_report_task) )
		{
			StopPeriodicTask(&status_report_task);
			printf("{%s} STOPPING %s to %s\n", GetPortName(port), tokens[0],  GetPortName(status_report_args.cbport));
		}
		printf("{%s} Send %s to %s each %ld ms\n",  GetPortName(port), tokens[0], GetPortName(target), period );

		// setup callback arguments
		status_report_args.cbuf = status_buffer;
		status_report_args.size = ARRAY_SIZE(status_buffer);
		status_report_args.cbport = target;

		//setup periodic task
		status_report_task.period  = period;
		status_report_task.oneshot = false;
		status_report_task.cbarg   = &status_report_args;
		status_report_task.callback= short_status != 0? short_status_callback : long_status_callback;
		status_report_task.thandle = INVALID_GPTIMER_HANDLE;

		if ( StartPeriodicTask(&status_report_task) != SC_RESULT_OK )
			return PrintCmdError("Failed to create Status Report task.\n");
	}

	return CmdAnswer;
}

void Cancel_Status_Reports()
{
	StopPeriodicTask(&status_report_task);
}

/*--------------------------------------------------------------------------------ECHO */
/* command format:
 *
 * 	ECHO ON or OFF
 *
 */
char *echo_cmd(char**tokens, int cnt, _ports_t port)
{
	char * retval="";

	if (cnt > 1)
	{
		int echo = find_binaryParam(tokens[1]);

		switch (echo )
		{
			case IS_ON:
				Cmd_Echo_On = true;
				retval = "<OK> - ECHO_ON\n";
				break;
			case IS_OFF:
				Cmd_Echo_On = false;
				break;
			default:
				retval = PrintCmdError( "Invalid argument." );
				break;
		}
	}
	return retval;
}

/*--------------------------------------------------------------------------------------IMUFREQ */
/* command format:
 *
 * 	IMUFREQ		- Returns current IMU Frequency
N *
 */
char *imufreq_cmd(char**tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"IMUFREQ %ld\n", imuFrequency);
		return CmdAnswer;
	}
	int32_t freq;

	if ( ToInt32(tokens[1], &freq) == 0 )
	{
		return PrintCmdError("Argument must be 0 to stop TDAS or between 50 and 200 Hz.");
	}

	if ( freq > MAX_IMU_FREQUENCY  || freq < 0  )
	{
		return "Frequency out of range 0 <= f <= 200";
	}
	if (freq == 0)
	{
		Stop_FSAS_Trigger_Frequency();
	}
	else
	{
		Set_FSAS_Trigger_Frequency(freq); // extend the period 20Hz to ensure sync with MARK1 time.
	}
	return PrintCmdOK();
}



/*------------------------------------------------------------------------------------- IMUFORMAT */
/* Command format:
 *
 * 	IMUFORMAT format
 *
 * 	Argument:
 * 		format 		FSAS, IMR, STIM, or RAW (startup default)
 *
 * 	Format is not saved to config. Always defaults to RAW on startup.
 *
 */
/*
int find_IMU_Format(const char*token)
{
	int fmt = find_KeywordConstant( ImuFormatDictionary, ARRAY_SIZE(ImuFormatDictionary), token);
	return fmt == IS_NEITHER? INVALID_IMU_FORMAT: fmt;
}
*/

char *imuformat_cmd(char**tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		return PrintCmdError( "Missing FORMAT argument." );
	}
	int format = find_KeywordConstant( ImuFormatDictionary, ARRAY_SIZE(ImuFormatDictionary), tokens[1]);


	if ( format == NOT_FOUND )
	{
		return PrintCmdError( "Unknown FORMAT argument.");
	}
	else
	{
		SetImuDataFormat((imu_format_t)format);
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU data format changed to %s\n", GetUpTimeStr(), tokens[1] );
	}
	return CmdAnswer;
}

/*------------------------------------------------------------------------------------- IMUTYPE */
/* Command format:
 *
 * 	IMUTYPE type
 *
 * 	Argument:
 * 		type 		FSAS or STIM
 *
 */

char *imutype_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		return PrintCmdError( "Missing TYPE argument.");
	}

	int key = find_KeywordConstant( ImuTypeList, ARRAY_SIZE(ImuTypeList), tokens[1]);

	if (key == NOT_FOUND )
	{
		return PrintCmdError("Missing IMU TYPE argument.");
	}
	else
	{
		bool save = false;
		char* saved = "NOT saved.";
		if (cnt == 3)
		{
			if (strcmp(tokens[2],"SAVE") == 0)
			{
				save = true;
				saved = "SAVED.";
			}
		}
		Set_IMU_Type((imu_type_t) key, save);

		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU TYPE changed to %s - %s\n", GetUpTimeStr(), tokens[1], saved );
	}
	return CmdAnswer;
}


/*------------------------------------------------------------------------------------- IMUCONNECT */
/* Command format:
 *
 * 	IMUCONNECT target
 *
 * 	Argument:
 * 		target 		PSOC \ OEM7700
 *
 */
char *imuconnect_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		return PrintCmdError( "Missing TYPE argument.");
	}

	int key = find_KeywordConstant( ImuComTargetList, ARRAY_SIZE(ImuComTargetList), tokens[1]);

	if (key == NOT_FOUND )
	{
		return PrintCmdError( "Unknown COM TARGET argument." );
	}
	else
	{
		Set_IMU_COM_Target(key);

		char* saved = "Not saved.";

		if (cnt > 2 && strcmp(tokens[2],"SAVE") == 0)
		{
			SaveConfig(&SysConfig);
			saved = "SAVED.";
		}
		Store_IMU_COM_Target((imu_target_t) key);
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU target changed to %s - %s\n", GetUpTimeStr(), tokens[1], saved );
	}
	return CmdAnswer;
}



/*-------------------------------------------------------------------------------------- LOG */
/* Command format:
 *
 * LOG	- Alone reports the log status
 * LOG -  [port] action
 *
 * Argunemts:
 * 		port		- The log target port. Default the calling port
 * 		action		- START or STOP (defaults to no arguments behavior)
 */

char *log_cmd(char**tokens, int cnt, _ports_t port)
{
	_ports_t target = port;		// default to calling port;
	 action_t action = ACTION_NONE;

	 /* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _p = FindPortID(tokens[i]);

		if (_p != INVALID_PORT)
		{
			target = _p;
			continue;
		}

		action_t _a = find_Action(tokens[i]);
		if (_a != ACTION_NONE)
		{
			action = _a;
			continue;
		}
	}

	if ( action == ACTION_NONE)
	{
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"LOG %s\n", TEST_BITS(LoggingPort,target)? "ON": "IDLE");
		return CmdAnswer;
	}

	if (action == ACTION_START)
	{
		Enable_IMU_Logging = false;
		Enable_GNSS_Logging = false;

		GPTimerDelay(100);

		PurgeBuffers();

		SET_BITS(LoggingPort, target);

		Enable_GNSS_Logging = true;
		SendStartLoggingMessages();

		GPTimerDelay(1500);
		Enable_IMU_Logging = true;

		printf("[%s] Starting log to port %s\n", GetUpTimeStr(),GetPortName(target) );
	}

	if (action == ACTION_STOP)
	{
		Enable_IMU_Logging = false;
		Enable_GNSS_Logging = false;

		GPTimerDelay(1000);

		CLR_BITS(LoggingPort,target);
		printf("[%s] Stopping log to port %s\n", GetUpTimeStr(),GetPortName(target) );
	}

	// Do not echo if logging to same port
	return (target != port)? PrintCmdOK(): NULL;
}


/*---------------------------------------------------------------------------------- VERSIONINFO */

char *version_cmd(char**tokens, int cnt, _ports_t port)
{
	return (char*)VersionString;
}

/*-------------------------------------------------------------------------------------- PORTID */

char pidbuf[20];

char *port_id_cmd(char**tokens,int cnt,_ports_t port)
{
	_ports_t target = port;

	if ( cnt > 1)
	{
		_ports_t _p = FindPortID(tokens[1]);

		if (_p != INVALID_PORT)
		{
			target = _p;
		}
	}
	snprintf(pidbuf,ARRAY_SIZE(pidbuf),"{%s} SC200\n", GetPortName(target));

	if ( target != port)
	{
		dbuf_t db;
		db.Count = strlen(pidbuf);
		db.Buffer = (uint8_t*) pidbuf;
		StoreReportRecord(&db, target);
		return PrintCmdOK();
	}
	return pidbuf;
}

/*---------------------------------------------------------------------------------- HELP */

char *help_id_cmd(char**tokens,int cnt, _ports_t port)
{
	char buf[100];

	for (int i = 0; i< ARRAY_SIZE(CmdDictionary); i++)
	{
		int nr = snprintf(buf, 100, "%s\n", CmdDictionary[i].cmd);
		SendToPort(port, (uint8_t*)buf, nr);
	}
	return PrintCmdOK();
}

/*---------------------------------------------------------------------------------- STANDBY */
/* Stop all reporting activity
 */
char *standby_cmd(char**tokens, int cnt, _ports_t port)
{
	printf("{%s} STANDBY command received.\n", GetPortName(port));

	Cancel_Status_Reports();
	Cancel_Pos_Reports();
	Cancel_All_Marktime_Reports();
	Cancel_All_Markpos_Reports();
	ReleaseAllGPTimers();

	LoggingPort = 0;	// stops logging

	return port_id_cmd(tokens, cnt, port);  // return port ID
}


/*---------------------------------------------------------------------------------- COPY CONSOLE (NOT WORKING) */
char *copy_console_cmd(char** tokens,int cnt, _ports_t port)
{
	_ports_t target = port;		// default to calling port;
	 action_t action = ACTION_ON;

	 /* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _p = FindPortID(tokens[i]);

		if (_p != INVALID_PORT)
		{
			target = _p;
			continue;
		}

		action_t _a = find_Action(tokens[i]);

		if (_a == ACTION_ON || _a == ACTION_OFF)
		{
			action = _a;
			continue;
		}
	}

	SetConsoleCopy((action == ACTION_ON)? target : INVALID_PORT );

	return PrintCmdOK();
}

/*----------------------------------------------------------------------------------- SYSTEMRESET */

void reset_worker(void* pv)
{
	__NVIC_SystemReset();
}

char *system_reset_cmd(char**tokens, int cnt, _ports_t port)
{
	uint16_t tout = 2000;

	GPT_HANDLE handle = CreateGPTimer(reset_worker, NULL, tout, true);
	StartGPTimer(handle);

	return "System will reset in 2seconds\n";	/* return never reached */
}


/*----------------------------------------------------------------------------------- MAPLBREPORT */
char *map_ladybug_report_cmd(char** tokens, int cnt, _ports_t port)
{
	 /* scan arguments */
	if (cnt < 2)
		return "Missing port info.\n";

	if ( MapLadyBugLogs(tokens[1]) )
		return PrintCmdError("Invalid PORT argument.\n");

	return PrintCmdOK();
}

/*----------------------------------------------------------------------------------- REQUESTGPSLOG */
char *request_gps_log_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
		return "Missing argument.\n";

	if ( RequestGpsLog(tokens[1]) )
			return PrintCmdError("Invalid LOG format.\n");

		return PrintCmdOK();
}

/*----------------------------------------------------------------------------------- SYSCONFIG */
/* Prints the config values stored in EPROM
 */
char *sysconfig_cmd(char** tokens, int cnt, _ports_t port)
{
	int count = sprintfconfig(CmdAnswer, ARRAY_SIZE(CmdAnswer));

	SendToPort( port, (uint8_t*)CmdAnswer, count );
	return PrintCmdOK();
}

/*------------------------------------------------------------------------------------ GNSSLOG
 * Command format:
 *
 *	GNSSLOG ENABLE | DISABLE
 *
 *	For IMU debug purposes, to log IMU records alone.
 */
char *gnss_log_cmd(char**tokens, int cnt,  _ports_t port)
{
	if (cnt > 1)
	{
		action_t action = find_Action(tokens[1]);

		if (action == ACTION_ENABLE || action == ACTION_DISABLE)
		{
			SysConfig.no_com2_logs_init = (action == ACTION_DISABLE);
			SaveConfig(&SysConfig);
		}
		else
		{
			return "Valid arguments are: ENABLE , DISABLE\n";
		}
	}

	return (SysConfig.no_com2_logs_init == false)? "GNSS_LOG_INIT ENABLED\n" : "GNSS_LOG_INIT DISABLED\n";
}


extern bool DisableImuLogs;

char *disableIMULog_cmd(char**tokens, int cnt,_ports_t port)
{
	if (cnt > 1)
	{
		action_t action = find_Action(tokens[1]);

		if (action == ACTION_ON || action == ACTION_OFF)
		{
			DisableImuLogs = (action == ACTION_ON);
			SaveConfig(&SysConfig);
		}
		else
		{
			return "Valid arguments are: ON , OFF\n";
		}
	}
	return ( DisableImuLogs == false)? "DISABLEIMULOGS is OFF\n" :  "DISABLEIMULOGS is ON\n";
}


/*-----------------------------------------------------------------------------------------
 * GPIO PIN Set, clear, pulse, monitor
 *-----------------------------------------------------------------------------------------*/

char *set_pin_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt > 2)
	{
		cyhal_gpio_t pin = find_pin_id(tokens[1]);

		if (pin != INVALID_PIN_ID)
		{
			int value = find_binaryParam(tokens[2]);

			if (value == IS_ON)
			{
				SET_GPIO_PIN(pin);
			}
			else if (value == IS_OFF)
			{
				CLEAR_GPIO_PIN(pin);
			}
			else
			{
				return "<ERROR> BAD INPUT VALUE\n";
			}
			snprintf(CmdAnswer,SIZEOF_CMD_BUFFER,"<OK> %s,%d\n", tokens[1],  TEST_HAL_PIN(pin));
			return CmdAnswer;
		}
		return "<ERROR> BAD PIN NAME. USE J6.2 - J6.5\n";
	}
	return "Usage: WRITEPIN J6.x ON or OFF\n";
}

/*--------------------------------------------------------------------------
 *  Pulse_pin_cmd(); - Warning: only one pin at a time can be pulsed
 *-------------------------------------------------------------------------*/
GPT_HANDLE pinPulseHandle = INVALID_GPTIMER_HANDLE;
cyhal_gpio_t calback_pulse_pin = INVALID_PIN_ID;

void PulsePinCallback(void * cbpin)
{
	TOGGLE_GPIO_PIN(calback_pulse_pin);
	//ReleaseGPTimer(pinPulseHandle);

	pinPulseHandle = INVALID_GPTIMER_HANDLE;
}

char *pulse_pin_cmd(char** tokens, int cnt, _ports_t port)
{
	uint16_t duration = 1000;

	if (cnt == 1)
	{
		return "Usage: PULSEPIN J6.x [millisec]\n";
	}
	calback_pulse_pin = find_pin_id(tokens[1]);

	if (calback_pulse_pin == INVALID_PIN_ID )
	{
		return "<ERROR> BAD PIN NAME. USE J6.2 - J6.5\n";
	}

	if (cnt > 2)
	{
		int32_t t;

		if ( ToInt32(tokens[2], &t) != 1 || t < 1)
		{
			return "<ERROR> IVALID PULSE DURATION\n";
		}
		duration = (uint16_t)t;
		}

	pinPulseHandle = CreateGPTimer(PulsePinCallback, &calback_pulse_pin, duration, true );

	if (pinPulseHandle == INVALID_GPTIMER_HANDLE)
	{
		return "<ERROR> GPTIMER CREATION FAILED\n";
	}
	TOGGLE_GPIO_PIN(calback_pulse_pin);

	StartGPTimer(pinPulseHandle);
	return PrintCmdOK();

}


char *read_pin_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt > 1)
	{
		cyhal_gpio_t pin = find_pin_id(tokens[1]);

		if (pin == INVALID_PIN_ID )
		{
			return "BAD PIN NAME. USE J6.2 - J6.5\n";
		}
		snprintf(CmdAnswer,SIZEOF_CMD_BUFFER,"#%s,%d\n", tokens[1],  TEST_HAL_PIN(pin));
		return CmdAnswer;
	}
	return "MISSING PARAMETERS\n";
}

/*-------------------------------------------------------------------
 * J6 PIN Monitor - Reports input pin state changes
 *-------------------------------------------------------------------*/
_ports_t monitor_port;
char monitor_buffer[20];

GPT_TASK_t monitor_pin_task =
{
	.thandle = INVALID_GPTIMER_HANDLE
};

cyhal_gpio_t calback_monitor_pin = INVALID_PIN_ID;


void pin_event_callback(const char* pinid, bool level)
{
	dbuf_t buf;
	// control message must start with #
	buf.Count = snprintf(monitor_buffer,sizeof(monitor_buffer), "#%s,%d\n", pinid, level);
	buf.Buffer = (uint8_t*) monitor_buffer;
	StoreReportRecord(&buf, monitor_port);
}

void MonitorPin_Callback(void * cbpin)
{
	cyhal_gpio_t pin  = *((cyhal_gpio_t*)cbpin);
	pin_event_callback(find_pin_name(pin), TEST_HAL_PIN(pin));
}


char *monitor_pin_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 1)
	{
		return "Usage: MONITORPIN J6.x [millisec] [port] [START/STOP]\n";
	}

	_ports_t target_port = port;		// default to calling port;
	uint16_t period = 1000;
	action_t action = ACTION_START;
	cyhal_gpio_t pin =  INVALID_PIN_ID;

	 /* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _t = FindPortID(tokens[i]);

		if (_t != INVALID_PORT)
		{
			target_port = _t;
			continue;
		}

		cyhal_gpio_t _p = find_pin_id(tokens[i]);

		if (_p != INVALID_PIN_ID )
		{
			pin = _p;
			continue;
		}

		action_t _a = find_Action(tokens[i]);
			if (_a != ACTION_NONE)
			{
				action = _a;
				continue;
			}

		int32_t t;

		if ( ToInt32(tokens[i], &t) == 1 && t >= 200)
		{
			period = (uint16_t)t;
			continue;
		}
	}

	if (pin == INVALID_PIN_ID )
	{
		return "BAD PIN NAME. USE J6.2 - J6.5\n";
	}

	if ( action == ACTION_STOP )
	{
		if (calback_monitor_pin == pin && monitor_port == target_port )
		{
			printf("Stopping monitor pin %s to %s\n", find_pin_name(pin), GetPortName(monitor_port) );
			StopPeriodicTask(&monitor_pin_task);
			return PrintCmdOK();
		}
		else
		{
			return PrintCmdError( "Invalid pin or port\n" );
		}
	}
	else
	{
		if (GPTaskRunning(&monitor_pin_task) )
		{
			printf("Stopping monitor pin %s to %s\n", find_pin_name(pin), GetPortName(monitor_port) );
			StopPeriodicTask(&monitor_pin_task);
		}

		printf("Start monitor pin %s to %s\n", find_pin_name(pin), GetPortName(target_port) );

		calback_monitor_pin = pin;
		monitor_port = target_port;

		//setup periodic task
		monitor_pin_task.period  = period;
		monitor_pin_task.oneshot = false;
		monitor_pin_task.cbarg   = &calback_monitor_pin;
		monitor_pin_task.callback= MonitorPin_Callback;
		monitor_pin_task.thandle = INVALID_GPTIMER_HANDLE;

		if ( StartPeriodicTask(&monitor_pin_task) != SC_RESULT_OK )
			return PrintCmdError("Failed to create MONITORPIN task.\n");
	}
	return PrintCmdOK();
}

char* cancel_pin_monitor_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt > 1)
	{
		cyhal_gpio_t pin = find_pin_id(tokens[1]);

		if (pin == INVALID_PIN_ID )
		{
			return "BAD PIN NAME. USE J6.2 - J6.5\n";
		}
		StopPeriodicTask(&monitor_pin_task);

		//Disable_Input_Pin_Event(pin);
	}
	return PrintCmdOK();
}


/*----------------------------------------------------------------------------- TEST MULTIPLE TIMERS */

GPT_HANDLE TimerHandles[MAX_TIMERS] =
{
		INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE,
		INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE,
};


void TimerTest_Callback(void *arg)
{
	GPT_HANDLE handle = *((GPT_HANDLE*)arg);
	uint32_t T = GetGPTImerPeriod(handle);
	printf("TIMER %ld - Elapsed: %ld\n", handle, T);
}

char* create_timer_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 1)
	{
		return "Usage: CREATETIMER duration_in_ms [TRUE for one shot]\n";
	}

	bool once = false;
	int32_t T;


	if ( ToInt32(tokens[1], &T) == 0 )
	{
		return "INVALID TIMER DURATION";
	}

	if (cnt == 3)
	{
		bool B =find_binaryParam(tokens[2]);
		if(B == IS_ON)
		{
			once = B;
		}
	}

	for(int i=0; i < ARRAY_SIZE(TimerHandles); i++)
	{
		if (TimerHandles[i] == INVALID_GPTIMER_HANDLE) // find first available timer
		{
			GPT_HANDLE h = CreateGPTimer(TimerTest_Callback, TimerHandles + i, T, once );
			if (h != INVALID_GPTIMER_HANDLE)
			{
				TimerHandles[i] = h;
				StartGPTimer(h);

				snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld CREATED\n", h);
				return CmdAnswer;
			}
		}
	}
	return "NO MORE TIMERS AVAILABLE\n";
}


char* release_timer_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 2)
	{
		int32_t T;

		if ( ToInt32(tokens[1], &T) == 0 )
		{
			return "INVALID TIMER NUMBER";
		}
		for(int i=0; i < ARRAY_SIZE(TimerHandles); i++ )
		{
			if(T == TimerHandles[i])
			{
				StopGPTimer(T);
				ReleaseGPTimer(T);
				TimerHandles[i] = INVALID_GPTIMER_HANDLE;

				snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld RELEASED\n", T);
				return CmdAnswer;
			}
		}
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld NOT FOUND\n", T);
		return CmdAnswer;
	}
	return "Usage: RELEASETIMER timer_nr\n";
}



/*------------------------------------------------------------------------------------ BENCHMARKS */
void BenchmarkMalloc(size_t bufsize, int count);
void BenchmarkRingBuffer(int bufsize, int count);
char* BenchmarkFlexRingBuffer(char**tokens,int cnt,_ports_t port);

char *benchmark_cmd(char**tokens,int cnt,_ports_t port)
{
	if (cnt < 4)
		return PrintCmdError(
				"BENCHMARK p1 p2 p3 pn\n"
				" p1 = 1 - BenchmarkMalloc()\n"
				" p1 = 2 - BenchmarkRingBuffer()\n"
				" p1 = 3 - BenchmarkFlexRingBuffer()\n"
				" p2, pn - benchmark parameters\n"
				);

	int32_t p1, p2, p3;

	if ( ToInt32(tokens[1], &p1) == 0 )
		return PrintCmdError( "bad parameter 1.");

	if ( ToInt32(tokens[2], &p2) == 0 )
		return PrintCmdError( "bad parameter 2.");

	if ( ToInt32(tokens[3], &p3) == 0 )
		return PrintCmdError( "bad parameter 3.");

	switch(p1)
	{
		case 1:
		printf("Malloc benchmark %ld %ld\n", p2, p3);
		BenchmarkMalloc(p2,p3);
		break;

		case 2:
		printf("BenchmarkRingBuffer  %ld %ld\n", p2, p3);
		BenchmarkRingBuffer(p2,p3);
		break;

		case 3:
		printf("BenchmarkFlexRingBuffer %ld %ld\n", p2, p3);
		return BenchmarkFlexRingBuffer(tokens, cnt, port);
		break;
	}

	return PrintCmdOK();
}


/*------------------------------------------------------------------------------------ TESTS */
char* test_imu_loopback( _ports_t port );
char* test_log_imu(char**tokens, int cnt, _ports_t port );
char* test_forward_imu( char* port );
char* test_bswap( _ports_t port );
char* test_init_bit( _ports_t port );
char* test_imu_pop( _ports_t port );

char *test_cmd(char**tokens, int cnt, _ports_t port)
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	if (cnt < 2)
	{
			snprintf(buf, size, "Test help:\n"
					"\tTEST 1 - IMU port loopback.\n"
					"\tTEST 2 - STIM300 Log to port _port_name_.\n"
					"\tTEST 3 - STIM300 FORWARD TO PORT.\n"
					"\tTEST 4 - Dequeue IMU Records.\n"
					"\tTEST 5 - Enable INIT_BIT PPS Output.\n"
					);

			return buf;
	}

	int32_t t1;

	if ( ToInt32(tokens[1], &t1) == 0 )
		return PrintCmdError( "bad parameter 1.");

	switch(t1)
	{
		case 1:
		{
			buf = test_imu_loopback(port);
			break;
		}
		case 2:
		{
			buf = test_log_imu(tokens, cnt, port);
			break;
		}

		case 3:
		{
			if (cnt >= 3)
				buf = test_forward_imu( tokens[2] );
			else
				return PrintCmdError("Forward STIM300 missing port param\n");
			break;
		}

		case 4:
			{
				buf = test_imu_pop( port );
				break;
			}
		case 5:
		{
			test_init_bit(port);
			break;
		}

		default:
			snprintf(buf, size, "Incorrect test # [%ld]\n", t1);
	}
	return buf;
}


/*----------------------------------------------- TEST - IMU PORT LOOPBACK */


char *GrabConsoleInput(uint8_t* buf, _ports_t port)
{
	if (buf[0] == 'x' || buf[0] == 'X') // Test for grab end
	{
		grab_input = NULL;

		if ( GrabDisconnect != NULL)
		{
			GrabDisconnect();
			GrabDisconnect = NULL;
		}

		return "Console disconnected.\n";
	}
	int cnt = strlen((char*)buf);
	Uart_IMU_Send(buf, cnt); 	// Send line to IMU port

	return "Data sent to IMU port.\n";
}

/*----------------------------------------*/
fnMessageProcessor 	old_processor;
uint32_t	old_isr_level;

void DisconnectLoopback()
{
	Reconfig_Uart_IMU(old_isr_level, false );
	SetImuMsgProcessor(old_processor);
	Enable_Uart_IMU();
}

/*----------------------------------------*/
uint8_t testBuf[128];
#define MAX_TEST_BUF 127
int tbIndex = 0;
bauds_t imu_bauds = B115200;

void TestMsgProcessor(uint8_t* buf, size_t cnt)
{
	memset(testBuf, 0, ARRAY_SIZE(testBuf) );
	memcpy(testBuf, buf, MIN(cnt,MAX_TEST_BUF));
	printf("IMU message: %d chars [%s]\n", cnt, testBuf );
	tbIndex =0;
}

char* test_imu_loopback( _ports_t port )
{
	snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU LOOPBACK TEST\nBidge TX+ to RX+ and TX- to RX-\n"
						"Send strings over this terminal, or 'X' to end\n");

	Init_Uart_IMU( imu_bauds );

	Set_IMU_COM_Target(Target_PSOC);
	old_isr_level = Reconfig_Uart_IMU(1, false); // One interrupt per 2 characterS
	old_processor = SetImuMsgProcessor(TestMsgProcessor );
	Enable_Uart_IMU();

	GrabDisconnect = DisconnectLoopback;
	grab_input = GrabConsoleInput;

	return CmdAnswer;
}

/*------------------------------------------------------------------------------------- IMUREDIR */
/* IMUREDIR - Redirects the raw IMU output to a port for diagnostics
 *
 * Command format:
 *
 * 	IMUREDIR [port]
 *
 *----------------------------------------------------*/
_ports_t redir_target;

void ImuRedirMsgProcessor(uint8_t* buf, size_t cnt)
{
	char str[100];

	SendToPort( redir_target,buf, cnt);
	GetUartErrorStr(str,100, Uart_IMU_Error);

	if ( Uart_IMU_Error != 0)
	{
		printf("IMU message: %d bytes, UART Error: %s\n", cnt, str );
		Uart_IMU_Error = 0;
	}
	tbIndex =0;
}

/*----------------------------------------------------*/

char *imuredir_cmd(char**tokens, int cnt, _ports_t port)	// debug command
{
	redir_target = port;		// default to calling port;

	if (cnt > 1)
	{
		_ports_t _p = FindPortID(tokens[1]);
		if (_p != INVALID_PORT)
		{
			redir_target = _p;
		}
	}
	snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU REDIR TO PORT\n"
						"Send commands over this terminal, or 'X' to end\n");

	Set_IMU_COM_Target(Target_PSOC);

	if (SysConfig.imu_type == IMUType_FSAS )
	{
		old_isr_level = Reconfig_Uart_IMU(FSAS_SN_RECORD_SIZE-1, false);
	}
	else if (SysConfig.imu_type == IMUType_STIM300 )
	{
		old_isr_level = Reconfig_Uart_IMU(STIM_RECORD_SIZE-1, false);
	}
	old_processor = SetImuMsgProcessor(ImuRedirMsgProcessor );
	GrabDisconnect = DisconnectLoopback;
	grab_input = GrabConsoleInput;

	Enable_Uart_IMU();

	return CmdAnswer;
}


/*----------------------------------------------------------------------- IMUBAUDS */
bool _cycfg_Uart_IMU_clock(uint32_t div, uint32_t frac );

typedef struct
{
	bauds_t clk;
	char* name;

}_t_baud_name;

_t_baud_name baud_names[] =
{
	{ 0, "CLOCK_DIV CLOCK_FRAC" },
	{ B115200, "115200"},
	{ B230400, "230400"},
	{ B460800, "460800"},
	{ B921600, "921600"},
	{ 5, "BYTE_COUNT_PULSE"},
};

char *imubauds_cmd(char** tokens, int cnt, _ports_t port)	// debug command
{
	if (cnt == 1)
	{
		for (int i = 0; i < ARRAY_SIZE(baud_names); i++ )
			printf("IMUBAUDS %d - > %s\n", i, baud_names[i].name );

		return "OK";
	}

	int32_t ndx = 0;

	ToInt32(tokens[1], &ndx);

	if (ndx == 0 && cnt == 4)
	{
		uint32_t div;
		uint32_t frac;

		ToUint32(tokens[2], &div);
		ToUint32(tokens[3], &frac);

		if ( _cycfg_Uart_IMU_clock(div, frac) == false )
		{
			snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU clock adjusted %ld / %ld\n", div, frac );
		}
		else
		{
			snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU clock adjust FAILED\n" );
		}
	}
	else if (ndx > 0 && ndx < 5 && cnt == 3)
	{
		imu_bauds = baud_names[ndx].clk;

		Disable_Uart_IMU();
		Set_Uart_IMU_Baudrate( imu_bauds);
		Enable_Uart_IMU();

		snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU Bauds = %s\n", baud_names[ndx].name );
	}
	else if (ndx == 5 && cnt == 3 )
	{
		uint32_t bytes;
		ToUint32(tokens[2], &bytes);
		uint32_t compare = UART_IMU_config.oversample * 10 * bytes; // total clock pulses for # of bytes

		Cy_TCPWM_PWM_SetCompare0Val(BAUDSMON_HW, BAUDSMON_NUM, compare);

		uint32_t freq = Cy_SysClk_PeriphGetFrequency(CLK2X16_IMU_UART_HW, CLK2X16_IMU_UART_HW);

		InitBaudsMonitor();

		 if (freq != 0)
		 {
			 double duration = (1000.0 * compare)/(double)freq;
			 snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "BAUDSMON (%ld clocks @ %ld KHz) will generate a %.2lf ms pulse of %ld bytes on TP6\n",
					 compare, freq, duration, bytes );
		 }
		 else
		 {
			 snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "Something went wrong, CLK2X16_IMU_UART reported 0 Hz!\n");
		 }
	}
	else
		snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "Incorrect # of arguments %d\n", cnt-1 );

	return CmdAnswer;
}

char *imu_record_size_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt >= 1)
	{
		uint32_t bytes;


		if (ToUint32(tokens[1], &bytes) == 1 )
		{
			Cy_SCB_SetRxFifoLevel(UART_IMU_HW, bytes );
			snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "Record size changed to %lu\n", bytes );
		}
		return CmdAnswer;
	}
	else
	{
		return "Format: IMURECSIZE record_size (>= 0)\n";
	}
}

char *imu_console_cmd(char**tokens, int cnt, _ports_t port)
{
	if (SpanStatus.ImuType == IMUType_KVH)
	{
		KVH_EnterConfigMode(port);

		GrabDisconnect = KVH_ExitConfigMode;
		grab_input = GrabConsoleInput;

		return "KVH Console. Send commands to KVH or 'X' to end.\n";
	}
	else
	{
		return "IMU Console only available for KVH\n";
	}
}

/*====================================================================== STIM300 TEST */

char *stim_status(char** tokens, int cnt, _ports_t port)
{
	GetStimStatus(CmdAnswer,  ARRAY_SIZE(CmdAnswer));
	return CmdAnswer;
}

char *GrabInput2(uint8_t* buf, _ports_t port)
{
	if (buf[0] == 'x' || buf[0] == 'X')
	{
		grab_input = NULL;
		Stim_Stop_Logging();
		Init_IMU_Interface(SysConfig.imu_type, SysConfig.imu_connect);

		return "End of Test.\n";
	}
	return "";
}

char* test_log_imu(char**tokens, int cnt, _ports_t port )
{
	Init_IMU_Interface(IMUType_STIM300, Target_PSOC);
	grab_input = GrabInput2;

	if(cnt < 3)
		return "Missing port ID to forward.\n";

	return test_forward_imu( tokens[2] );

}


char* test_forward_imu( char* port )
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	grab_input = GrabInput2;

	_ports_t log_port = FindPortID(port);

	if (log_port == INVALID_PORT )
	{
		snprintf(buf, size, "STIM300 FORWARD: %s Not a valid port (COM1,USB1,USB2,CONSOLE,UART_J6)\n",port );
		return buf;
	}

	Stim_Log_To_Port(log_port);

	snprintf(buf, size, "STIM300 LOG to PORT %s STARTED - X to end.\n", port );

	return (buf);
}

/*------------------------------------------------------------------------
 * Find error in IMU records pop during LOG command
 */

extern RDBUF_HANDLE HandleImuRecords;

char* test_imu_pop( _ports_t port )
{
	printf("Reading back Queued IMU records.\n");

	if (HandleImuRecords == NULL)
	{
		return "ERROR: HandleImuRecords == NULL\n";
	}

	int records = FlexRingBufferItems( HandleImuRecords );
	uint16_t cnt = 0;

	printf("HandleImuRecords Record count: %d\n", records);

	uint8_t  *pdn=0;

	while ( records != 0 )
	{
		SET_DEBUG_TP(TP6);
		cnt++;

		printf("Reading record %d of %d ...\n", cnt, records);

		ring_data_t R;
		PopFromFlexRingBuffer(HandleImuRecords, &R);

		uint16_t size = R.Size;
		uint8_t  *pd = R.Data;
		pdn = pd + size;

		CLEAR_DEBUG_TP(TP6);

		char * roll = (pd != pdn)? "OK" : "Roll";

		printf("\tRecord size: %d. %hhn Next: %hhn - %s\n", size, pd, pdn, roll);

		records = FlexRingBufferItems( HandleImuRecords );
	}
	return "---< DONE >----\n";
}

/*-------------------------------------------------------------
 *  Test IMU Init Bit line
 */
bool initBitInitialized = false;
bool InitBitEnabled = false;

char* test_init_bit( _ports_t port )
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	if (initBitInitialized == false)
	{
		Init_IMU_INIT_BIT_PWM();
		initBitInitialized = true;
		InitBitEnabled = true;
	}
	InitBitEnabled = !InitBitEnabled;

	Enable_IMU_INIT_BIT_PWM(InitBitEnabled);

	snprintf(buf, size, "INIT BIT is %s.\n", InitBitEnabled?"ENABLED":"DISABLED" );

	return (buf);
}

/*====================================================================== BYTESWAP TEST */

char *GrabInput3(uint8_t* buf, _ports_t port)
{
	char *ans = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);


	if (buf[0] == 'x' || buf[0] == 'X')
	{
		grab_input = NULL;
		SysConfig.imu_connect = Target_NovAtel;

		return "End of Test.\n";
	}

	uint32_t val, c3;

	sscanf((char*)buf,"%lu", &val);
	c3 = _bswap32(val);

	snprintf(ans, size, "Received  \t %.8lX\n__bswap32()\t = %.8lX\n",val,c3);

	return ans;
}

char* test_bswap( _ports_t port )
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	snprintf(buf, size,"Byte swap test. Enter a 32 bit number - X or x to end\n");

	grab_input = GrabInput3;

	return buf;
}

/*----------------------------------------------------------------------------- GENOUTPUT */
/* GenerateS a random string with a CRC at the end "RANDOMCHARS[CRC]\n"
 */
char* genoutput_cmd(char** tokens, int cnt, _ports_t port)
{
	_ports_t target = port;
	int32_t nch = 25;

	if(cnt < 2)
		return "Usage: GENOUTPUT nchars [port]\n";

	/* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _p = FindPortID(tokens[i]);

		if (_p != INVALID_PORT)
		{
			target = _p;
			continue;
		}
		else
		{
			if ( ToInt32(tokens[i], &nch) == 0 )
				return  "bad parameter.\n";
		}
	}

	/* generate a buffer random chars and CRC */
	uint8_t* pbuf = malloc(nch+1);

	for (int i = 0; i < nch; i++)
	{
		pbuf[i] = '@' + rand() % 26;
	}
	pbuf[nch] = '\n';

	uint16_t crc = crc16_ccitt(pbuf,nch);
	SendToPort(target, pbuf,nch);
	int n = snprintf(CmdBuffer,sizeof(CmdBuffer),"[%u]\n", crc );
	SendToPort(target, (uint8_t*)CmdBuffer,(size_t)n);

	return  PrintCmdOK();
}

/*============================================================================== ECHO2 */

char *echo2_cmd(char**tokens, int cnt, _ports_t port )
{
	if (cnt < 2 )
		return "Usage: ECHO2 string_to_echo\n";

	SendToPort(port, (uint8_t*)tokens[1], strlen(tokens[1]));
	return  PrintCmdOK();
}

/*** EOF ***/

