/*
 * cmdprocessor.c
 *
 * Console  Command processor.
 *
 *  Created on: Jun 23, 2022
 *      Author: Guillermo
 */
#include "common.h"
#include "tests.h"

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
		while(count-- > 0)
		{
			char c = *buffer++;
			console_putchar(c);
		}
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
char *imu_errors_cmd(char**, int, _ports_t);

char *system_reset_cmd(char**, int, _ports_t);
char *sysconfig_cmd(char**, int, _ports_t);
char *port_id_cmd(char**,int,_ports_t);
char *help_id_cmd(char**,int,_ports_t);
char *copy_console_cmd(char**,int, _ports_t);

char *standby_cmd(char**,int,_ports_t);
char *gnss_log_cmd(char**, int, _ports_t);
char *pos_report_cmd(char**, int, _ports_t);
char *vel_report_cmd(char**, int, _ports_t);
char *delta_dist_report_cmd(char** tokens, int cnt, _ports_t port);
char *marktime_report_cmd(char**, int, _ports_t);
char *markpos_report_cmd(char** tokens, int cnt, _ports_t port);
char *map_ladybug_report_cmd(char** tokens, int cnt, _ports_t port);
char *request_gps_log_cmd(char** tokens, int cnt, _ports_t port);

char *set_pin_cmd(char** tokens, int cnt, _ports_t port);
char *pulse_pin_cmd(char** tokens, int cnt, _ports_t port);
char *read_pin_cmd(char** tokens, int cnt, _ports_t port);
char *monitor_pin_cmd(char** tokens, int cnt, _ports_t port);
char *cancel_pin_monitor_cmd(char** tokens, int cnt, _ports_t port);
char *echo2_cmd(char**, int, _ports_t);

char *imu_log_cmd(char**,int,_ports_t);

char *verbose_cmd(char**,int,_ports_t);

char* release_timer_cmd(char** tokens, int cnt, _ports_t port);
char* create_timer_cmd(char** tokens, int cnt, _ports_t port);
char* clock_monitor_cmd(char** tokens, int cnt, _ports_t port);
char* imuaccel_scale_cmd(char** tokens, int cnt, _ports_t port);
char* imugyro_scale_cmd(char** tokens, int cnt, _ports_t port);

/*--------------------------------------------------------------------------- command dictionary */
typedef struct
{
	const char* 	cmd;
	fn_command_t	fn;

}fn_dictionary_t;

const fn_dictionary_t CmdDictionary[] =
{
		{"LOG", 			log_cmd},
		{"VERSIONINFO", 	version_cmd},
		{"STATUS",			status_cmd},
		{"SHORTSTATUS",		status_cmd},		// sends short status <STATUS,a,b,c,d...x>\r
		{"ECHO",			echo_cmd},
		{"IMUFREQ",			imufreq_cmd},
		{"IMUFORMAT",		imuformat_cmd},
		{"IMUTYPE",			imutype_cmd},
		{"IMUCONNECT",		imuconnect_cmd},
		{"IMUACCELSCALE", 	imuaccel_scale_cmd},
		{"IMUGYROSCALE", 	imugyro_scale_cmd},
		{"SYSTEMRESET", 	system_reset_cmd},
		{"SYSCONFIG", 		sysconfig_cmd},
		{"STANDBY",		standby_cmd},
		{"PORTID",			port_id_cmd},
		{"?",				help_id_cmd},
		//{"COPYCONSOLE",		copy_console_cmd},
		{"POSREPORT", 		pos_report_cmd},		// Send periodic position reports
		{"VELOCITYREPORT", vel_report_cmd},		// Send periodic velocity and acceleration reports
		{"DELTADISTREPORT", delta_dist_report_cmd},// Send periodic delta distance reports
		{"MARKTIMEREPORT", marktime_report_cmd},	// Send  reports
		{"MARKPOSREPORT",  markpos_report_cmd},	// Send  reports
		{"MAPLBREPORT",    map_ladybug_report_cmd},// Send Ladybug GPS logs to PORT#
		{"REQUESTGPSLOG",  request_gps_log_cmd},	// Send Ladybug GPS logs to PORT#

		{"SETPIN",  		set_pin_cmd},			// SET port pin high or low
		{"WRITEPIN",  		set_pin_cmd},			// SET port pin high or low
		{"PULSEPIN",  		pulse_pin_cmd},			// Pulse port pin
		{"READPIN", 		read_pin_cmd},			// read port pin
		{"MONITORPIN", 	monitor_pin_cmd},		// read port pin
		{"CANCELPINMONITOR",cancel_pin_monitor_cmd},

		/* debug commands */
		{"VERBOSE",  	 verbose_cmd},			// Echo all commands and answer to the console.
		{"GENOUTPUT",   genoutput_cmd },		// generates a n output of n bytes with crc16
		{"ECHO2",       echo2_cmd	},			// Echoes back what's received by the port
		{"GNSSLOGS",    gnss_log_cmd}, 		// enables (default) / disables GNSS logs 
		{"IMULOGS", 	 imu_log_cmd},			// enables (default) / disables IMU logs
		{"BENCHMARK",	 benchmark_cmd},  		// executes several benchmarks
		{"TEST",		 test_cmd },				// executes several test commands
		{"CREATETIMER", create_timer_cmd},
		{"RELEASETIMER", release_timer_cmd},
		{"IMUREDIR",    imuredir_cmd},
		{"IMUBAUDS",    imubauds_cmd},
		{"STIMSTATUS",  stim_status},
		{"IMURECSIZE",  imu_record_size_cmd},
		{"IMUCONSOLE",  imu_console_cmd},
		{"IMUERRORS",   imu_errors_cmd},
		{"CLOCKMONITOR", clock_monitor_cmd},
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

/*--------------------------------------------------- Actions */
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
int find_KeywordConstant ( const tkeywrd_t* array, size_t size, const char* word)
{
	for (int i=0; i < size;i++)
	{
		if (strcmp(word, array[i].name) == 0)
			return array[i].key;
	}

	return NOT_FOUND;	// no key found
}


const char* find_KeywordName ( const tkeywrd_t* list, size_t size, const int key)
{
	for(int i = 0; i < size; i++ )
	{
		if( list[i].key == key )
			return list[i].name;
	}
	return "UNKNOWN";
}


/****************************************************************************
* Command decoder
*****************************************************************************/
char CmdAnswer[SIZEOF_CMD_BUFFER];

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
grab_fn _grab_input = NULL;
_gab_disconnect _release_grab = NULL;

void _grab_console(grab_fn grab, _gab_disconnect disconnect)
{
	_grab_input = grab;
	_release_grab = disconnect;
}

void _release_console()
{
	_grab_input = NULL;

	if (_release_grab != NULL)
		_release_grab();
}


char* ProcessCommand(char* buffer, _ports_t sender)
{
	if (_grab_input != NULL)
		return _grab_input((uint8_t*)buffer, sender);

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
volatile int cmdIndex = 0;

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
* Asynchronous messages processing 
*****************************************************************************/
#define MAX_ASYNC_MESSAGES 10
typedef struct
{
	uint32_t size; 
	uint8_t* data; /* pointer to data buffer */
	_ports_t port; /* target port for the message */
	 
}async_message_t;

async_message_t AsyncMessageBuffer[MAX_ASYNC_MESSAGES];
int async_message_index = 0;
int async_message_count = 0;

void ProcessAsyncMessages() // serviced by the main loop
{
	while (async_message_count > 0)
	{
		async_message_t *msg = &AsyncMessageBuffer[ (async_message_index - async_message_count + MAX_ASYNC_MESSAGES) % MAX_ASYNC_MESSAGES ];

		SendToPort(msg->port, msg->data, msg->size);

		async_message_count--;
	}
}

void PostAssyncMessage(uint8_t *buf, int cnt, _ports_t receiver)
{
	if (async_message_count < MAX_ASYNC_MESSAGES)
	{
		AsyncMessageBuffer[async_message_index].data = buf;
		AsyncMessageBuffer[async_message_index].size = cnt;
		AsyncMessageBuffer[async_message_index].port = receiver;

		async_message_index = (async_message_index + 1) % MAX_ASYNC_MESSAGES;
		async_message_count++;
	}

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

char* PrintCmdError( const char *string )
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


/*------------------------------------------------------------------------------------- VELOCITYREPORT */
/* Sends velocity and acceleration reports 
 * 
 * 	VELOCITYREPORT  [port] [action] [frequency]
 *
 * 	Parameters:
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
 * 		frequency	- frequency in Hertz (default 1 Hz) max 20 Hz
 *----------------------------------------------------------------------------------------------------*/
_ports_t velreport_target = INVALID_PORT;
double  velreport_frequency = 1.0; // default 1 Hz
#define MAX_VELOCITY_REPORT_FREQUENCY_HZ 20
char velreport_buf[256];

void velocity_report_callback(pvelocity_rep_t vel)
{
	int cnt = snprintf(velreport_buf, ARRAY_SIZE(velreport_buf), 
	"#VEL,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n", 
	vel->WeekSeconds, 
	vel->HorSpeed, 
	vel->HorAccel, 
	vel->VertSpeed, 
	vel->VertAccel);

	SendToPort(velreport_target, (uint8_t*)velreport_buf, cnt);
}

char *vel_report_cmd(char** tokens, int cnt, _ports_t port)
{
	velreport_target = port;		// default to calling port;
	 action_t action = ACTION_START;

	if ( cnt > 1 ) /* if no arguments use defaults above */
	{
			/* scan arguments */
		for (int i = 1; i < cnt; i++ )
		{
			_ports_t _p = FindPortID(tokens[i]);

			if (_p != INVALID_PORT)
			{
				velreport_target = _p;
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
				if(_v > MAX_VELOCITY_REPORT_FREQUENCY_HZ)
					velreport_frequency = MAX_VELOCITY_REPORT_FREQUENCY_HZ;
				else
					velreport_frequency =  _v;
			}
		}
	}

	if (action == ACTION_STOP)
	{
			printf("{%s} Stopping %s to %s\n", GetPortName(port), tokens[0], GetPortName(velreport_target) );
			CancelVelocityReport();
	}
	else
	{
		RegisterVelocityReport(velreport_frequency, 
			SysConfig.speed_cutoff, 
			SysConfig.vel_smoth_factor, 
			SysConfig.accel_smoth_factor,
			 velocity_report_callback );
	}

	return PrintCmdOK();
}

/*------------------------------------------------------------------------------------- DELTADISTREPORT */
/* Sends delta distance travel reports 
 * 
 * 	DELTADISTREPORT  [port] [action] [distance]
 *
 * 	Parameters:
 * 		port		- destination port. Default sender port
 * 		action		- START or STOP		Default START
 * 		distance	- distance in meters (default 10 meters)
 *----------------------------------------------------------------------------------------------------*/
#define MIN_DELTA_DISTANCE_METERS 1
 _ports_t delta_dist_target = INVALID_PORT;
 int32_t  delta_dist = 10; 				// default 10 meters
double report_frequency = 10.0;			// default 10 Hz
double distance_travelled = 0.0;

char distreport_buf[200];

double DistanceTravelled(double speed, double accel, double period)
{
	// distance = speed * time + 0.5 * accel * time^2
	return (speed + 0.5 * accel * period) * period;
}

double TimeToDistance(double speed, double accel, double distance)
{
	// Solve the quadratic equation: 0.5 * accel * t^2 + speed * t - distance = 0
	double a = 0.5 * accel;
	double b = speed;
	double c = -distance;

	double discriminant = b * b - 4 * a * c;

	if (discriminant < 0)
	{
		// No real solution, return a large time to indicate it's not reachable
		return INFINITY;
	}

	double sqrt_discriminant = sqrt(discriminant);
	double t1 = (-b + sqrt_discriminant) / (2 * a);
	double t2 = (-b - sqrt_discriminant) / (2 * a);

	// Return the positive time value
	return (t1 > 0) ? t1 : t2;
}


void distance_report_callback(pvelocity_rep_t vel)
{
	double period = 1.0 / report_frequency; // period in seconds
	// Update the distance travelled based on the current speed and acceleration
	distance_travelled += DistanceTravelled(vel->HorSpeed, vel->HorAccel, period); // period in seconds

	// Calculate the time required to reach the target distance
	double timetodist =	0;
	
	if (distance_travelled < delta_dist)
		timetodist = TimeToDistance(vel->HorSpeed, vel->HorAccel, delta_dist - distance_travelled);

	if (timetodist < period)
	{	
		distance_travelled = 0; // Already reached the target distance

		int cnt = snprintf(distreport_buf, ARRAY_SIZE(distreport_buf), 
			"#DELTADIST,%.3lf,%.3lf,%.3lf,%.3lf\n", 
			timetodist,			// time to reach target distance before next report
			vel->WeekSeconds, 	// current week seconds
			vel->HorSpeed, 		// current horizontal speed
			vel->HorAccel 		// current horizontal acceleration
			);

		SendToPort(delta_dist_target, (uint8_t*)distreport_buf, cnt);
	}
}

char *delta_dist_report_cmd(char** tokens, int cnt, _ports_t port)
{
	 delta_dist_target = port;		// default to calling port;
	 action_t action = ACTION_START;

	if ( cnt > 1 ) /* if no arguments use defaults above */
	{
			/* scan arguments */
		for (int i = 1; i < cnt; i++ )
		{
			_ports_t _p = FindPortID(tokens[i]);

			if (_p != INVALID_PORT)
			{
				delta_dist_target = _p;
				continue;
			}

			action_t _a = find_Action(tokens[i]);
			if (_a != ACTION_NONE)
			{
				action = _a;
				continue;
			}

			int32_t _d;

			if ( ToInt32(tokens[i], &_d) != 0 )
			{
				if(_d >= MIN_DELTA_DISTANCE_METERS)
					delta_dist = _d;
				else
					delta_dist = MIN_DELTA_DISTANCE_METERS;
			}
		}
	}

	if (action == ACTION_STOP)
	{
			printf("{%s} Stopping %s to %s\n", GetPortName(port), tokens[0], GetPortName(delta_dist_target) );
			CancelVelocityReport();
	}
	else
	{
		RegisterVelocityReport(report_frequency, 
			SysConfig.speed_cutoff, 
			SysConfig.vel_smoth_factor, 
			SysConfig.accel_smoth_factor,
			 distance_report_callback );
	}

return PrintCmdOK();
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
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"IMUFREQ %lf\n", IMU_Frequency);
		return CmdAnswer;
	}
	int32_t freq;

	if ( ToInt32(tokens[1], &freq) == 0 )
	{
		return PrintCmdError("Argument must be 0 to stop TDAS or between 50 and 1000 Hz.");
	}

	if ( freq > 1000  || freq < 0  )
	{
		return "Frequency out of range 0 <= f <= 1000\n";
	}
	if (freq == 0)
	{
		Stop_IMU_Trigger_Frequency();
	}
	else
	{
		Set_IMU_Trigger_Frequency(freq); // extend the period 20Hz to ensure sync with MARK1 time.
	}
	return PrintCmdOK();
}



/*------------------------------------------------------------------------------------- IMUFORMAT */
/* Command format:
 *
 * 	IMUFORMAT format
 *
 * 	Argument:
 * 		format 		FSAS, IMR, STIM, KVH or NRAW (=NovAtel raw startup default)
 *
 * 	Format is saved to config for debug purposes. 
 *  For NON SPAN configurations the format must be converted to NovAtel Raw NRAW  
 *-------------------------------------------------------------------------------------------------*/
char *imuformat_cmd(char**tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		return PrintCmdError( "Usage: IMUFORMAT <format> [SAVE]" );
	}
	int format = find_KeywordConstant( ImuFormatsDictionary, ImuFormatsCount, tokens[1]);

	if ( format == NOT_FOUND )
	{
		return PrintCmdError( "Unknown FORMAT argument.");
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
		SetImuDataFormat((imu_format_t)format, save);

		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU data format changed to %s - %s\n", GetUpTimeStr(), tokens[1], saved	);
	}
	return CmdAnswer;
}



/*------------------------------------------------------------------------------------- IMUTYPE */
/* Command format:
 *
 * 	IMUTYPE type [SAVE]
 *
 * 	Argument:
 * 		Imu types supported	FSAS, STIM, KVH, etc.
 *
 */

char *imutype_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		PrintImuType();
		return "";
	}

	int key = find_KeywordConstant( ImuTypeList, ImuTypeCount, tokens[1]);

	if (key == NOT_FOUND )
	{
		PrintValidImuTypes();
		return PrintCmdError("\nInvalidIMU TYPE argument.\nFormat: IMUTYPE type [SAVE]");
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
 * 	IMUCONNECT target [SAVE]
 *
 * 	Argument:
 * 		target 		PSOC \ OEM7700
 */
char *imuconnect_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"IMU Connected to: %s\nValid targets: PSOC, OEM770\n", find_KeywordName(ImuComTargets, ImuComTargetsCount, SysConfig.imu_connect)  );
		return CmdAnswer;
	}

	int key = find_KeywordConstant( ImuComTargets, ImuComTargetsCount, tokens[1]);

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


/*------------------------------------------------------------------------------------- IMUACCELSCALE */
/* Command format:
 *
 * 	IMUACCELSCALE scale [SAVE]
 *
 * 	Argument:
 * 		scale - float value
 */
char* imuaccel_scale_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"IMU ACCEL Adjustment Scale: %.3f\nValid scales: float value (0.0 = default) [SAVE]\n", SysConfig.imu_accel_scale );
		return CmdAnswer;
	}

	float scale = 0.0;

	if ( sscanf( tokens[1], "%f", &scale ) == 0)
	{
		return PrintCmdError( "INVALID Scale Argument\n" );
	}

	char* saved = "NOT saved.";
	SysConfig.imu_accel_scale = scale;

	if (cnt > 2 && strcmp(tokens[2],"SAVE") == 0)
	{
		SaveConfig(&SysConfig);
		saved = "SAVED.";
	}
	snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU ACCEL SCALE changed to %s - %s\n", GetUpTimeStr(), tokens[1], saved );

	return CmdAnswer;
}

/*------------------------------------------------------------------------------------- IMUACCELSCALE */
/* Command format:
 *
 * 	IMUGYROSCALE scale [SAVE]
 *
 * 	Argument:
 * 		scale - float value
 */
char* imugyro_scale_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt < 2)
	{
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"IMU GYRO Adjustment Scale: %.3f\nValid scales: float value (0.0 = default) [SAVE]\n", SysConfig.imu_gyro_scale );
		return CmdAnswer;
	}

	float scale = 0.0;

	if ( sscanf( tokens[1], "%f", &scale ) == 0)
	{
		return PrintCmdError( "INVALID Scale Argument\n" );
	}
	
	char* saved = "NOT saved.";
	SysConfig.imu_gyro_scale = scale;

	if (cnt > 2 && strcmp(tokens[2],"SAVE") == 0)
	{
		SaveConfig(&SysConfig);
		saved = "SAVED.";
	}
	snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer),"[%s] IMU GYRO SCALE changed to %s - %s\n", GetUpTimeStr(), tokens[1], saved );

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
bool default_IMU_Logging = true;
bool default_GNSS_Logging = true;

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

		Enable_GNSS_Logging = default_GNSS_Logging;
		SendStartLoggingMessages();

		GPTimerDelay(1500);
		Enable_IMU_Logging = default_IMU_Logging;

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

/*------------------------------------------------------------------------------------ GNSSLOGS
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
			default_GNSS_Logging = (action == ACTION_ENABLE);
		//	SaveConfig(&SysConfig);
		}
		else
		{
			return "Usage: GNSSLOGS [ENABLE | DISABLE]\n";
		}
	}

	return (default_GNSS_Logging == true)? "GNSSLOGS are ENABLED\n" : "GNSSLOGS are DISABLED\n";
}


/*---------------------------------------------------------------------------------- IMULOG 
 * Command format:
 *
 *	IMULOG ENABLE | DISABLE
 *
 *	For IMU debug purposes, to log GNSS records alone.
 */
char *imu_log_cmd(char**tokens, int cnt,  _ports_t port)
{
	if (cnt > 1)
	{
		action_t action = find_Action(tokens[1]);

		if (action == ACTION_ON || action == ACTION_OFF)
		{
			default_IMU_Logging = (action == ACTION_ON);
		}
		else
		{
			return "Usage: IMULOGS [ON | OFF]\n";
		}
	}
	return ( default_IMU_Logging == true)? "IMULOGS are ON\n" :  "IMULOGS are OFF\n";
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
		return "Usage: REQUESTGPSLOG <novatel_log_command>\n";

	if ( RequestGpsLog(tokens[1]) )
		return PrintCmdError("Invalid NOVATEL LOG format.\n");

	return PrintCmdOK();
}

/*----------------------------------------------------------------------------------- SYSCONFIG */
/* Prints the config values stored in EPROM
 */
char *sysconfig_cmd(char** tokens, int cnt, _ports_t port)
{
	int count = PrintSysConfig(CmdAnswer, ARRAY_SIZE(CmdAnswer));

	SendToPort( port, (uint8_t*)CmdAnswer, count );
	return PrintCmdOK();
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


/*----------------------------------------------------------------------- IMUBAUDS */
typedef struct
{
	bauds_t clk;
	char* name;

}_t_baud_name;

bauds_t imu_bauds = B115200;

_t_baud_name baud_names[] =
{
	{ 0, "CLOCK_DIV CLOCK_FRAC OVERSAMPLE" },
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

	if (ndx == 0 && cnt == 5)
	{
		int32_t div;
		int32_t frac;
		
		ToInt32(tokens[2], &div);
		ToInt32(tokens[3], &frac);
		int32_t oversample;
		ToInt32(tokens[4], &oversample);

		if ( _cycfg_Uart_IMU_clock(div, frac, oversample) == false )
		{
			snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU clock adjusted %ld / %ld Oversample: %ld\n", div, frac, oversample );
		}
		else
		{
			snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU clock adjust FAILED\n" );
		}
	}
	else if (ndx > 0 && ndx < 5 && cnt == 2)
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

		uint32_t freq = Cy_SysClk_PeriphGetFrequency(CLK_IMU_UART_HW, CLK_IMU_UART_HW);

		InitBaudsMonitor();

		 if (freq != 0)
		 {
			 double duration = (1000.0 * compare)/(double)freq;
			 snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "BAUDSMON (%lu clocks @ %lu KHz) will generate a %.2lf ms pulse of %lu bytes on TP6\n",
			 					 compare, freq/1000, duration, bytes );

			 TriggerBaudsMonitor(compare);
		 }
		 else
		 {
			 snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "Something went wrong, CLK_IMU_UART reported 0 Hz!\n");
		 }
	}
	else
		snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "Incorrect # of arguments %d\n", cnt-1 );

	return CmdAnswer;
}

/*------------------------------------------------------------------------------------- IMURECSIZE */
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

/*------------------------------------------------------------------------------------- IMUCONSOLE */
_ports_t consolePort;
typedef void (*format_ouptut_fn)(const char_t* buf);
format_ouptut_fn _format_output = Uart_IMU_SendString;


char *GrabIMUConsoleInput(uint8_t* buf, _ports_t port)
{
	if (buf[0] == 'x' || buf[0] == 'X') // Test for grab end
	{
		_release_console();
		return "IMU Console disconnected.\n";
	}
	_format_output((char_t*)buf); 	// Send line to IMU port

	return NULL;
}


void LeaveKvhConsole()
{
	if (SpanStatus.ImuType == IMUType_KVH)
	{
		KVH_ExitConfigMode();
	}
}

char *imu_console_cmd(char**tokens, int cnt, _ports_t port)
{
	consolePort = port;
	char msg[] = "Entering IMU Console mode. Type X to exit.\n";

	if (SpanStatus.ImuType == IMUType_KVH)
	{
		SendToPort(port, (uint8_t*)msg, strlen(msg));
		_format_output = KVH_SendConfigCommand;
		_grab_console(GrabIMUConsoleInput, LeaveKvhConsole);
		KVH_EnterConfigMode(port);
		return NULL;
	}
	else
	{
		return "IMU Console not available for this IMU type\n";
	}
}

/*------------------------------------------------------------------------------------- IMUERRORS */
char *imu_errors_cmd(char** tokens, int cnt, _ports_t port)
{
	GetUartErrorStr(CmdAnswer, sizeof(CmdAnswer), Uart_IMU_Error);
	Uart_IMU_Error = 0;
	return CmdAnswer;
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


/*============================================================================== ECHO2 */

char *echo2_cmd(char**tokens, int cnt, _ports_t port )
{
	if (cnt < 2 )
		return "Usage: ECHO2 string_to_echo\n";

	SendToPort(port, (uint8_t*)tokens[1], strlen(tokens[1]));
	return  PrintCmdOK();
}

/*** EOF ***/

