/*
 * cmdprocessor.h
 *
 *  Created on: Jun 23, 2022
 *      Author: Guillermo
 */
 #pragma once
#include "system.h"

/************************************************************* PORTS */
#define SIZEOF_CMD_BUFFER 400
extern char CmdAnswer[SIZEOF_CMD_BUFFER];
extern char CmdBuffer[SIZEOF_CMD_BUFFER];
extern volatile bool Cmd_Echo_On;
extern volatile int cmdIndex;

void SendToPort(_ports_t port, uint8_t* buffer, size_t count);
void SendStringToPort(_ports_t port, char* str);
const char* GetPortName(_ports_t port);
_ports_t    FindPortID( char* token);


/********************************* System Config Keywords */
const char* GetImuTypeName( imu_type_t type);
const char* GetImuConnectName(imu_target_t target);
int PrintStatusLong(char* buffer, size_t size);
char* ScanCommandLine(uint8_t *buf, int cnt, _ports_t sender );
char* ProcessCommand(char* buffer, _ports_t sender);

void ProcessAsyncMessages(); // serviced by the main loop
void PostAssyncMessage(uint8_t *buf, int cnt, _ports_t receiver);

/*------------------------------------keyword search */
int find_KeywordConstant ( const tkeywrd_t* , size_t , const char* );
const char* find_KeywordName ( const tkeywrd_t*, size_t, const int );

char* PrintCmdOK();
char* PrintCmdError(const char* msg);   

/*--------------------------------------------------- Actions */
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

action_t find_Action(const char *action_name);

/*---------------------------------------------------- responses ON / OFF - YES / NO **/
enum
{
	IS_OFF = 0,
	IS_ON  = 1,
	IS_NEITHER = -1,
	NOT_FOUND = -2,
};
int find_binaryParam(char* token);

/*---------------------------------------------------------------- Value Converters */
static inline int ToInt32(const char* token, int32_t *pvalue)
{ return sscanf(token, "%ld", pvalue);}

static inline int ToUint32(const char* token, uint32_t *pvalue)
{ return sscanf(token, "%lu", pvalue); }

static inline float ToFloat(const char* token, float* pvalue)
{ return sscanf(token, "%f", pvalue); }

static inline double ToDouble(const char* token, double* pdvalue)
{ return sscanf(token, "%lf", pdvalue); }

/*---------------------------------------------------------------- Console Ggrab */
typedef char* (*grab_fn)(uint8_t*, _ports_t);
typedef void (*_gab_disconnect)();

void _grab_console(grab_fn grab, _gab_disconnect disconnect);
void _release_console();
