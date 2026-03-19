/*
 * cmdprocessor.h
 *
 *  Created on: Jun 23, 2022
 *      Author: Guillermo
 */

#ifndef CMDPROCESSOR_H_
#define CMDPROCESSOR_H_

#include "common.h"

/************************************************************* PORTS */

void SendToPort(_ports_t port, uint8_t* buffer, size_t count);
void SendStringToPort(_ports_t port, char* str);
const char* GetPortName(_ports_t port);
_ports_t    FindPortID( char* token);


/********************************* System Config Keywords */
const char* GetImuTypeName( imu_type_t type);
const char* GetImuConnectName(imu_target_t target);

int ToInt32(char*token, int32_t *pvalue);

extern volatile bool Cmd_Echo_On;

int PrintStatusLong(char* buffer, size_t size);
char* ScanCommandLine(uint8_t *buf, int cnt, _ports_t sender );
char* ProcessCommand(char* buffer, _ports_t sender);

/*------------------------------------keyword search */
int find_KeywordConstant ( const tkeywrd_t* , size_t , const char* );
const char* find_KeywordName ( const tkeywrd_t*, size_t, const int );


#endif /* CMDPROCESSOR_H_ */

