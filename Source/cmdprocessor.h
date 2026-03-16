/*
 * cmdprocessor.h
 *
 *  Created on: Jun 23, 2022
 *      Author: Guillermo
 */

#ifndef SOURCE_CMDPROCESSOR_H_
#define SOURCE_CMDPROCESSOR_H_

#include "globaldefs.h"

/************************************************************* PORTS */

void SendToPort(_ports_t port, uint8_t* buffer, size_t count);
void SendStringToPort(_ports_t port, char* str);
const char* GetPortName(_ports_t port);
_ports_t    FindPortID( char* token);


/********************************* System Config Keywords */
const char* GetImuTypeName( imu_type_t type);
const char* ImuConnectName(imu_target_t target);

int ToInt32(char*token, int32_t *pvalue);

extern volatile bool Cmd_Echo_On;

int PrintStatusLong(char* buffer, size_t size);
char* ScanCommandLine(uint8_t *buf, int cnt, _ports_t sender );
char* ProcessCommand(char* buffer, _ports_t sender);

#endif /* SOURCE_CMDPROCESSOR_H_ */

