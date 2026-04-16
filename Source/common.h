/*----------------------------------------------------------
 * common.h - Include for all C / C++ modules
 *
 * Copyright (C) 2021 - G2 AIRBORNE SYSTEMS
 * All Rights Reserved
 *
 *  Created on: Sep 10, 2021
 *      Author: G. Gallo
 *--------------------------------------------------------*/

#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "eprom.h"

/*--------------------------------- project includes */
#include "globaldefs.h"
#include "cfifo.h"
#include "gpio.h"
#include "nmea.h"
#include "Uarts.h"
#include "Timers.h"
#include "Novatel.h"
#include "imu.h"
#include "imu_stim.h"
#include "imu_kvh.h"
#include "usb_cdc.h"
#include "retarget_console.h"
#include "cmdprocessor.h"

/*------------------------------------------------------------ Dictionary */

typedef struct
{
	const int 	key;
	const char* str;

} dictionary_t;

/*----------------------------------------------------------------------------
 * SearchDictionary()  - in cmdprocessor.c
 * input
 * 		pd - pointer to dictionary
 * 		count - number of entries use ARRAY_SIZE(dictionary)
 * 		key  - search item
 * 	Returns
 * 		string matching key or _default if not found
 *--------------------------------------------------------------------------*/
const char* SearchDictionary(dictionary_t *pd, size_t cnt, const int key, const char *_def);


/*-------------------------------------------------------------- Interrupt configurator for UARTs */
/* WARNING do not use for GPIO  (cy_stc_sysint_t) MUST BE GLOBAL FOR GPIO PINS */

static inline void ConfigureInterrupt(IRQn_Type src, uint32_t priority, cy_israddress isr_address)
{
	const cy_stc_sysint_t intr_config =
	{
		.intrSrc 	  = src,            /* Source of interrupt signal */
		.intrPriority = priority 		/* Interrupt priority */
	};

	Cy_SysInt_Init(&intr_config, isr_address);
	NVIC_ClearPendingIRQ(intr_config.intrSrc);
	NVIC_EnableIRQ(intr_config.intrSrc);
}

static inline void ConfigureInterruptGPIO(const cy_stc_sysint_t* pintr_config, cy_israddress isr_address)
{
	Cy_SysInt_Init(pintr_config, isr_address);
	NVIC_ClearPendingIRQ(pintr_config->intrSrc);
	NVIC_EnableIRQ(pintr_config->intrSrc);
}



/************************************************************************************* main.c */
extern const char VersionString[];
extern const char _consoleHeader[];
extern _ports_t LoggingPort;
extern uint32_t imuFrequency;
extern bool log_gnss_records;

void Send_GNSS_Record( void* pdata, size_t data_size);
void Purge_GNSS_Buffer();

//void SendImuReport( dbuf_t* record);
void Send_IMU_Record( void* pdata, size_t data_size);
void PurgeBuffers();

void StoreReportRecord( dbuf_t *buf, _ports_t port);
void PrintWithTime(const char *msg);

#endif /* COMMON_H_ */
