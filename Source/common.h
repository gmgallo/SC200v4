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
#include "globaldefs.h"

/*----------------------------------------------------------------------------- numeric converters */
inline bool isdec(char c)	{ return (c >='0' && c <= '9'); }
inline bool ishex(char c) 	{ return (isdec(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')); }
inline int dec2bin(char c)  { return c - '0'; }
inline int hex2bin(char c)  { return ((c >= 'a') ? c - 'W' : ((c >= 'A') ? c - '7' : c - '0')); }

#ifdef __GNUC__
#if __GNUC_PREREQ (4, 3)
static __inline unsigned short _bswap16(unsigned short __bsx) { return __builtin_bswap16 (__bsx);  }
static __inline unsigned int   _bswap32(unsigned int __bsx)   { return __builtin_bswap32 (__bsx);  }
#elif
inline uint16_t ToEndian16(uint16_t a) { return (a >> 8) | (a << 8); }
inline uint32_t ToEndian32(uint32_t a) { return (a >> 24) | ((a & 0xff0000) >> 8) | ((a & 0xff00)<<8) | ((a & 0xff)<<24); }
#endif

#endif

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

/************************************************************************************* main.c */
void Send_GNSS_Record( void* pdata, size_t data_size);
void Purge_GNSS_Buffer();

//void SendImuReport( dbuf_t* record);
void Send_IMU_Record( void* pdata, size_t data_size);
void PurgeBuffers();

void StoreReportRecord( dbuf_t *buf, _ports_t port);
void PrintWithTime(char *msg);

#endif /* COMMON_H_ */
