/*----------------------------------------------------------
 * globaldefs.h - Include for all header files
 *
 * Copyright (C) 2022 - G2 AIRBORNE SYSTEMS
 * All Rights Reserved
 *
 *  Created on: June 3, 2022
 *      Author: G. Gallo
 *--------------------------------------------------------*/

#ifndef GLOBALDEFS_H_
#define GLOBALDEFS_H_

#define SC_RESULT_OK	(0)

/*-------------------------------- Common defs and inlines */
#define IS_LINE_TERMINATOR(c)	((c) == '\r' || (c) == '\n')

#define MAX(a,b)	(((a)>(b))?(a):(b))
#define MIN(a,b)	(((a)<(b))?(a):(b))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)	 (sizeof(a)/sizeof(a[0]))
#endif

// ----------------------------------------------- Defines moved to GPIO.H
#ifndef _GPIO_CONTROL_
#define _GPIO_CONTROL_

#define SET_BITS(a,b)	 ((a) |=  (b))
#define CLR_BITS(a,b)	 ((a) &= ~(b))
#define TOGGLE_BITS(a,b) ((a) ^= (b))
#define TEST_BITS(a,b)	 ((a) & (b))


__STATIC_INLINE void SET_GPIO_PIN(cyhal_gpio_t pin )	 { cyhal_gpio_write(pin, true); }
__STATIC_INLINE void CLEAR_GPIO_PIN(cyhal_gpio_t pin )	 { cyhal_gpio_write(pin, false); }
__STATIC_INLINE void TOGGLE_GPIO_PIN(cyhal_gpio_t pin )	 { cyhal_gpio_toggle(pin); }

//#define TOGGLE_GPIO_PIN(a,b) (Cy_GPIO_Inv((GPIO_PRT_Type *)(a),(uint32_t )(b)))
//#define CLEAR_GPIO_PIN(a,b)  (Cy_GPIO_Clr((GPIO_PRT_Type *)(a),(uint32_t )(b)))

#ifndef TEST_PIN
#define TEST_PIN(a,b)	   Cy_GPIO_Read((GPIO_PRT_Type *)(a),(uint32_t )(b))
#endif

#ifndef TEST_HAL_PIN
#define TEST_HAL_PIN(pin)	 (cyhal_gpio_read(pin))		// returns bool
#endif

#endif /* _GPIO_CONTROL_ */

/* debug helpers */
#ifdef DEBUG
#define SET_DEBUG_TP(Pin)		cyhal_gpio_write(Pin, true);
#define CLEAR_DEBUG_TP(Pin)	    cyhal_gpio_write(Pin, false);
#define TOGGLE_DEBUG_TP(Pin)	cyhal_gpio_toggle(Pin);
#else
#define SET_DEBUG_TP(Pin)
#define CLEAR_DEBUG_TP(Pin)
#endif

/*-------------------- pin definintions handled by Device Configurator 
#define USB_V_SENS2		P5_7		IN configured in main()  (in prototyping board only)
#define USB_V_SENS  	P3_3		IN configured in main()
#define SI_TP1			P3_5		OUT
#define SI_TP2			P3_4		OUT
#define SI_TP3			P7_3		OUT
------------------------------------------------------------------------------*/


typedef struct
{
	const char* name;
	int			key;
} keyword_t;


/*------------------------------------ defined in cmddprocessor.c */

int find_KeywordConstant ( const keyword_t* , size_t , const char* );
const char* find_KeywordName ( const keyword_t*, size_t, const int );

/*--------------------------------- local includes */
#include "cfifo.h"
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

/*--------------------------------- main globals */
extern const char VersionString[];
extern const char _consoleHeader[];
extern _ports_t LoggingPort;
extern uint32_t imuFrequency;
extern bool log_gnss_records;

#endif /* GLOBALDEFS_H_ */
