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
#include <stdint.h>
#include <stdbool.h>

#define SC_RESULT_OK (0)

/*-------------------------------- Common defs and inlines */
#define IS_LINE_TERMINATOR(c)	((c) == '\r' || (c) == '\n')

#define MAX(a,b)	(((a)>(b))?(a):(b))
#define MIN(a,b)	(((a)<(b))?(a):(b))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)	 (sizeof(a)/sizeof(a[0]))
#endif

typedef struct _tag_keyword
{
	const char* name;
	int			key;
} tkeywrd_t;

#define DICTIONARY_SIZE(dict) (sizeof(dict)/sizeof(dict[0]))

/*----------------------------------------------------------------------------- numeric converters */
inline bool isdec(char c)	{ return (c >='0' && c <= '9'); }
inline bool ishex(char c) 	{ return (isdec(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')); }
inline int dec2bin(char c)  { return c - '0'; }
inline int hex2bin(char c)  { return ((c >= 'a') ? c - 'W' : ((c >= 'A') ? c - '7' : c - '0')); }

#ifdef __GNUC__
	#if __GNUC_PREREQ (4, 2)
	static __inline unsigned short _bswap16(unsigned short __bsx) { return __builtin_bswap16 (__bsx);  }
	static __inline unsigned int   _bswap32(unsigned int __bsx)   { return __builtin_bswap32 (__bsx);  }
	#else
	static __inline uint16_t ToEndian16(uint16_t a) { return (a >> 8) | (a << 8); }
	static __inline uint32_t ToEndian32(uint32_t a) { return (a >> 24) | ((a & 0xff0000) >> 8) | ((a & 0xff00)<<8) | ((a & 0xff)<<24); }
	#endif
#endif

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

/*----------------------------------------- serial ports */
typedef enum _Ports 	// port ID bit fields
{
	INVALID_PORT = 0X00,
	UART_COM1    = 0x01,
	USB_COM1 	 = 0x02,
	USB_COM2	 = 0x04,
	UART_OEM7700 = 0x08,
	UART_CONSOLE = 0x10,
	UART_IMU	 = 0x20,
	UART_J6		 = 0x40,

}_ports_t;

typedef enum // Predefined UART baud rates
{
	B115200 = 0,
	B230400 = 1,
	B460800 = 2,
	B921600 = 3,
} bauds_t;

/*------------------------------------- imu types */
typedef enum
{
	IMUType_INVALID = 0,
	IMUType_FSAS 	= 1,
	IMUType_STIM300 = 2,
	IMUType_KVH		= 3,

} imu_type_t;

typedef enum
{
	Target_INVALID  = 0,
	Target_PSOC 	= 1,
	Target_NovAtel 	= 2,

} imu_target_t;

typedef enum
{
	INVALID_FORMAT = 0,
	fmtFSAS_NATIVE = 1,
	fmtNOVATEL_RAW = 2,		// == RAWIMUSX Default logging format for Inertial Explorer processing
	fmtNOVATEL_IMR = 3,		// Novatel neutral format
	fmt_STIM300	   = 4,
	fmt_KVH_NATIVE = 5,

} imu_format_t;				// This is the IMU format to report to the logger app

/*------------------------------ smoth filters defaults */
#define DEFAULT_VELOCITY_SMOOTH_FACTOR  0.25
#define DEFAULT_ACCEL_SMOOTH_FACTOR  	0.25
#define DEFAULT_SPEED_CUTOFF         	0.1		/*m/s*/

#endif /* GLOBALDEFS_H_ */
