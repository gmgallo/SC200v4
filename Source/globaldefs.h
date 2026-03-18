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

/* ----------------------------------------------- Defines moved to GPIO.H
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

#endif  _GPIO_CONTROL_ */

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


/*------------------------------------ defined in cmddprocessor.c */

// int find_KeywordConstant ( const tkeywrd_t* , size_t , const char* );
// const char* find_KeywordName ( const tkeywrd_t*, size_t, const int );


#endif /* GLOBALDEFS_H_ */
