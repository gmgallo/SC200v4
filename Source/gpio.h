/*
 * gpio.h
 *
 *  Created on: Apr 19, 2025
 *      Author: Guillermo
 */
#include "common.h"

#ifndef _GPIO_H_
#define _GPIO_H_


// ----------------------------------------------- Defines also in globaldefs.h
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


/*------------------------------------------------------------------------
 *  Connector J6 GPIO
 *------------------------------------------------------------------------*/
#define INVALID_PIN_ID 0

typedef enum j6pins		// Connector to port Pin Mappings
{
	J6_2 = P13_3,		// Default IN
	J6_3 = P13_2,		// Default OUT
	J6_4 = P13_1,		// Default OUT
	J6_5 = P13_0,		// Default IN

} j6pin_t;

int find_pin_index(const char *pin_name);
cyhal_gpio_t find_pin_id(const char *pin_name);
const char* find_pin_name(cyhal_gpio_t id);

/* Pin ISR callback function.
 * pin is the one triggered the ISR, level will show the NEW level (HI/LOW) of the pin.
 */
typedef void (*pincb_f)(char* pin, bool level);

bool Enable_Input_Pin_Event(j6pin_t _pin_, pincb_f fn ); // return true on success
void Disable_Input_Pin_Event(j6pin_t _pin);

void Init_Input_Pin(cyhal_gpio_t _pin_, cyhal_gpio_callback_data_t *pcbdata );


#endif /* _GPIO_H_ */
