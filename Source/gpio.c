/*
 * gpio.c
 *
 *  Created on: Apr 19, 2025
 *      Author: Guillermo
 */
#include "common.h"
#include "gpio.h"

/****************************************************************************
* J6 Connector Input pin change detect
*
* Programs a Pin from J6 as input and enables its Change Detect ISR
* Valid Pins are J6.2 to J6.5
*****************************************************************************/


typedef struct _pin_id
{
	const char*  name;
	cyhal_gpio_t pin;

} pin_id_t;

const pin_id_t PinNameDictionary[] =
{
		{"J6.2", J6_2 },				// IN Used in Digital IO interface add-on board
		{"J6.3", J6_3 },				// OUT Used in Digital IO interface add-on board
		{"J6.4", J6_4 },				// OUT
		{"J6.5", J6_5 },				// IN
};


typedef struct _fcb_args
{

	pin_id_t pin_id;
	pincb_f fcb;

} fcb_args_t;

fcb_args_t fcbArray[4] =
{
		{{NULL,INVALID_PIN_ID},NULL},
		{{NULL,INVALID_PIN_ID},NULL},
		{{NULL,INVALID_PIN_ID},NULL},
		{{NULL,INVALID_PIN_ID},NULL},
};

void j6_isr_cb(void *arg, cyhal_gpio_event_t event)
{
	fcb_args_t *pcbargs = (fcb_args_t*)arg;

	bool level = (event & CYHAL_GPIO_IRQ_RISE)? true: false;

    printf("PIN %s Event %s\n", pcbargs->pin_id.name, (level? "HIGH":"LOW"));

    pcbargs->fcb((char*)pcbargs->pin_id.name , level); // Call user function
}


cyhal_gpio_callback_data_t  j6_cb_data =
{
		j6_isr_cb,
		NULL, NULL, 0,
};

bool Enable_Input_Pin_Event(j6pin_t _pin, pincb_f fn )
{
	int i = 0;
	for( ; i < ARRAY_SIZE(fcbArray); i++)
	{
		if (fcbArray[i].pin_id.pin == INVALID_PIN_ID ) // find first empty slot
			break;
	}
	if (i == ARRAY_SIZE(fcbArray) )
		return false;

	fcbArray[i].pin_id.pin = (cyhal_gpio_t)_pin;
	fcbArray[i].pin_id.name = find_pin_name(_pin);
	fcbArray[i].fcb = fn;

	j6_cb_data.callback_arg = fcbArray + i;
	Init_Input_Pin(_pin, &j6_cb_data );
	return true;
}

void Disable_Input_Pin_Event(j6pin_t _pin)
{
	for( int i = 0 ; i < ARRAY_SIZE(fcbArray); i++)
	{
		if (fcbArray[i].pin_id.pin == (cyhal_gpio_t)_pin ) // find in the list
		{
			cyhal_gpio_register_callback(_pin, NULL);
			fcbArray[i].pin_id.pin = INVALID_PIN_ID;
		}
	}
}



cyhal_gpio_t find_pin_id(const char *pin_name)
{
	for (int i = 0 ; i < ARRAY_SIZE(PinNameDictionary); i++)
	{
		if (strcmp(pin_name, PinNameDictionary[i].name ) == 0)
			return PinNameDictionary[i].pin;
	}
	return INVALID_PIN_ID;
}

int find_pin_index(const char *pin_name)
{
	for (int i = 0 ; i < ARRAY_SIZE(PinNameDictionary); i++)
	{
		if (strcmp(pin_name, PinNameDictionary[i].name ) == 0)
			return i;
	}
	return INVALID_PIN_ID;
}


const char* find_pin_name(cyhal_gpio_t id)
{
	for (int i = 0 ; i < ARRAY_SIZE(PinNameDictionary); i++)
		{
			if ( PinNameDictionary[i].pin == id )
				return PinNameDictionary[i].name;
		}
		return "Jx.?";
}


/*------------------------------------------------------------------------------------
 *  Init_Input_Pin()
 *  Programs a pin as Input and optionally enables its
 *  change detect ISR if the second argument is not NULL
 */

void Init_Input_Pin(cyhal_gpio_t _pin_, cyhal_gpio_callback_data_t *pcbdata)
{
	/* Initialize the State Change Detect*/
	cyhal_gpio_init(_pin_, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);

	if (pcbdata != NULL )
	{
		cyhal_gpio_enable_event(_pin_, CYHAL_GPIO_IRQ_BOTH, CYHAL_ISR_PRIORITY_DEFAULT, true);
		cyhal_gpio_register_callback(_pin_, pcbdata);
	}
}


