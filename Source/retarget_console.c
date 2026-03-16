/***************************************************************************//**
* \file cy_retarget_console
*
* \brief
* Provides APIs for retargeting stdio to UART hardware
*  modified version of Cypress retarget_io
*
********************************************************************************
* \copyright
* Copyright 2018-2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "retarget_console.h"
#include "cyhal_hw_types.h"
#include "cyhal_uart.h"

 #define CONSOLE_INT_PRIORITY   5

#if defined(__cplusplus)
extern "C" {
#endif

/* UART HAL object used by BSP for Debug UART port */
cyhal_uart_t console_uart_obj;

/* Tracks the previous character sent to output stream */
#ifdef CY_RETARGET_IO_CONVERT_LF_TO_CRLF
static char cy_retarget_io_stdout_prev_char = 0;
#endif /* CY_RETARGET_IO_CONVERT_LF_TO_CRLF */

uint8_t console_getchar(void)
{
    uint8 c = 0;
   	cyhal_uart_getc(&console_uart_obj, &c, 0);
    return c;
}

uint32_t GetConsoleInputBufferCount()
{
	return Cy_SCB_GetNumInRxFifo( console_uart_obj.base );
}

typedef void (*_putc_fn)( uint8_t value);


_putc_fn fn_putc = NULL;

void SetConsoleCopy(_ports_t port)
{
	if (port == UART_COM1)
		fn_putc = Uart_COM_Putc;

	else if (port == USB_COM1)
		fn_putc = USB_COM1_Putc;

	else if (port == USB_COM2)
		fn_putc = USB_COM2_Putc;
	else
		fn_putc = NULL;
}

void console_putchar(char c)
{
    cyhal_uart_putc(&console_uart_obj, c);

    if (fn_putc != NULL)
    	fn_putc(c);
}

     /* Add an explicit reference to the floating point printf library to allow
    the usage of floating point conversion specifier. */
    __asm (".global _printf_float");
    /***************************************************************************
    * Function Name: _write
    ***************************************************************************/
     __attribute__((weak)) int _write (int fd, const char *ptr, int len)
    {
         int nChars = 0;
        (void)fd;
        if (ptr != NULL)
        {
            for (/* Empty */; nChars < len; ++nChars)
            {
            #ifdef CY_RETARGET_IO_CONVERT_LF_TO_CRLF
                if (*ptr == '\n' && cy_retarget_io_stdout_prev_char != '\r')
                {
                    console_putchar('\r');
                }

                cy_retarget_io_stdout_prev_char = *ptr;
            #endif /* CY_RETARGET_IO_CONVERT_LF_TO_CRLF */
                console_putchar((uint32_t)*ptr);
                ++ptr;
            }
        }
        return (nChars);
    }

/* Add an explicit reference to the floating point scanf library to allow
the usage of floating point conversion specifier. */
__asm (".global _scanf_float");
__attribute__((weak)) int _read (int fd, char *ptr, int len)
{
	int nChars = 0;
	(void)fd;
	if (ptr != NULL)
	{
		for(/* Empty */;nChars < len;++ptr)
		{
			*ptr = (char)console_getchar();
			++nChars;
			if((*ptr == '\n') || (*ptr == '\r'))
			{
				break;
			}
		}
	}
	return (nChars);
}

cy_rslt_t retarget_console_init(cyhal_gpio_t tx, cyhal_gpio_t rx)
{
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0,
    };

    cy_rslt_t result = cyhal_uart_init(&console_uart_obj, tx, rx, NC, NC, NULL, &uart_config);

    if (result == CY_RSLT_SUCCESS)
    {
        result = cyhal_uart_set_baud(&console_uart_obj, CONSOLE_BAUDRATE, NULL);

        /*
        cyhal_uart_register_callback(&uart_obj, uart_event_handler, NULL);

        // Enable required UART events
        cyhal_uart_enable_event(&console_uart_obj,
                                (cyhal_uart_event_t)( CYHAL_UART_IRQ_TX_DONE |
                                                     CYHAL_UART_IRQ_RX_DONE),
													 CONSOLE_INT_PRIORITY, true);
		*/
    }

    return result;
}

void retarget_console_deinit()
{
    cyhal_uart_free(&console_uart_obj);
}

#if defined(__cplusplus)
}
#endif
