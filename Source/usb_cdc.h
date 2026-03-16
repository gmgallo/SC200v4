/*
 * usb_cfg.h
 *
 *  Created on: Oct 6, 2021
 *      Author: Guillermo
 */

#ifndef USB_CDC_H_
#define USB_CDC_H_

#include "globaldefs.h"

#define USBUART_COM1    (0U)
#define USBUART_COM2    (1U)

void   	 Init_USB_VBUS_Detect();
bool     USB_CDC_Init();
uint32_t Send_USB_CDC_Data( uint32_t port,  uint8_t *buffer, size_t count);
uint32_t Send_USB_CDC_String( uint32_t port, const char_t * string);
uint32_t Read_USB_CDC_Data( uint32_t port, uint8_t *buffer, size_t buffer_size);
bool   	 USB_CDC_DataReady( uint32_t port );
void   	 USB_CDC_Putc(uint32_t port, uint8_t c);
bool     IsUSB_CDC_PortOpen(  uint32_t port);

extern bool USBUART1_LC_Changed;
extern bool USBUART2_LC_Changed;

inline bool USBUART_StatusChanged()	{ return (USBUART1_LC_Changed || USBUART2_LC_Changed); }

void 	 Monitor_USB_CDC_Status();


static inline void USB_COM1_Putc(uint8_t c)  { USB_CDC_Putc(USBUART_COM1, c); }
static inline void USB_COM2_Putc(uint8_t c)  { USB_CDC_Putc(USBUART_COM2, c); }

extern volatile bool USB_Active;

#endif /* USB_CDC_H_ */
