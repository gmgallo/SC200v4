/*
 * usb_io.c
 *
 *  Created on: Oct 4, 2021
 *      Author: Guillermo
 *
 *      ## USB Device Specific Instructions

		The user must ensure that the parameters selected in the USB Device personality
		are aligned with the descriptor configuration in the USB Configurator, because
		there is no connection between the USB Device personality in the Device
		Configurator and USB Configurator.

		Specifically, parameter "Endpoints Mask" in the USB personality must be aligned
		with the endpoints selected in the USB Configurator. If DMA Automatic mode is
		selected, parameter  "Endpoint Buffer Size" must be aligned with the total size
		of the endpoint buffers allocated in the USB Configurator.
 *
 */

#include "common.h"

#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"

//#define DEBUG_USB_ISR
//#define	DEBUG_USB_TP	_TP1

/*******************************************************************************
* Macros
********************************************************************************/
#define USBUART_BUFFER_SIZE (64U) // This must match the End Point buffer size
#define USBUART_COM1    (0U)
#define USBUART_COM2    (1U)

bool DTR_Status[2] = { 0, 0 };

uint32_t USB1_LineControl = 0;
uint32_t USB2_LineControl = 0;
bool USBUART1_LC_Changed = false;
bool USBUART2_LC_Changed = false;

bool IsUSB_CDC_PortOpen( uint32_t port )
{
	return DTR_Status[port];
}
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void usb_high_isr(void);
void usb_medium_isr(void);
void usb_low_isr(void);

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_cdc_context_t    usb_cdcContext;


/****************************************************************************
* USB Reconnect ISR
*****************************************************************************/
volatile bool USB_Active = false;
volatile bool USB_Must_Connect = false;


#define RECONNECT_PERIOD_MS 	5000	// try to reconnect every 5 seconds
#define CONNECT_TIMEOUT_MS		5000	// 5 sec wait for CPU enumerate the arrival


volatile GPT_HANDLE _Uthandle = INVALID_GPTIMER_HANDLE;

void reconnect_usb_isr(void *arg)
{
   (void) arg;

   if ( TEST_HAL_PIN( USB_V_SENS) ) // as long as VBUS is present try to reconnect
   {
	   USB_Must_Connect = true;
	   PrintWithTime("Attempt to connect USB\n");
   }
}


/****************************************************************************
* USB VBUS Detect
*****************************************************************************/

void usb_detect_cb(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;

    if(event & CYHAL_GPIO_IRQ_RISE)
    {
    	if ( TEST_HAL_PIN( USB_V_SENS) )
    	{
    		PrintWithTime("+ USB ARRIVAL +\n");
   			USB_Must_Connect = true;
        }
    }
    else
	{
    	PrintWithTime("- USB DISCONNECT - \n");
    	Cy_USB_Dev_Disconnect(&usb_devContext);
		USB_Must_Connect = false;
	}
}

cyhal_gpio_callback_data_t usb_detect_cb_data =
{
		usb_detect_cb,
		NULL, 0, 0,
};

void Init_USB_VBUS_Detect()
{
	/* Initialize the USB Detect Pin P3.3 */
	cyhal_gpio_init(USB_V_SENS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	cyhal_gpio_enable_event(USB_V_SENS, CYHAL_GPIO_IRQ_BOTH, CYHAL_ISR_PRIORITY_DEFAULT, true);
	cyhal_gpio_register_callback(USB_V_SENS, &usb_detect_cb_data);
}


/*******************************************************************************
* Global Variables
********************************************************************************/
/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};


/***************************************************************************
 * USB_CDC_Init()
 ***************************************************************************/
bool USB_CDC_Init()
{
	USB_Active = false;
	USB_Must_Connect = false;

	/* Initialize the USB device */
	Cy_USB_Dev_Init(USBDEV_HW, &USBDEV_config, &usb_drvContext,
					&usb_devices[0], &usb_devConfig, &usb_devContext);

	/* Initialize the CDC Class */
	Cy_USB_Dev_CDC_Init(&usb_cdcConfig, &usb_cdcContext, &usb_devContext);

	/* Initialize the USB interrupts */
	Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
	Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
	Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);

	/* Enable the USB interrupts */
	NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
	NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
	NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

	Init_USB_VBUS_Detect();

	/* if we have VBUS try to connect to the host */

	if ( TEST_HAL_PIN( USB_V_SENS) )
	{
	 /* Make device appear on the bus. This function call is blocking,
	   it waits till the device enumerates up to 15 seconds timeout */
		 if ( Cy_USB_Dev_Connect(true, CONNECT_TIMEOUT_MS, &usb_devContext) == CY_USB_DEV_SUCCESS )
		 {
			USB_Active = true;
		 }
		 else /* connection failed. Start reconnect timer */
		 {
			/* create a periodic timer to reconnect USB when VBUS goes ON (USB wire connect or CPU power ON) */
			/* the ISR will signal to trying to connect periodically until the host enumerates this device */

			_Uthandle = CreateGPTimer(reconnect_usb_isr, NULL, RECONNECT_PERIOD_MS, false);
			StartGPTimer(_Uthandle);
		 }
	}
	return USB_Active;
}


/***************************************************************************
* Function Name: Monitor_USB_CDC_Status()
*
* called from main loop to monitor port connect / disconnect
****************************************************************************/
void Monitor_USB_CDC_Status()
{

	if( USB_Active ) /* we are connected */
	{
		if ( USBUART1_LC_Changed )
		{
			if ( DTR_Status[USBUART_COM1] == true )
			{
				PrintWithTime("USB1 Connected" );
				printf(" @ %ld Bauds\n", Cy_USB_Dev_CDC_GetDTERate(USBUART_COM1,& usb_cdcContext ));
				Cy_SysLib_Delay(50);
				char cmnd[] ="PORTID";
				Send_USB_CDC_String( USBUART_COM1, ProcessCommand(cmnd, USB_COM1) );
			}
			else
				PrintWithTime("USB1 Disconnected\n");

			USBUART1_LC_Changed = false;
		}

		if ( USBUART2_LC_Changed )
		{
			if ( DTR_Status[USBUART_COM2] == true )
			{
				PrintWithTime("USB2 Connected");
				printf(" @ %ld Bauds\n", Cy_USB_Dev_CDC_GetDTERate(USBUART_COM2, & usb_cdcContext ));
				Cy_SysLib_Delay(50);
				Send_USB_CDC_String( USBUART_COM2, ProcessCommand("PORTID", USB_COM2) );
			}
			else
				PrintWithTime("USB2 Disconnected\n");

			USBUART2_LC_Changed = false;
		}
	}
	else if ( USB_Must_Connect )
	{
		if ( Cy_USB_Dev_Connect(true, CONNECT_TIMEOUT_MS, &usb_devContext) == CY_USB_DEV_SUCCESS )
		{
			USB_Active = true;
			USB_Must_Connect = false;

			USBUART1_LC_Changed = false;
			USBUART2_LC_Changed = false;

			if (_Uthandle != INVALID_GPTIMER_HANDLE)
			{
				StopGPTimer(_Uthandle);
				ReleaseGPTimer(_Uthandle);
				_Uthandle = INVALID_GPTIMER_HANDLE;
			}
		 }
		 else /* connection failed. Start reconnect timer */
		 {
			/* create a periodic timer to reconnect USB when VBUS goes ON (USB wire connect or CPU power ON) */
			/* the ISR will signal to trying to connect periodically until the host enumerates this device */
			 if (_Uthandle == INVALID_GPTIMER_HANDLE)
				 _Uthandle = CreateGPTimer(reconnect_usb_isr, NULL, RECONNECT_PERIOD_MS, false);
			 StartGPTimer(_Uthandle);
		 }
	}
}

/***************************************************************************
 * Send_USB_CDC_Data()
 ***************************************************************************/
uint32_t Send_USB_CDC_Data(uint32_t port, uint8_t *buffer, size_t count)
{
	size_t sentcnt = 0;
	size_t sendchunk;

	do {

		sendchunk = count - sentcnt;

		/* limit each send to packets less than IN buffer size */
		if ( sendchunk >= USBUART_BUFFER_SIZE )
			sendchunk = USBUART_BUFFER_SIZE;

		do {
			if (!IsUSB_CDC_PortOpen(port))
				return CY_USB_DEV_DRV_HW_DISABLED;
		} while (0u == Cy_USB_Dev_CDC_IsReady(port, &usb_cdcContext));

		/* Send data back to host */
		Cy_USB_Dev_CDC_PutData(port, buffer + sentcnt, sendchunk, &usb_cdcContext);

		sentcnt += sendchunk;

	} while (sentcnt < count);

	/* Send zero-length packet to PC. */
	if (sendchunk == USBUART_BUFFER_SIZE)
	{
		do {
			if (!IsUSB_CDC_PortOpen(port))
				return CY_USB_DEV_DRV_HW_DISABLED;

		} while (0u == Cy_USB_Dev_CDC_IsReady(port, &usb_cdcContext));


		Cy_USB_Dev_CDC_PutData(port, NULL, 0u, &usb_cdcContext);
	}
    return CY_USB_DEV_SUCCESS;
}

/***************************************************************************
 * Send_USB_CDC_String()
 ***************************************************************************/
uint32_t Send_USB_CDC_String(uint32_t port, char_t const* string)
{
	/* Wait until component is ready to send data to host */
	do {
		if (!IsUSB_CDC_PortOpen(port))
			return CY_USB_DEV_DRV_HW_DISABLED;

	} while (0u == Cy_USB_Dev_CDC_IsReady(port, &usb_cdcContext));

	cy_en_usb_dev_status_t status = Cy_USB_Dev_CDC_PutString(port, string, 100,  &usb_cdcContext);

    /* Send zero-length packet to PC. */
//    Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, NULL, 0u, &usb_cdcContext);

	return status;
}

/***************************************************************************
 * Read_USB_CDC_Data()
 ***************************************************************************/
uint32_t Read_USB_CDC_Data(uint32_t port, uint8_t *buffer, size_t buffer_size)
{
	size_t count = 0;

    /* Check if host sent any data */
    if (Cy_USB_Dev_CDC_IsDataReady(port, &usb_cdcContext))
    {
        /* Get number of bytes */
        count = Cy_USB_Dev_CDC_GetAll(port, buffer, buffer_size, &usb_cdcContext);
    }

    return count;
}

/***************************************************************************
 * USB_CDC_DataReady()
 ***************************************************************************/
bool USB_CDC_DataReady(uint32_t port)
{
	return Cy_USB_Dev_CDC_IsDataReady(port, &usb_cdcContext);
}

/***************************************************************************
 * USB_CDC_Putc()
 ***************************************************************************/
void  USB_CDC_Putc(uint32_t port, uint8_t c)
{
	 Cy_USB_Dev_CDC_PutChar(port, c,  &usb_cdcContext);
}

/***************************************************************************
* Function Name: usb_high_isr
****************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBDEV_HW,
    						Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USBDEV_HW),
                            &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
*  Trigger when there is data transmission
***************************************************************************/
void usb_medium_isr(void)
{
#ifdef DEBUG_USB_ISR
	TOGGLE_DEBUG_TP(DEBUG_USB_TP)
#endif

    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USBDEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
*  Trigger when line control or line encoding status changed.
**************************************************************************/
void usb_low_isr(void)
{
    /* Call interrupt processing */

    Cy_USBFS_Dev_Drv_Interrupt(USBDEV_HW,
    		Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USBDEV_HW), &usb_drvContext);

    /* we are only interested in DTR */
    uint32_t lctl = (Cy_USB_Dev_CDC_GetLineControl(USBUART_COM1, &usb_cdcContext) & CY_USB_DEV_CDC_LINE_CONTROL_DTR);

    if ( lctl != USB1_LineControl )
    {
    	USB1_LineControl = lctl;

    	DTR_Status[USBUART_COM1] = lctl? true : false;

    	USBUART1_LC_Changed = true;
    }

    lctl =  (Cy_USB_Dev_CDC_GetLineControl(USBUART_COM2, &usb_cdcContext) & CY_USB_DEV_CDC_LINE_CONTROL_DTR);

     if ( lctl != USB2_LineControl )
     {
     	USB2_LineControl = lctl;

    	DTR_Status[USBUART_COM2] = lctl? true : false;

     	USBUART2_LC_Changed = true;
     }
}

/*--- EOF ---*/
