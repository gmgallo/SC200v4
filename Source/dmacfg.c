/*
 * dmacfg.c
 *
 *  Created on: Jul 18, 2022
 *      Author: Guillermo
 */

#include "cyhal.h"
#include "cy_pdl.h"
#include "dmacfg.h"


/*******************************************************************************
* Function Name: configure_dma_interrup() - helper function
********************************************************************************/
static inline void configure_interrupt(IRQn_Type src, uint32_t priority, cy_israddress isr_address)
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


/*******************************************************************************
* Function Name: configure_dma_channel
********************************************************************************
*
* Summary:
* Configures a DMA channel for operation.
*
* Parameters:
* 		dma_channel_init_t 	- structure populated by the caller
*
* Return:
*  		dma_init_status		- = CY_DMA_SUCCESS if no errors detected
*
*******************************************************************************/
cy_en_dma_status_t configure_dma_channel(dma_channel_init_t* cfg)
{
    cy_en_dma_status_t dma_init_status;

    for (int i = 0; i < cfg->descriptor_count; i++ )
    {
    	dma_desc_init_t *pd = cfg->descriptor_list[i];

    	dma_init_status = Cy_DMA_Descriptor_Init(pd->pdescriptor, pd->pdesc_cfg);

		if (dma_init_status!=CY_DMA_SUCCESS)
		{
		   return dma_init_status;
		}

	    /* Set source and destination address for descriptor 0 */
	    Cy_DMA_Descriptor_SetSrcAddress(pd->pdescriptor, pd->pdesc_src_addr );
	    Cy_DMA_Descriptor_SetDstAddress(pd->pdescriptor, pd->pdesc_dest_addr );
    }

    dma_init_status = Cy_DMA_Channel_Init(cfg->base, cfg->channel, cfg->channelConfig);

    if (dma_init_status!=CY_DMA_SUCCESS)
    {
    	return dma_init_status;
    }

    if (cfg->descriptor_count == 1)
    {
    	/* ensure that there is no channel wondering */
    	Cy_DMA_Descriptor_SetNextDescriptor(Cy_DMA_Channel_GetCurrentDescriptor(cfg->base, cfg->channel), NULL);
    }
    else
    {
    	/* Start with descriptor 0 */
    	Cy_DMA_Channel_SetDescriptor(cfg->base, cfg->channel, cfg->descriptor_list[0]->pdescriptor );
    }

    /* Initialize and enable interrupt from RxDma */
    configure_interrupt( cfg->irq_type, cfg->intr_prioroty, cfg->isr_address);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(cfg->base, cfg->channel, CY_DMA_INTR_MASK);

    /* Enable channel and DMA block to start descriptor execution process */
    Cy_DMA_Channel_Enable(cfg->base, cfg->channel );
    Cy_DMA_Enable(cfg->base);

    return dma_init_status;
}


/*******************************************************************************
* Function Name: Configure_UART_With_DMA
********************************************************************************
*
* Summary:
* Configures a UART to operate with DMA.
*
* Parameters:
* 		uart_dma_config_t 	- structure populated by the caller
*
* Return:
*  		cy_en_scb_uart_status_t	- = CY_SCB_UART_SUCCESS if no errors detected
*
*******************************************************************************/
int Configure_UART_With_DMA(uart_dma_config_t *pcfg)
{
	cy_en_dma_status_t dma_status;

	if ( pcfg->rx_dma_channel != NULL)
	{
		dma_status = configure_dma_channel( pcfg->rx_dma_channel );

		if (dma_status  != CY_DMA_SUCCESS )
				return dma_status;
	}

	if (  pcfg->tx_dma_channel != NULL )
	{
		dma_status = configure_dma_channel( pcfg->tx_dma_channel );

		if (dma_status  != CY_DMA_SUCCESS )
				return dma_status;
	}
	cy_en_scb_uart_status_t init_status = Cy_SCB_UART_Init(pcfg->base , pcfg->config , pcfg->context);

	if ( init_status == CY_SCB_UART_SUCCESS )
	{
		 configure_interrupt( pcfg->irq_type, pcfg->intr_prioroty, pcfg->isr_address );
		 Cy_SCB_UART_Enable( pcfg->base );
	}

	return init_status;
}


/*** EOF ***/
