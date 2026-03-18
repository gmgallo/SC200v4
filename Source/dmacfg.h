/*
 * dmacfg.h
 *
 *  Created on: Jul 18, 2022
 *      Author: Guillermo
 */

#ifndef SOURCE_DMACFG_H_
#define SOURCE_DMACFG_H_


/*******************************************************************************
*         DMA config structures
*******************************************************************************/
typedef struct
{
   /* descriptor 0 used for RX and TX */
	cy_stc_dma_descriptor_t 				*pdescriptor;
	cy_stc_dma_descriptor_config_t const	*pdesc_cfg;
	uint32_t 								*pdesc_src_addr;
	uint32_t 					   			*pdesc_dest_addr;

} dma_desc_init_t;


typedef struct
{
	DW_Type       							*base;
	uint32_t 								channel;
    cy_stc_dma_channel_config_t const 		*channelConfig;
    IRQn_Type								irq_type;
    uint32_t        						intr_prioroty;
    cy_israddress							isr_address;		/* your ISR function */

    uint32_t								descriptor_count;
    dma_desc_init_t							*descriptor_list[];

} dma_channel_init_t;



typedef struct
{
	CySCB_Type 						*base;
	cy_stc_scb_uart_config_t const 	*config;
	cy_stc_scb_uart_context_t 		*context;
	IRQn_Type						irq_type;
	uint32_t        				intr_prioroty;
	cy_israddress					isr_address;

	dma_channel_init_t				*rx_dma_channel;
	dma_channel_init_t				*tx_dma_channel;

} uart_dma_config_t;




/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
cy_en_dma_status_t configure_dma_channel(dma_channel_init_t * cfg);  /* should return CY_DMA_SUCCESS */

int Configure_UART_With_DMA(uart_dma_config_t *pcfg);			 /* should return CY_SCB_UART_SUCCESS */



#endif /* SOURCE_DMACFG_H_ */
