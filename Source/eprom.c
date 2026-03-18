/*
 * eprom.c
 *
 *  Created on: Aug 27, 2022
 *      Author: Guillermo
 */

#include "common.h"


#define EPROM_ERROR	  (eepromReturnValue != CY_EM_EEPROM_SUCCESS) // TRUE on Error
#define EPROM_INIT_OK (eepromReturnValue == CY_EM_EEPROM_SUCCESS)

#define FLASH_EPROM_SIZE (CY_EM_EEPROM_GET_PHYSICAL_SIZE(EEPROM_SIZE, CONFIG_MODE, WEAR_LEVELLING_MODE, REDUNDANT_COPY_MODE))


/*******************************************************************************
 * Global variables
 ******************************************************************************/
bool eprom_init = false;

cy_stc_eeprom_context_t Em_EEPROM_context;

/* Return status for EEPROM. */
 cy_en_em_eeprom_status_t eepromReturnValue;

 uint32_t EEPROM_Capacity = EEPROM_SIZE;

#if (FLASH_REGION_TO_USE == AUX_FLASH_REGION_32K)

  CY_SECTION(".cy_em_eeprom")	/* this maps the EPROM to the 32KB Auxiliary flash area */

#endif /* #if(FLASH_REGION_TO_USE) */

 CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)

const uint8_t EepromStorageArea[FLASH_EPROM_SIZE] = {0u};



 /*-------------------------------------------------------------------------
  * Init_Eprom_Storage()
  *-------------------------------------------------------------------------*/
bool Init_Eprom_Storage()
{
	/* EEPROM configuration and context structure. */
	cy_stc_eeprom_config_t Em_EEPROM_config =
	{
		.eepromSize 		= FLASH_EPROM_SIZE,
		.simpleMode			= CONFIG_MODE,
		.blockingWrite 		= BLOCKING_WRITE_MODE,
		.redundantCopy 		= REDUNDANT_COPY_MODE,
		.wearLevelingFactor = WEAR_LEVELLING_MODE,
	};


	/* Initialize the flash start address in EEPROM configuration structure. */
	Em_EEPROM_config.userFlashStartAddr = (uint32_t) EepromStorageArea;

	eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);

	eprom_init = EPROM_INIT_OK;

	return EPROM_ERROR;
}

/*-------------------------------------------------------------------------
 *  Eprom_Read(uint32_t start_addr, void* pdata, uint32_t size)
 *-------------------------------------------------------------------------*/
bool Eprom_Read(uint32_t start_addr, void* pdata, uint32_t size)
{
	eepromReturnValue =	Cy_Em_EEPROM_Read(start_addr, pdata, size, &Em_EEPROM_context);

	return EPROM_ERROR;
}

/*-------------------------------------------------------------------------
 * Eprom_Write(uint32_t start_addr, void* pdata, uint32_t size)
 *-------------------------------------------------------------------------*/
bool Eprom_Write(uint32_t start_addr, void* pdata, uint32_t size)
{
	eepromReturnValue = Cy_Em_EEPROM_Write(start_addr, pdata, size, &Em_EEPROM_context);

	return EPROM_ERROR;
}

/*------------------------------------------------------------------------
 *  Read / Save app configuration
 * -----------------------------------------------------------------------*/
uint16_t crc16wseed(uint8_t* buf, size_t len, unsigned short seed); /* use in IMU.c */

#define SYS_CONFIG_ADDRESS			(0x0000)		// 0 offset from EPROM start addr.
#define SYS_CONFIG_ID				(0xCBD3)
#define SYS_CONFIG_CRC_SEED			(0x3DBC)
#define SYS_CONFIG_CRC_OFFSET		(sizeof(sys_config_t) - sizeof(uint16_t))

#define VAR_EPROM_DATA_ADDRESS		(0x0400)		// 1KB starts general data address

bool ReadConfig(sys_config_t* cfg)
{
	if (eprom_init == false)
	{
		Init_Eprom_Storage();
	}

	if (Eprom_Read(SYS_CONFIG_ADDRESS, cfg, sizeof(sys_config_t)) == false )
	{
		return cfg->Crc != crc16wseed((uint8_t*)cfg, SYS_CONFIG_CRC_OFFSET,SYS_CONFIG_CRC_SEED);
	}
	return true;
}

bool SaveConfig(sys_config_t* cfg)
{
	cfg->ConfigID 	= SYS_CONFIG_ID;
	cfg->Size		= sizeof(sys_config_t);
	cfg->Crc		= crc16wseed((uint8_t*)cfg,SYS_CONFIG_CRC_OFFSET,SYS_CONFIG_CRC_SEED);

	if (eprom_init == false)
	{
		Init_Eprom_Storage();
	}

	return Eprom_Write(SYS_CONFIG_ADDRESS, cfg, sizeof(sys_config_t));
}

