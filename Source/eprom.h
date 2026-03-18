/*
 * eprom.h
 *
 *  Created on: Aug 27, 2022
 *      Author: Guillermo
 */

#ifndef EPROM_H_
#define EPROM_H_
#include "cy_em_eeprom.h"

/*
 * PSoC 6 MCU: CY8C62x6,CY8C62x7 Datasheet
 *
 * PSOC6 Flash Data
 *
 * Row size			512 bytes
 * Aux Flash size	32kb
 * Write endurance	100,000 cycles min
 *
 * ARM Mmemory map regioons  (Page 12)
 *
 * 0x0000 0000 – 0x1FFF FFFF		CODE executable ARM code region
 * 0x0800 0000 – 0x0804 7FFF		SRAM for CM4 and CM0+ up to 288KB (INSIDE ARM CODE region)
 * 0x1000 0000 – 0x100F FFFF		Application Flash up to 1MB
 * 0x1400 0000 – 0x1400 7FFF		Auxiliary flash (can be used for EEPROM emulation)
 * 0x1600 0000 – 0x1600 7FFF		Supervisory flash
 * 0x2000 0000 – 0x3FFF FFFF		ARM SRAM region (not used in PSOC6)
 * 0x4000 0000 – 0x5FFF FFFF		PERIPHERALS
 * 0x6000 0000 – 0x9FFF FFFF		EXTERNAL RAM (code can execute here too)
 * 0xE000 0000 – 0xE00F FFFF		Private peripheral BUS
 * 0xE010 0A000 – 0xFFFF FFFF		Device specific registers
 *
 */

/* sFlash USER DATA AREA
 * There are four rows in SFlash that are available for user data –
 * 0x16000800, 0x16000A00, 0x16000C00, and 0x1600E00 (total 512 x 4 = 2048 bytes).
 * The automatic SFlash memory allocation is always shown in project linker files after building, as follows:

 * The following regions define device specific SFLASH memory regions and must not be changed.

	sflash_user_data  (rx)    : ORIGIN = 0x16000800, LENGTH = 0x800      // Supervisory flash: User data 2KB

	sflash_nar        (rx)    : ORIGIN = 0x16001A00, LENGTH = 0x200      // Supervisory flash: Normal Access Restrictions (NAR) 512Bytes

	sflash_public_key (rx)    : ORIGIN = 0x16005A00, LENGTH = 0xC00 	 // Supervisory flash: Public Key

	sflash_toc_2      (rx)    : ORIGIN = 0x16007C00, LENGTH = 0x200      // Supervisory flash: Table of Content # 2

	sflash_rtoc_2     (rx)    : ORIGIN = 0x16007E00, LENGTH = 0x200      // Supervisory flash: Table of Content # 2 Copy

	To place data in the sflash_user_data preceed the declaration with:

	CY_SECTION(".cy_sflash_user_data")
*/

/*******************************************************************************
 * Macros
 *******************************************************************************/
 /* EEPROM Configuration details. All the sizes mentioned are in bytes.
 * For details on how to configure these values refer to cy_em_eeprom.h. The
 * library documentation is provided in Em EEPROM API Reference Manual. The user
 * access it from ModusToolbox IDE Quick Panel > Documentation>
 * Cypress Em_EEPROM middleware API reference manual
 */

#define SFLASH_USER_DATA SIZE	(0x0800) /* 2KB in Supevisory flash (sFlash) for user data */
#define AUX_FLASH_SIZE 			(0x8000) /* 32KB Auxiliary flash data */

#define USER_FLASH_REGION       (0u)	/* Creates EPROM area in user flash (code) region */
#define AUX_FLASH_REGION_32K   	(1u)	/* Creates EPROM area in aux area 32KB above code RECOMENDED */

#define REDUNDANT_COPY          (1u)	/* doubles the flash requirement */
#define NO_REDUNDANT_COPY       (0u)

#define WEAR_LEVELLING_FACTOR   (2u)	/* multiplies the size of flash required */
#define NO_WEAR_LEVELLING_FACTOR (1u)

#define BLOCKING_WRITE          (1u)
#define NO_BLOCKING_WRITE       (0u)

#define CONFIG_SIMPLE_MODE      (1u)	/* Disables all of the above */
#define CONFIG_HIREL_MODE      	(1u)	/* Enable all of the above */

/* -------------------------------------------------------------------------
 * select in the defines below how the EEPROM should be configured
 *  CONFIG_SIMPLE_MODE disables all data protection options
 */
#define REDUNDANT_COPY_MODE		NO_REDUNDANT_COPY
#define WEAR_LEVELLING_MODE		NO_WEAR_LEVELLING_FACTOR
#define BLOCKING_WRITE_MODE		NO_BLOCKING_WRITE
#define CONFIG_MODE				CONFIG_SIMPLE_MODE
#define FLASH_REGION_TO_USE     AUX_FLASH_REGION_32K
#define EEPROM_SIZE				AUX_FLASH_SIZE 			/* 32KB */


extern cy_en_em_eeprom_status_t eepromReturnValue;

extern uint32_t EEPROM_Capacity;

bool Init_Eprom_Storage();

bool Eprom_Read(uint32_t start_addr, void* pdata, uint32_t size);
bool Eprom_Write(uint32_t start_addr, void* pdata, uint32_t size);


#pragma pack(push,1)
typedef struct
{
	uint16_t ConfigID;		// verification
	uint16_t Size;			// Struct size, won't match if fields are added

	//-------------------------- payload
	int8_t soft_reset;			// App reset reset requested
	int8_t no_com2_logs_init;	// Do not init COM2 (NOVATEL) logs (will be set externally)
	int8_t imu_type;			// ID of IMU selected
	int8_t imu_connect;			// IMU COM directed to OEM7700 or PSOC6 (default)
	int8_t enable_ins;			// for receivers with SPAN firmware

	//-------------------------- end payload
	uint16_t Crc;			// Integrity check

} sys_config_t;
#pragma pack(pop)

extern sys_config_t SysConfig;		/* defined in main.c */

bool ReadConfig(sys_config_t* cfg);
bool SaveConfig(sys_config_t* cfg);


#endif /* EPROM_H_ */
