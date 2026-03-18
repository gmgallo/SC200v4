/*
 * rtclock.c
 *
 *  Created on: Sep 11, 2021
 *      Author: Guillermo
 */

#include <common.h>

bool RTC_Initialized = false;

/*
const cy_stc_rtc_config_t RTC_config =
{
	.sec = 0U,
	.min = 0U,
	.hour = 12U,
	.amPm = CY_RTC_AM,
	.hrFormat = CY_RTC_24_HOURS,
	.dayOfWeek = CY_RTC_SUNDAY,
	.date = 1U,
	.month = CY_RTC_JANUARY,
	.year = 0U,
};
*/

uint8_t Init_RTC( cy_stc_rtc_config_t const * config)
{
	cy_en_rtc_status_t result = Cy_RTC_Init(config);

	if (result == CY_RTC_SUCCESS)
		RTC_Initialized = true;

	return (result != CY_RTC_SUCCESS);
}

uint16_t GetDateTimeString(uint8_t *buf, size_t size)
{
	cy_stc_rtc_config_t dateTime;
	uint16_t cnt = 0;

	 Cy_RTC_GetDateAndTime( &dateTime );

	 if (size >= 20 )
	 {
		 cnt = snprintf((char*)buf, size,"20%.2ld-%.2ld-%.2ld %.2ld:%.2ld:%.2ld",
			dateTime.year, dateTime.month,dateTime.date,
			dateTime.hour, dateTime.min, dateTime.sec );
	 }
	 return cnt;
}

