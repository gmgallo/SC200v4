/*
 * rtclock.h
 *
 *  Created on: Sep 11, 2021
 *      Author: Guillermo
 */

#ifndef RTCLOCK_H_
#define RTCLOCK_H_

#include "globaldefs.h"

extern bool RTC_Initialized;

uint8_t Init_RTC( cy_stc_rtc_config_t const * config);
uint16_t GetDateTimeString(uint8_t *buf, size_t size);

#endif /* RTCLOCK_H_ */
