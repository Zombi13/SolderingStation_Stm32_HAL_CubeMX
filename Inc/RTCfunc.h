/*
 * RTCfunc.h
 *
 *  Created on: 9 Apr 2019
 *      Author: Z13
 */

#ifndef RTCFUNC_H_
#define RTCFUNC_H_

#include "stm32f0xx_hal.h"

typedef struct
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
}RTC_TimePointTypeDef;

uint16_t TimePointCompare(RTC_TimePointTypeDef *point0, RTC_TimePointTypeDef *point1);

#endif /* RTCFUNC_H_ */
