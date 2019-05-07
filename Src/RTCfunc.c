/*
 * RTCfunc.c
 *
 *  Created on: 9 Apr 2019
 *      Author: Z13
 */

#ifndef RTCFUNC_C_
#define RTCFUNC_C_

#include "RTCfunc.h"

uint16_t TimePointCompare(RTC_TimePointTypeDef *point0, RTC_TimePointTypeDef *point1){
	int16_t temp=0;
	if (point0->sDate.Date == point1->sDate.Date){
		temp = ((((point1->sTime.Hours - point0->sTime.Hours)*60 \
				+(point1->sTime.Minutes - point0->sTime.Minutes))*60 \
						+(point1->sTime.Seconds - point0->sTime.Seconds)));
	} else{
		temp= ((((24 + point1->sTime.Hours - point0->sTime.Hours)*60				//min
					+(point1->sTime.Minutes - point0->sTime.Minutes))*60			//sec
							+(point1->sTime.Seconds - point0->sTime.Seconds)));		//sec
	}
	if(temp <0){
		//error???
	}
	return temp;
}

#endif /* RTCFUNC_C_ */
