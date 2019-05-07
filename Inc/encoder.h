/*
 * encoder.h
 *
 *  Created on: 8 Apr 2019
 *      Author: Z13
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f0xx_hal.h"
#include "RTCfunc.h"

typedef struct
{
	int16_t 				Previous_Values;
	int16_t					Difference;
	uint8_t	 				ButtonIsPressed;
	uint8_t					HoldButtonTime;
	uint8_t					HoldButtonDetect;
	TIM_HandleTypeDef*		htim;
	RTC_TimePointTypeDef	TPoint[2];
//	RTC_TimeTypeDef 		sTime[2];
//	RTC_DateTypeDef 		sDate[2];
}EncoderTypeDef;

/*
typedef struct
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
}RTC_TimePointTypeDef;
*/


uint16_t encoder(EncoderTypeDef *encoder);
void EncoderButton_Handler(EncoderTypeDef *encoder, RTC_HandleTypeDef *hrtc);

#endif /* ENCODER_H_ */
