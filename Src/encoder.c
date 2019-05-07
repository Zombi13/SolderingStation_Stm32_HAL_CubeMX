/*
 * encoder.c
 *
 *  Created on: 8 Apr 2019
 *      Author: Z13
 */

#include "encoder.h"

uint16_t encoder(EncoderTypeDef *encoder){
    if (encoder->Previous_Values != encoder->htim->Instance->CNT){
      encoder->Difference = encoder->Previous_Values - encoder->htim->Instance->CNT;            //encoder_difference
      encoder->Previous_Values = encoder->htim->Instance->CNT;

      if(encoder->Difference == encoder->htim->Init.Period)               //processing, cyclic shift of the timer
    	  encoder->Difference =-2;
      if(encoder->Difference == -(encoder->htim->Init.Period) )
    	  encoder->Difference = 2;
    }else encoder->Difference = 0;
    return encoder->Difference;
}


void EncoderButton_Handler(EncoderTypeDef *encoder, RTC_HandleTypeDef *hrtc)
{
	if(encoder->ButtonIsPressed)
	{
		HAL_RTC_GetTime(hrtc, &encoder->TPoint[1].sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(hrtc, &encoder->TPoint[1].sDate, RTC_FORMAT_BCD);
		encoder->ButtonIsPressed = 0;

		if (TimePointCompare(&encoder->TPoint[0], &encoder->TPoint[1]) >= encoder->HoldButtonTime){
			encoder->HoldButtonDetect = 1;
		}
	}
	else
	{
		HAL_RTC_GetTime(hrtc, &encoder->TPoint[0].sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(hrtc, &encoder->TPoint[0].sDate, RTC_FORMAT_BCD);
		encoder->ButtonIsPressed = 1;
	}


//	EncoderButtonPushed = 1;
/*	if(SpecMod){
		SpecMod++;

	}*/
}

/*
void HoldButton_Detect()
{
	uint32_t temp = 0;

	switch (SpecMod){
	case 0:
		if(EncoderButtonPushed)
		{
			if (HAL_GPIO_ReadPin(EncoderButton_Port, EncoderButton_Pin)){
				HAL_RTC_GetTime(&hrtc, &Point[3].sTime, RTC_FORMAT_BCD);
				HAL_RTC_GetDate(&hrtc, &Point[3].sDate, RTC_FORMAT_BCD);

				if (TimePointCompare(&Point[2], &Point[3]) >= 15){
					SpecMod = 1;
					SolderPWM_T.Instance->CCR1 = 0;
					DimmerValue = 0;
					CalibrCount = 0;
				}
			}
		}
	break;
	case 1:
		calibr[0+CalibrCount]=SolderPWM_T.Instance->CCR1;
		calibr[1+CalibrCount]= T_Solder_Set;
		for(uint8_t i1=0;i1<=255;i1++){
			temp += tempArr[i];
		}
		calibr[2+CalibrCount] =temp / 255;
		SolderPWM_T.Instance->CCR1 =100*CalibrCount;
		CalibrCount+=3;
		if(CalibrCount >= 30) SpecMod=0;
	break;
	}
}*/
