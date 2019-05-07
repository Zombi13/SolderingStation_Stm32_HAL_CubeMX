/*
 * u8g2_arm.h
 *
 *  Created on: Apr 16, 2019
 *      Author: Z13
 */

#ifndef U8G2_ARM_H_
#define U8G2_ARM_H_

#include "stm32f0xx_hal.h"
#include <U8g2.h>

void u8g2_SPI_Set(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t u8xx_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
uint8_t u8xx_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif /* U8G2_ARM_H_ */
