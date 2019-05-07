/*
 * u8g2_arm.c
 *
 *  Created on: Apr 16, 2019
 *      Author: Z13
 */
//#include <U8g2lib.h>

#include <st7920.h>
#include <u8g2_arm.h>

SPI_HandleTypeDef 	*hspi_u8g2;
GPIO_TypeDef* 		CS_GPIOx;
uint16_t 			CS_GPIO_Pin;

void u8g2_SPI_Set(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	hspi_u8g2		= hspi;
	CS_GPIOx 		= GPIOx;
	CS_GPIO_Pin 	= GPIO_Pin;
}

//uint8_t u8xx_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//  switch(msg)
//  {
//    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
//      break;							// can be used to setup pins
//    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
//      break;
//    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
//      break;
//    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
//      break;
//    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
//      break;
//    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
//      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
//    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
//    //case U8X8_MSG_GPIO_SPI_CLOCK:
//      break;
//    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
//    //case U8X8_MSG_GPIO_SPI_DATA:
//      break;
//    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
//      break;							// arg_int=1: Input dir with pullup high for I2C clock pin
//    case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
//      break;							// arg_int=1: Input dir with pullup high for I2C data pin
//    case U8X8_MSG_GPIO_MENU_SELECT:
//      u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
//      break;
//    case U8X8_MSG_GPIO_MENU_NEXT:
//      u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
//      break;
//    case U8X8_MSG_GPIO_MENU_PREV:
//      u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
//      break;
//    case U8X8_MSG_GPIO_MENU_HOME:
//      u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
//      break;
//    default:
//      u8x8_SetGPIOResult(u8x8, 1);			// default return value
//      break;
//  }
//  return 1;
//}

uint8_t u8xx_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
	  //MicroDelay(10);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    //HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    //HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
    break;
  }
  return 1;
}


uint8_t u8xx_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
	HAL_SPI_Transmit(hspi_u8g2, (uint8_t *) arg_ptr, arg_int, 10);
	 break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);
	break;
  case U8X8_MSG_BYTE_END_TRANSFER:
	  //MicroDelay(100);
	  HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_RESET);
    break;
  default:
    return 0;
  }
  return 1;
}
