/*
 * ST7920.h
 *
 *  Created on: 17 ���. 2019 �.
 *      Author: Z13
 */

#ifndef ST7920_H_
#define ST7920_H_

#include "stm32f0xx_hal.h"

/**
  * @brief  GPIO Bit SET and Bit RESET enumeration
  */
typedef enum
{
  CMD = 0U,
  DATA
}St7920_DataType;

// ����� ������
#define ST7920_Cmd_FunctionSet              	0x20    // ����� �������� ����������, ���������� Extended Mode � Graphic Mode
// ����� ������ Basic
#define ST7920_CmdBasic_Clear                   0x01   // // ������� DDRAM							//1.6 ms
#define ST7920_CmdBasic_Home                    0x02   // 0.072 ms // ������� ������� � ������ �������			// 0.072 ms
#define ST7920_CmdBasic_EntryMode               0x04   // 0.072 ms // ��������� ���������� ������� � ������	// 0.072 ms
#define ST7920_CmdBasic_DisplayOnOff            0x08   // 0.072 ms // ���������� �������� � ��������			// 0.072 ms
#define ST7920_CmdBasic_CursorDisplayControl    0x10   // 0.072 ms //

#define ST7920_CmdBasic_SetCGRAMaddr            0x40   // 0.072 ms // ��������� ������ � CGRAM
#define ST7920_CmdBasic_SetDDRAMaddr            0x80   // 0.072 ms // ��������� ������ � DDRAM
// ����� ������ Extended
#define ST7920_CmdExt_StandBy                   0x01    // ������� � ����� StandBy
#define ST7920_CmdExt_SelScrollOrRamAddr        0x02    // ����� ������ ���� ������ � ������
#define ST7920_CmdExt_Reverse                   0x04    // ������ ����� �� 4 ����� � DDRAM

#define ST7920_CmdExt_SetIRAMOrSccrollAddr      0x40    // ��������� ������ � IRAM ��� ����� �������
#define ST7920_CmdExt_SetGDRAMAddr              0x80    // ��������� ������ � GDRAM (������ ������������ ������)
//==============================================================================


void spi_lcd_send(uint8_t Data, uint8_t RS);
void St7920_SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void ST7920_Basic_SetCursorPosition(uint8_t Line, uint8_t Position);
void ST7920_Write_Str (uint8_t *pData, uint16_t Size);
void intToASCI(uint32_t data, uint8_t FixedLength);

#endif /* ST7920_H_ */
