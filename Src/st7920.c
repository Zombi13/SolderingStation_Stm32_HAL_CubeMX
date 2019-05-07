/*
 * ST7920.c
 *
 *  Created on: 16 лют. 2019 р.
 *      Author: Z13
 */
#include <st7920.h>

SPI_HandleTypeDef* 	hspi_St7920;
GPIO_TypeDef* 		CS_St7920_GPIOx;
uint16_t 			CS_St7920_GPIO_Pin;


/**
 * @brief  Init st7920 with SPI interface
 *
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  GPIOx where x can be (A..H) to select the GPIO peripheral for STM32F0 family
 * @param  GPIO_Pin specifies the port bit to be written.
 */
void St7920_SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	hspi_St7920			= hspi;
	CS_St7920_GPIOx 	= GPIOx;
	CS_St7920_GPIO_Pin 	= GPIO_Pin;

	HAL_Delay(100);
	spi_lcd_send(0x03, 0);	//b0010_0000
	spi_lcd_send(0x03, 0);	//b0010_0000
	spi_lcd_send(0x0C, 0); //b0000_1111
	spi_lcd_send(0x01, 0);
	spi_lcd_send(0x06, 0); //??

}


void spi_lcd_send(uint8_t Data, St7920_DataType RS)
{
	uint8_t buf[3];

	buf[0]= RS<<1 | 0xF8 ;//11111000
	buf[1]= Data & 0xF0;
	buf[2]= (uint8_t)(Data<<4  & 0xF0);

	HAL_GPIO_WritePin(CS_St7920_GPIOx, CS_St7920_GPIO_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(hspi_St7920, buf, 3, 10);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CS_St7920_GPIOx, CS_St7920_GPIO_Pin, GPIO_PIN_RESET);
}

void ST7920_Write_Str (uint8_t *pData, uint16_t Size)
{
	while (Size > 0U){
		spi_lcd_send(*pData++, DATA);
		Size--;
	}
}
void ST7920_write (int8_t Data, St7920_DataType Type){
	spi_lcd_send(Data, Type );
}

/**
 * @brief  Convert Integer to ASCI cod
 *
 * @param  data 		Integer to Convert
 * @param  FixedLength  The number of characters to display(1-5).
 * 							For floating length must be zero.
 */
void intToASCI(uint32_t data, uint8_t FixedLength){

	uint8_t string[5]={0};
	uint8_t dataCount = 0;

	while(data){
		string[dataCount] = data %10 + 0x30;
		data /= 10;
		dataCount++;
	}

	if(FixedLength){
		dataCount = FixedLength;
	}

	for(dataCount -= 1; dataCount >= 0; dataCount--){
		spi_lcd_send(string[dataCount], DATA);
	}

}
//==============================================================================
// Basic-команда очистки диспле€ в текстовом режиме
//==============================================================================
void ST7920_Basic_Clear(void)
{
	ST7920_write(ST7920_CmdBasic_Clear, CMD);
}
//==============================================================================


//==============================================================================
// Basic-команда дл€ установки курсора в текстовом режиме в начало
//==============================================================================
void ST7920_Basic_Home(void)
{
  ST7920_write(ST7920_CmdBasic_Home, CMD);
}
//==============================================================================


//==============================================================================
// Basic-команда дл€ установки параметров сдвига курсора и экрана
//==============================================================================
void ST7920_Basic_EntryMode(uint8_t ShiftOn, uint8_t MoveRight)
{
  uint8_t Data = ST7920_CmdBasic_EntryMode;

  Data |= ((ShiftOn & 0x01 )  << 0);
  Data |= ((MoveRight & 0x01) << 1);

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Basic-команда дл€ включени€/отключени€ диспле€ и управлени€ отображением
// курсора в текстовом режиме
//==============================================================================
void ST7920_Basic_DisplayOnOff(uint8_t DisplayOn, uint8_t CursorOn, uint8_t BlinkOn)
{
  uint8_t Data = ST7920_CmdBasic_DisplayOnOff;

  if (DisplayOn)
    Data |= (1 << 2);
  if (CursorOn)
    Data |= (1 << 1);
  if (BlinkOn)
    Data |= (1 << 0);

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Basic-команда дл€ установки параметров сдвига курсора
//==============================================================================
void ST7920_Basic_CursorDisplayControl(uint8_t DisplayMoveRight, uint8_t CursorMoveRight)
{
  uint8_t Data = ST7920_CmdBasic_CursorDisplayControl;

  if (DisplayMoveRight)
    Data |= (1 << 3);
  if (CursorMoveRight)
    Data |= (1 << 2);

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Basic-команда дл€ управлени€ текущим режимом (Basic-Extended).
// “акже она устанавливает битность параллельного интерфейса.
//==============================================================================
void ST7920_Basic_FunctionSet(uint8_t Mode)
{
  uint8_t Data = ST7920_Cmd_FunctionSet;

#if (ST7920_IF == ST7920_IF_Parallel_8bit)
  Data |= (1 << 4);
#endif

  if (Mode)
    Data |= (1 << 2);

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Basic-команда установки указател€ в CGRAM
//==============================================================================
void ST7920_Basic_SetCGRAMaddr(uint8_t Addr)
{
  uint8_t Data = ST7920_CmdBasic_SetCGRAMaddr;
  Data |= (Addr & 0x3F);
  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
/**@brief  Basic-команда установки указател€ в DDRAM - позиыи€ курсора
 *
 * First line AC range is 80H..8FH
 * Second line AC range is 90H..9FH
 * Third line AC range is A0H..AFH
 * Fourth line AC range is B0H..BFH
 *
 * @param 	Line 1-4
 * @param 	Position 0-F
 */

//==============================================================================
void ST7920_Basic_SetCursorPosition(uint8_t Line, uint8_t Position)		//DDRAMaddr(uint8_t Line, uint8_t Position)
{
  uint8_t Data = ST7920_CmdBasic_SetDDRAMaddr;
  Data |= (((Line-1)<<4) + Position) & 0x3F;
  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда перевода в режим сна
//==============================================================================
void ST7920_Ext_StandBy(void)
{
  ST7920_write(ST7920_CmdExt_StandBy, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда выбирает указатель на IRAM или Scroll-адрес
//==============================================================================
void ST7920_Ext_SelScrollOrRamAddr(uint8_t SelectScroll)
{
  uint8_t Data = ST7920_CmdExt_SelScrollOrRamAddr;

  if (SelectScroll)
    Data |= 0x01;

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда включает инверсию 1 из 4 строк. ѕовторный вызов отключает инверсию
//==============================================================================
void ST7920_Ext_Reverse(uint8_t Row)
{
  uint8_t Data = ST7920_CmdExt_Reverse;
  Data |= (Row & 0x03);
  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда дл€ управлени€ текущим режимом (Basic-Extended), управлени€
// графическим режимом. “акже она устанавливает битность параллельного интерфейса.
//==============================================================================
void ST7920_Ext_FunctionSet(uint8_t ExtendedMode, uint8_t GraphicMode)
{
  uint8_t Data = ST7920_Cmd_FunctionSet;

#if (ST7920_IF == ST7920_IF_Parallel_8bit)
  Data |= (1 << 4);
#endif

  if (ExtendedMode)
    Data |= (1 << 2);

  if (GraphicMode)
    Data |= (1 << 1);

  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда установки указател€ в IRAM или Scroll-адреса
//==============================================================================
void ST7920_Ext_SetIRAMOrSccrollAddr(uint8_t Addr)
{
  uint8_t Data = ST7920_CmdExt_SetIRAMOrSccrollAddr;
  Data |= (Addr & 0x3F);
  ST7920_write(Data, CMD);


}
//==============================================================================


//==============================================================================
// Extended-команда установки указател€ в буфере кадра графического режима
//==============================================================================
void ST7920_Ext_SetGDRAMAddr(uint8_t VertAddr, uint8_t HorizAddr)
{
  uint8_t Data = ST7920_CmdExt_SetGDRAMAddr;
  Data |= (VertAddr & 0x7F);
  ST7920_write(1, Data);

  Data = ST7920_CmdExt_SetGDRAMAddr;
  Data |= (HorizAddr & 0x0F);
  ST7920_write(Data, CMD);


}
//==============================================================================

