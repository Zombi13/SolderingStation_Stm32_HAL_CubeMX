/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <st7920.h>
//#include "arm_math.h"
#include "pid.h"
#include "encoder.h"
#include "RTCfunc.h"
#include <U8g2.h>
#include <u8g2_arm.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define		debug				1

#define		ZCD_Pin				GPIO_PIN_3
#define		FenRelePort			GPIOA
#define		FenRelePin			GPIO_PIN_2

#define		DimmerTim			htim14
#define		DimmerTimCh			TIM_CHANNEL_1
#define		FenFanPWM_T			htim1
#define		FenFanPWM_ch		TIM_CHANNEL_2
#define		SolderPWM_T			htim1
#define		SolderPWM_ch		TIM_CHANNEL_1

#define		EncoderTimer		htim3
#define		EncoderButton_Port	GPIOA
#define		EncoderButton_Pin	GPIO_PIN_5

#define		Gerkon_Port			GPIOA
#define		Gerkon_Pin			GPIO_PIN_12
#define		Button1_Pin			GPIO_PIN_10
#define		Button2_Pin			GPIO_PIN_1
#define		Button3_Pin			GPIO_PIN_15
#define		RollBall_Pin		GPIO_PIN_0
#define 	Lcd_CS				GPIOB, GPIO_PIN_6

//#define		SolderMod			0
//#define		FenMod				1
//#define		FenFanMod			2
#define		Off					0
#define		On					1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
u8g2_t u8g2;
EncoderTypeDef enc0;
RTC_TimePointTypeDef Point[4]={0};


typedef enum
{
	OFF = 0U,
    ON
}StateType;

typedef enum {
	SolderMod,
	FenMod,
	FenFanMod,
	SpecMod
}WorkModType ,WorkMod1;
typedef struct
{
	uint16_t		Nadc[16];
	//uint16_t		Uadc[16];			//millivolts from ADC
	uint16_t		Text[16];			//from external measurementp;

}ThermocoupleCalibrationTypeDef;

ThermocoupleCalibrationTypeDef	ThermocoupleProfile[8];

#define		RollBall_IE	0x01
#define		Gerkon_IE	0x10

struct {
	//0000 0001		solder 	RollBall
	//0001 0000		fen 	Gerkon
	uint8_t		IE;
	WorkModType WorkMod;
	uint8_t		Step;
	uint8_t		MoveDetect;
	ThermocoupleCalibrationTypeDef	*ThermocoupleProfile;
	int32_t 	CPUtemperature;
}Station;

uint8_t	Profile;
int16_t SMtemp;

typedef struct
{
	uint8_t 			Status;
	uint8_t				GerkonStatus;
	uint8_t				DimerStatus;
	uint16_t 			DimmerValue;
	uint8_t 			T_Measured;
	int16_t 			T_Set;
	uint16_t			T_Max;
	uint8_t 			T_Min;
	uint8_t 			T_FanOff;
	uint8_t 			MaxPower;			//Max power limit, [%]
	uint16_t 			CalibrPoint0;
	uint16_t 			CalibrPoint1;
	uint8_t				error;
	TIM_HandleTypeDef	*htim;
	uint32_t 			Channel;
}ChannelTypeDef;

typedef struct
{
	uint8_t 			Status;
	int16_t 			P_Set;
	uint8_t				P_Max;
	uint8_t 			P_Min;
	uint8_t				error;
	TIM_HandleTypeDef	*htim;
	uint32_t 			Channel;
	uint16_t			FanStep;
}FanTypeDef;

typedef struct
{
	uint8_t 			Status;
	uint8_t 			T_Measured;
	int16_t 			T_Set;
	uint16_t			T_Max;
	uint8_t 			T_Min;
	uint8_t				error;
	TIM_HandleTypeDef	*htim;
	uint32_t 			Channel;
	uint8_t				MaxPower;
}SolderTypeDef;


SolderTypeDef 	Solder;
ChannelTypeDef 	Fen;
FanTypeDef 		FenFan;

struct
{
	uint32_t		buffer[30];
	uint8_t			bufferCaunt;
	float			step;
	uint32_t		Sfen;
	uint32_t		Ssolder;
	uint32_t		Ssensor;

}ADCData;

//int16_t T_Fen_Set=0;
//int16_t T_Solder_Set=0;
//int16_t FenFanSpid=0;
//
//uint8_t SolderTMin = 0;			//C
//uint16_t SolderTMax = 350;		//C
//uint8_t FenTMin = 0;			//C
//uint16_t FenTMax = 450;			//C
//uint8_t FenFanSpidMin = 0; 	//%
//uint8_t FenFanSpidMax = 100;	//%
//uint8_t Step = 5;
//uint16_t FanStep = 0;

//uint8_t 	WorkMod =0;


//uint16_t 	DimmerValue=0;
//uint8_t 	FenStatus = 0;
//uint8_t 	FenFanStatus =0,
uint8_t	FanSpeedMax=3,FanOff=0;
//uint16_t 	testValue=0;
uint8_t		CountForButton = 0;

//uint16_t 	adc_value[3];
//uint16_t 	Tsrns;
//float 		adc_step=3.3/4095;
//uint16_t	tempArr[256],calibr[30];
//uint8_t		i = 0;
//uint8_t		EncoderButtonPushed = 0;
//uint8_t		CalibrCount;

uint8_t DispleiMod =0;
char buffer[12];
uint16_t i=250 ,x=120, y=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void SetValue(int16_t T);
void PID_Fen();
void PID_Solder();
void DimmerControl(ChannelTypeDef* ChStrct);
void ZCD_Handler();
void test();
void SolderControl(SolderTypeDef* solder);
//uint16_t TimePointCompare(RTC_TimePointTypeDef *point0, RTC_TimePointTypeDef *point1);
void FanControl(uint8_t status);

void StationConfig ();
void ErrorControl();
uint16_t temp01();
void Srednie();
void displaiInit();
void MoweDetect();
void InactivityDetect ();
void adc_to_t();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//lcd init
  StationConfig ();

  u8g2_SPI_Set(&hspi1, Lcd_CS);
  u8g2_Setup_st7920_s_128x64_2(&u8g2, U8G2_R0, u8xx_byte_hw_spi, u8xx_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
//
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);	//start timet in encoder mod + intrrupt
  HAL_TIM_PWM_Start_IT(&SolderPWM_T, TIM_CHANNEL_3);
  //arm_pid_f32();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //ErrorControl();
	  SetValue(encoder(&enc0) * Station.Step);
	  if(enc0.Difference != 0)
		  MoweDetect();
	  InactivityDetect ();

	  if(Station.WorkMod != SpecMod) test();
	  adc_to_t();
	  displaiInit();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 20000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA3 PA5 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void displaiInit (){

	//utoa(i, buffer, 10);

    u8g2_FirstPage(&u8g2);
    do
    {
/*
    	u8g2_DrawFrame(&u8g2, 1, 5, 5, 3);
    	u8g2_DrawFrame(&u8g2, 7, 4, 7, 4);
    	u8g2_DrawFrame(&u8g2, 16, 3, 9, 5);
    	u8g2_DrawFrame(&u8g2, 27, 3, 11, 5);
    	u8g2_DrawFrame(&u8g2, 40, 2, 13, 6);
    	u8g2_DrawFrame(&u8g2, 55, 2, 15, 6);
    	u8g2_DrawFrame(&u8g2, 72, 1, 17, 7);
    	u8g2_DrawFrame(&u8g2, 91, 1, 19, 7);
    	u8g2_DrawFrame(&u8g2, 111, 0, 21, 8);
*/
    	u8g2_DrawTriangle(&u8g2, 1, 10, x, 10, x, y);

		u8g2_DrawHLine(&u8g2, 0, 10, 128);

		u8g2_DrawLine(&u8g2, 0, 43, 6, 64);
		u8g2_DrawLine(&u8g2, 42, 43, 36, 64);
		u8g2_DrawLine(&u8g2, 6, 63, 36, 63);

		u8g2_DrawLine(&u8g2, 42, 43, 48, 63);
		u8g2_DrawLine(&u8g2, 85, 43, 79, 63);
		u8g2_DrawLine(&u8g2, 48, 63, 79, 63);

		u8g2_DrawLine(&u8g2, 85, 44, 91, 63);
		u8g2_DrawLine(&u8g2, 127, 44, 121, 63);
		u8g2_DrawLine(&u8g2, 91, 63, 121, 63);

		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 5, 53, "Solder");
		utoa(Solder.T_Measured, buffer, 10);
		u8g2_DrawStr(&u8g2, 12, 63, buffer);
		u8g2_DrawStr(&u8g2, 54, 53, "Fen");
		utoa(Fen.T_Measured, buffer, 10);
		u8g2_DrawStr(&u8g2, 54, 63, buffer);
		u8g2_DrawStr(&u8g2, 98, 53, "Fan");
		utoa(FenFan.P_Set, buffer, 10);
		u8g2_DrawStr(&u8g2, 98, 63, buffer);
		u8g2_DrawUTF8(&u8g2, 110, 63, "%");
//		u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
		u8g2_DrawUTF8(&u8g2, 29, 60, "°");
		u8g2_DrawUTF8(&u8g2, 71, 68, "°");
//---------------------------------------------------------------------------
		if (Solder.error || Fen.error || FenFan.error){
			u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
			if (Solder.error){
				u8g2_DrawStr(&u8g2, 35, 19, "Solder error");
			}
			if (Fen.error){
				u8g2_DrawStr(&u8g2, 35, 29, "Fen error");
			}
			if (FenFan.error){
				u8g2_DrawStr(&u8g2, 35, 39, "FenFan error");
			}
		}else{
			switch (DispleiMod){
			case 0:
				if(Solder.Status && Fen.Status){
					switch (Station.WorkMod){
					case SolderMod:
						u8g2_DrawHLine(&u8g2, 42, 43, 44);
						u8g2_DrawHLine(&u8g2, 85, 43, 42);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
						utoa(Solder.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 5, 39, buffer);
						u8g2_DrawCircle(&u8g2, 64, 14, 2, U8G2_DRAW_ALL);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB18_tr);
						utoa(Fen.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 70, 39, buffer);
						u8g2_DrawCircle(&u8g2, 115, 20, 2, U8G2_DRAW_ALL);
						break;
					case FenMod:
						u8g2_DrawHLine(&u8g2, 85, 43, 42);
					case FenFanMod:
						u8g2_DrawHLine(&u8g2, 0, 43, 42);
						if(Station.WorkMod == FenFanMod)
							u8g2_DrawHLine(&u8g2, 42, 43, 44);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB18_tr);
						utoa(Solder.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 5, 39, buffer);
						u8g2_DrawCircle(&u8g2, 50, 20, 2, U8G2_DRAW_ALL);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
						utoa(Fen.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 60, 39, buffer);
						u8g2_DrawCircle(&u8g2, 119, 14, 2, U8G2_DRAW_ALL);
						break;
					case SpecMod:
						break;

					}

				}else{
					u8g2_DrawCircle(&u8g2, 95, 14, 2, U8G2_DRAW_ALL);
					switch (Station.WorkMod){
					case SolderMod:
						u8g2_DrawHLine(&u8g2, 42, 43, 44);
						u8g2_DrawHLine(&u8g2, 85, 43, 42);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
						utoa(Solder.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 35, 39, buffer);
						break;
					case FenMod:
						u8g2_DrawHLine(&u8g2, 85, 43, 42);
					case FenFanMod:
						u8g2_DrawHLine(&u8g2, 0, 43, 42);
						if(Station.WorkMod == FenFanMod)
							u8g2_DrawHLine(&u8g2, 42, 43, 44);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
						utoa(Fen.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 35, 39, buffer);
						break;
					case SpecMod:
						break;
					}

				}
				break;
			case 1:
				switch (Station.WorkMod){
				case SolderMod:
					u8g2_DrawHLine(&u8g2, 42, 43, 44);
					u8g2_DrawHLine(&u8g2, 85, 43, 42);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
					if (Station.MoveDetect == 1){
						utoa(Solder.T_Set, buffer, 10);
						u8g2_DrawStr(&u8g2, 5, 39, buffer);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
						u8g2_DrawStr(&u8g2, 115, 30, "<");
					}else{
						utoa(Solder.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 5, 39, buffer);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
						u8g2_DrawStr(&u8g2, 115, 40, "<");
					}
					u8g2_DrawCircle(&u8g2, 64, 14, 2, U8G2_DRAW_ALL);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
					u8g2_DrawStr(&u8g2, 75, 20, (Solder.Status)? "on" : "off");
					u8g2_DrawStr(&u8g2, 75, 30, "Set:");
					utoa(Solder.T_Set, buffer, 10);
					u8g2_DrawStr(&u8g2, 100, 30, buffer);
					u8g2_DrawStr(&u8g2, 75, 40, "Cur:");
					utoa(Solder.T_Measured, buffer, 10);
					u8g2_DrawStr(&u8g2, 100, 40, buffer);
					break;
				case FenMod:
					u8g2_DrawHLine(&u8g2, 0, 43, 42);
					u8g2_DrawHLine(&u8g2, 85, 43, 42);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
					if (Station.MoveDetect == 1){
						utoa(Fen.T_Set, buffer, 10);
						u8g2_DrawStr(&u8g2, 60, 39, buffer);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
						u8g2_DrawStr(&u8g2, 45, 30, "<");
					}else{
						utoa(Fen.T_Measured, buffer, 10);
						u8g2_DrawStr(&u8g2, 60, 39, buffer);
						u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
						u8g2_DrawStr(&u8g2, 45, 40, "<");
					}
					u8g2_DrawCircle(&u8g2, 119, 14, 2, U8G2_DRAW_ALL);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
					u8g2_DrawStr(&u8g2, 5, 20, (Fen.Status)? "on" : "off");
					u8g2_DrawStr(&u8g2, 5, 30, "Set:");
					utoa(Fen.T_Set, buffer, 10);
					u8g2_DrawStr(&u8g2, 30, 30, buffer);
					u8g2_DrawStr(&u8g2, 5, 40, "Cur:");
					utoa(Fen.T_Measured, buffer, 10);
					u8g2_DrawStr(&u8g2, 30, 40, buffer);
					break;
				case FenFanMod:
					u8g2_DrawHLine(&u8g2, 0, 43, 42);
					u8g2_DrawHLine(&u8g2, 42, 43, 44);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB24_tr);
					utoa(FenFan.P_Set, buffer, 10);
					u8g2_DrawStr(&u8g2, 60, 39, buffer);
					u8g2_DrawCircle(&u8g2, 119, 14, 2, U8G2_DRAW_ALL);
					u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
					u8g2_DrawStr(&u8g2, 5, 20, (FenFan.Status)? "on" : "off");
					u8g2_DrawStr(&u8g2, 5, 30, "Set:");
					utoa(FenFan.P_Set, buffer, 10);
					u8g2_DrawStr(&u8g2, 30, 30, buffer);
					//u8g2_DrawStr(&u8g2, 5, 40, "Cur:");
					//u8g2_DrawStr(&u8g2, 30, 40, "250");
					break;
				case SpecMod:
					break;
				}
				break;
			}
		}

    } while (u8g2_NextPage(&u8g2));
}


void StationConfig ()
{
	Station.IE 			&= ~(RollBall_IE + Gerkon_IE);
	Station.Step 		= 1;
	enc0.htim 			= &htim3;
	enc0.HoldButtonTime = 60;

	Solder.htim 		= &htim1;
	Solder.Channel 		= TIM_CHANNEL_1;
	Solder.MaxPower 	= 95;
	Solder.T_Min		= 0;
	Solder.T_Max		= 450;


	Fen.htim 			= &htim14;
	Fen.Channel 		= TIM_CHANNEL_1;
	Fen.MaxPower 		= 30;
	Fen.T_Min			= 0;
	Fen.T_Max			= 450;
	Fen.T_FanOff		= 50;


	FenFan.htim 		= &htim1;
	FenFan.Channel 		= TIM_CHANNEL_2;
	FenFan.P_Min		= 25;
	FenFan.P_Max		= 100;

	FenFan.FanStep 		= FenFanPWM_T.Init.Period/100;

	ADCData.step = (float)3300/4095;



}

void test()
{
	switch(Station.WorkMod){
		case SolderMod:
			Solder.htim->Instance->CCR1 =Solder.T_Set;
			Solder.htim->Instance->CCR3 =Solder.T_Set +(Solder.htim->Init.Period - Solder.T_Set)/2;
			break;
		case FenMod:
			Fen.DimmerValue = Fen.T_Set;
			break;
		case FenFanMod:
			FenFan.htim->Instance->CCR2 = FenFan.P_Set * FenFan.FanStep;
			break;
		case SpecMod:

			break;

	}
}

void adc_to_t(){
	Srednie();
	Solder.T_Measured = ADCData.Ssolder * ADCData.step;
	Fen.T_Measured = ADCData.Sfen * ADCData.step;

//#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
//#define VDD_CALIB ((uint32_t) (4095))
//#define VDD_APPLI ((uint32_t) (3300))
//#define AVG_SLOPE ((uint32_t) (5336)) //AVG_SLOPE in ADC conversion step
//(@3.3V)/°C multiplied by 1000 for precision on the division
//int32_t temperature; /* will contain the temperature in degrees Celsius */

 // AVG_SLOPE= 4.3mV/C
 // V30= 1.43V
//Station.CPUtemperature = ((uint32_t) *TEMP30_CAL_ADDR
//- ((uint32_t) ADCData.buffer[ADCData.bufferCaunt+2] * VDD_APPLI / VDD_CALIB)) * 1000;

//Station.CPUtemperature = ((uint32_t) *TEMP30_CAL_ADDR - ADCData.buffer[ADCData.bufferCaunt+2]*ADCData.step);
//Station.CPUtemperature = (Station.CPUtemperature / AVG_SLOPE) + 30;
	Station.CPUtemperature = (1325 - ADCData.buffer[ADCData.bufferCaunt+2]*ADCData.step)*1000;
	Station.CPUtemperature = (Station.CPUtemperature /4200) + 30;
///

}

void SetValue(int16_t Value){
	switch(Station.WorkMod){
		case SolderMod:
			Solder.T_Set += Value*100;
			if(Solder.T_Set <= Solder.T_Min) Solder.T_Set = Solder.T_Min;
			//if(Solder.T_Set >= Solder.T_Max) Solder.T_Set = Solder.T_Max;
			break;
		case FenMod:
			Fen.T_Set += Value*100;
			if(Fen.T_Set <= Fen.T_Min) Fen.T_Set = Fen.T_Min;
			//if(Fen.T_Set >= Fen.T_Max) Fen.T_Set = Fen.T_Max;
			break;
		case FenFanMod:
			FenFan.P_Set += Value;
			if(FenFan.P_Set <= FenFan.P_Min) FenFan.P_Set = FenFan.P_Min;
			if(FenFan.P_Set >= FenFan.P_Max) FenFan.P_Set = FenFan.P_Max;
			break;
		case SpecMod:
			SMtemp += Value;
			break;
	}
}

struct
{
	uint8_t		step;
	uint8_t		i;
}CalibrationData;

void ThermocoupleCalibration (){
	// вывести еккран преветствия
	//----------------------------//
	// выберете профель			  //
	//0.Fen		4.пусто		1	  //
	//1.Fen2?	5.пусто		1	  //
	//2.Solder	6.пусто		1	  //
	//3.пусто	7.пусто		1	  //
	//----------------------------//



	CalibrationData.step = 0;

		switch(CalibrationData.step){
		case 0:
			if(SMtemp > 8) SMtemp =8;
			if(SMtemp < 0) SMtemp =0;
			Profile = SMtemp;
			break;
		case 1:
			Srednie();
			ThermocoupleProfile[Profile].Nadc[0] = temp01();
			ThermocoupleProfile[Profile].Text[0] = SMtemp;

			break;
		case 3:
			if (CalibrationData.i ==0)
			{
				SMtemp = 100;
				CalibrationData.i++;
			}
			Solder.htim->Instance->CCR1 =SMtemp;
			Solder.htim->Instance->CCR3 =SMtemp +(Solder.htim->Init.Period - SMtemp)/2;
			break;
		case 4:
			if (CalibrationData.i ==0)
			{
				SMtemp = 50;
				CalibrationData.i++;
			}
			Srednie();
			ThermocoupleProfile[Profile].Nadc[0] = temp01();
			ThermocoupleProfile[Profile].Text[0] = SMtemp;
			break;
		case 5:
			if (CalibrationData.i ==0)
			{
				SMtemp = 200;
				CalibrationData.i++;
			}
			Solder.htim->Instance->CCR1 =SMtemp;
			Solder.htim->Instance->CCR3 =SMtemp +(Solder.htim->Init.Period - SMtemp)/2;
			break;
		case 6:
			if (CalibrationData.i ==0)
			{
				SMtemp = 100;
				CalibrationData.i++;
			}
			Srednie();
			ThermocoupleProfile[Profile].Nadc[0] = temp01();
			ThermocoupleProfile[Profile].Text[0] = SMtemp;
			break;


		}


}

uint16_t temp01(){
	if (Profile<2){
		if (ADCData.Sfen - ADCData.buffer[ADCData.bufferCaunt+1] < 15 && ADCData.buffer[ADCData.bufferCaunt+1]-ADCData.Sfen <15){
			return ADCData.buffer[ADCData.bufferCaunt+1];
		}
	}
	else{
		if (ADCData.Ssolder - ADCData.buffer[ADCData.bufferCaunt] < 15 && ADCData.buffer[ADCData.bufferCaunt]-ADCData.Ssolder <15){
			return ADCData.buffer[ADCData.bufferCaunt];
		}
	}
	return 0;
}

void Srednie(){
	ADCData.Ssolder = ADCData.buffer[0];
	ADCData.Sfen 	= ADCData.buffer[1];
	ADCData.Ssensor = ADCData.buffer[2];

	for (int i=3; i < sizeof(ADCData.buffer)/4; i+=3){
		ADCData.Ssolder += ADCData.buffer[i];
		ADCData.Sfen 	+= ADCData.buffer[i+1];
		ADCData.Ssensor += ADCData.buffer[i+2];
	}
	ADCData.Sfen 	/= sizeof(ADCData.buffer)/4/3;
	ADCData.Ssolder /= sizeof(ADCData.buffer)/4/3;
	ADCData.Ssensor /= sizeof(ADCData.buffer)/4/3;
}

void MoweDetect(){
	DispleiMod =1;
	Station.MoveDetect = 1;
	HAL_RTC_GetTime(&hrtc, &Point[2].sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &Point[2].sDate, RTC_FORMAT_BCD);
}

void InactivityDetect (){
	if(Station.MoveDetect){
		HAL_RTC_GetTime(&hrtc, &Point[3].sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &Point[3].sDate, RTC_FORMAT_BCD);

		if(Station.MoveDetect == 1){
			if (TimePointCompare(&Point[2], &Point[3]) >= 2){
				Station.MoveDetect = 2;
			}
		}
		if(Station.MoveDetect == 2){
			if (TimePointCompare(&Point[2], &Point[3]) >= 5){
				Station.MoveDetect = 3;
				DispleiMod =0;
			}
		}
		if(Station.MoveDetect == 3){
			if (TimePointCompare(&Point[2], &Point[3]) >= 60){
				Station.MoveDetect = 0;
				//slipp
			}

		}
	}
}

uint8_t PushButton(uint8_t workMod){
	if(Station.WorkMod != workMod) CountForButton = 0;
	if (CountForButton == 0) {
		HAL_RTC_GetTime(&hrtc, &Point[0].sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &Point[0].sDate, RTC_FORMAT_BCD);
		CountForButton++;
		Station.WorkMod = workMod;
		return On;
	}else{
		HAL_RTC_GetTime(&hrtc, &Point[1].sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &Point[1].sDate, RTC_FORMAT_BCD);
		if (TimePointCompare(&Point[0], &Point[1]) < 10){
			CountForButton = 0;
			return Off;
		} else {
			Station.WorkMod = workMod;
			HAL_RTC_GetTime(&hrtc, &Point[0].sTime, RTC_FORMAT_BCD);
			HAL_RTC_GetDate(&hrtc, &Point[0].sDate, RTC_FORMAT_BCD);
			return On;
		}
	}
}


void PID_Fen(){

}

void PID_Solder(){

}
void ErrorControl(){
	if(Fen.T_Measured >= Fen.T_Max){
		Fen.error = 1;
	}
//	if(Fen.htim->Instance->CCR1 >= (Fen.htim->Instance->ARR ) * Fen.MaxPower /100){	//Проверка на превкшение мощности
//		Fen.error = 1;
//	}

	if(Solder.htim->Instance->CCR1 >= (Solder.htim->Instance->ARR /100) * Solder.MaxPower){	//Проверка на превкшение мощности
		Solder.error = 1;
	}

	if(Solder.T_Measured >= Solder.T_Max){
		Solder.error = 1;
	}

	if (Fen.error){
		HAL_GPIO_WritePin(FenRelePort, FenRelePin, GPIO_PIN_RESET);
		Fen.DimerStatus = OFF;
		FanControl(FanSpeedMax);
		//ERROR msg
		//Sound alarm
	}
	if (FenFan.error){
		HAL_GPIO_WritePin(FenRelePort, FenRelePin, GPIO_PIN_RESET);
		Fen.DimerStatus = OFF;
		FanControl(OFF);
		//ERROR msg
		//Sound alarm
	}
	if (Solder.error){
		HAL_TIM_PWM_Stop_IT(&SolderPWM_T, SolderPWM_ch);
		//ERROR msg
		//Sound alarm
	}
}

void SolderControl(SolderTypeDef* solder){
	if(solder->Status == On && solder->error == 0){
		HAL_TIM_PWM_Start_IT(&SolderPWM_T, SolderPWM_ch);
	}
	else{
		//solder->Status = OFF;
		HAL_TIM_PWM_Stop_IT(&SolderPWM_T, SolderPWM_ch);
	}
}

void Gerkon_Handler(){
	if ((Station.IE & Gerkon_IE) && Fen.Status  ){
		Fen.GerkonStatus = HAL_GPIO_ReadPin(Gerkon_Port, Gerkon_Pin);
		Fen.Status = Fen.GerkonStatus;
		DimmerControl(&Fen);
	}
}

void DimmerControl(ChannelTypeDef* ChStrct)
{
	if(ChStrct->Status == On && ChStrct->error == 0)
	{
		ChStrct->DimerStatus = ON;
		FanControl(On);
		HAL_GPIO_WritePin(FenRelePort, FenRelePin, GPIO_PIN_SET);
		HAL_TIM_OnePulse_Start_IT(&DimmerTim, DimmerTimCh);

	}
	else
	{
		//Fen.Status = OFF;
		ChStrct->DimerStatus = OFF;
		HAL_GPIO_WritePin(FenRelePort, FenRelePin, GPIO_PIN_RESET);
	}
}

// peredelat ?
void FanControl(uint8_t status)
{
	if (status == FanSpeedMax){
		FenFanPWM_T.Instance->CCR2 = FenFanPWM_T.Instance->ARR;
		HAL_TIM_PWM_Start(&FenFanPWM_T, FenFanPWM_ch);
	}else if(status == Off){
		if(debug){													//проверка температуры
			HAL_TIM_PWM_Stop(&FenFanPWM_T, FenFanPWM_ch);
		}
	}
	else
	{
		FenFanPWM_T.Instance->CCR2 = FenFan.P_Set;
		HAL_TIM_PWM_Start(&FenFanPWM_T, FenFanPWM_ch);
	}
}

void ZCD_Handler()
{
	if(Fen.DimerStatus == On && Fen.error == 0){
		Fen.htim->Instance->CR1 |= TIM_CR1_CEN;		//Timer Start

		//Zcd ++;
/*      switch (mode){
		  case 0:
			TIM14->CR1 |= TIM_CR1_CEN;
			Zcd = 0;
			break;
		  case 1:
			if(Zcd == 2){
			  TIM14->CR1 |= TIM_CR1_CEN;
			  Zcd = 0;
			}
			break;
		  case 2:
			if(Zcd == 3){
			  TIM14->CR1 |= TIM_CR1_CEN;
			  Zcd = 0;
			}
			break;
		  case 3:
			TIM14->CR1 |= TIM_CR1_CEN;
			break;
		  default :
			Zcd = 0;
			break;
		  }*/
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (Station.WorkMod != SpecMod){
		switch (GPIO_Pin){
			case RollBall_Pin:
				if( Solder.Status && (Station.IE & RollBall_IE)){
					Solder.Status = ON;
					SolderControl(&Solder);
					MoweDetect();
				}
				break;
			case ZCD_Pin:		// GPIO_PIN_3://ZCD
				ZCD_Handler();
				break;
			case EncoderButton_Pin:
				EncoderButton_Handler(&enc0, &hrtc);
				MoweDetect();
				break;
			case Gerkon_Pin:
				Gerkon_Handler();
				MoweDetect();
				break;
			case Button1_Pin:
				Solder.Status = PushButton(SolderMod);
				SolderControl(&Solder);
				MoweDetect();
				break;
			case Button2_Pin:
				Fen.Status = PushButton(FenMod);
				DimmerControl(&Fen);
				MoweDetect();
				break;
			case Button3_Pin:
				FenFan.Status = PushButton(FenFanMod);
				FanControl(FenFan.Status);					// peredelat // not off in this place
				MoweDetect();
				break;
		}
	}else
	{
		switch (GPIO_Pin){
			case RollBall_Pin:
				MoweDetect();
				break;
			case ZCD_Pin:
				ZCD_Handler();
				break;
			case EncoderButton_Pin:
				CalibrationData.step++;
				CalibrationData.i = 0;
				MoweDetect();
				break;
			case Gerkon_Pin:
				MoweDetect();
				break;
			case Button1_Pin:
				MoweDetect();
				break;
			case Button2_Pin:
				MoweDetect();
				break;
			case Button3_Pin:
				MoweDetect();
				break;
		}
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  if(htim == Fen.htim){
	  htim->Instance->ARR = Fen.DimmerValue+50;
	  htim->Instance->CCR1 = Fen.DimmerValue;
	  htim->Instance->EGR |= TIM_EGR_UG;			// Reinitialize the counter and generates an update of the registers.
  }
  if(htim == Solder.htim){
	  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	  {
//		  if(i>=256) i =0;
//		  tempArr[i]= adc_value[1];
//		  i++;
		  ADCData.bufferCaunt +=3;
		  if (ADCData.bufferCaunt >= 30) ADCData.bufferCaunt = 0;
		  HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADCData.buffer[ADCData.bufferCaunt], 3);
//		  HAL_ADC_Start(&hadc);
//		  HAL_ADC_PollForConversion(&hadc, 50);
//		  adc_value[0] = HAL_ADC_GetValue(&hadc);

	  }
  }

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
