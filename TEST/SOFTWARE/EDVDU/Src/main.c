/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FONT/font/f5x7.h"
#include "FONT/font/AF9x10.h"
#include "FONT/font/AF12x16.h"
#include "FONT/font/f10x20.h"
#include "BMP/Logo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Input_DB_GLCD(void);
void Output_DB_GLCD(void);
void Write_DB_GLCD(unsigned char x);
unsigned char Read_DB_GLCD(void);
unsigned int _delay_us(unsigned int Delay);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned char Data;
char data[37];
char Ctemp[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t keysValue = 0;
uint8_t keyNumber = 0;
char str[128];
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin); HAL_Delay(100);
	HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin); HAL_Delay(100);

	// Calibrate The ADC On Power-Up For Better Accuracy
  HAL_ADCEx_Calibration_Start(&hadc1);



	KS108_Init(NON_INVERTED); 	_delay_ms(100);
//	KS108_CLSx();

	KS108_DrawBitmap(Logo, 0, 0, TRANS);
	_delay_ms(1500);
	KS108_CLSx();
	
//	KS108_FillRect(128-20, 64-40, 10, 20, BLACK);
//	KS108_InvertRect(8, 8, 5, 7);
	//KS108_SetInverted(INVERTED);
//	_delay_ms(1500);
//	KS108_CLSx();
//	Line(10, 10, 120, 20, BLACK);
	//Rectangle(15, 15, 60, 40, WHITE);
//	RoundRectangle(40, 40, 63, 63, 4,1 ,BLACK);
//	Circle_Fill(30,30,10,0,BLACK);
//	_delay_ms(2500);
//	KS108_CLSx();

	SetLetter(E_LETTER);
	LcdFont(f5x7);


//	TextBox (0, 0,  48,  9, "EDVDU V1",  0);
//	TextBox (0, 9,  48,  18, "EDVDU V2",  0);
//	TextBox (0, 18,  48,  27, "EDVDU V3",  0);
//	TextBox (0, 27,  48,  36, "EDVDU V4",  0);
//	TextBox (0, 36,  48,  45, "EDVDU V5",  0);
//	TextBox (0, 45,  48,  54, "EDVDU V6",  0);
//	TextBox (0, 54,  48,  63, "EDVDU V7",  0);	
	
	HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin); HAL_Delay(100);
	HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin); HAL_Delay(100);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
/*		
	 // Start ADC Conversion
		HAL_ADC_Start(&hadc1);
	 // Poll ADC1 Perihperal & TimeOut = 1mSec
		HAL_ADC_PollForConversion(&hadc1, 1);
	 // Read The ADC Conversion Result & Map It To PWM DutyCycle
		keysValue = HAL_ADC_GetValue(&hadc1);		
//		HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin); HAL_Delay(1000);
*/

		KS108_CLSx();
		
    HAL_ADC_Start_DMA(&hadc1, &keysValue, 1);
		
		sprintf(str, "Value of Keys = %d", keysValue);
		TextBox (0, 0,  48,  9, str,  0);

		if(keysValue==0){TextBox (0, 9,  48,  18, "GO",  0); keyNumber = 1;}
		else
		if(500<keysValue&&keysValue<600){TextBox (0, 9,  48,  18, "BACK",  0); keyNumber = 2;}
		else
		if(1300<keysValue&&keysValue<1400){TextBox (0, 9,  48,  18, "UP",  0); keyNumber = 3;}
		else
		if(2000<keysValue&&keysValue<2100){TextBox (0, 9,  48,  18, "SELECT",  0); keyNumber = 4;}
		else
		if(2900<keysValue&&keysValue<3000){TextBox (0, 9,  48,  18, "DOWN",  0); keyNumber = 5;}
		else {TextBox (0, 9,  48,  18, "NO KEYS",  0); keyNumber = 0;}
		
		HAL_Delay(100);

//		KS108_FillRect(0, 54, 42, 9, WHITE);
//		KS108_FillRect(80, 45, 47, 9, WHITE);

		
//		
//		switch (keysValue) {
//				case (0):
//						TextBox (0, 9,  48,  18, "GO",  0);
//						break;
//				case (500):
//						printf("Value is 8");
//						break;
//				case 9:
//						printf("Value is 9");
//						break;
//				default:
//						printf("Out of range");
//						break;
//		}
//		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SDCARD_SELECT_Pin|EEPROM_SELECT_Pin|TR_EN_Pin|FLASH_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GLCD_D0_Pin|GLCD_D1_Pin|GLCD_D2_Pin|GLCD_RS_Pin
                          |GLCD_E_Pin|GLCD_RST_Pin|GLCD_RW_Pin|GLCD_D3_Pin
                          |GLCD_D4_Pin|GLCD_D5_Pin|GLCD_D6_Pin|GLCD_D7_Pin
                          |GLCD_CS1_Pin|GLCD_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ALARM_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCARD_SELECT_Pin EEPROM_SELECT_Pin TR_EN_Pin FLASH_SELECT_Pin */
  GPIO_InitStruct.Pin = SDCARD_SELECT_Pin|EEPROM_SELECT_Pin|TR_EN_Pin|FLASH_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GLCD_D0_Pin GLCD_D1_Pin GLCD_D2_Pin GLCD_RS_Pin
                           GLCD_E_Pin GLCD_RST_Pin GLCD_RW_Pin GLCD_D3_Pin
                           GLCD_D4_Pin GLCD_D5_Pin GLCD_D6_Pin GLCD_D7_Pin
                           GLCD_CS1_Pin GLCD_CS2_Pin */
  GPIO_InitStruct.Pin = GLCD_D0_Pin|GLCD_D1_Pin|GLCD_D2_Pin|GLCD_RS_Pin
                          |GLCD_E_Pin|GLCD_RST_Pin|GLCD_RW_Pin|GLCD_D3_Pin
                          |GLCD_D4_Pin|GLCD_D5_Pin|GLCD_D6_Pin|GLCD_D7_Pin
                          |GLCD_CS1_Pin|GLCD_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Input_DB_GLCD(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void Output_DB_GLCD(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Write_DB_GLCD(unsigned char x)
{
	HAL_GPIO_WritePin(KS108_DB_PRT,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(KS108_DB_PRT,(x),GPIO_PIN_SET);
}

unsigned char Read_DB_GLCD(void)
{ 
	Data=0;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_0))
		Data|=1;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_1))
		Data|=2;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_2))
		Data|=4;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_3))
		Data|=8;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_4))
		Data|=16;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_5))
		Data|=32;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_6))
		Data|=64;
	if(HAL_GPIO_ReadPin(KS108_DB_PRT,GPIO_PIN_7))
		Data|=128;
	return Data;
}
unsigned int _delay_us(unsigned int Delay)
{
		//Delay*=100;
		unsigned int Sum=0;
		for(unsigned int Counter=0;Counter<=Delay;Counter++)
		{
				Sum+=Counter;
		}
		return Sum;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		sprintf(str, "Value of DMA = %d", keysValue);
		TextBox (0, 18,  48,  27, str,  0);
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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
