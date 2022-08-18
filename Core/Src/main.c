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
#include "cmsis_os.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

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
#define KEY_PRESSED     0x00
#define KEY_NOT_PRESSED 0x01

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

/* USER CODE BEGIN PV */
osThreadId communicationTaskHandle;
osThreadId indicatorTaskHandle;
osThreadId controlTaskHandle;

unsigned char Data;
char data[37];
char Ctemp[20];

uint8_t ubKeyNumber = 0x0;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void GLCD(void const * argument);
void BLINK(void const * argument);
void CANBUS(void const * argument);

//static void LED_Display(uint8_t LedStatus);

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
}

/* USER CODE BEGIN 4 */
void Input_DB_GLCD(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void Output_DB_GLCD(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Write_DB_GLCD(unsigned char x)
{
	HAL_GPIO_WritePin(KS108_DB_PRT,0xFF,GPIO_PIN_RESET); 
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

/**
  * @brief  Turns ON/OFF the dedicated LED.
  * @param  LedStatus: LED number from 0 to 3
  * @retval None
  */
//static void LED_Display(uint8_t LedStatus)
//{
  /* Turn OFF all LEDs */
//  BSP_LED_Off(LED1);
//  BSP_LED_Off(LED2);
//  BSP_LED_Off(LED3);
//  BSP_LED_Off(LED4);

//  switch(LedStatus)
//  {
//    case (1):
//      /* Turn ON LED1 */
////      BSP_LED_On(LED1);
//      break;

//    case (2):
//      /* Turn ON LED2 */
////      BSP_LED_On(LED2);
//      break;

//    case (3):
//      /* Turn ON LED3 */
////      BSP_LED_On(LED3);
//      break;

//    case (4):
//      /* Turn ON LED4 */
////      BSP_LED_On(LED4);
//      break;
//    default:
//      break;
//  }
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
  while (1)
  {
		HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin);
		osDelay(25);
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
