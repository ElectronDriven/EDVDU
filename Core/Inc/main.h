/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALARM_Pin GPIO_PIN_13
#define ALARM_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_4
#define TEMP_GPIO_Port GPIOA
#define SD_SELECT_Pin GPIO_PIN_5
#define SD_SELECT_GPIO_Port GPIOA
#define FLASH_SELECT_Pin GPIO_PIN_6
#define FLASH_SELECT_GPIO_Port GPIOA
#define DOUT_Pin GPIO_PIN_7
#define DOUT_GPIO_Port GPIOA
#define GLCD_D0_Pin GPIO_PIN_0
#define GLCD_D0_GPIO_Port GPIOB
#define GLCD_D1_Pin GPIO_PIN_1
#define GLCD_D1_GPIO_Port GPIOB
#define GLCD_D2_Pin GPIO_PIN_2
#define GLCD_D2_GPIO_Port GPIOB
#define LOAD_Pin GPIO_PIN_8
#define LOAD_GPIO_Port GPIOA
#define KEYS_Pin GPIO_PIN_15
#define KEYS_GPIO_Port GPIOA
#define GLCD_D3_Pin GPIO_PIN_3
#define GLCD_D3_GPIO_Port GPIOB
#define GLCD_D4_Pin GPIO_PIN_4
#define GLCD_D4_GPIO_Port GPIOB
#define GLCD_D5_Pin GPIO_PIN_5
#define GLCD_D5_GPIO_Port GPIOB
#define GLCD_D6_Pin GPIO_PIN_6
#define GLCD_D6_GPIO_Port GPIOB
#define GLCD_D7_Pin GPIO_PIN_7
#define GLCD_D7_GPIO_Port GPIOB
#define GLCD_CS1_Pin GPIO_PIN_8
#define GLCD_CS1_GPIO_Port GPIOB
#define GLCD_CS2_Pin GPIO_PIN_9
#define GLCD_CS2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
