/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

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
#define RELAY1_Pin GPIO_PIN_0
#define RELAY1_GPIO_Port GPIOD
#define RELAY2_Pin GPIO_PIN_1
#define RELAY2_GPIO_Port GPIOD
#define RELAY3_Pin GPIO_PIN_2
#define RELAY3_GPIO_Port GPIOD
#define RELAY4_Pin GPIO_PIN_3
#define RELAY4_GPIO_Port GPIOD
#define RELAY5_Pin GPIO_PIN_4
#define RELAY5_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* Following the Board Pinout definition */
#define SSRELAY_GPIO_Port GPIOE
#define SSRELAY12_Pin GPIO_PIN_11
#define SSRELAY11_Pin GPIO_PIN_10
#define SSRELAY10_Pin GPIO_PIN_9
#define SSRELAY9_Pin GPIO_PIN_8
#define SSRELAY8_Pin GPIO_PIN_7
#define SSRELAY7_Pin GPIO_PIN_6
#define SSRELAY6_Pin GPIO_PIN_5
#define SSRELAY5_Pin GPIO_PIN_4
#define SSRELAY4_Pin GPIO_PIN_3
#define SSRELAY3_Pin GPIO_PIN_2
#define SSRELAY2_Pin GPIO_PIN_1
#define SSRELAY1_Pin GPIO_PIN_0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
