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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define switch_7_Pin GPIO_PIN_1
#define switch_7_GPIO_Port GPIOA
#define switch_6_Pin GPIO_PIN_4
#define switch_6_GPIO_Port GPIOA
#define switch_5_Pin GPIO_PIN_5
#define switch_5_GPIO_Port GPIOA
#define switch_3_Pin GPIO_PIN_7
#define switch_3_GPIO_Port GPIOA
#define switch_2_Pin GPIO_PIN_0
#define switch_2_GPIO_Port GPIOB
#define switch_1_Pin GPIO_PIN_1
#define switch_1_GPIO_Port GPIOB
#define relay_1_Pin GPIO_PIN_12
#define relay_1_GPIO_Port GPIOB
#define relay_2_Pin GPIO_PIN_13
#define relay_2_GPIO_Port GPIOB
#define relay_3_Pin GPIO_PIN_14
#define relay_3_GPIO_Port GPIOB
#define relay_4_Pin GPIO_PIN_15
#define relay_4_GPIO_Port GPIOB
#define relay_5_Pin GPIO_PIN_8
#define relay_5_GPIO_Port GPIOA
#define relay_6_Pin GPIO_PIN_9
#define relay_6_GPIO_Port GPIOA
#define relay_7_Pin GPIO_PIN_10
#define relay_7_GPIO_Port GPIOA
#define relay_8_Pin GPIO_PIN_15
#define relay_8_GPIO_Port GPIOA
#define switch_8_Pin GPIO_PIN_3
#define switch_8_GPIO_Port GPIOB
#define switch_4_Pin GPIO_PIN_4
#define switch_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
