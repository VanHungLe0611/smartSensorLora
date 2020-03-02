/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define ACC_INT_1_Pin GPIO_PIN_2
#define ACC_INT_1_GPIO_Port GPIOC
#define ACC_INT_2_Pin GPIO_PIN_3
#define ACC_INT_2_GPIO_Port GPIOC
#define LORA_RST_Pin GPIO_PIN_4
#define LORA_RST_GPIO_Port GPIOC
#define LORA_DIO5_Pin GPIO_PIN_5
#define LORA_DIO5_GPIO_Port GPIOC
#define LORA_DIO3_Pin GPIO_PIN_0
#define LORA_DIO3_GPIO_Port GPIOB
#define LORA_DIO4_Pin GPIO_PIN_1
#define LORA_DIO4_GPIO_Port GPIOB
#define LORA_DIO0_Pin GPIO_PIN_2
#define LORA_DIO0_GPIO_Port GPIOB
#define LORA_DIO1_Pin GPIO_PIN_10
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_DIO2_Pin GPIO_PIN_11
#define LORA_DIO2_GPIO_Port GPIOB
#define GPIO0_Pin GPIO_PIN_6
#define GPIO0_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_7
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_8
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_9
#define GPIO3_GPIO_Port GPIOC
#define SUP_EN_Pin GPIO_PIN_8
#define SUP_EN_GPIO_Port GPIOA
#define US_Trigger_Pin GPIO_PIN_15
#define US_Trigger_GPIO_Port GPIOA
#define US_Echo_Pin GPIO_PIN_12
#define US_Echo_GPIO_Port GPIOC
#define US42V2_Trigger_Pin GPIO_PIN_2
#define US42V2_Trigger_GPIO_Port GPIOD
#define US42V2_Echo_Pin GPIO_PIN_4
#define US42V2_Echo_GPIO_Port GPIOB
#define GPIO8_Pin GPIO_PIN_5
#define GPIO8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
