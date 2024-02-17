/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADDR0_Pin GPIO_PIN_13
#define ADDR0_GPIO_Port GPIOC
#define OC_MON_Pin GPIO_PIN_3
#define OC_MON_GPIO_Port GPIOA
#define SD_B_Pin GPIO_PIN_4
#define SD_B_GPIO_Port GPIOA
#define nBRK_B_Pin GPIO_PIN_5
#define nBRK_B_GPIO_Port GPIOA
#define SD_C_Pin GPIO_PIN_6
#define SD_C_GPIO_Port GPIOA
#define nBRK_C_Pin GPIO_PIN_7
#define nBRK_C_GPIO_Port GPIOA
#define ADDR2_Pin GPIO_PIN_12
#define ADDR2_GPIO_Port GPIOB
#define nBRK_A_Pin GPIO_PIN_14
#define nBRK_A_GPIO_Port GPIOB
#define SD_A_Pin GPIO_PIN_15
#define SD_A_GPIO_Port GPIOB
#define CROSS_DET_A_Pin GPIO_PIN_6
#define CROSS_DET_A_GPIO_Port GPIOC
#define CROSS_DET_B_Pin GPIO_PIN_7
#define CROSS_DET_B_GPIO_Port GPIOC
#define CROSS_DET_C_Pin GPIO_PIN_8
#define CROSS_DET_C_GPIO_Port GPIOC
#define ADDR1_Pin GPIO_PIN_12
#define ADDR1_GPIO_Port GPIOA
#define IBUS_MON_JMP_Pin GPIO_PIN_11
#define IBUS_MON_JMP_GPIO_Port GPIOC
#define STATUS_IND_Pin GPIO_PIN_8
#define STATUS_IND_GPIO_Port GPIOB
#define ERROR_IND_Pin GPIO_PIN_9
#define ERROR_IND_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
