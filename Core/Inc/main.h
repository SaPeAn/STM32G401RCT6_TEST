/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define PWR_OFF_Pin GPIO_PIN_13
#define PWR_OFF_GPIO_Port GPIOC
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOA
#define IU_TX_Pin GPIO_PIN_2
#define IU_TX_GPIO_Port GPIOA
#define IU_RX_Pin GPIO_PIN_3
#define IU_RX_GPIO_Port GPIOA
#define V_BAT_Pin GPIO_PIN_4
#define V_BAT_GPIO_Port GPIOA
#define DISP_SCK_Pin GPIO_PIN_5
#define DISP_SCK_GPIO_Port GPIOA
#define BR_PWM_Pin GPIO_PIN_6
#define BR_PWM_GPIO_Port GPIOA
#define DISP_MOSI_Pin GPIO_PIN_7
#define DISP_MOSI_GPIO_Port GPIOA
#define DISP_RS_Pin GPIO_PIN_0
#define DISP_RS_GPIO_Port GPIOB
#define DISP_RSE_Pin GPIO_PIN_1
#define DISP_RSE_GPIO_Port GPIOB
#define DISP_CS_Pin GPIO_PIN_10
#define DISP_CS_GPIO_Port GPIOB
#define MEM_NSS_Pin GPIO_PIN_12
#define MEM_NSS_GPIO_Port GPIOB
#define MEM_SCK_Pin GPIO_PIN_13
#define MEM_SCK_GPIO_Port GPIOB
#define MEM_MISO_Pin GPIO_PIN_14
#define MEM_MISO_GPIO_Port GPIOB
#define MEM_MOSI_Pin GPIO_PIN_15
#define MEM_MOSI_GPIO_Port GPIOB
#define BTN_0_Pin GPIO_PIN_8
#define BTN_0_GPIO_Port GPIOA
#define BTN_1_Pin GPIO_PIN_9
#define BTN_1_GPIO_Port GPIOA
#define BTN_2_Pin GPIO_PIN_10
#define BTN_2_GPIO_Port GPIOA
#define SOUND__Pin GPIO_PIN_4
#define SOUND__GPIO_Port GPIOB
#define SOUND_B5_Pin GPIO_PIN_5
#define SOUND_B5_GPIO_Port GPIOB
#define UTX_Pin GPIO_PIN_6
#define UTX_GPIO_Port GPIOB
#define U_RX_Pin GPIO_PIN_7
#define U_RX_GPIO_Port GPIOB
#define BTN_3_Pin GPIO_PIN_8
#define BTN_3_GPIO_Port GPIOB
#define BTN_J_Pin GPIO_PIN_9
#define BTN_J_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
