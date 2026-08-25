/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"

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
#define INA1_Pin LL_GPIO_PIN_12
#define INA1_GPIO_Port GPIOB
#define INA2_Pin LL_GPIO_PIN_13
#define INA2_GPIO_Port GPIOB
#define INB2_Pin LL_GPIO_PIN_14
#define INB2_GPIO_Port GPIOB
#define INB1_Pin LL_GPIO_PIN_15
#define INB1_GPIO_Port GPIOB
#define LA_Pin LL_GPIO_PIN_6
#define LA_GPIO_Port GPIOC
#define LB_Pin LL_GPIO_PIN_7
#define LB_GPIO_Port GPIOC
#define PWMA_Pin LL_GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin LL_GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define RB_Pin LL_GPIO_PIN_3
#define RB_GPIO_Port GPIOD
#define RA_Pin LL_GPIO_PIN_4
#define RA_GPIO_Port GPIOD
#define SPI1_CS_Pin LL_GPIO_PIN_7
#define SPI1_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
