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

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define ENC_1Y_Pin LL_GPIO_PIN_2
#define ENC_1Y_GPIO_Port GPIOE
#define ENC_2Y_Pin LL_GPIO_PIN_3
#define ENC_2Y_GPIO_Port GPIOE
#define L0_Pin LL_GPIO_PIN_14
#define L0_GPIO_Port GPIOC
#define L1_Pin LL_GPIO_PIN_15
#define L1_GPIO_Port GPIOC
#define L2_Pin LL_GPIO_PIN_9
#define L2_GPIO_Port GPIOF
#define L3_Pin LL_GPIO_PIN_10
#define L3_GPIO_Port GPIOF
#define L4_Pin LL_GPIO_PIN_1
#define L4_GPIO_Port GPIOF
#define L5_Pin LL_GPIO_PIN_0
#define L5_GPIO_Port GPIOC
#define L6_Pin LL_GPIO_PIN_1
#define L6_GPIO_Port GPIOC
#define L7_Pin LL_GPIO_PIN_2
#define L7_GPIO_Port GPIOC
#define Q5_Pin LL_GPIO_PIN_3
#define Q5_GPIO_Port GPIOC
#define Q0_Pin LL_GPIO_PIN_0
#define Q0_GPIO_Port GPIOA
#define Q1_Pin LL_GPIO_PIN_1
#define Q1_GPIO_Port GPIOA
#define Q2_Pin LL_GPIO_PIN_2
#define Q2_GPIO_Port GPIOA
#define Q3_Pin LL_GPIO_PIN_3
#define Q3_GPIO_Port GPIOA
#define Q13_Pin LL_GPIO_PIN_5
#define Q13_GPIO_Port GPIOA
#define Q8_Pin LL_GPIO_PIN_6
#define Q8_GPIO_Port GPIOA
#define Q9_Pin LL_GPIO_PIN_7
#define Q9_GPIO_Port GPIOA
#define Q10_Pin LL_GPIO_PIN_4
#define Q10_GPIO_Port GPIOC
#define Q11_Pin LL_GPIO_PIN_5
#define Q11_GPIO_Port GPIOC
#define Q7_Pin LL_GPIO_PIN_1
#define Q7_GPIO_Port GPIOB
#define Q12_Pin LL_GPIO_PIN_2
#define Q12_GPIO_Port GPIOB
#define L_PWM_Pin LL_GPIO_PIN_11
#define L_PWM_GPIO_Port GPIOE
#define L_DIR_Pin LL_GPIO_PIN_12
#define L_DIR_GPIO_Port GPIOE
#define R_PWM_Pin LL_GPIO_PIN_13
#define R_PWM_GPIO_Port GPIOE
#define R_DIR_Pin LL_GPIO_PIN_14
#define R_DIR_GPIO_Port GPIOE
#define Q14_Pin LL_GPIO_PIN_11
#define Q14_GPIO_Port GPIOB
#define Q6_Pin LL_GPIO_PIN_12
#define Q6_GPIO_Port GPIOB
#define Q4_Pin LL_GPIO_PIN_14
#define Q4_GPIO_Port GPIOB
#define Q15_Pin LL_GPIO_PIN_15
#define Q15_GPIO_Port GPIOB
#define SW_R_Pin LL_GPIO_PIN_15
#define SW_R_GPIO_Port GPIOD
#define SW_L_Pin LL_GPIO_PIN_6
#define SW_L_GPIO_Port GPIOC
#define SW_D_Pin LL_GPIO_PIN_7
#define SW_D_GPIO_Port GPIOC
#define SW_U_Pin LL_GPIO_PIN_8
#define SW_U_GPIO_Port GPIOC
#define ENC_3Y_Pin LL_GPIO_PIN_11
#define ENC_3Y_GPIO_Port GPIOA
#define ENC_4Y_Pin LL_GPIO_PIN_12
#define ENC_4Y_GPIO_Port GPIOA
#define SPI1_CS_Pin LL_GPIO_PIN_7
#define SPI1_CS_GPIO_Port GPIOD
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
