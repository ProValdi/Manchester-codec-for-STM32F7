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
#include "stm32f7xx_hal.h"

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
#define LNA_LOW2_Pin GPIO_PIN_2
#define LNA_LOW2_GPIO_Port GPIOE
#define LNA_LOW1_Pin GPIO_PIN_3
#define LNA_LOW1_GPIO_Port GPIOE
#define VSUP_GEN_Pin GPIO_PIN_4
#define VSUP_GEN_GPIO_Port GPIOE
#define AMP_LOW_Pin GPIO_PIN_5
#define AMP_LOW_GPIO_Port GPIOE
#define TX_MANCHERSTER_Pin GPIO_PIN_6
#define TX_MANCHERSTER_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define CH_EN1_Pin GPIO_PIN_1
#define CH_EN1_GPIO_Port GPIOF
#define CH_EN2_Pin GPIO_PIN_2
#define CH_EN2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define ERR_Pin GPIO_PIN_12
#define ERR_GPIO_Port GPIOF
#define TMP_LED_TX_Pin GPIO_PIN_0
#define TMP_LED_TX_GPIO_Port GPIOG
#define DBG_ERR_UART_Pin GPIO_PIN_1
#define DBG_ERR_UART_GPIO_Port GPIOG
#define DBG_TX_Pin GPIO_PIN_7
#define DBG_TX_GPIO_Port GPIOE
#define DBG_RX_Pin GPIO_PIN_8
#define DBG_RX_GPIO_Port GPIOE
#define DBG_ERR_Pin GPIO_PIN_14
#define DBG_ERR_GPIO_Port GPIOE
#define DBG_UART_Pin GPIO_PIN_15
#define DBG_UART_GPIO_Port GPIOE
#define DBG_RXB11_Pin GPIO_PIN_11
#define DBG_RXB11_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LNA_HIGH1_Pin GPIO_PIN_10
#define LNA_HIGH1_GPIO_Port GPIOC
#define LNA_HIGH2_Pin GPIO_PIN_11
#define LNA_HIGH2_GPIO_Port GPIOC
#define VSUP_REC_Pin GPIO_PIN_2
#define VSUP_REC_GPIO_Port GPIOD
#define AMP_HIGH_Pin GPIO_PIN_11
#define AMP_HIGH_GPIO_Port GPIOG
#define GEN_HIGH_Pin GPIO_PIN_6
#define GEN_HIGH_GPIO_Port GPIOB
#define GEN_LOW_Pin GPIO_PIN_7
#define GEN_LOW_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_9
#define RX_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_1
#define TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
