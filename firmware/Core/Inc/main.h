/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define GPIO_5V_EN_Pin GPIO_PIN_13
#define GPIO_5V_EN_GPIO_Port GPIOC
#define GPIO_12V_PG_Pin GPIO_PIN_14
#define GPIO_12V_PG_GPIO_Port GPIOC
#define GPIO_12V_EN_Pin GPIO_PIN_15
#define GPIO_12V_EN_GPIO_Port GPIOC
#define GPIO_PM_B_SCK_Pin GPIO_PIN_0
#define GPIO_PM_B_SCK_GPIO_Port GPIOA
#define GPIO_LED_Pin GPIO_PIN_1
#define GPIO_LED_GPIO_Port GPIOA
#define GPIO_PM_B_MISO_Pin GPIO_PIN_4
#define GPIO_PM_B_MISO_GPIO_Port GPIOA
#define GPIO_PM_A_SCK_Pin GPIO_PIN_5
#define GPIO_PM_A_SCK_GPIO_Port GPIOA
#define GPIO_PM_B_CONV_Pin GPIO_PIN_6
#define GPIO_PM_B_CONV_GPIO_Port GPIOA
#define GPIO_PM_A_MISO_Pin GPIO_PIN_7
#define GPIO_PM_A_MISO_GPIO_Port GPIOA
#define GPIO_PM_A_CONV_Pin GPIO_PIN_0
#define GPIO_PM_A_CONV_GPIO_Port GPIOB
#define ADC1_5V_SENSE_Pin GPIO_PIN_1
#define ADC1_5V_SENSE_GPIO_Port GPIOB
#define ADC1_12V_SENSE_Pin GPIO_PIN_2
#define ADC1_12V_SENSE_GPIO_Port GPIOB
#define ADC1_24V_SENSE_Pin GPIO_PIN_10
#define ADC1_24V_SENSE_GPIO_Port GPIOB
#define GPIO_PM_B_CTRL_1_Pin GPIO_PIN_11
#define GPIO_PM_B_CTRL_1_GPIO_Port GPIOB
#define GPIO_PM_B_CTRL_2_Pin GPIO_PIN_12
#define GPIO_PM_B_CTRL_2_GPIO_Port GPIOB
#define GPIO_PM_B_CTRL_3_Pin GPIO_PIN_13
#define GPIO_PM_B_CTRL_3_GPIO_Port GPIOB
#define GPIO_PM_B_CTRL_4_Pin GPIO_PIN_14
#define GPIO_PM_B_CTRL_4_GPIO_Port GPIOB
#define GPIO_PM_A_CTRL_1_Pin GPIO_PIN_15
#define GPIO_PM_A_CTRL_1_GPIO_Port GPIOB
#define GPIO_PM_A_CTRL_2_Pin GPIO_PIN_8
#define GPIO_PM_A_CTRL_2_GPIO_Port GPIOA
#define GPIO_PM_A_CTRL_3_Pin GPIO_PIN_6
#define GPIO_PM_A_CTRL_3_GPIO_Port GPIOC
#define GPIO_PM_A_CTRL_4_Pin GPIO_PIN_7
#define GPIO_PM_A_CTRL_4_GPIO_Port GPIOC
#define GPIO_LNA_B_EN_Pin GPIO_PIN_15
#define GPIO_LNA_B_EN_GPIO_Port GPIOA
#define GPIO_PA_B_EN_Pin GPIO_PIN_0
#define GPIO_PA_B_EN_GPIO_Port GPIOD
#define GPIO_LNA_A_EN_Pin GPIO_PIN_1
#define GPIO_LNA_A_EN_GPIO_Port GPIOD
#define GPIO_PA_A_EN_Pin GPIO_PIN_2
#define GPIO_PA_A_EN_GPIO_Port GPIOD
#define GPIO_ATTEN_LE_Pin GPIO_PIN_3
#define GPIO_ATTEN_LE_GPIO_Port GPIOD
#define SPI3_ATTEN_SCK_Pin GPIO_PIN_3
#define SPI3_ATTEN_SCK_GPIO_Port GPIOB
#define GPIO_TDD_CTRL_B_Pin GPIO_PIN_4
#define GPIO_TDD_CTRL_B_GPIO_Port GPIOB
#define SPI3_ATTEN_MOSI_Pin GPIO_PIN_5
#define SPI3_ATTEN_MOSI_GPIO_Port GPIOB
#define GPIO_TDD_CTRL_A_Pin GPIO_PIN_6
#define GPIO_TDD_CTRL_A_GPIO_Port GPIOB
#define GPIO_PORT1_EN_Pin GPIO_PIN_7
#define GPIO_PORT1_EN_GPIO_Port GPIOB
#define GPIO_PORT2_EN_Pin GPIO_PIN_8
#define GPIO_PORT2_EN_GPIO_Port GPIOB
#define GPIO_5V_PG_Pin GPIO_PIN_9
#define GPIO_5V_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum { false, true } bool;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
