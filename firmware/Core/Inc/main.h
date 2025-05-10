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
#define PWR_5V_EN_Pin GPIO_PIN_13
#define PWR_5V_EN_GPIO_Port GPIOC
#define PWR_12V_PG_Pin GPIO_PIN_14
#define PWR_12V_PG_GPIO_Port GPIOC
#define PWR_12V_EN_Pin GPIO_PIN_15
#define PWR_12V_EN_GPIO_Port GPIOC
#define SPI2_SCK_PM_B_SCK_Pin GPIO_PIN_0
#define SPI2_SCK_PM_B_SCK_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define USART2_TX_DEBUG_Pin GPIO_PIN_2
#define USART2_TX_DEBUG_GPIO_Port GPIOA
#define SPI2_MISO_PM_B_SDO_Pin GPIO_PIN_3
#define SPI2_MISO_PM_B_SDO_GPIO_Port GPIOA
#define SPI1_SCK_PM_A_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_PM_A_SCK_GPIO_Port GPIOA
#define SPI1_MISO_PM_A_SDO_Pin GPIO_PIN_6
#define SPI1_MISO_PM_A_SDO_GPIO_Port GPIOA
#define PM_B_CONV_Pin GPIO_PIN_7
#define PM_B_CONV_GPIO_Port GPIOA
#define PM_A_CONV_Pin GPIO_PIN_0
#define PM_A_CONV_GPIO_Port GPIOB
#define ADC1_5V_SENSE_Pin GPIO_PIN_1
#define ADC1_5V_SENSE_GPIO_Port GPIOB
#define ADC1_12V_SENSE_Pin GPIO_PIN_2
#define ADC1_12V_SENSE_GPIO_Port GPIOB
#define ADC1_24V_SENSE_Pin GPIO_PIN_10
#define ADC1_24V_SENSE_GPIO_Port GPIOB
#define PM_B_SW1_V1_Pin GPIO_PIN_11
#define PM_B_SW1_V1_GPIO_Port GPIOB
#define PM_B_SW1_V2_Pin GPIO_PIN_12
#define PM_B_SW1_V2_GPIO_Port GPIOB
#define PM_B_SW2_VCTL_Pin GPIO_PIN_13
#define PM_B_SW2_VCTL_GPIO_Port GPIOB
#define PM_B_SW3_VCTL_Pin GPIO_PIN_14
#define PM_B_SW3_VCTL_GPIO_Port GPIOB
#define PM_A_SW1_V1_Pin GPIO_PIN_15
#define PM_A_SW1_V1_GPIO_Port GPIOB
#define PM_A_SW1_V2_Pin GPIO_PIN_8
#define PM_A_SW1_V2_GPIO_Port GPIOA
#define PM_A_SW2_VCTL_Pin GPIO_PIN_6
#define PM_A_SW2_VCTL_GPIO_Port GPIOC
#define PM_A_SW3_VCTL_Pin GPIO_PIN_7
#define PM_A_SW3_VCTL_GPIO_Port GPIOC
#define LNA_B_EN_Pin GPIO_PIN_15
#define LNA_B_EN_GPIO_Port GPIOA
#define PA_B_EN_Pin GPIO_PIN_0
#define PA_B_EN_GPIO_Port GPIOD
#define LNA_A_EN_Pin GPIO_PIN_1
#define LNA_A_EN_GPIO_Port GPIOD
#define PA_A_EN_Pin GPIO_PIN_2
#define PA_A_EN_GPIO_Port GPIOD
#define ATTEN_LE_Pin GPIO_PIN_3
#define ATTEN_LE_GPIO_Port GPIOD
#define SPI3_SCK_ATTEN_SCK_Pin GPIO_PIN_3
#define SPI3_SCK_ATTEN_SCK_GPIO_Port GPIOB
#define TDD_MCU_B_Pin GPIO_PIN_4
#define TDD_MCU_B_GPIO_Port GPIOB
#define SPI3_MOSI_ATTEN_MOSI_Pin GPIO_PIN_5
#define SPI3_MOSI_ATTEN_MOSI_GPIO_Port GPIOB
#define TDD_MCU_A_Pin GPIO_PIN_6
#define TDD_MCU_A_GPIO_Port GPIOB
#define PWR_PORT1_EN_Pin GPIO_PIN_7
#define PWR_PORT1_EN_GPIO_Port GPIOB
#define PWR_PORT2_EN_Pin GPIO_PIN_8
#define PWR_PORT2_EN_GPIO_Port GPIOB
#define PWR_5V_PG_Pin GPIO_PIN_9
#define PWR_5V_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum { false, true } bool;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
