/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// XXX this needs to be tidied up...

#include <stdio.h>
#include <string.h>

#include "usbd_cdc_if.h"

#include "util.h"
#include "func_sys.h"
#include "func_rf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VER     0.2

// use USB CDC for printf()? If not set, UART2 will be used.
// side effect: an active USB connection is required for things to work
// as otherwise the TX ACK will never be generated and printf() will hang.
// not really a problem if your use case has a USB cable plugged in all the time.
#define PRINTF_USB_CDC

// Potential divider ratio calculations for Vsense
#define RATIO_5V   ((26.1 + 26.1) / 26.1)  // R107 and R108
#define RATIO_12V  ((75.0 + 26.1) / 26.1) // R105 and R106
#define RATIO_24V  ((191.0 + 26.1) / 26.1) // R103 and R104
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t EndOfConversion;
uint16_t adc_data[3];

float vsense_5v, vsense_12v, vsense_24v;

extern rf_sw_sp3t IC904, IC1004;
extern rf_sw_spdt IC903, IC905, IC1003, IC1005, IC1501_1502, IC1601_1602;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void _float_to_string(float, char*);
void resp_ok();
void resp_error();
void resp_bool(bool);
void resp_float(float);
void handle_command(char*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //setbuf(stdout, NULL); // no need for a \n for printf()
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_USB_Device_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t)&adc_data, 3);
  HAL_TIM_Base_Start(&htim3);
  led_on();

  // XXX: TDD operation is not implemented yet, so for now we just ignore it
  set_sw_pos_spdt(IC1501_1502, SW_POS_2);
  set_sw_pos_spdt(IC1601_1602, SW_POS_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(EndOfConversion == 1)
	  {
		  // this should run every second with TIM3 parameters set
		  vsense_5v = ((float)adc_data[0] / 4095.0) * 3.283 * RATIO_5V;
		  vsense_12v = ((float)adc_data[1] / 4095.0) * 3.283 * RATIO_12V;
		  vsense_24v = ((float)adc_data[2] / 4095.0) * 3.283 * RATIO_24V;
		  EndOfConversion = 0;
	  }

	  //////////////
	  // Test Cases
	  //////////////

	  ////// SDR TX -> PA -> TRX IO
/*
	  //// PM block
	  // sw1: j2
	  set_sw_pos_sp3t(IC904, SW_POS_2);
	  set_sw_pos_sp3t(IC1004, SW_POS_2);
      // sw2: don't care
	  set_sw_pos_spdt(IC905, SW_POS_1);
	  set_sw_pos_spdt(IC1005, SW_POS_1);
      // sw3@ don't care
	  set_sw_pos_spdt(IC903, SW_POS_1);
	  set_sw_pos_spdt(IC1003, SW_POS_1);

	  //// TDD block
	  set_sw_pos_spdt(IC1501_1502, SW_POS_2);
	  set_sw_pos_spdt(IC1601_1602, SW_POS_2);
*/

	  ////// SDR TX -> Power Meter
/*
	  //// PM block
	  // sw1: j3
	  set_sw_pos_sp3t(IC904, SW_POS_3);
	  set_sw_pos_sp3t(IC1004, SW_POS_3);
      // sw2: don't care
	  set_sw_pos_spdt(IC905, SW_POS_1);
	  set_sw_pos_spdt(IC1005, SW_POS_1);
      // sw3: j2
	  set_sw_pos_spdt(IC903, SW_POS_2);
	  set_sw_pos_spdt(IC903, SW_POS_2);
*/
/*
	  setRFPowerMeter(RF_CH_A, POWER_METER_SDR);
	  setRFPowerMeter(RF_CH_B, POWER_METER_SDR);

	  uint16_t r = getRFPowerLevelRawBitbang(RF_CH_A);
	  printf("SDR CH A Result: %d\r\n", r);
	  r = getRFPowerLevelRawBitbang(RF_CH_B);
	  printf("SDR CH B Result: %d\r\n----------------\r\n", r);
*/

	  ////// Ext u.FL -> Power Meter
/*
	  //// PM block
	  // sw1: don't care - j2: SDR_TX -> TX_OUT
	  set_sw_pos_sp3t(IC904, SW_POS_2);
	  set_sw_pos_sp3t(IC1004, SW_POS_2);
      // sw2: don't care - j2: RX_IN -> SDR_RX
	  set_sw_pos_spdt(IC905, SW_POS_1);
	  set_sw_pos_spdt(IC1005, SW_POS_1);
      // sw3: j1
	  set_sw_pos_spdt(IC903, SW_POS_1);
	  set_sw_pos_spdt(IC903, SW_POS_1);

	  uint16_t r = getRFPowerLevelRawBitbang(RF_CH_A);
	  printf("Ext CH A Result: %d\r\n", r);
	  r = getRFPowerLevelRawBitbang(RF_CH_B);
	  printf("Ext CH B Result: %d\r\n----------------\r\n", r);
*/

	  ////// RX A IN -> RX A OUT (SDR) with attenuator
/*
	  //// TDD block
	  set_sw_pos_spdt(IC1501_1502, SW_POS_2);
	  set_sw_pos_spdt(IC1601_1602, SW_POS_2);
	  //// PM block
	  // sw1: don't care - j2: SDR_TX -> TX_OUT
	  set_sw_pos_sp3t(IC904, SW_POS_2);
	  set_sw_pos_sp3t(IC1004, SW_POS_2);
      // sw2: j2: RX_IN -> SDR_RX
	  set_sw_pos_spdt(IC905, SW_POS_2);
	  set_sw_pos_spdt(IC1005, SW_POS_2);
      // sw3: don't care
	  set_sw_pos_spdt(IC903, SW_POS_1);
	  set_sw_pos_spdt(IC903, SW_POS_1);
	  //// Attenuator
	  setRxAttenuation(RF_CH_A, 20);
	  setRxAttenuation(RF_CH_B, 20);
*/

	  //HAL_Delay(2000);

	  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_5V_EN_Pin|GPIO_12V_EN_Pin|GPIO_PM_A_CTRL_3_Pin|GPIO_PM_A_CTRL_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PM_B_SCK_Pin|GPIO_LED_Pin|GPIO_PM_A_SCK_Pin|GPIO_PM_B_CONV_Pin
                          |GPIO_PM_A_CTRL_2_Pin|GPIO_LNA_B_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PM_A_CONV_Pin|GPIO_PM_B_CTRL_1_Pin|GPIO_PM_B_CTRL_2_Pin|GPIO_PM_B_CTRL_3_Pin
                          |GPIO_PM_B_CTRL_4_Pin|GPIO_PM_A_CTRL_1_Pin|GPIO_TDD_CTRL_B_Pin|GPIO_TDD_CTRL_A_Pin
                          |GPIO_PORT1_EN_Pin|GPIO_PORT2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PA_B_EN_Pin|GPIO_LNA_A_EN_Pin|GPIO_PA_A_EN_Pin|GPIO_ATTEN_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_5V_EN_Pin GPIO_12V_EN_Pin GPIO_PM_A_CTRL_3_Pin GPIO_PM_A_CTRL_4_Pin */
  GPIO_InitStruct.Pin = GPIO_5V_EN_Pin|GPIO_12V_EN_Pin|GPIO_PM_A_CTRL_3_Pin|GPIO_PM_A_CTRL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_12V_PG_Pin */
  GPIO_InitStruct.Pin = GPIO_12V_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_12V_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PM_B_SCK_Pin GPIO_LED_Pin GPIO_PM_A_SCK_Pin GPIO_PM_B_CONV_Pin
                           GPIO_PM_A_CTRL_2_Pin GPIO_LNA_B_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PM_B_SCK_Pin|GPIO_LED_Pin|GPIO_PM_A_SCK_Pin|GPIO_PM_B_CONV_Pin
                          |GPIO_PM_A_CTRL_2_Pin|GPIO_LNA_B_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PM_B_MISO_Pin GPIO_PM_A_MISO_Pin */
  GPIO_InitStruct.Pin = GPIO_PM_B_MISO_Pin|GPIO_PM_A_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PM_A_CONV_Pin GPIO_PM_B_CTRL_1_Pin GPIO_PM_B_CTRL_2_Pin GPIO_PM_B_CTRL_3_Pin
                           GPIO_PM_B_CTRL_4_Pin GPIO_PM_A_CTRL_1_Pin GPIO_TDD_CTRL_B_Pin GPIO_TDD_CTRL_A_Pin
                           GPIO_PORT1_EN_Pin GPIO_PORT2_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PM_A_CONV_Pin|GPIO_PM_B_CTRL_1_Pin|GPIO_PM_B_CTRL_2_Pin|GPIO_PM_B_CTRL_3_Pin
                          |GPIO_PM_B_CTRL_4_Pin|GPIO_PM_A_CTRL_1_Pin|GPIO_TDD_CTRL_B_Pin|GPIO_TDD_CTRL_A_Pin
                          |GPIO_PORT1_EN_Pin|GPIO_PORT2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PA_B_EN_Pin GPIO_LNA_A_EN_Pin GPIO_PA_A_EN_Pin GPIO_ATTEN_LE_Pin */
  GPIO_InitStruct.Pin = GPIO_PA_B_EN_Pin|GPIO_LNA_A_EN_Pin|GPIO_PA_A_EN_Pin|GPIO_ATTEN_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_5V_PG_Pin */
  GPIO_InitStruct.Pin = GPIO_5V_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_5V_PG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
#ifdef PRINTF_USB_CDC
    while(CDC_Transmit_FS((uint8_t*)ptr, len) != USBD_OK); // this doesn't work?
	//CDC_Transmit_FS((uint8_t*)ptr, len); // not all characters are printed with this?
#else
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
#endif

    return len;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	EndOfConversion = 1;
}


void resp_ok()
{
	printf("OK\n");
}

void resp_error()
{
	printf("ERROR\n");
}

void resp_int(int val)
{
  printf("%d\n", val);
}

void resp_uint16(uint16_t val)
{
  printf("%u\n", (unsigned int)val);
}

void resp_bool(bool val)
{
	if (val == true){
		printf("TRUE\n");
	} else {
		printf("FALSE\n");
	}
}

void resp_float(float val)
{
	char buf[8];
    _float_to_string(val, buf);
    printf("%s\n", buf);
}

void handle_command(char *cmd)
{
  // Check for commands with parameters
  // only one for now: RXATTEN_{channel}_{value}
  const char *rxatten_prefix = "RXATTEN_";
  
  if (strncmp(cmd, rxatten_prefix, strlen(rxatten_prefix)) == 0) {
    // Find the position of the first underscore after the prefix
    char *underscore_pos = strchr(cmd + strlen(rxatten_prefix), '_');
    if (underscore_pos != NULL) {
      // Extract and check the channel character
      char channel = *(underscore_pos - 1);
      if (channel != 'A' && channel != 'B') {
        // Invalid channel character
        printf("ERROR:CHN\n");
        return;
      }
      // Attempt to extract a float value from the end of the command
      float float_value;
      if (sscanf(underscore_pos + 1, "%f", &float_value) == 1) {
        printf("c: %c,v: %.2f\n", channel, float_value);
      } else {
        // Failed to extract a numeric value from the command.
        printf("ERROR:VAL\n");
      }
    } else {
      // Failed to find the channel and value separator underscore.
      printf("ERROR:FMT\n");
    }
    return;
  }

  // Check for commands without parameters
  if (strcmp(cmd, "VERSION") == 0) {
    resp_float(FW_VER);  
  } else if (strcmp(cmd, "LED_ON") == 0) {
    led_on();
    resp_ok();
  } else if (strcmp(cmd, "LED_OFF") == 0) {
    led_off();
    resp_ok();
  } else if (strcmp(cmd, "VSENSE_5V") == 0) {
    resp_float(vsense_5v);
  } else if (strcmp(cmd, "VSENSE_12V") == 0) {
    resp_float(vsense_12v);
  } else if (strcmp(cmd, "VSENSE_24V") == 0) {
    resp_float(vsense_24v);
  } else if (strcmp(cmd, "5V_ON") == 0) {
    pwr_5v_on();
    resp_ok();
  } else if (strcmp(cmd, "12V_ON") == 0) {
    pwr_12v_on();
    resp_ok();
  } else if (strcmp(cmd, "5V_OFF") == 0) {
    pwr_5v_off();
    resp_ok();
  } else if (strcmp(cmd, "12V_OFF") == 0) {
    pwr_12v_off();
    resp_ok();
  } else if (strcmp(cmd, "5V_PG") == 0) {
    if (pwr_5v_pg()){
      resp_ok();
    } else {
      resp_error();
    }
  } else if (strcmp(cmd, "12V_PG") == 0) {
    if (pwr_12v_pg()){
      resp_ok();
    } else {
      resp_error();
    }
  } else if (strcmp(cmd, "RELAY1_ON") == 0) {
    relay_port1_on();
    resp_ok();
  } else if (strcmp(cmd, "RELAY1_OFF") == 0) {
    relay_port1_off();
    resp_ok();
  } else if (strcmp(cmd, "RELAY2_ON") == 0) {
    relay_port2_on();
    resp_ok();
  } else if (strcmp(cmd, "RELAY2_OFF") == 0) {
    relay_port2_off();
    resp_ok();
  } else if (strcmp(cmd, "LNA_A_ACTIVE") == 0) {
    setLNA(RF_CH_A, LNA_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "LNA_A_BYPASS") == 0) {
    setLNA(RF_CH_A, LNA_BYPASS);
    resp_ok();
  } else if (strcmp(cmd, "LNA_B_ACTIVE") == 0) {
    setLNA(RF_CH_B, LNA_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "LNA_B_BYPASS") == 0) {
    setLNA(RF_CH_B, LNA_BYPASS);
    resp_ok();
  } else if (strcmp(cmd, "PA_A_ACTIVE") == 0) {
    setPA(RF_CH_A, PA_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "PA_A_BYPASS") == 0) {
    setPA(RF_CH_A, PA_BYPASS);
    resp_ok();
  } else if (strcmp(cmd, "PA_B_ACTIVE") == 0) {
    setPA(RF_CH_B, PA_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "PA_B_BYPASS") == 0) {
    setPA(RF_CH_B, PA_BYPASS);
    resp_ok();
  } else if (strcmp(cmd, "TXINHIBIT_A_ACTIVE") == 0) {
    setTxInhibit(RF_CH_A, RF_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXINHIBIT_A_INACTIVE") == 0) {
    setTxInhibit(RF_CH_A, RF_INACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXINHIBIT_B_ACTIVE") == 0) {
    setTxInhibit(RF_CH_B, RF_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXINHIBIT_B_INACTIVE") == 0) {
    setTxInhibit(RF_CH_B, RF_INACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXRXLOOP_A_ACTIVE") == 0) {
    setTxRxLoopback(RF_CH_A, RF_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXRXLOOP_A_INACTIVE") == 0) {
    setTxRxLoopback(RF_CH_A, RF_INACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXRXLOOP_B_ACTIVE") == 0) {
    setTxRxLoopback(RF_CH_B, RF_ACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "TXRXLOOP_B_INACTIVE") == 0) {
    setTxRxLoopback(RF_CH_B, RF_INACTIVE);
    resp_ok();
  } else if (strcmp(cmd, "PWRLEVEL_A_READ") == 0) {
    uint16_t pwrlevel = getRFPowerLevelRawBitbang(RF_CH_A);
	resp_uint16(pwrlevel);
  } else if (strcmp(cmd, "PWRLEVEL_B_READ") == 0) {
	uint16_t pwrlevel = getRFPowerLevelRawBitbang(RF_CH_B);
	resp_uint16(pwrlevel);
  } else if (strcmp(cmd, "PWRMEAS_A_OFF") == 0) {
    setRFPowerMeter(RF_CH_A, POWER_METER_OFF);
    resp_ok();
  } else if (strcmp(cmd, "PWRMEAS_A_SDR") == 0) {
    setRFPowerMeter(RF_CH_A, POWER_METER_SDR);
    resp_ok();
  } else if (strcmp(cmd, "PWRMEAS_A_EXT") == 0) {
    setRFPowerMeter(RF_CH_A, POWER_METER_EXT);
    resp_ok();
  } else if (strcmp(cmd, "PWRMEAS_B_OFF") == 0) {
    setRFPowerMeter(RF_CH_B, POWER_METER_OFF);
    resp_ok();
  } else if (strcmp(cmd, "PWRMEAS_B_SDR") == 0) {
    setRFPowerMeter(RF_CH_B, POWER_METER_SDR);
    resp_ok();
  } else if (strcmp(cmd, "PWRMEAS_B_EXT") == 0) {
    setRFPowerMeter(RF_CH_B, POWER_METER_EXT);
    resp_ok();
  } else if (strcmp(cmd, "RESET_A") == 0) {
    rf_reset(RF_CH_A);
    resp_ok();
  } else if (strcmp(cmd, "RESET_B") == 0) {
    rf_reset(RF_CH_B);
    resp_ok();
  } else {
    //printf("Unknown command: %s\n", cmd);
    printf("ERROR:CMD\n");
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
