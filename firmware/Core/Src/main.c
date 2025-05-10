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
#include <stdio.h>
#include <stdarg.h>  // va_list, va_start, va_end
#include <stdlib.h>  // strtof
#include <string.h>
#include "usbd_cdc_if.h"
#include "util.h"
#include "func_sys.h"
#include "func_rf.h"
#include "test_modes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VER     0.3

#define DEBUG_PRINT

// use USB CDC for printf()? If not set, UART2 will be used.
// side effect: an active USB connection is required for things to work
// as otherwise the TX ACK will never be generated and printf() will hang.
// not really a problem if your use case has a USB cable plugged in all the time.
#define PRINTF_USB_CDC

// For our custom cdcprintf() functionality
#define CDC_QUEUE_SIZE 2048
static char cdc_queue[CDC_QUEUE_SIZE] = {0};
static size_t cdc_queue_len = 0;

// Potential divider ratio calculations for Vsense
#define RATIO_5V   ((26.1 + 26.1) / 26.1)  // R107 and R108
#define RATIO_12V  ((75.0 + 26.1) / 26.1)  // R105 and R106
#define RATIO_24V  ((191.0 + 26.1) / 26.1) // R103 and R104

#define MAX_CMD_QUEUE 16
typedef struct {
    char* commands[MAX_CMD_QUEUE];
    uint8_t head;
    uint8_t tail;
} CommandQueue;

static CommandQueue cmd_queue = {0};

static int enqueue_command(const char* cmd) {
    uint8_t next_head = (cmd_queue.head + 1) % MAX_CMD_QUEUE;
    if(next_head == cmd_queue.tail) return -1; // Queue full
    
    cmd_queue.commands[cmd_queue.head] = malloc(strlen(cmd)+1);
    strcpy(cmd_queue.commands[cmd_queue.head], cmd);
    cmd_queue.head = next_head;
    return 0;
}

static char* dequeue_command(void) {
    if(cmd_queue.tail == cmd_queue.head) return NULL; // Queue empty
    char* cmd = cmd_queue.commands[cmd_queue.tail];
    cmd_queue.tail = (cmd_queue.tail + 1) % MAX_CMD_QUEUE;
    return cmd;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t EndOfADCConversion;
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
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int cdcprintf(const char* format, ...);
int dbgprintf(const char* format, ...);
void cdc_process_queue(void);
void resp_ok();
void resp_error();
void resp_bool(bool);
void resp_float(float);
void resp_param_error();
void queue_command(char*);
void process_command(char*);
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Start continuous ADC conversion
  HAL_ADC_Start_DMA(&hadc1, (uint32_t)&adc_data, 3);
  HAL_TIM_Base_Start(&htim3);

  // Start with a known RF setup
  rf_reset(RF_CH_A);
  rf_reset(RF_CH_B);

  // Open for business!
  led_on();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Process cdcprintf() output queue
    cdc_process_queue();

    // Commands that need to talk to talk SPI etc take a long time, causing
    // the CDC comms to break, so we have this queue to process them
    char* next_cmd = dequeue_command();
    if(next_cmd) {
        process_command(next_cmd);
        free(next_cmd);
    }

    // Update ADC values, should update every second with TIM3 parameters set
    if(EndOfADCConversion == 1) {
      vsense_5v = ((float)adc_data[0] / 4095.0) * 3.283 * RATIO_5V;
      vsense_12v = ((float)adc_data[1] / 4095.0) * 3.283 * RATIO_12V;
      vsense_24v = ((float)adc_data[2] / 4095.0) * 3.283 * RATIO_24V;
      EndOfADCConversion = 0;
    }

/*
    // Test CH A switch GPIO
    const uint32_t xdelay_ms = 500;
    // 1. Toggle PM_A_SW1_V1 (PB15)
    //HAL_GPIO_WritePin(PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(xdelay_ms);

    // 2. Toggle PM_A_SW1_V2 (PA8)
    //HAL_GPIO_WritePin(PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(xdelay_ms);

    // 3. Toggle PM_A_SW2_VCTL (PC6)
    //HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(xdelay_ms);

    // 4. Toggle PM_A_SW3_VCTL (PC7)
    //HAL_GPIO_WritePin(PM_A_SW3_VCTL_GPIO_Port, PM_A_SW3_VCTL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(xdelay_ms); // Delay after the last toggle before resetting

    // --- Reset All Pins ---
    // Set all specified pins back to LOW (Reset state)

    HAL_GPIO_WritePin(PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PM_A_SW3_VCTL_GPIO_Port, PM_A_SW3_VCTL_Pin, GPIO_PIN_RESET);
    HAL_Delay(xdelay_ms); // Delay after the last toggle before resetting
*/

    // Test 1 - RX Path, LNA Bypass, 0dB attenuation
    //test_mode_1(); HAL_Delay(3000);

    // Test 2 - RX Path, LNA Enable, 0dB attenuation
    //test_mode_2(); HAL_Delay(3000);

    // Test 3 - TX Path, PA Bypass
    //test_mode_3(); HAL_Delay(3000);

    // Test 4 - TX Path, PA Enable
    //test_mode_4(); HAL_Delay(3000);

    //  Test 5 - RX Path, Variable Attenuation
    //test_mode_5();

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, PWR_5V_EN_Pin|PWR_12V_EN_Pin|PM_A_SW2_VCTL_Pin|PM_A_SW3_VCTL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|PM_B_CONV_Pin|PM_A_SW1_V2_Pin|LNA_B_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PM_A_CONV_Pin|PM_B_SW1_V1_Pin|PM_B_SW1_V2_Pin|PM_B_SW2_VCTL_Pin
                          |PM_B_SW3_VCTL_Pin|PM_A_SW1_V1_Pin|TDD_MCU_B_Pin|TDD_MCU_A_Pin
                          |PWR_PORT1_EN_Pin|PWR_PORT2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PA_B_EN_Pin|LNA_A_EN_Pin|PA_A_EN_Pin|ATTEN_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PWR_5V_EN_Pin PWR_12V_EN_Pin PM_A_SW2_VCTL_Pin PM_A_SW3_VCTL_Pin */
  GPIO_InitStruct.Pin = PWR_5V_EN_Pin|PWR_12V_EN_Pin|PM_A_SW2_VCTL_Pin|PM_A_SW3_VCTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_12V_PG_Pin */
  GPIO_InitStruct.Pin = PWR_12V_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_12V_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin PM_B_CONV_Pin PM_A_SW1_V2_Pin LNA_B_EN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|PM_B_CONV_Pin|PM_A_SW1_V2_Pin|LNA_B_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PM_A_CONV_Pin PM_B_SW1_V1_Pin PM_B_SW1_V2_Pin PM_B_SW2_VCTL_Pin
                           PM_B_SW3_VCTL_Pin PM_A_SW1_V1_Pin TDD_MCU_B_Pin TDD_MCU_A_Pin
                           PWR_PORT1_EN_Pin PWR_PORT2_EN_Pin */
  GPIO_InitStruct.Pin = PM_A_CONV_Pin|PM_B_SW1_V1_Pin|PM_B_SW1_V2_Pin|PM_B_SW2_VCTL_Pin
                          |PM_B_SW3_VCTL_Pin|PM_A_SW1_V1_Pin|TDD_MCU_B_Pin|TDD_MCU_A_Pin
                          |PWR_PORT1_EN_Pin|PWR_PORT2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_B_EN_Pin LNA_A_EN_Pin PA_A_EN_Pin ATTEN_LE_Pin */
  GPIO_InitStruct.Pin = PA_B_EN_Pin|LNA_A_EN_Pin|PA_A_EN_Pin|ATTEN_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_5V_PG_Pin */
  GPIO_InitStruct.Pin = PWR_5V_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_5V_PG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Add formatted text to the CDC queue
int cdcprintf(const char* format, ...) {
    char temp[256];
    va_list args;
    int chars_written;

    va_start(args, format);
    chars_written = vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);

    // Check if there's enough space in the queue
    if (cdc_queue_len + chars_written < CDC_QUEUE_SIZE - 1) {
        strcat(cdc_queue + cdc_queue_len, temp);
        cdc_queue_len += chars_written;
    }

    return chars_written;
}

int dbgprintf(const char* format, ...) {
  int chars_written = 0;

#ifdef DEBUG_PRINT
  char temp[256];
  char timestamped_temp[280]; // Larger to accommodate timestamp and prefix
  va_list args;

  uint32_t timestamp_ms = HAL_GetTick(); // Get current timestamp in milliseconds

  va_start(args, format);
  chars_written = vsnprintf(temp, sizeof(temp), format, args);
  va_end(args);

  // Create the formatted string with timestamp prefix
  int prefix_len = snprintf(timestamped_temp, sizeof(timestamped_temp), 
                           "#[%lu] ", timestamp_ms);
  
  // Copy the original message after the prefix
  strncpy(timestamped_temp + prefix_len, temp, sizeof(timestamped_temp) - prefix_len);
  
  // Calculate total length including prefix
  int total_len = prefix_len + chars_written;
  
  // Check if there's enough space in the queue
  if (cdc_queue_len + total_len < CDC_QUEUE_SIZE - 1) {
      strcat(cdc_queue + cdc_queue_len, timestamped_temp);
      cdc_queue_len += total_len;
  }
#endif

  return chars_written;
}

// Process the CDC queue in your main loop
void cdc_process_queue(void) {
    // Maximum bytes to send in one transmission
    const size_t max_chunk_size = 64; // Adjust based on your CDC endpoint size

    if (cdc_queue_len > 0) {
        // First check if there's a newline within the first max_chunk_size characters
        size_t bytes_to_send = max_chunk_size;

        // Find the first newline in our potential chunk
        for (size_t i = 0; i < bytes_to_send && i < cdc_queue_len; i++) {
            if (cdc_queue[i] == '\n') {
                // Found a newline, send up to and including this newline
                bytes_to_send = i + 1;
                break;
            }
        }

        // If no newline found or beyond our chunk size, just use the max size
        if (bytes_to_send > cdc_queue_len) {
            bytes_to_send = cdc_queue_len;
        }

        // Only transmit if CDC is ready
        if (CDC_Transmit_FS((uint8_t*)cdc_queue, bytes_to_send) == USBD_OK) {
            // Shift remaining data to the beginning of the buffer
            if (bytes_to_send < cdc_queue_len) {
                memmove(cdc_queue, cdc_queue + bytes_to_send, cdc_queue_len - bytes_to_send);
            }

            cdc_queue_len -= bytes_to_send;
            cdc_queue[cdc_queue_len] = '\0'; // Ensure null termination
        }
    }
}

// printf() calls this
int _write(int file, char *ptr, int len) {
#ifdef PRINTF_USB_CDC
	// Just transmit, trying to be too clever here causes timeouts/problems...
	// THIS WILL NOT WORK WITH MULTIPLE NEWLINES - use cdcprintf() instead!
	CDC_Transmit_FS((uint8_t*)ptr, len);
#else
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
#endif

    return len;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	EndOfADCConversion = 1;
}

////////////////////////////////////////////

void resp_ok()
{
	cdcprintf("OK\n");
}

void resp_error()
{
	cdcprintf("ERROR\n");
}

void resp_int(int val)
{
	cdcprintf("%d\n", val);
}
void resp_uint16(uint16_t val)
{
cdcprintf("%u\n", (unsigned int)val);
}

void resp_bool(bool val)
{
	if (val == true){
		cdcprintf("TRUE\n");
	} else {
		cdcprintf("FALSE\n");
	}
}

void resp_float(float val)
{
	char buf[12];
    sprintf(buf, "%.2f", val);
    cdcprintf("%s\n", buf);
}

void resp_param_error()
{
    cdcprintf("ERROR:PARAM\n");
}

void queue_command(char *cmd)
{
    if (enqueue_command(cmd) != 0) {
        dbgprintf("ERROR:CMD_QUEUE\n");
    }
}

void process_command(char *cmd)
{
	dbgprintf("Received command: %s\n", cmd);

    // Tokenize on : , or \n
	char* tokens[4];
	char* token = strtok(cmd, ":,\n");

    int token_count = 0;
    while (token_count < 4 && token != NULL) {
        tokens[token_count++] = token;
        token = strtok(NULL, ":,\n");
    }

    dbgprintf("<tokenizer>\n");
    if (token_count == 0) {
      resp_param_error();
      dbgprintf("  no tokens!\n");
      return;
    } else {
        dbgprintf("  %d token(s)\n", token_count);
    }
    for (int i = 0; i < token_count; i++) {
        dbgprintf("  token %d: %s\n", i, tokens[i]);
    }
    dbgprintf("</tokenizer>\n");
    /////////////////////

    // Handle version command
    if (strcmp(tokens[0], "VERSION") == 0) {
        resp_float(FW_VER);
        return;
    }

    // Handle LED commands
    if (strcmp(tokens[0], "LED") == 0) {
        if (token_count < 2) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            resp_ok();
            led_on();
        } else if (strcmp(tokens[1], "OFF") == 0) {
            resp_ok();
        	led_off();
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle voltage sensing
    if (strcmp(tokens[0], "VSENSE") == 0) {
        if (token_count < 2) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "5V") == 0) {
            resp_float(vsense_5v);
        } else if (strcmp(tokens[1], "12V") == 0) {
            resp_float(vsense_12v);
        } else if (strcmp(tokens[1], "24V") == 0) {
            resp_float(vsense_24v);
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle power control
    if (strcmp(tokens[0], "PWR") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            if (strcmp(tokens[2], "5V") == 0) {
                resp_ok();
                pwr_5v_on();
            } else if (strcmp(tokens[2], "12V") == 0) {
                resp_ok();
                pwr_12v_on();
            } else {
                resp_param_error();
            }
        } else if (strcmp(tokens[1], "OFF") == 0) {
            if (strcmp(tokens[2], "5V") == 0) {
                resp_ok();
                pwr_5v_off();
            } else if (strcmp(tokens[2], "12V") == 0) {
                resp_ok();
                pwr_12v_off();
            } else {
                resp_param_error();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle relay control
    if (strcmp(tokens[0], "RELAY") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            if (strcmp(tokens[2], "1") == 0) {
                resp_ok();
                relay_port1_on();
            } else if (strcmp(tokens[2], "2") == 0) {
                resp_ok();
                relay_port2_on();
            } else {
                resp_param_error();
            }
        } else if (strcmp(tokens[1], "OFF") == 0) {
            if (strcmp(tokens[2], "1") == 0) {
                resp_ok();
                relay_port1_off();
            } else if (strcmp(tokens[2], "2") == 0) {
                resp_ok();
                relay_port2_off();
            } else {
                resp_param_error();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle LNA control
    if (strcmp(tokens[0], "LNA") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_lna(RF_CH_A, LNA_ACTIVE);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_lna(RF_CH_B, LNA_ACTIVE);
            } else {
                resp_param_error();
            }
        } else if (strcmp(tokens[1], "OFF") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_lna(RF_CH_A, LNA_BYPASS);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_lna(RF_CH_B, LNA_BYPASS);
            } else {
                resp_param_error();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle PA control
    if (strcmp(tokens[0], "PA") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_pa(RF_CH_A, PA_ACTIVE);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_pa(RF_CH_B, PA_ACTIVE);
            } else {
                resp_param_error();
            }
        } else if (strcmp(tokens[1], "OFF") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_pa(RF_CH_A, PA_BYPASS);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_pa(RF_CH_B, PA_BYPASS);
            } else {
                resp_param_error();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle TDD control
    if (strcmp(tokens[0], "TDD") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        if (strcmp(tokens[1], "ON") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_tdd_mode(RF_CH_A, RF_ACTIVE);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_tdd_mode(RF_CH_B, RF_ACTIVE);
            } else {
                resp_param_error();
            }
        } else if (strcmp(tokens[1], "OFF") == 0) {
            if (strcmp(tokens[2], "A") == 0) {
                resp_ok();
                set_tdd_mode(RF_CH_A, RF_INACTIVE);
            } else if (strcmp(tokens[2], "B") == 0) {
                resp_ok();
                set_tdd_mode(RF_CH_B, RF_INACTIVE);
            } else {
                resp_param_error();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // Handle RXATTEN commands
    if (strcmp(tokens[0], "RXATTEN") == 0) {
        dbgprintf(">> parsing RXATTEN\n");
        if (token_count < 3) {
            resp_param_error();
            return;
        }

        rf_channel_t channel;
        if (strcmp(tokens[1], "A") == 0) {
            channel = RF_CH_A;
        } else if (strcmp(tokens[1], "B") == 0) {
            channel = RF_CH_B;
        } else {
            resp_param_error();
            return;
        }

        char *endptr;
        float atten_value = strtof(tokens[2], &endptr);
        if (endptr == tokens[2] || *endptr != '\0') {
            resp_param_error();
            return;
        }

        resp_ok();

        set_rx_atten(channel, atten_value);

        dbgprintf(">> set RXATTEN %s %f\n", tokens[1], atten_value);
        return;
    }

    // Handle Reset
    if (strcmp(tokens[0], "RESET") == 0) {
    	if (token_count < 2) {
          resp_param_error();
          return;
      }

      rf_channel_t channel;
      if (strcmp(tokens[1], "A") == 0) {
          channel = RF_CH_A;
      } else if (strcmp(tokens[1], "B") == 0) {
          channel = RF_CH_B;
      } else {
          resp_param_error();
          return;
      }

      resp_ok();
      
      rf_reset(channel);

      return;
    }

    // Handle SW:<name>:<pos> command
    if (strcmp(tokens[0], "SW") == 0) {
        if (token_count < 3) {
            resp_param_error();
            return;
        }
        // Map switch name to switch object and type
        rf_switch_pos_t pos = SW_POS_NONE;
        if (strcmp(tokens[2], "J1") == 0) pos = SW_POS_1;
        else if (strcmp(tokens[2], "J2") == 0) pos = SW_POS_2;
        else if (strcmp(tokens[2], "J3") == 0) pos = SW_POS_3;
        else {
            resp_param_error();
            return;
        }

        // Switches: SW1A, SW1B, SW2A, SW2B, SW3A, SW3B, TDD_A, TDD_B
        if (strcmp(tokens[1], "SW1A") == 0) {
            set_sw_pos_sp3t(IC904, pos);
            resp_ok();
        } else if (strcmp(tokens[1], "SW1B") == 0) {
            set_sw_pos_sp3t(IC1004, pos);
            resp_ok();
        } else if (strcmp(tokens[1], "SW2A") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC905, pos);
                resp_ok();
            }
        } else if (strcmp(tokens[1], "SW2B") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC1005, pos);
                resp_ok();
            }
        } else if (strcmp(tokens[1], "SW3A") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC903, pos);
                resp_ok();
            }
        } else if (strcmp(tokens[1], "SW3B") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC1003, pos);
                resp_ok();
            }
        } else if (strcmp(tokens[1], "TDDA") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC1501_1502, pos);
                resp_ok();
            }
        } else if (strcmp(tokens[1], "TDDB") == 0) {
            if (pos == SW_POS_3 || pos == SW_POS_NONE) {
                resp_param_error();
            } else {
                set_sw_pos_spdt(IC1601_1602, pos);
                resp_ok();
            }
        } else {
            resp_param_error();
        }
        return;
    }

    // we shouldn't get here...
    cdcprintf("ERROR:CMD\n");
}

////////////////////////////////////////////
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
