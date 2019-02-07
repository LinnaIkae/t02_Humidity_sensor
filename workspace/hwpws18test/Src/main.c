
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tm_stm32_hd44780.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId auxTaskHandle;
uint32_t auxTaskFxnBuffer[ 512 ];
osStaticThreadDef_t auxTaskFxnControlBlock;
osMessageQId defaultQueueHandle;
uint8_t defaultQueueBuffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t defaultQueueControlBlock;
osMessageQId auxQueueHandle;
uint8_t auxQueueBuffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t auxQueueControlBlock;
osTimerId timer01Handle;
osStaticTimerDef_t timer01ControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void defaultTaskFxn(void const * argument);
void auxTaskFxn(void const * argument);
void timer01Callback(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

struct Option{
	char name[8];
	uint32_t value;
	uint32_t incr;
	uint32_t min;
	uint32_t max;
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of timer01 */
  osTimerStaticDef(timer01, timer01Callback, &timer01ControlBlock);
  timer01Handle = osTimerCreate(osTimer(timer01), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, defaultTaskFxn, osPriorityNormal, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of auxTask */
  osThreadStaticDef(auxTask, auxTaskFxn, osPriorityNormal, 0, 512, auxTaskFxnBuffer, &auxTaskFxnControlBlock);
  auxTaskHandle = osThreadCreate(osThread(auxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of defaultQueue */
  osMessageQStaticDef(defaultQueue, 4, uint16_t, defaultQueueBuffer, &defaultQueueControlBlock);
  defaultQueueHandle = osMessageCreate(osMessageQ(defaultQueue), NULL);

  /* definition and creation of auxQueue */
  osMessageQStaticDef(auxQueue, 4, uint16_t, auxQueueBuffer, &auxQueueControlBlock);
  auxQueueHandle = osMessageCreate(osMessageQ(auxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 190;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 55;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 8000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D3_Pin|D4_Pin|D5_Pin|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D2_Pin|D0_Pin|GPIO_PIN_7|RS_Pin 
                          |LCD_EN_Pin|sensor_enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_Pin|D7_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D3_Pin */
  GPIO_InitStruct.Pin = D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin PA9 PA10 
                           PA11 PA12 */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D0_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : rotary_left_Pin btn0_Pin btn1_Pin */
  GPIO_InitStruct.Pin = rotary_left_Pin|btn0_Pin|btn1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 RS_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|RS_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : rotaty_right_Pin */
  GPIO_InitStruct.Pin = rotaty_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(rotaty_right_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : sensor_enable_Pin */
  GPIO_InitStruct.Pin = sensor_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(sensor_enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D1_Pin */
  GPIO_InitStruct.Pin = D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

uint8_t db_button1_block = 0;
uint8_t db_button2_block = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t token = pdFALSE;
	uint16_t data = GPIO_Pin;
	xQueueSendToBackFromISR(auxQueueHandle, &data, &token);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_defaultTaskFxn */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_defaultTaskFxn */
void defaultTaskFxn(void const * argument)
{

  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	/*
	 */
#define NUM_OPT 4

	uint16_t data = 0;
	uint32_t contrast = 992; //contrast 0-4096
	uint32_t brightness = 4000;
	uint32_t sens_pwm_counter;
	uint32_t sens_pwm_compare;
	uint32_t hum = 0;
	uint32_t hum_l = 0;
	uint32_t hum_r = 0;
	uint32_t temp= 0;
	uint32_t temp_l= 0;
	uint32_t temp_r= 0;
	uint32_t temp_mV;

	//Option = {"name", value, incr, min, max}
	struct Option opt_list[NUM_OPT] = {
			{"brightn", 4000, 100, 0, 4096},
			{"contrst", 992, 100, 0, 4096},
			{"s pwm f", 250000, 5000, 200000, 500000}, //sensor pwm frequency
			{"s pwm D", 15, 2, 0, 100} //sensor pwm duty cycle 0-100%
	};

	//mode = 0: show humidity and temp
	//mode = 1: options
	uint8_t mode = 0;
	uint8_t opt_sel= 0;
	uint8_t selected = 0;

	unsigned char hum_str[16] = "";
	unsigned char temp_str[16] = "";




	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // to brightness
	HAL_GPIO_WritePin(sensor_enable_GPIO_Port, sensor_enable_Pin, GPIO_PIN_SET);
	TM_HD44780_Init(16, 2);
	for(;;)
	{

		unsigned char text[25] = "";
		data = 0;
		xQueueReceive(defaultQueueHandle, &data, 500);

		if(mode == 0){
			switch (data) {
			case 1024: //rotary encoder left
				snprintf(text, 15, "Left");
				break;

			case 256: //rotary encoder right
				snprintf(text, 15, "Right");
				break;

			case 16: // button on rotary encoder
				break;

			case 32: // lone button
				mode = 1;
				selected = 0;
				break;
			default:
				break;
			}

			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // to sensor
			osDelay(100);
			HAL_ADC_Start(&hadc1);
			HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 100);
			while(status == HAL_BUSY) {
			}
			osDelay(10);
			hum = HAL_ADC_GetValue(&hadc1);
			hum = hum / 16;// compensation for oversampling
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			hum_l = hum / 100;
			hum_r = hum - hum_l * 100;
			osDelay(100);

			HAL_ADC_Start(&hadc2);
			status = HAL_ADC_PollForConversion(&hadc2, 100);
			while(status == HAL_BUSY) {
			}
			temp = HAL_ADC_GetValue(&hadc2);
			temp = temp / 16; //compensation for oversampling
			float x = 725.0/812.0;
			temp_mV = temp * x; // to millivolts
			if(temp_mV < 500){
				temp = 111111111; //for debug
			}
			else{
				temp = (temp_mV - 500);  // to celcius*10
			}

			temp_l = temp / 10; //left side
			temp_r = temp - temp_l*10; //right side, after decimal point


			snprintf(hum_str, 16, "Hum:  %lu.%lu         ", (unsigned long)hum_l, (unsigned long)hum_r);
			if(temp == 111111111){
				snprintf(temp_str, 16, "Temp: No reading!       ");
			}
			else{
				snprintf(temp_str, 16, "Temp: %lu.%lu %cC       ", (unsigned long)temp_l, (unsigned long)temp_r, (char)223);
			}
			TM_HD44780_Puts(0, 0, hum_str);
			TM_HD44780_Puts(0, 1, temp_str);
			HAL_UART_Transmit(&huart2, hum_str,
					sizeof(hum_str) ,1);
		}

		else if(mode == 1){
			//currently shown option is indicated by opt_sel
			//the value can be changed with rotary if selected = true

			struct Option sel1 = opt_list[opt_sel];
			struct Option sel2 = opt_list[opt_sel + 1];

			switch (data) {
			case 1024: //rotary encoder left
				if(selected){
					if(sel1.value <= (sel1.min +sel1.incr)){
						sel1.value = sel1.min;
					}
					else{
						sel1.value = sel1.value - sel1.incr;
					}

					opt_list[opt_sel] = sel1;
				}
				else{
					if(opt_sel <= 1){
						opt_sel = 0;
					}
					else{
						opt_sel = opt_sel - 1;
					}
				}
				break;

			case 256: //rotary encoder right
				if(selected){
					sel1.value = sel1.value + sel1.incr;

					if(sel1.value > sel1.max){
						sel1.value = sel1.max;
					}
					opt_list[opt_sel] = sel1;
				}
				else{
					opt_sel = opt_sel + 1;
					if(opt_sel > NUM_OPT-1){
						opt_sel = NUM_OPT-1;
					}
				}
				break;

			case 16: // button on rotary encoder
				selected = (selected == 1 ? 0: 1); //toggle
				if(selected){
					TM_HD44780_CursorOn();
				}
				else{
					TM_HD44780_CursorOff();
				}
				break;

			case 32: // lone button
				mode = 0;
				opt_sel = 0;
				break;
			default:
				break;
			}


			sel1 = opt_list[opt_sel];
			sel2 = opt_list[opt_sel + 1];


			unsigned char lcd_top[16] = "";
			unsigned char lcd_bot[16] = "";
			snprintf(lcd_top, 16, "%c%s %lu             ",(char)126, sel1.name, sel1.value);
			if(opt_sel + 1 == NUM_OPT){
				// when the last option is on top row, just print empty line on the bottom
				snprintf(lcd_bot, 16, "                   ");
			}
			else{
				snprintf(lcd_bot, 16, "%s %lu             ", sel2.name, sel2.value);
			}
			TM_HD44780_Puts(0, 0, lcd_top);
			TM_HD44780_Puts(0, 1, lcd_bot);
		}

		brightness = opt_list[0].value;
		contrast  = opt_list[1].value;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, contrast);
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, brightness);

		//setting sensor pwm options
		float divisor = opt_list[2].value * 1.25; //kHz * ns = 10^-6
		float counterf = 100000000.0 / divisor;
		sens_pwm_counter = counterf; // 1/(kHz * us)
		sens_pwm_compare = (opt_list[3].value * sens_pwm_counter) / 100;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, sens_pwm_compare);
		__HAL_TIM_SET_AUTORELOAD(&htim4, sens_pwm_counter);
		//HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
		//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

		HAL_UART_Transmit(&huart2, text,
				sizeof(text) ,1);
		osDelay(10);
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_auxTaskFxn */
/**
 * @brief Function implementing the auxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_auxTaskFxn */
void auxTaskFxn(void const * argument)
{
  /* USER CODE BEGIN auxTaskFxn */

	// Handles the interrupts from the inputs and performs debouncing before
	// sending the data on to the default task queue.

	uint16_t db_button0 = 0;
	uint16_t db_button1 = 0;
	uint16_t db_rot1 = 0;
	uint16_t db_rot2 = 0;
	uint16_t pr_rot1 = 0; // task ticks since the last rot1 trigger
	uint16_t pr_rot2 = 0;

	uint16_t pr_max = 100;
	uint16_t db_max = 30;

	int8_t direction = -1;
	uint16_t direction_timeout = 0;
	uint16_t direction_timeout_max = 100;

	for(;;)
	{
		uint16_t button = 0;
		xQueueReceive(auxQueueHandle, &button, 0);
		switch(button) {
		case 16:{
			if(db_button0 == 0) {
				db_button0 = 200;
				xQueueSend(defaultQueueHandle, &button, 0);
			}
			break;
		}
		case 32:{
			if(db_button1 == 0) {
				db_button1 = 200;
				xQueueSend(defaultQueueHandle, &button, 0);
			}
			break;
		}
		case 1024:{
			if(db_rot1 == 0) {
				db_rot1 = db_max;
				pr_rot1 = 0;
				if(pr_rot2 < pr_max && (direction == -1 || direction == 0)){
					direction = 0;
					direction_timeout = direction_timeout_max;
					xQueueSend(defaultQueueHandle, &button, 0);
				}
			}
			break;
			}
		case 256:{
			if(db_rot2 == 0) {
				db_rot2 = db_max;
				pr_rot2 = 0;
				if(pr_rot1 < pr_max && (direction == -1 || direction == 1)){
					direction = 1;
					direction_timeout = direction_timeout_max;
					xQueueSend(defaultQueueHandle, &button, 0);
				}
			}
			break;
		}
		}
		if(db_button0 > 0) {
			db_button0 = db_button0 - 1;
		}
		if(db_button1 > 0) {
			db_button1 = db_button1 - 1;
		}
		if(db_rot1 > 0) {
			db_rot1 = db_rot1 - 1;
		}
		if(db_rot2 > 0) {
			db_rot2 = db_rot2 - 1;
		}
		if(direction_timeout > 0) {
			direction_timeout = direction_timeout - 1;
		}
		if(pr_rot1 < pr_max) {
			pr_rot1 = pr_rot1 + 1;
		}
		if(pr_rot2 < pr_max) {
			pr_rot2 = pr_rot2 + 1;
		}
		if(direction_timeout == 0) {
			direction = -1;
		}
		vTaskDelay(1);
	}
  /* USER CODE END auxTaskFxn */
}

/* timer01Callback function */
void timer01Callback(void const * argument)
{
  /* USER CODE BEGIN timer01Callback */

  /* USER CODE END timer01Callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(0)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
