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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static uint8_t RxBuffer[256];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

int trap_120_map[6][3] = {
    {0,1,-1}, //1
    {-1,1,0}, //2
    {-1,0,1}, //3
    {0,-1,1}, //4
    {1,-1,0}, //5
    {1,0,-1}  //6
};

// PI terms
float user_velocity = 0;
float velocity_target = 0;
float kP = 0.06;
float kI = 0.15;
float set_velocity = 0;
float I_vel = 0;

// helper functions
void adjust_PWM_DC(TIM_HandleTypeDef* const pwmHandle, int channel, const float DC);
void set_phase_voltages(float Uq, float Ud, float angle);
void initialize_angle();

// control functions
void run_motor();
void velocity_control_loop(float target);
void pi_loop();
void stop_motor();
void enable_motor(void);
void disable_motor();

// status
int running = 0;
int ADDR0, ADDR1, ADDR2;

uint8_t i2c_addr = 0x00;

// user constants
int pole_pairs;

// constants
float quantized_e_angle;
float phase_e_angle;

// system variables
float shaft_angle;
float shaft_velocity;
uint32_t ccr_dc = 0;
uint32_t bus_current;
int phase_state;
float uq_mag;

uint32_t now, ol_ts;
double elapsed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  htim1.Instance->CCR1 = 0;
  htim1.Instance->CCR2 = 0;
  htim1.Instance->CCR3 = 0;

  // T200 motor constants
  pole_pairs = 14;
  phase_e_angle = 360 / (float)pole_pairs;
  quantized_e_angle = 360/(6 * (float)pole_pairs);
  uq_mag = 0.15;

  // initialize I2C
  //ADDR0
  if(HAL_GPIO_ReadPin(GPIOC, ADDR0) == GPIO_PIN_SET){
	  i2c_addr = (i2c_addr | 0x01);
  }
  //ADDR1
  if(HAL_GPIO_ReadPin(GPIOA, ADDR1) == GPIO_PIN_SET){
  	  i2c_addr = (i2c_addr | 0x02);
  }
  //ADDR2
  if(HAL_GPIO_ReadPin(GPIOB, ADDR2) == GPIO_PIN_SET){
  	  i2c_addr = (i2c_addr | 0x04);
  }

  i2c_addr = 0x22;

  HAL_I2C_EnableListen_IT(&hi2c1);

  MX_I2C1_Init();

  // enable listen interrupt
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
      /* Transfer error in reception process */
      Error_Handler();
   }

  user_velocity = 0;
  enable_motor();
  initialize_angle();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  uint8_t tr = 0x00;

  /* USER CODE END 2 */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Ext_IT11;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 22;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 15;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 640-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_B_Pin|SD_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nBRK_B_Pin|nBRK_C_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nBRK_A_GPIO_Port, nBRK_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_A_Pin|STATUS_IND_Pin|ERROR_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADDR0_Pin CROSS_DET_A_Pin CROSS_DET_B_Pin CROSS_DET_C_Pin */
  GPIO_InitStruct.Pin = ADDR0_Pin|CROSS_DET_A_Pin|CROSS_DET_B_Pin|CROSS_DET_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OC_MON_Pin ADDR1_Pin */
  GPIO_InitStruct.Pin = OC_MON_Pin|ADDR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_B_Pin SD_C_Pin */
  GPIO_InitStruct.Pin = SD_B_Pin|SD_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : nBRK_B_Pin nBRK_C_Pin */
  GPIO_InitStruct.Pin = nBRK_B_Pin|nBRK_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADDR2_Pin */
  GPIO_InitStruct.Pin = ADDR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADDR2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nBRK_A_Pin */
  GPIO_InitStruct.Pin = nBRK_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nBRK_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_A_Pin */
  GPIO_InitStruct.Pin = SD_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IBUS_MON_JMP_Pin */
  GPIO_InitStruct.Pin = IBUS_MON_JMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IBUS_MON_JMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_IND_Pin ERROR_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin|ERROR_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/*
  // Main control loop
  if (htim == &htim2 )
  {
    run_motor();
  }
  // PI control loop
  if (htim == &htim3 )
  {
    pi_loop();
  }
  */
  // LEDs & check for stall or other bad condition
  if (htim == &htim4 )
  {
	  // status LED
	  HAL_GPIO_TogglePin (GPIOB, STATUS_IND_Pin);

	  // run LED
	  HAL_GPIO_TogglePin (GPIOB, ERROR_IND_Pin);
  }

}

// start pwm for a timer channel
void start_pwm(TIM_HandleTypeDef * tim_handler, int channel){
	switch(channel){
	    case 1:
			HAL_TIM_PWM_Start(tim_handler, TIM_CHANNEL_1); // A
			HAL_TIMEx_PWMN_Start(tim_handler, TIM_CHANNEL_1); // A
	    case 2:
	    	HAL_TIM_PWM_Start(tim_handler, TIM_CHANNEL_2); // A
	    	HAL_TIMEx_PWMN_Start(tim_handler, TIM_CHANNEL_2); // A
	    case 3:
	    	HAL_TIM_PWM_Start(tim_handler, TIM_CHANNEL_3); // A
	    	HAL_TIMEx_PWMN_Start(tim_handler, TIM_CHANNEL_3); //
	}

}

void enable_motor(){

	// set PWMs to safe states
	htim1.Instance->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // A
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // A
	htim1.Instance->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // B
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // B
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // C
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // C

	// release brake function and enable drivers
	HAL_GPIO_WritePin(GPIOB, nBRK_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, SD_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, nBRK_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, SD_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, nBRK_C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, SD_C_Pin, GPIO_PIN_SET);

}

void disable_motor(){

	// set PWMs to safe states
	htim1.Instance->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // A
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // A
	htim1.Instance->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // B
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // B
	htim1.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // C
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // C

	// TODO: disable drivers

}

void initialize_angle(){
	shaft_angle = 0;
	phase_state = 0;

	// this will set to phase state = 2 and electrical angle to quant. 1 clockwise
	set_phase_voltages(0.15, 0, quantized_e_angle);
	HAL_Delay(4);
	// set this to 0!
	user_velocity = 0;
}

// 50ms
void pi_loop(){

	// check for change
	if(user_velocity != velocity_target){
		velocity_target = user_velocity;
	}

	float error = velocity_target - set_velocity;
	float P_Term = kP * error;
	I_vel = I_vel + error;
	float I_Term = kI * I_vel;

	set_velocity = (P_Term + I_Term);

	uq_mag = fabs(set_velocity) / 4000;

}

// 10us
void run_motor(){

	// set to new velocity
	velocity_control_loop(set_velocity);
}

void stop_motor(){
	set_phase_voltages(0,0,0);
}

// this is called by the run motor function
void velocity_control_loop(float target_v){
	// find time since last call
	// calculate desired angle needed to achieve velocity

	// calculate desired angle to achieve velocity
	float e_angle = (target_v * 0.00001);

	if(target_v == 0){
		uq_mag = 0;
	}

	set_phase_voltages(uq_mag, 0, e_angle);

}

int get_phase(float angle){
	return (int)((angle - (((int)(angle / phase_e_angle)) * phase_e_angle)) / quantized_e_angle) + 1;
}

// this implements the hardware layer outputs from commutate command
void set_phase_voltages(float Uq, float Ud, float angle){

	// calculate duty cycle
	float dc_set = Uq * 100;

	if(angle == 0){
		dc_set = 0;
		if(trap_120_map[phase_state - 1][0] == 1){
			adjust_PWM_DC(&htim1, 1, dc_set);
		}
		if(trap_120_map[phase_state - 1][1] == 1){
			adjust_PWM_DC(&htim1, 2, dc_set);
		}
		if(trap_120_map[phase_state - 1][2] == 1){
			adjust_PWM_DC(&htim1, 3, dc_set);
		}
	}

	shaft_angle = shaft_angle + angle;
	if(shaft_angle > 360){
		shaft_angle = shaft_angle - 360;
	}
	if(shaft_angle < 0){
		shaft_angle = 360 + shaft_angle;
	}

	// get new phase state based on angle
	int new_phase = get_phase(shaft_angle);

	// just update duty cycles if no phase change applies
	if(new_phase == phase_state){
		return;
	}

	phase_state = new_phase;

	// switch phase
	switch(trap_120_map[phase_state - 1][0]){
		case 1:
			adjust_PWM_DC(&htim1, 1, dc_set);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
			break;
		case 0:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
			break;
		case -1:
			adjust_PWM_DC(&htim1, 1, 0);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			break;
	}
	switch(trap_120_map[phase_state - 1][1]){
		case 1:
			adjust_PWM_DC(&htim1, 2, dc_set);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
			break;
		case 0:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
			break;
		case -1:
			adjust_PWM_DC(&htim1, 2, 0);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			break;
	}
	switch(trap_120_map[phase_state - 1][2]){
		case 1:
			adjust_PWM_DC(&htim1, 3, dc_set);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
			break;
		case 0:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
			break;
		case -1:
			adjust_PWM_DC(&htim1, 3, 0);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			break;
	}

}


// this implements the actual duty cycle setting blindly (be careful!)
void adjust_PWM_DC(TIM_HandleTypeDef* const pwmHandle, int channel, float DC)
{

    /* The duty cycle value is a percentage of the reload register value (ARR). Rounding is used.*/
    uint32_t newRegVal = (uint32_t)roundf((pwmHandle->Instance->ARR) * (DC * 0.01F));

    /*In case of the DC being calculated as higher than the reload register, cap it to the reload register*/
    if(newRegVal > pwmHandle->Instance->ARR){
        newRegVal = pwmHandle->Instance->ARR;
    }

    // set the variable for all channels
    ccr_dc = newRegVal;

	//Assign the new DC count to the capture compare register immediately
    switch(channel){
		case 1:
			pwmHandle->Instance->CCR1 = newRegVal;
		case 2:
			pwmHandle->Instance->CCR2 = newRegVal;
		case 3:
			pwmHandle->Instance->CCR3 = newRegVal;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	//if(AddrMatchCode == 22){
		//HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, , 1, I2C_FIRST_AND_LAST_FRAME);
		//HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, RxBuffer, 1, I2C_NEXT_FRAME);
	//}

	  if( TransferDirection==I2C_DIRECTION_TRANSMIT ) {
	      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxBuffer, 1, I2C_NEXT_FRAME);

	  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Init(&hi2c1);
	HAL_I2C_EnableListen_IT(&hi2c1);
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

