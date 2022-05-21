/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MOBILE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for TMTC */
osThreadId_t TMTCHandle;
uint32_t TMTCBuffer[ 128 ];
osStaticThreadDef_t TMTCControlBlock;
const osThreadAttr_t TMTC_attributes = {
  .name = "TMTC",
  .cb_mem = &TMTCControlBlock,
  .cb_size = sizeof(TMTCControlBlock),
  .stack_mem = &TMTCBuffer[0],
  .stack_size = sizeof(TMTCBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MAG */
osThreadId_t MAGHandle;
uint32_t MAGBuffer[ 128 ];
osStaticThreadDef_t MAGControlBlock;
const osThreadAttr_t MAG_attributes = {
  .name = "MAG",
  .cb_mem = &MAGControlBlock,
  .cb_size = sizeof(MAGControlBlock),
  .stack_mem = &MAGBuffer[0],
  .stack_size = sizeof(MAGBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myRC */
osThreadId_t myRCHandle;
uint32_t myRCBuffer[ 128 ];
osStaticThreadDef_t myRCControlBlock;
const osThreadAttr_t myRC_attributes = {
  .name = "myRC",
  .cb_mem = &myRCControlBlock,
  .cb_size = sizeof(myRCControlBlock),
  .stack_mem = &myRCBuffer[0],
  .stack_size = sizeof(myRCBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ECU */
osThreadId_t ECUHandle;
uint32_t ECUBuffer[ 128 ];
osStaticThreadDef_t ECUControlBlock;
const osThreadAttr_t ECU_attributes = {
  .name = "ECU",
  .cb_mem = &ECUControlBlock,
  .cb_size = sizeof(ECUControlBlock),
  .stack_mem = &ECUBuffer[0],
  .stack_size = sizeof(ECUBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MAGdata */
osMessageQueueId_t MAGdataHandle;
uint8_t MAGdataBuffer[ 1 * sizeof( MAGNET ) ];
osStaticMessageQDef_t MAGdataControlBlock;
const osMessageQueueAttr_t MAGdata_attributes = {
  .name = "MAGdata",
  .cb_mem = &MAGdataControlBlock,
  .cb_size = sizeof(MAGdataControlBlock),
  .mq_mem = &MAGdataBuffer,
  .mq_size = sizeof(MAGdataBuffer)
};
/* Definitions for RCdata */
osMessageQueueId_t RCdataHandle;
uint8_t RCdataBuffer[ 1 * sizeof( RC ) ];
osStaticMessageQDef_t RCdataControlBlock;
const osMessageQueueAttr_t RCdata_attributes = {
  .name = "RCdata",
  .cb_mem = &RCdataControlBlock,
  .cb_size = sizeof(RCdataControlBlock),
  .mq_mem = &RCdataBuffer,
  .mq_size = sizeof(RCdataBuffer)
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
void StartTMTC(void *argument);
void StartMAG(void *argument);
void StartRC(void *argument);
void StartECU(void *argument);

/* USER CODE BEGIN PFP */
float map(float target, float from_min, float from_max, float to_min, float to_max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile unsigned long gTick = 0;

TELEM tx_pack = {0,};
TELEM rx_pack = {0,};

#define MOBILE
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MAGdata */
  MAGdataHandle = osMessageQueueNew (1, sizeof(MAGNET), &MAGdata_attributes);

  /* creation of RCdata */
  RCdataHandle = osMessageQueueNew (1, sizeof(RC), &RCdata_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TMTC */
  TMTCHandle = osThreadNew(StartTMTC, NULL, &TMTC_attributes);

  /* creation of MAG */
  MAGHandle = osThreadNew(StartMAG, NULL, &MAG_attributes);

  /* creation of myRC */
  myRCHandle = osThreadNew(StartRC, NULL, &myRC_attributes);

  /* creation of ECU */
  ECUHandle = osThreadNew(StartECU, NULL, &ECU_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 180-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 18-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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

  HAL_TIM_Base_MspInit(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

#ifdef MOBILE
  HAL_UART_Receive_DMA(&huart1, rx_pack.array, sizeof(Comm_struct)+2);
#else
	HAL_UART_Receive_DMA(&huart1, rx_pack.array, sizeof(Comm_struct));
#endif

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

	HAL_UART_Receive_DMA(&huart3,rc_byte_data, 16);

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


float map(float target, float from_min, float from_max, float to_min, float to_max){
    float mult = (float)(to_max - to_min) / (float)(from_max - from_min);
    target = target - from_min;
    return to_min + (target * mult);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartx){
	if(huartx->Instance == huart1.Instance){
		tx_pack.tx.mode = rx_pack.rx.comm;
#ifdef MOBILE
		HAL_UART_Receive_DMA(&huart1, rx_pack.array, sizeof(Comm_struct)+2);
#else
		HAL_UART_Receive_DMA(&huart1, rx_pack.array, sizeof(Comm_struct));
#endif
	}else if(huartx->Instance == huart3.Instance){
		Spektrum_Read();

		HAL_UART_Receive_DMA(&huart3, rc_byte_data, 16);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTMTC */
/**
* @brief Function implementing the TMTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTMTC */
void StartTMTC(void *argument)
{
  /* USER CODE BEGIN 5 */
  tx_pack.tx.header = 0xFF;

#ifdef MOBILE
  uint8_t text_buff[100] = {0,};
#endif
  /* Infinite loop */
  for(;;)
  {
#ifdef MOBILE
	sprintf(text_buff, "angle : %f | mode : %f\r\n", tx_pack.tx.angle, tx_pack.tx.mode);

	for(int i = 0; i < sizeof(text_buff); i++){
		if(text_buff[i] == 0) break;
		HAL_UART_Transmit(&huart1, &text_buff[i], 1, 0);
	}

	osDelay(1);
#else
	for(int i = 0; i < sizeof(Telem_struct); i++){
		HAL_UART_Transmit(&huart1, &tx_pack.array[i], 1, HAL_TIMEOUT);
	}
	osDelay(1);
#endif
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMAG */
/**
* @brief Function implementing the MAG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMAG */
void StartMAG(void *argument)
{
  /* USER CODE BEGIN StartMAG */

  MAGNET mag = {0, };

  as5147_Init(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);
  as5147_setZeroPosition();
  //setOffset(&mag);

  sens_time = 0;
  sens_start = 0;
  osDelay(1);

  /* Infinite loop */
  for(;;)
  {
	as5147_chip_num = 0;
	sens_time = gTick - sens_start;
	updatePosition(&mag);
	sens_start = gTick;
	osMessageQueuePut(MAGdataHandle, &mag, 0, 0);

	osDelay(1);
  }
  /* USER CODE END StartMAG */
}

/* USER CODE BEGIN Header_StartRC */
/**
* @brief Function implementing the myRC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRC */
void StartRC(void *argument)
{
  /* USER CODE BEGIN StartRC */
  RC rc_cmd = {0, };

  /* Infinite loop */
  for(;;)
  {
	if(rc.channel[THROTTLE].pos != 0){
		rc_cmd.thro = map(rc.channel[THROTTLE].pos, RC_MIN, RC_MAX, PWM_MIN, PWM_MAX);     // thro command 				m/s

		rc_cmd.d_pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, 100, -100);			// d(pi)/dt command			deg/s
		rc_cmd.d_theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -100, 100);		// d(theta)/dt command		deg/s
		rc_cmd.d_psi = map(rc.channel[RUDDER].pos, RC_MIN, RC_MAX, -100, 100);			// d(psi)/dt command		deg/s

		rc_cmd.pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, 45, -45);			// d(pi)/dt command			deg/s
		rc_cmd.theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -45, 45);		// d(theta)/dt command		deg/s

		if(SWITCH_POS(rc.channel[AUX1].pos) > 0){
			rc_cmd.arm = 1;
		}
		else{
			rc_cmd.arm = 0;
		}

		rc_cmd.mode = SWITCH_POS(rc.channel[AUX2].pos);

		osMessageQueuePut(RCdataHandle, &rc_cmd, 0, 0);
	}
	osDelay(1);
  }
  /* USER CODE END StartRC */
}

/* USER CODE BEGIN Header_StartECU */
/**
* @brief Function implementing the ECU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartECU */
void StartECU(void *argument)
{
  /* USER CODE BEGIN StartECU */
  RC rc_cmd = {0, };

  ECS ecu;
  ecu.safty_arm = 0;
  ecu.state = ECU_OK;

  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(RCdataHandle, &rc_cmd, 0, 0);
	tx_pack.tx.throttle = rc_cmd.thro;
	osMessageQueueGet(MAGdataHandle, &ecu.sensor.mag, 0, 0);
	tx_pack.tx.angle = ecu.sensor.mag.shaft_position;


	osDelay(1);
  }
  /* USER CODE END StartECU */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {
    gTick++;
  }

  /* USER CODE END Callback 1 */
}

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
