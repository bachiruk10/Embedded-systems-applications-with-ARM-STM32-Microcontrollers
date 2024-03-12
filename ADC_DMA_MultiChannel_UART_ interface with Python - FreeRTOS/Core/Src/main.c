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

	/***************************** Tutorial Tags ******************************
	 ADC triggered by Timer
	 UART data communication to PC
	 Python for control STM32 and reading/ plotting data
	 FreeRTOS implmentation:
	 Tasks (priorities, states),
	 Events,
	 Queues,
	 Semaphores,
	 Memory pool
	 **************************************************************************/

	/* *************************************************************************
	* Application: ADC_DMA_MultiChannel_UART_ interface with Python - FreeRTOS
	*

    We repeat implementation of the previous project "ADC_DMA_MultiChannel_UART_ interface with Python"
    with FreeRTOS implementation.

    Configure three tasks (threads): one task with High priority is triggered with an interrupt
    sent from Python. This task start ADC and Timer for sampling three potentiometers. The second task
    set with Above normal priority convert the ADC data into float, and calculacte the average of 10
    measurements for each of the three ADC channels. The averaged data is sent in a memory queue to Task
    3, which set in normal priority that is resposnsible to send the average data via UART to PC, where
    Python code process and plot the data.

    Python is able to start or stop ADC acquisition via flags sent to MCU. (more information can be found from
    previous project with bare metal implementation)

    Demonstration of this project is shown with figures in the subfolder results of this project

    Python code can be found in the subfolder  "Python client"

    ****************************************************************************
    Calculation of specs used by this application:
    ADC freq  = APB2/4 = 90 MHz/4 = 22.5 MHz
    Max Sample rate for ADC = 22.5 MHz / (15 cycles) = 1.5 MS/s
    Max allowed freq timer for ADC trig = 1.5 M / N_Channels scanned
    Freq_Timer = 100 Hz.
    So for 1 scanned Channel: DMA for 10 measurements is 10 Hz
    for 3 scanned Channels: DMA for 30 measurements is 10 Hz
    because channels are sampled sequentially at ADC sampling rate. :-)
	***************************************************************************/



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<math.h>
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Size of DMA buffer*/
#define length 30
/*Number of ADC Channels*/
#define NCh 3

#define FLAG 0x05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/********************************Definition tasks******************************/
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/****************************  Definition Semaphopres  **************************/
osSemaphoreId_t CountingSem1Handle;
const osSemaphoreAttr_t CountingSem1_attributes = {
  .name = "CountingSem1",
};
/******************************** Definition Events  ***************************/
osEventFlagsId_t EventHandle;
const osEventFlagsAttr_t Event_attributes = {
  .name = "Event",
};


/********************************Definition queues******************************/
osMessageQueueId_t Queue1Handle;
osMessageQueueAttr_t Queue1_attributes = {
		.name = "Queue1",
};


/********************************Definition Memory pool***************************/
osMemoryPoolId_t MemoryPoolHandle;
osMemoryPoolAttr_t MemoryPool_attributes = {
		.name = "MemoryPool",
};



/* USER CODE BEGIN PV */
/*DMA Data buffer for ADC conversions*/
uint16_t  pData[length];

uint16_t counter = 0;

float ADC_Ave [NCh], Vref = 2.92;

float ADC_CONV [NCh];

/*Counter for number of end of conversions by the ADC for size of DMA buffer*/

/*Byte to be received from COM port by UART*/
uint8_t recData;
//uint8_t recData = 'g';


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartTask3(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	  int i = 0;
	  for (i=0; i<len; i++)
		  ITM_SendChar((*ptr++));
	  return len;
}
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  printf("PROGRAM START \n\r");
  // Clear timer update flag in case it is set at start
  if (__HAL_TIM_GET_FLAG(&htim8 , TIM_FLAG_UPDATE) != RESET)
  __HAL_TIM_CLEAR_FLAG(&htim8 , TIM_FLAG_UPDATE);

  /*Start UART in interrupt mode, to waiting for message from Python client */
  HAL_UART_Receive_IT(&huart1, &recData, 1);


  //HAL_TIM_Base_Start(&htim8);
//	  /*Start ADC with DMA*/
//	  /* Start ADC sampling using DMA */
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *) pData, length);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CountingSem1 */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  Queue1Handle =  osMessageQueueNew(1, sizeof(ADC_CONV), &Queue1_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(StartTask3, NULL, &Task3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Event */
  EventHandle = osEventFlagsNew(&Event_attributes);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 18000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  // USER CODE BEGIN 5

  int ret;


	// Infinite loop
  for(;;)
  {
	osSemaphoreAcquire(CountingSem1Handle, osWaitForever);



	ret = osMessageQueueGet(Queue1Handle, ADC_CONV , (uint8_t*) osPriorityNormal, osWaitForever);
	if (ret!=0){
		printf("Error retrieving data \n");
	}
	else {
		//Transmit data to client over UART
		if(ADC_CONV[0] > 0){
			HAL_UART_Transmit(&huart1, (uint8_t*) ADC_CONV, 12, 5000);
			HAL_Delay(100);
		}
	}

    osDelay(1);
  }
   //USER CODE END 5
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
   //USER CODE BEGIN StartTask2

	int ret;
	float * ADC_Ave_V;

	MemoryPoolHandle = osMemoryPoolNew(10, sizeof(ADC_Ave_V), &MemoryPool_attributes);
   //Infinite loop
  for(;;)
  {

	osSemaphoreAcquire(CountingSem1Handle, osWaitForever);

	ADC_Ave_V =  (float *) osMemoryPoolAlloc(MemoryPoolHandle, 0U);


	uint32_t average = 0;

	for (int i = 0; i < NCh; i++){
	for (int j = 0; j < length/NCh; j++){
		average += pData[j*NCh+i];
	}
	//Digital value 0-4095
	ADC_Ave[i] = (float) (average / (length/NCh));
	//Convert to voltage
	ADC_Ave_V[i] = (float) (Vref/pow(2,12))* ADC_Ave [i];
	average = 0;
	}

	ret = osMessageQueuePut(Queue1Handle, ADC_Ave_V, osPriorityNormal, osWaitForever);

	if (ret!=0){
		  printf("Error putting data in queue\n");
	}
	else {
		// Free the block of memory
		ret = osMemoryPoolFree(MemoryPoolHandle , ADC_Ave_V);
		if (ret !=0)
			printf("Error freeing memory from memory pool\n");
	}

    ///osThreadYield();
	  osDelay(1);
  }
  // USER CODE END StartTask2
}

/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void StartTask3(void *argument)
{
  /* USER CODE BEGIN StartTask3 */

  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(EventHandle, FLAG, osFlagsWaitAny, osWaitForever);
	  printf("inside task \n");
		if(recData == 'g'){
			  /*Message to start ADC received!, turn on the red led*/
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
			  /*Start the timer*/
			HAL_TIM_Base_Start(&htim8);
			  /*Start ADC with DMA*/
			  /* Start ADC sampling using DMA */
		    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) pData, length);
		}

		else if (recData == 'r'){
			/*Turn off red led*/
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
			/*Turn off red led*/
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
			/*Stop the timer*/
			HAL_TIM_Base_Stop(&htim8);
		}

		osEventFlagsClear(EventHandle, FLAG);

		 // Need to put this to generate an interrupt for the next reception of byte


		recData = 'n';
		// Terminate the task and move to the next one.

		// osThreadTerminate(Task3Handle);

		osDelay(1000);
  }
  /* USER CODE END StartTask3 */
}


/*Callback function to receive message from Python client*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Set flag when MCU receives message from python client

	printf("inside interrupt \n\r");
	osEventFlagsSet(EventHandle, FLAG);
	__NOP();
	HAL_UART_Receive_IT(&huart1 , &recData , 1);
}

/*Callback function for data acquisition*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	CountingSem1Handle = osSemaphoreNew(2, 2, &CountingSem1_attributes);
	/*Toggle green led for every ADC conversion*/
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
