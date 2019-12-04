/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart_rx;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch4_up;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
osThreadId defaultTaskHandle;
osMessageQId myQueue01Handle;
osTimerId myTimer01Handle;
osSemaphoreId Bin_SemHandle;
/* USER CODE BEGIN PV */
QueueHandle_t xQueue = NULL;

typedef struct 
{ uint8_t Value;} Data;
Data Rx_UART_Data; 
uint8_t Rx_Queue_Data, myRxData[8]  ,  myQueueData[32] ;
uint8_t TimerBuffer[32] = " Time Elapsed [sec]      \n";
int  size = 0 ,  timer_conter=0;
//int i =0;  
__IO uint32_t uwPrescalerValue = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
void Data_Processing(void const * argument);
void Callback01(void const * argument);

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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&hlpuart1, myRxData , 2);
  /* USER CODE END 2 */
  
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  
  /* Create the semaphores(s) */
  /* definition and creation of Bin_Sem */
  osSemaphoreDef(Bin_Sem);
  Bin_SemHandle = osSemaphoreCreate(osSemaphore(Bin_Sem), 1);
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  
  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);
  
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  
  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 256, uint8_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xQueue = xQueueCreate(256, sizeof(uint8_t));
  if (xQueue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */
  
  /* Create the thread(s) */
  /* definition and creation of Data_Processing */
  osThreadDef(defaultTask, Data_Processing, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief LPUART1 Initialization Function
* @param None
* @retval None
*/
static void MX_LPUART1_UART_Init(void)
{
  
  /* USER CODE BEGIN LPUART1_Init 0 */
  
  /* USER CODE END LPUART1_Init 0 */
  
  /* USER CODE BEGIN LPUART1_Init 1 */
  
  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  
  /* USER CODE END LPUART1_Init 2 */
  
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
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = uwPrescalerValue;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  //htim3.Init.RepetitionCounter = 0;
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
* Enable DMA controller clock
* Configure DMA for memory to memory transfers
*   hdma_memtomem_dma1_channel1
*/
static void MX_DMA_Init(void) 
{
  
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  
  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }
  
  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);
  
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
}

/* USER CODE BEGIN 4 */



/************** rx complete callback ***********/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  xQueueSendFromISR(xQueue, &myRxData[1] , NULL);
  xSemaphoreGiveFromISR(Bin_SemHandle, (long*const)Data_Processing);  // give semaphore to task to process data 
  HAL_UART_Receive_DMA(&hlpuart1, myRxData,2);
}

/************** rx half complete callback ***********/

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  xQueueSendFromISR(xQueue, &myRxData[0] , NULL);
  xSemaphoreGiveFromISR(Bin_SemHandle, (long*const)Data_Processing);  // give semaphore to task to process data 
  HAL_UART_Receive_DMA(&hlpuart1, myRxData,2);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief  Function implementing the Data_Processing thread.
* @param  argument: Not used 
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void Data_Processing(void const * argument)
{
  /* USER CODE BEGIN 5 */
  int j; 
  osTimerStart(myTimer01Handle,1000); 
  /* Infinite loop */
  for(;;)
  {
    
    if(xSemaphoreTakeFromISR(Bin_SemHandle, (long*const)Data_Processing)) // take semaphore coming from callback functions
    {
//    osDelay(500); // Task Periodicity 
    size = 256 - uxQueueSpacesAvailable(xQueue); // get the size of received data 
    if(size != 0)
    {
      for(j=0; j < size ; j++)  // loop to dequeue all received data 
      {
        xQueueReceive(xQueue, &Rx_Queue_Data, 0);  
        HAL_UART_Transmit_IT(&hlpuart1 , &Rx_Queue_Data , 1);
        while (hlpuart1.gState != 32)   // Wait for the end of the transfer to tansfer a new one  
        {
        }
        if(Rx_Queue_Data =='S')
        {
          HAL_TIM_Base_Start_IT(&htim3);
        }
        else if(Rx_Queue_Data == 's')
        {
          HAL_TIM_Base_Stop_IT(&htim3);
          timer_conter=0;
        }
        
      }     
    }
    }
    
  }
  /* USER CODE END 5 */ 
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  
  /* USER CODE END Callback01 */
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
  else
  {
    timer_conter++;
    snprintf((char *) TimerBuffer+20, 30, "%d", timer_conter);
    HAL_UART_Transmit_IT(&hlpuart1, TimerBuffer ,26);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
