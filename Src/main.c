/**
  ******************************************************************************
  *  Name        : main.c
  *  Author      : Alejandro Borghero
  *  Version     :
  *  Copyright   : GPLv3
  *  Description : Planificación Óptima de un Sistema Multiprocesador de Tiempo 
  *                Real con Restricciones de Precedencia, Comunicación y Energía
  ******************************************************************************
  * 
  * Copyright (c) 2016 Alejandro Borghero <alejandro.borghero@uns.edu.ar>
  *
  *  Este programa es software libre: Usted puede redistribuirlo y/o 
  *  modificarlo bajo los términos de la versión 3 de la licencia GPL (GNU 
  *  General Public License) publicada por la Free Software Foundation.
  * 
  *  Debería haber recibido una copia de la GNU General Public License junto
  *  con este programa. Si no es así, consulte <http://www.gnu.org/licenses/>
  * 
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_host.h"


//#define RTOS 'C' 				// Usar CMSIS_OS
#define RTOS 'F'				// Usar FreeRTOS

#if RTOS == 'C'
#define CMSISOS
#endif

#ifdef CMSISOS
#include "cmsis_os.h"
#else
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#endif


/* USER CODE BEGIN Includes */
#define 	LED_GPIO_Port 	GPIOD
#define		LED_NARANJA		LD3_Pin
#define		LED_VERDE		LD4_Pin
#define		LED_ROJO	 	LD5_Pin
#define		LED_AZUL 		LD6_Pin
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

#ifdef CMSISOS
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
#endif

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum SysFreq{F_025, F_050, F_075, F_100};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);

HAL_StatusTypeDef CAN_Polling(void);

#ifdef CMSISOS
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
#else
static void Task_Body( void* pvParams );
#endif
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
#ifdef CMSISOS
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);
#else
  xTaskCreate( Task_Body, NULL, 128, NULL, configMAX_PRIORITIES-1, NULL );
#endif
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
#ifdef CMSISOS
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 16, uint16_t);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);
#else
  //PROGRAMAR LAS COLAS!!
#endif

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
#ifdef CMSISOS
  osKernelStart();
#else
  vTaskStartScheduler();
#endif
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(;;);

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
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

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

CAN_FilterConfTypeDef  sFilterConfig;
static CanTxMsgTypeDef        TxMessage;
static CanRxMsgTypeDef        RxMessage;

  hcan1.Instance = CAN1;
  hcan1.pTxMsg = &TxMessage;
  hcan1.pRxMsg = &RxMessage;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;

  // Ver RM0090 - pag. 1097
  //	BaudRate = 1 / NominalBitTime
  //	NominalBitTime = tq + tBS1 + tBS2
  //	tBS1 = tq x ( TS1[3:0] + 1 )
  //	tBS2 = tq x ( TS2[2:0] + 1 )
  //	tq = ( BRP[9:0] + 1 ) x tPCLK


//  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;		// Modo loopback, restaurar NORMAL al finalizar prueba


  //  Prescaler  =  PCLK1 / ( BaudRate * total_de_tq )
  //  total_de_tq = CAN_SJW + CAN_BS1 + CAN_BS2
  //  total de tq = ( PCKL1 / Prescaler ) / BaudRate

//  Rango de valores ( RM0090 - pag. 1096 )
//  	  CAN_SJW = 1 a  4 TQ        --> Típicamente 1TQ
//  	  CAN_BS1 = 1 a 16 TQ
//  	  CAN_BS2 = 1 a  8 TQ
//  	  Prescaler = 1 a 1024

//  El muestreo se utiliza tipicamente entre el 75% (ARINC 825) y 87.5% (CANopen)
//  Por lo tanto (( CAN_SJW + CAN_BS1 ) / ( CAN_SJW + CAN_BS1 + CAN_BS2 )) = 0.875 ( RM0090 - pag. 1097 )

//  total de tq = ( PCKL1 / Prescaler ) / BaudRate = ( 42000000 / 1000000 ) / Prescaler
  // 		 para Prescaler  = 2	=>	total_de_tq = 21
  // 		 para Prescaler  = 3	=>	total_de_tq = 14
  // 		 para Prescaler  = 6	=>	total_de_tq = 7
  // 		 para Prescaler  = 7	=>	total_de_tq = 6
  // 		 para Prescaler  = 14	=>	total_de_tq = 3

//  Usando CAN_SJW = 1TQ , CAN_BS1 = 11TQ , CAN_BS2 = 2TQ resulta:
//  	  	  total_de_tq = CAN_SJW + CAN_BS1 + CAN_BS2 = 14
//  	  	  muestreo = (( CAN_SJW + CAN_BS1 ) / ( CAN_SJW + CAN_BS1 + CAN_BS2 )) = 0.857
//	Prescaler = PCLK1 / ( BaudRate * total_de_tq ) = ( 42000000 / 14 ) / 1000000 = 3

  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_11TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.Prescaler = 3;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
	  HAL_GPIO_WritePin(GPIOD, LED_AZUL, GPIO_PIN_SET);
    Error_Handler();
  }

//Filtros
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef CAN_Polling(void)
{



//  MX_CAN1_Init();

  // OJO - CONFIGURAR FILTROS!!!!

// Inicio de transmisión
  hcan1.pTxMsg->StdId = 0x11;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->DLC = 2;
  hcan1.pTxMsg->Data[0] = 0xCA;
  hcan1.pTxMsg->Data[1] = 0xFE;

  if(HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
  {
    /* Transmission Error */
    Error_Handler();
  }

  if(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY)
  {
    return HAL_ERROR;
  }

// Inicio de Recepción
  if(HAL_CAN_Receive(&hcan1, CAN_FIFO0,10) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  if(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY)
  {
    return HAL_ERROR;
  }

  if (hcan1.pRxMsg->StdId != 0x11)
  {
    return HAL_ERROR;
  }

  if (hcan1.pRxMsg->IDE != CAN_ID_STD)
  {
    return HAL_ERROR;
  }

  if (hcan1.pRxMsg->DLC != 2)
  {
    return HAL_ERROR;
  }

  if ((hcan1.pRxMsg->Data[0]<<8|hcan1.pRxMsg->Data[1]) != 0xCAFE)
  {
    return HAL_ERROR;
  }

  return HAL_OK; /* Test Passed */


}


/* USER CODE END 4 */

#ifdef CMSISOS
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOD, LD3_Pin|LD4_Pin|LD5_Pin|LD6_Pin);
	  osDelay(100);
  }
  /* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}
#else
static void Task_Body( void* pvParams )
{
	/* A pointer to the subject task's name, which is a standard NULL terminated
	 * C string. */
	char *pcTaskName = pcTaskGetTaskName( NULL );

	portTickType xLastWakeTime=(portTickType)(0);

	for (;;)
	{
//		sprintf( "%s -- %d\n\r", pcTaskName, xTaskGetTickCount() );

		// Destello de le Naranja como primer ensayo
		HAL_GPIO_TogglePin(GPIOD, LED_NARANJA);


// 		Envío y rececpción de datos en modo Loopback por polling
		if ( CAN_Polling() == HAL_ERROR )
				HAL_GPIO_WritePin(GPIOD, LED_ROJO, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOD, LED_VERDE, GPIO_PIN_SET);


		vTaskDelayUntil(&xLastWakeTime,100);
		//vTaskDelay(1000);
	}

	/* If the tasks ever leaves the for cycle, kill it. */
	vTaskDelete( NULL );
}
#endif

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOD, LED_ROJO, GPIO_PIN_SET);
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/****************** Copyright (c) 2016 Alejandro Borghero  *****END OF FILE****/
