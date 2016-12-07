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
  *  Copyright (c) 2016 Alejandro Borghero <alejandro.borghero@uns.edu.ar>
  *
  *  This program is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, either version 3 of the License.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  * 
  ******************************************************************************
  *
  * TODO:
  * 			Verify the can TxMessage and RxMessage  to the handler (hcan)
  * 			Program Queues for tasks
  *
  */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_host.h"


//#define RTOS 'C' 				// Use CMSIS_OS
#define RTOS 'F'					// Use FreeRTOS

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
#define 	LED_GPIO_Port 			GPIOD
#define		LED_ORANGE					LD3_Pin
#define		LED_GREEN						LD4_Pin
#define		LED_RED	 						LD5_Pin
#define		LED_BLUE 						LD6_Pin
#define		LED_ORANGE_ON()			HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET)
#define		LED_GREEN_ON()			HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET)
#define		LED_RED_ON()				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET)
#define		LED_BLUE_ON()				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET)
#define		LED_ORANGE_OFF()		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET)
#define		LED_GREEN_OFF()			HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET)
#define		LED_RED_OFF()				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET)
#define		LED_BLUE_OFF()			HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET)
#define		LED_ORANGE_TOGGLE()	HAL_GPIO_TogglePin(GPIOD, LD3_Pin)
#define		LED_GREEN_TOGGLE()	HAL_GPIO_TogglePin(GPIOD, LD4_Pin)
#define		LED_RED_TOGGLE()		HAL_GPIO_TogglePin(GPIOD, LD5_Pin)
#define		LED_BLUE_TOGGLE()		HAL_GPIO_TogglePin(GPIOD, LD6_Pin)

#define		BUTTON_GPIO_Port	GPIOA
#define		BUTTON				B1_Pin
#define		BUTTON_PRESSED		1
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
void Error_Handler(uint8_t);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
HAL_StatusTypeDef ALE_CAN_Receive_IT(CAN_HandleTypeDef *hcan, uint8_t FIFONumber);


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
    Error_Handler(1);
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
    Error_Handler(1);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler(1);
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

// Configuration of interface
  hcan1.Instance = CAN1;
  hcan1.pTxMsg = &TxMessage;
  hcan1.pRxMsg = &RxMessage;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;

  // See RM0090 - pag. 1097
  //	BaudRate = 1 / NominalBitTime
  //	NominalBitTime = tq + tBS1 + tBS2
  //	tBS1 = tq x ( TS1[3:0] + 1 )
  //	tBS2 = tq x ( TS2[2:0] + 1 )
  //	tq = ( BRP[9:0] + 1 ) x tPCLK


//  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;		// Loopback mode, restore NORMAL mode at the end of the test


  //  Prescaler  =  PCLK1 / ( BaudRate * total_of_tq )
  //  total_of_tq = CAN_SJW + CAN_BS1 + CAN_BS2
  //  total of tq = ( PCKL1 / Prescaler ) / BaudRate

//  Values ranges ( RM0090 - pag. 1096 )
//  	  CAN_SJW = 1 a  4 TQ        --> Typically 1TQ
//  	  CAN_BS1 = 1 a 16 TQ
//  	  CAN_BS2 = 1 a  8 TQ
//  	  Prescaler = 1 a 1024

//  Sampling is typically used between 75% (ARINC 825) and 87.5% (CANopen)
//  Thus (( CAN_SJW + CAN_BS1 ) / ( CAN_SJW + CAN_BS1 + CAN_BS2 )) = 0.875 ( RM0090 - pag. 1097 )

//  total of tq = ( PCKL1 / Prescaler ) / BaudRate = ( 42000000 / 1000000 ) / Prescaler
  // 		 for Prescaler  = 2	=>	total_of_tq = 21
  // 		 for Prescaler  = 3	=>	total_of_tq = 14
  // 		 for Prescaler  = 6	=>	total_of_tq = 7
  // 		 for Prescaler  = 7	=>	total_of_tq = 6
  // 		 for Prescaler  = 14	=>	total_of_tq = 3

//  Using CAN_SJW = 1TQ , CAN_BS1 = 11TQ , CAN_BS2 = 2TQ result:
//  	  	  total_of_tq = CAN_SJW + CAN_BS1 + CAN_BS2 = 14
//  	  	  sampling = (( CAN_SJW + CAN_BS1 ) / ( CAN_SJW + CAN_BS1 + CAN_BS2 )) = 0.857
//	Prescaler = PCLK1 / ( BaudRate * total_of_tq ) = ( 42000000 / 14 ) / 1000000 = 3

  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_11TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.Prescaler = 3;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler(1);
  }

// Filter configuration
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
    Error_Handler(1);
  }
// Configure transmission process
  hcan1.pTxMsg->StdId = 0x11;
  hcan1.pTxMsg->ExtId = 0x0;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->DLC = 2;
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
    Error_Handler(1);
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
    Error_Handler(1);
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
    Error_Handler(1);
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


// Transmission start
  hcan1.pTxMsg->Data[0] = 0xCA;
  hcan1.pTxMsg->Data[1] = 0xFE;

  if(HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
  {
    /* Transmission Error */
    Error_Handler(1);
  }

  if(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY)
  {
    return HAL_ERROR;
  }

// Reception start
  if(HAL_CAN_Receive(&hcan1, CAN_FIFO0,10) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler(1);
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

	uint8_t idx=0;

// Initiates the interrupt reception process
//  if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
//  {
//    /* Reception Error */
//    Error_Handler(1);
//  }
 ALE_CAN_Receive_IT(&hcan1, CAN_FIFO0);

	for (;;)
	{
//		sprintf( "%s -- %d\n\r", pcTaskName, xTaskGetTickCount() );

		// The led flashes to show that the systems is working
		HAL_GPIO_TogglePin(GPIOD, LED_ORANGE);

/*
// 		Transmission and reception in Loopback mode (Polling)
		if ( CAN_Polling() == HAL_ERROR )
				HAL_GPIO_WritePin(GPIOD, LED_RED, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOD, LED_GREEN, GPIO_PIN_SET);
*/

		if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON) == BUTTON_PRESSED)
		{
			idx%=3;
			// Set the data to be transmitted
			hcan1.pTxMsg->Data[0] = ++idx;
			hcan1.pTxMsg->Data[1] = 0xAD;

//			HAL_GPIO_WritePin(GPIOD, LED_BLUE, GPIO_PIN_SET);
//			HAL_CAN_Transmit(&hcan1, 10);

			// Transmits the data
			if(HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
			{
				/* Transmition Error */
				Error_Handler(1);
			}
			HAL_Delay(10);
		}
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
  * @brief  This function is executed in case of error occurrence. The red led
  * will flash nerror times.
  * @param  nerror : Number of error.
  * @retval None
  */
void Error_Handler(uint8_t nerror)
{
	uint8_t i;
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	LED_RED_OFF();
  while(1) 
  {
		for(i=0; i < nerror; ++i)
		{
			LED_RED_TOGGLE();
			HAL_Delay(500);
			LED_RED_TOGGLE();
			HAL_Delay(500);
		}
		HAL_Delay(1500);
  /* USER CODE END Error_Handler */ 
  }

}
// CAN Rx complete callback
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  if ((hcan->pRxMsg->StdId == 0x11)&&(hcan->pRxMsg->IDE == CAN_ID_STD) && (hcan->pRxMsg->DLC == 2))
  {
  	HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_BLUE, GPIO_PIN_RESET);
    switch(hcan->pRxMsg->Data[0])
    {
    /* Shutdown leds */
    case 0:
    	HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_BLUE, GPIO_PIN_RESET);
    	break;
    case 1:
    	HAL_GPIO_WritePin(GPIOD, LED_GREEN, GPIO_PIN_SET);
    	break;
    case 2:
    	HAL_GPIO_WritePin(GPIOD, LED_BLUE, GPIO_PIN_SET);
    	break;
    case 3:
    	HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_BLUE, GPIO_PIN_SET);
    	break;
    case 4:
    	break;
    default:
    	break;
    }
  }

//  while(HAL_CAN_Receive_IT(hcan, CAN_FIFO0)  != HAL_OK)
//  	{
//  	HAL_GPIO_TogglePin(GPIOD, LED_BLUE);
// 		HAL_Delay(50);
//
//  	}




// Rearm the receive interrupt
  if(ALE_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler(3);
  }
}

/**
  * @brief  Receives a correct CAN frame. Modified to unlock the can interface.
  * @param  hcan:       Pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  FIFONumber: Specify the FIFO number
  * @retval HAL status
  */
HAL_StatusTypeDef ALE_CAN_Receive_IT(CAN_HandleTypeDef* hcan, uint8_t FIFONumber)
{

  /* Check the parameters */
  assert_param(IS_CAN_FIFO(FIFONumber));


	if(hcan->State == HAL_CAN_STATE_BUSY_TX)
	{
		/* Change CAN state */
		hcan->State = HAL_CAN_STATE_BUSY_TX_RX;
	}
	else
	{
		/* Change CAN state */
		hcan->State = HAL_CAN_STATE_BUSY_RX;

		/* Set CAN error code to none */
		hcan->ErrorCode = HAL_CAN_ERROR_NONE;

		/* Enable Error warning Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG);

		/* Enable Error passive Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EPV);

		/* Enable Bus-off Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_BOF);

		/* Enable Last error code Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_LEC);

		/* Enable Error Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_ERR);
	}

	if(FIFONumber == CAN_FIFO0)
	{
		/* Enable FIFO 0 message pending Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
	}
	else
	{
		/* Enable FIFO 1 message pending Interrupt */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP1);
	}

  /* Return function status */
  return HAL_OK;
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
