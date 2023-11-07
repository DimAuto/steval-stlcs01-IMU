/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();

  FusionInit();


  if (lsm6_bus_init() != 0){
	  uint8_t Test[] = "Failed to init I2C bus\r\n";
	  uart_write_debug(Test, 10);
  }
  else{
		  if (lsm6_acc_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LSM6 acc\r\n";
			  uart_write_debug(Test, 10);
		  }
		  if (gyro_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LSM6 gyro\r\n";
			  uart_write_debug(Test, 10);
		  }
		  if (magn_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LIS3 magn\r\n";
			  uart_write_debug(Test, 10);
		  }
  }
  if (ublox_i2c_bus_init() != HAL_OK){
	  uart_write_debug("Failed to Initialize ublox bus\r\n", 10);
  }
  else{
	  UBLOX_transResult res;
	  res = ubloxInit();
	  if (res != UBX_ACK){
		  uart_write_debug("Failed to Initialize UBLOX\r\n", 10);
	  }
	  else{
		  uart_write_debug("Ublox Initialized!\r\n", 10);
	  }
  }

  /* Init scheduler */
  osKernelInitialize();
  /* USER CODE BEGIN RTOS_MUTEX */
  debugUartMutex = osMutexNew(&uartMutex_attributes);
  i2cMutex = osMutexNew(&i2cMutex_attributes);
  /* USER CODE END RTOS_MUTEX */
  memsQueueHandle = osMessageQueueNew (8, sizeof(mems_data_t), &memsQueue_attributes);
  outputQueueHandle = osMessageQueueNew (4, sizeof(FusionEuler), &outputQueue_attributes);
  messageQueueHandle = osMessageQueueNew (8, RB_SIZE, &messageQueue_attributes);
  coorsQueueHandle = osMessageQueueNew (8, sizeof(gps_data_t), &coorsQueue_attributes);
//  magnVectorQueueHandle = osMessageQueueNew (8, sizeof(double), &magnVectorQueue_attributes);

  /* EVENT FLAG FOR ACK RECEIVE */
  ack_rcvd = osEventFlagsNew(NULL);
  wait_for_ack = osEventFlagsNew(NULL);
  magnetic_interf = osEventFlagsNew(NULL);
  magnCalibStart = osEventFlagsNew(NULL);
  //							//

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  readMemsTaskHandle = osThreadNew(readMemsTask, NULL, &readMemsTask_attributes);

//  calcHeadingTaskHandle = osThreadNew(calcHeadingTask, NULL, &calcHeadingTask_attributes);

  printOutTaskHandle = osThreadNew(printOutTask, NULL, &printOutTask_attributes);

//  getCoorsTaskHandle = osThreadNew(getCoorsTask, NULL, &getCoorsTask_attributes);

//  sendMessageTaskHandle = osThreadNew(sendMessageTask, NULL, &sendMessageTaskHandle_attributes);

  readMessageTaskHandle = osThreadNew(readMessageTask, NULL, &readMessageTaskHandle_attributes);

  gyroCalibrationTaskHandle = osThreadNew(gyroCalibrationTask, NULL, &gyroCalibrationTaskHandle_attributes);

  magnCalibrationTaskHandle = osThreadNew(magnCalibrationTask, NULL, &magnCalibrationTaskHandle_attributes);

  checkforInterruptsTaskHandle = osThreadNew(checkForInterrupt, NULL, &checkforInterruptTaskHandle_attributes);

  /*Suspend the gyro-calibration task*/
  osThreadSuspend(gyroCalibrationTaskHandle);
  osThreadSuspend(magnCalibrationTaskHandle);

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
}


/* CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
//	uint32_t magneticCalibStart = 0;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB,LED2_Pin);
		uart_receive_it(UART_NYX);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

void checkForInterrupt(void *argument){
	for(;;){
		//Check for enabled interrupt flags
		if (osEventFlagsWait(magnCalibStart, 0x00000001U, osFlagsWaitAny, 10) == 1){
			osThreadResume(magnCalibrationTaskHandle);;
			osEventFlagsSet(magnCalibStart, 0x00000000U);
		}
		osDelay(1000);
	}
}

void calcHeadingTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	osStatus_t status;
	FusionInit();

	for(;;)
	{
		status = osMessageQueueGet(memsQueueHandle, &mems_data, NULL, 5U);   // wait for message
	    if (status == osOK) {
	    	FusionCalcHeading(&mems_data, &euler);
	    	osMessageQueuePut(outputQueueHandle, &euler, 0U, 5U);
	    }
		osDelay(30);
	}
}

void readMemsTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	double magn_vector;
	for(;;)
	{
		tick_gyro(&mems_data);
		FusionCalcHeading(&mems_data, &euler);
		osMessageQueuePut(outputQueueHandle, &euler, 0U, 0U);
		osDelay(MEMS_SR);
	}
	osDelay(MEMS_SR);
}


void printOutTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	int trans_field_flag;
	double vector;
	uint8_t text[50] = "";
	osStatus_t status, status2;
	uint32_t magnetic_rej_flag = 0;

	for(;;)
	{
		status = osMessageQueueGet(outputQueueHandle, &euler, NULL, 5U);   // wait for message
		if (status == osOK) {
			magnetic_rej_flag = osEventFlagsWait(magnetic_interf, 0x00000001U, osFlagsWaitAny, 10);
			trans_field_flag = get_magn_transient_field();
			if (magnetic_rej_flag == 1){
				if (trans_field_flag != 0){
					sprintf(text, "HEAD: %f * #\r\n", euler.angle.yaw);
				}
				else{
					sprintf(text, "HEAD: %f *\r\n", euler.angle.yaw);
				}
			}else{
				if (trans_field_flag != 0){
					sprintf(text, "HEAD: %f #\r\n", euler.angle.yaw);
				}
				else{
					sprintf(text, "HEAD: %f\r\n", euler.angle.yaw);
				}
			}
			uart_write_debug(text,50);
			memset(text,0,sizeof(text));
		}
		vector = get_magn_vector_magnitude();
		sprintf(text, "magn: %lf\r\n", vector);
		uart_write_debug(text,50);
		memset(text,0,sizeof(text));
		osDelay(140);
	}
}


void getCoorsTask(void *argument){
	gps_data_t data;
	for(;;)
	{
		ublox_tick();
		osMessageQueuePut(coorsQueueHandle, &data, 0U, 0U);
		osDelay(1700);
	}
}

void readMessageTask(void *argument){
	osStatus_t status;
	uint32_t ack_flag, wait_flag;
	uint8_t message_buffer[RB_SIZE] = {0};
	for(;;){
		status = osMessageQueueGet(messageQueueHandle, message_buffer, NULL, osWaitForever);   // wait for message
		if (status == osOK) {
			tick_Handler(message_buffer);
			wait_flag = osEventFlagsWait(wait_for_ack, 0x00000001U, osFlagsWaitAny, 20);
			if (wait_flag == 1){
				ack_flag = osEventFlagsWait(ack_rcvd, ACK_FLAG, osFlagsWaitAny, 150);
				if (ack_flag != 1){
					tick_Handler(message_buffer);
				}
				osEventFlagsSet(wait_for_ack, 0x00000000U);
			}
		}
		osDelay(200);
	}
}

void gyroCalibrationTask(void *argument){
	mems_data_t mems_data;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	osDelay(100);
	uart_write_debug("Gyro Calibration: Hold the device still\r\n", 50);
	for(;;){
		if (gyro_offset_calculation(&mems_data) == 0){
			uart_write_debug("Gyro Calibration: Finished!\r\n", 50);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osThreadSuspend(gyroCalibrationTaskHandle);
		}
		osDelay(10);
	}
}

void magnCalibrationTask(void *argument){
	mems_data_t mems_data;
	uint8_t mag_sample_res, err_coef_res = 1;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	uart_write_debug("Magnetometer Calibration: Rotate the device multiple times on each axis\r\n", 100);
	for(;;){
		if (magneto_sample(&mems_data) == 0){
			SetMagnCalibratingFlag(false);
			uart_write_debug("Magnetometer Calibration: Finished!\r\n", 50);
			osDelay(200);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osDelay(200);
			osThreadSuspend(magnCalibrationTaskHandle);
		}
		osDelay(50);
	}
}

void accCalibrationTask(void *argument){
	mems_data_t mems_data;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	uart_write_debug("Accelerometer Calibration: Rotate the device multiple times on each axis\r\n", 100);
	for(;;){
		if (acc_sample(&mems_data) == 0){
			uart_write_debug("Accelerometer Calibration: Finished!\r\n", 50);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osThreadTerminate(accCalibrationTaskHandle);
		}
		osDelay(50);
	}
}










/////////////////////////////////////////////SYSTEM INIT/////////////////////////////////////////////////



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.Timing = 0x00702991;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
