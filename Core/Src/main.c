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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
static void imuPolling();
static float tempConversion(uint8_t* byteTemp8);
static float accData();
static uint8_t accRange();
static float accConversion(uint16_t* byteTemp16, uint8_t acc_range);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);

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
  MX_SPI1_Init();
  MX_UART4_Init();
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
deks
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



static void imuPolling(){

	#define MESSAGE_LEN 3
	#define TIMEOUT_DELAY 100
	#define TEMP_MSB_REG 0x22
	#define READ_COMMAND_MASK 0x80

	char uart_buf[50];
	int uart_buf_len;
	char uart_buf_fail[50];
	int uart_buf_fail_len;
	char uart_result_temp[50];
	int uart_result_temp_len;

	uart_buf_len = sprintf(uart_buf, "Begun \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf, uart_buf_len, 100);

	const uint8_t TRANSMIT_MESSAGE[MESSAGE_LEN] = {TEMP_MSB_REG | READ_COMMAND_MASK, 0x00, 0x00};
	const uint8_t RECEIVE_MESSAGE[MESSAGE_LEN];
	HAL_StatusTypeDef status;

	uart_buf_fail_len = sprintf(uart_buf_fail, "Failure \r\n");

	//Setting the initial value for the CSB
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	for(;;){

// According to the data sheet PS or Pin (#07) should be grounded for SPI and set to VDDIO for I2C.

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);


		//SPI command to read data from specific register
		status = HAL_SPI_TransmitReceive(&hspi1, TRANSMIT_MESSAGE, RECEIVE_MESSAGE, MESSAGE_LEN, TIMEOUT_DELAY);


		//Error to console if status is not OK
		if(status!=HAL_OK){
			HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf_fail, uart_buf_fail_len, 100);
		}


		// SET CSB to Low
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);


		// Temperature Conversions according to data sheet
		float regular_temp = tempConversion(RECEIVE_MESSAGE);

		//Printing temperature results to console
		uart_result_temp_len = sprintf(uart_result_temp, "Temperature: %.2f°C\r\n", regular_temp);
		HAL_UART_Transmit(&huart4, (uint8_t *)uart_result_temp, uart_result_temp_len, 100);

		vTaskDelay(pdMS_TO_TICKS(1280));
	}

}

static float tempConversion(uint8_t* byteTemp8){

	uint16_t raw_temp = ((byteTemp8[1] * 8) + (byteTemp8[2]/32));
	uint16_t temp_int11 = (raw_temp > 1023) ? raw_temp - 2048 : raw_temp;
	float normal_temp = (temp_int11 * 0.125) + 23;

	return normal_temp;
}

static void accData(){

	#define MESSAGE_LEN 7
	#define TIMEOUT_DELAY 100
	#define ACC_LSB_REG 0x22
	#define READ_COMMAND_MASK 0x80

	char uart_buf[50];
	int uart_buf_len;
	char uart_buf_fail[50];
	int uart_buf_fail_len;
	char uart_result_acc[50];
	int uart_result_acc_len;

	uart_buf_len = sprintf(uart_buf, "Begun \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf, uart_buf_len, 100);


	//Receive message is 3 long since the first byte is a dummy byte, second is the MSB, and third is LSB
	const uint8_t TRANSMIT_MESSAGE[MESSAGE_LEN] = {ACC_LSB_REG | READ_COMMAND_MASK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const uint8_t RECEIVE_MESSAGE[MESSAGE_LEN];
	HAL_StatusTypeDef status;

	uart_buf_fail_len = sprintf(uart_buf_fail, "Failure \r\n");

	uint8_t acceleration_range = accRange();

	//Setting the initial value for the CSB
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	for(;;){

	// According to the data sheet PS or Pin (#07) should be grounded for SPI and set to VDDIO for I2C.

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);


			//SPI command to read data from specific register
			status = HAL_SPI_TransmitReceive(&hspi1, TRANSMIT_MESSAGE, RECEIVE_MESSAGE, MESSAGE_LEN, TIMEOUT_DELAY);


			//Error to console if status is not OK
			if(status!=HAL_OK){
				HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf_fail, uart_buf_fail_len, 100);
			}


			// SET CSB to Low
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);


			// Temperature Conversions according to data sheet
			float regular_acc = accConversion(RECEIVE_MESSAGE, acceleration_range);

			//Printing temperature results to console
			uart_result_acc_len = sprintf(uart_result_acc, "Acceleration: %.2f°C\r\n", regular_acc);
			HAL_UART_Transmit(&huart4, (uint8_t *)uart_result_acc, uart_result_acc_len, 100);

			vTaskDelay(pdMS_TO_TICKS(1280));
	}

}

static uint8_t accRange(){
	#define MESSAGE_LEN 2
	#define TIMEOUT_DELAY 100
	#define ACC_RANGE_REG 0x41
	#define READ_COMMAND_MASK 0x80

	const uint8_t TRANSMIT_MESSAGE[MESSAGE_LEN] = {ACC_RANGE_REG | READ_COMMAND_MASK, 0x00};
	const uint8_t RECEIVE_MESSAGE[MESSAGE_LEN];

	//Setting the initial value for the CSB
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);


	//SPI command to read data from specific register
	HAL_SPI_TransmitReceive(&hspi1, TRANSMIT_MESSAGE, RECEIVE_MESSAGE, MESSAGE_LEN, TIMEOUT_DELAY);

	// SET CSB to Low
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);


	uint8_t acc_range = RECEIVE_MESSAGE[1];

	//Maybe should include this, reserve command masking, acc_range &= 0xFC;

	return acc_range;
}

static float accConversion(uint16_t* byteTemp16, uint8_t acc_range){

	uint16_t Accel_X_int16 = byteTemp16[2]*256 + byteTemp16[1];
	uint16_t Accel_Y_int16 = byteTemp16[4]*256 + byteTemp16[3];
	uint16_t Accel_Z_int16 = byteTemp16[6]*256 + byteTemp16[5];

	float Accel_X_in_mg = Accel_X_int16/32768*1000*(2^(acc_range + 1))*1.5;
	float Accel_Y_in_mg = Accel_Y_int16/32768*1000*(2^(acc_range + 1))*1.5;
	float Accel_Z_in_mg = Accel_Z_int16/32768*1000*(2^(acc_range + 1))*1.5;

	float normal_acc = ((Accel_X_in_mg^2) + (Accel_X_in_mg^2) + (Accel_X_in_mg^2))^0.5;

	return normal_acc;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
