/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "bmp390.h"
#include "flight_data.h"
#include "data_manager.h"
#include "flight_algorithm.h"
#include "uart_handler.h"
#include "sensor_fusion.h"
#include "quaternion.h"
#include "kalman_filter.h"
#include "backup_domain.h"
#include "w25q.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define UKB_TEST_WITH_TYPE_C
#ifdef UKB_TEST_WITH_TYPE_C
	#define TEST_UART &huart2
    #define TEST_USARTx USART2
#else
    #define TEST_UART &huart3
    #define TEST_USARTx USART3
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
BMI088_HandleTypeDef bmi088;
BMP390_HandleTypeDef bmp390;

FlightData flight_data = { 0 };

UartHandler_HandleTypeDef uart_handler;

Quaternion_HandleTypeDef quaternion;
KalmanFilter_HandleTypeDef kalman;

Backup_Domain_HandleTypeDef backup_domain;

W25Q_HandleTypeDef w25q;
uint8_t flash_data_packet[W25Q_PACKET_SIZE];

volatile uint8_t timer_flag_100ms = 0;

char tx_buffer[200];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void BMI088_IMU_Init(void);
static void BMP390_Baro_Init(void);

static void W25Q_Flash_LogData(void);
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
	MX_SPI2_Init();
	MX_RTC_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	MX_USART3_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
	/* use for 100ms interrupt */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/* use to calculate dt in microsecond */
	HAL_TIM_Base_Start(&htim5);

	BackupDomain_Init(&backup_domain, &hrtc);

	if (backup_domain_is_reset_occured(&backup_domain))
	{
		BMI088_IMU_Init();
		BMI088_Config(&bmi088);
		BackupDomain_ReadSRAM(&backup_domain, (uint8_t*) bmi088.configuration.offsets.acc_offset, BACKUP_DOMAIN_ACC_OFFSET, 12);
		BackupDomain_ReadSRAM(&backup_domain, (uint8_t*) bmi088.configuration.offsets.gyro_offset, BACKUP_DOMAIN_GYRO_OFFSET, 12);
		HAL_Delay(20);
		BMI088_Update(&bmi088);

		Quaternion_Recover(&quaternion, -bmi088.data.acc_x, -bmi088.data.acc_y, bmi088.data.acc_z, &backup_domain);

		BMP390_Baro_Init();
		BMP390_Config(&bmp390);
		BackupDomain_ReadSRAM(&backup_domain, (uint8_t*) &bmp390.data.base_altitude, BACKUP_DOMAIN_BASE_ALTITUDE, 4);
		HAL_Delay(50);
		BMP390_Update(&bmp390);
		flight_data.raw.base_altitude = bmp390.data.base_altitude;

		UartHandler_Start(&uart_handler, TEST_UART);

		FlightAlgorithm_Recover(&flight_data, &backup_domain);

		BackupDomain_ReadSRAM(&backup_domain, (uint8_t*) &uart_handler.system_mode, BACKUP_DOMAIN_SYSTEM_MODE, 1);

		KalmanFilter_Init(&kalman);
		KalmanFilter_Recover(&kalman, &backup_domain);

		W25Q_Recover(&w25q, &hspi1, W25Q_CS_GPIO_Port, W25Q_CS_Pin, TEST_UART, &backup_domain);
	}
	else
	{
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		HAL_Delay(4000);
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

		BMI088_IMU_Init();
		BMI088_Config(&bmi088);
		BMI088_GetOffset(&bmi088);
		BackupDomain_WriteSRAM(&backup_domain, (uint8_t*) bmi088.configuration.offsets.acc_offset, BACKUP_DOMAIN_ACC_OFFSET, 12);
		BackupDomain_WriteSRAM(&backup_domain, (uint8_t*) bmi088.configuration.offsets.gyro_offset, BACKUP_DOMAIN_GYRO_OFFSET, 12);
		HAL_Delay(20);
		BMI088_Update(&bmi088);

		Quaternion_Init(&quaternion, -bmi088.data.acc_x, -bmi088.data.acc_y, bmi088.data.acc_z);

		BMP390_Baro_Init();
		BMP390_Config(&bmp390);
		BMP390_SetBaseAltitude(&bmp390);
		BackupDomain_WriteSRAM(&backup_domain, (uint8_t*) &bmp390.data.base_altitude, BACKUP_DOMAIN_BASE_ALTITUDE, 4);
		HAL_Delay(50);
		BMP390_Update(&bmp390);
		flight_data.raw.base_altitude = bmp390.data.base_altitude;

		UartHandler_Start(&uart_handler, TEST_UART);

		FlightAlgorithm_Reset(&flight_data);

		KalmanFilter_Init(&kalman);

		W25Q_Init(&w25q, &hspi1, W25Q_CS_GPIO_Port, W25Q_CS_Pin, TEST_UART);

		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		BMI088_Update(&bmi088);
		BMP390_Update(&bmp390);

		DataManager_UpdateFlightData(&flight_data, &bmi088, &bmp390);

		SensorFusion_QuaternionUpdate(&flight_data, &quaternion);

		UartHandler_ProcessPacket(&uart_handler, &w25q);

		if (uart_handler_get_is_command_packet_ready_flag(&uart_handler) == 1)
		{
			uart_handler_clear_is_command_packet_ready_flag(&uart_handler);
			FlightAlgorithm_Reset(&flight_data);
			KalmanFilter_Reset(&kalman);
		}

		if (timer_flag_100ms)
		{
			timer_flag_100ms = 0;

			switch (uart_handler.system_mode)
			{
				case NORMAL_MODE:
					SensorFusion_KalmanUpdate(&flight_data, &kalman, &uart_handler);
					FlightAlgorithm_Update(&flight_data);
					break;

				case SIT_MODE:
					SensorFusion_KalmanUpdate(&flight_data, &kalman, &uart_handler);
					DataManager_UpdateSitPacket(&flight_data, &uart_handler);
					UartHandler_SendSitPacket(&uart_handler);
					break;

				case SUT_MODE:
					DataManager_UpdateFlightDataWithSutPacket(&flight_data, &uart_handler);
					SensorFusion_KalmanUpdate(&flight_data, &kalman, &uart_handler);
					FlightAlgorithm_Update(&flight_data);
					DataManager_UpdateSutStatePacket(&flight_data, &uart_handler);
					UartHandler_SendSutStatePacket(&uart_handler);
					break;
			}

            W25Q_CommandHandler(&w25q);
            W25Q_Flash_LogData();

			DataManager_UpdateBKPSRAMPacket(&flight_data, &uart_handler, &backup_domain, &w25q);
			BackupDomain_WriteSRAM(&backup_domain, flight_data.bkpsram_packet, 0, BKPSRAM_PACKET_SIZE);
		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */
	const uint32_t MAGIC_NUMBER = 0x29AE74BF;
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == MAGIC_NUMBER)
	{
		return;
	}
	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x14;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
	sDate.Month = RTC_MONTH_JULY;
	sDate.Date = 0x5;
	sDate.Year = 0x26;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, MAGIC_NUMBER);
	/* USER CODE END RTC_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 7999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
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
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 79;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

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
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, BUZZER_Pin | RECOVERY_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, RECOVERY_ADC_1_Pin | RECOVERY_ADC_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RECOVERY_2_Pin | USER_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GYRO_CS_Pin | W25Q_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ACC_CS_Pin | BARO_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RECOVERY_STAT_1_Pin */
	GPIO_InitStruct.Pin = RECOVERY_STAT_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RECOVERY_STAT_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RECOVERY_1_Pin */
	GPIO_InitStruct.Pin = RECOVERY_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RECOVERY_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RECOVERY_ADC_1_Pin RECOVERY_ADC_2_Pin */
	GPIO_InitStruct.Pin = RECOVERY_ADC_1_Pin | RECOVERY_ADC_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : RECOVERY_STAT_2_Pin */
	GPIO_InitStruct.Pin = RECOVERY_STAT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RECOVERY_STAT_2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RECOVERY_2_Pin */
	GPIO_InitStruct.Pin = RECOVERY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RECOVERY_2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GYRO_CS_Pin W25Q_CS_Pin */
	GPIO_InitStruct.Pin = GYRO_CS_Pin | W25Q_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ACC_INT_Pin */
	GPIO_InitStruct.Pin = ACC_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ACC_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SENSOR_EN_Pin */
	GPIO_InitStruct.Pin = SENSOR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SENSOR_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ACC_CS_Pin BARO_CS_Pin */
	GPIO_InitStruct.Pin = ACC_CS_Pin | BARO_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : GYRO_INT_Pin BARO_INT_Pin */
	GPIO_InitStruct.Pin = GYRO_INT_Pin | BARO_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_LED_Pin */
	GPIO_InitStruct.Pin = USER_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_NVIC_DisableIRQ(ACC_INT_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(GYRO_INT_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BARO_INT_EXTI_IRQn);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		timer_flag_100ms++;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == TEST_USARTx)
	{
		uart_handler_dma_cplt_callback(&uart_handler);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == TEST_USARTx)
	{
		uart_handler_rx_event_callback(&uart_handler, Size);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ACC_INT_Pin)
	{
		bmi088_set_accel_INT(&bmi088);
	}
	if (GPIO_Pin == GYRO_INT_Pin)
	{
		bmi088_set_gyro_INT(&bmi088);
	}
	if (GPIO_Pin == BARO_INT_Pin)
	{
		bmp390_set_INT(&bmp390);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hspi1.Instance)
	{
		w25q_dma_cplt_callback(&w25q);
	}
}

static void BMP390_Baro_Init(void)
{
	bmp390.configuration.spi = &hspi2;
	bmp390.configuration.cs_gpio_port = BARO_CS_GPIO_Port;
	bmp390.configuration.cs_gpio_pin = BARO_CS_Pin;
	bmp390.configuration.IRQ = BARO_INT_EXTI_IRQn;

	bmp390.configuration.mode = BMP390_NORMAL_MODE;
	bmp390.configuration.odr = BMP390_ODR_50;
	bmp390.configuration.osr_pressure = BMP390_PRESSURE_OSR_4;
	bmp390.configuration.osr_temperature = BMP390_TEMPERATURE_OSR_1;
	bmp390.configuration.iir_filter_coefficient = BMP390_IIR_FILTER_COEFFICIENT_0;

	BMP390_Init(&bmp390);
}

static void BMI088_IMU_Init(void)
{
	bmi088.configuration.spi = &hspi2;
	bmi088.configuration.accel_cs_gpio_port = ACC_CS_GPIO_Port;
	bmi088.configuration.accel_cs_gpio_pin = ACC_CS_Pin;
	bmi088.configuration.gyro_cs_gpio_port = GYRO_CS_GPIO_Port;
	bmi088.configuration.gyro_cs_gpio_pin = GYRO_CS_Pin;
	bmi088.configuration.acc_IRQ = ACC_INT_EXTI_IRQn;
	bmi088.configuration.gyro_IRQ = GYRO_INT_EXTI_IRQn;
	bmi088.configuration.htim = &htim5;

	bmi088.configuration.acc_power_mode = BMI088_ACC_PWR_MODE_ACTIVE;
	bmi088.configuration.acc_range = BMI088_ACC_RANGE_24;
	bmi088.configuration.acc_odr = BMI088_ACC_ODR_200;
	bmi088.configuration.acc_bandwidth = BMI088_ACC_BANDWIDTH_OSR_4;

	bmi088.configuration.gyro_pwr_mode = BMI088_GYRO_PWR_MODE_NORMAL;
	bmi088.configuration.gyro_range = BMI088_GYRO_RANGE_2000;
	bmi088.configuration.gyro_bandwidth = BMI088_GYRO_BANDWIDTH_116;

	BMI088_Init(&bmi088);
}

static void W25Q_Flash_LogData(void)
{
    if (flight_algorithm_get_flight_phase() == PHASE_IDLE || flight_algorithm_get_flight_phase() == PHASE_LANDED)
    {
        static uint8_t flash_log_counter = 0;
        flash_log_counter++;
        if (flash_log_counter >= 10)
        {
            flash_log_counter = 0;
            DataManager_PackFlashData(&flight_data, flash_data_packet, &uart_handler, &backup_domain, &kalman, &bmp390, &quaternion);
            W25Q_LogData(&w25q, flash_data_packet);
        }
    }
    else
    {
        DataManager_PackFlashData(&flight_data, flash_data_packet, &uart_handler, &backup_domain, &kalman, &bmp390, &quaternion);
        W25Q_LogData(&w25q, flash_data_packet);
    }
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
#ifdef USE_FULL_ASSERT
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
