/*
 * bmp390.c
 *
 *  Created on: Dec 27, 2025
 *      Author: bilal
 * 
 *  References:
 *          https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
 */

#include "main.h"
#include "bmp390.h"
#include <math.h>
#include <string.h>


#define BMP390_DATA_READY		(uint8_t)0x60 // Used for bit masking when checking if data is ready

#define BMP390_BUFFER_MAX_SIZE	21

#define BMP390_INT_TIMEOUT_MS   100

/* --- Powers of 2 for BMP390 Calibration Coefficients --- */
/* Negative exponents (Positive equivalents are used to multiply instead of divide) */
#define BMP390_POW_2_8          256.0f              		// 2^8
#define BMP390_POW_2_3          8.0f                		// 2^3

/* Positive exponents */
#define BMP390_POW_2_6          64.0f               		// 2^6
#define BMP390_POW_2_14         16384.0f           			// 2^14
#define BMP390_POW_2_15         32768.0f            		// 2^15
#define BMP390_POW_2_20         1048576.0f          		// 2^20
#define BMP390_POW_2_29         536870912.0f        		// 2^29
#define BMP390_POW_2_30         1073741824.0f       		// 2^30
#define BMP390_POW_2_32         4294967296.0f      		 	// 2^32
#define BMP390_POW_2_37         137438953472.0f     		// 2^37
#define BMP390_POW_2_48         281474976710656.0f  		// 2^48
#define BMP390_POW_2_65         36893488147419103232.0f 	// 2^65


/* --- Private Functions Declaration --- */
/* SPI Communication Functions */
static inline HAL_StatusTypeDef BMP390_Write(BMP390_HandleTypeDef *BMP390, uint8_t MemAddress, uint8_t *pData);
static inline HAL_StatusTypeDef BMP390_Read(BMP390_HandleTypeDef *BMP390, uint8_t MemAddress, uint8_t *rxBuffer, uint16_t Size);
static void CS_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void CS_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* Sensor Configuration Functions */
static HAL_StatusTypeDef bmp390_get_trimming_coefficients(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_int_ctrl(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_if_conf(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_pwr_ctrl(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_osr(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_odr(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_set_iir_filter_coefficient(BMP390_HandleTypeDef *BMP390);
static HAL_StatusTypeDef bmp390_softreset(BMP390_HandleTypeDef *BMP390);
static uint8_t bmp390_check_drdy(BMP390_HandleTypeDef *BMP390);

/* Data Processing Functions */
static void bmp390_get_data(BMP390_HandleTypeDef *BMP390);
static void bmp390_calculate_altitude(BMP390_HandleTypeDef *BMP390);


/**
 * @brief Initializes the BMP390 sensor and checks communication.
 * @param BMP390 Pointer to the handle structure.
 * @retval bmp390_status_t.
 */
bmp390_status_t BMP390_Init(BMP390_HandleTypeDef *BMP390)
{
	// The SPI interface is activated when CS pin is pulled down
	CS_Low(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);
	HAL_Delay(5);
	CS_High(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);
	HAL_Delay(50);

	BMP390->flags.is_data_ready = 0;
	BMP390->flags.is_data_updated = 0;
    BMP390->flags.is_altitude_new = 0;
    BMP390->flags.is_updated_with_int = 0;
	BMP390->data.base_altitude = 0;
    BMP390->data.last_int_time = HAL_GetTick();

	uint8_t data;
	BMP390_Read(BMP390, BMP390_REG_CHIP_ID, &data, 1);

	// 0x60 is reset value for CHIP_ID
	if (data == 0x60)
	{
		return BMP390_OK;
	}
	else
	{
		return BMP390_ERROR;
	}
}

/**
 * @brief Fully configures sensor parameters.
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_Config(BMP390_HandleTypeDef *BMP390)
{
	HAL_NVIC_DisableIRQ(BMP390->configuration.IRQ);

	bmp390_softreset(BMP390);
	HAL_Delay(5);

	bmp390_get_trimming_coefficients(BMP390);
	bmp390_set_int_ctrl(BMP390);
	bmp390_set_if_conf(BMP390);
	bmp390_set_osr(BMP390);
	bmp390_set_odr(BMP390);
	bmp390_set_iir_filter_coefficient(BMP390);
	bmp390_set_pwr_ctrl(BMP390);
	HAL_Delay(50);

	HAL_NVIC_EnableIRQ(BMP390->configuration.IRQ);
	HAL_Delay(50);
}

/**
 * @brief Processes data if the software 'is_data_ready' flag is set (useful for Interrupt mode).
 * @param BMP390 Pointer to the handle structure.
 * @note The data ready interrupt is the normal path and 'last_int_time' is stamped every time it
 *       delivers a sample. When no interrupt arrives for BMP390_INT_TIMEOUT_MS the interrupt line
 *       is treated as lost and the status register is polled instead.
 * @note 'is_updated_with_int' reports which of the two paths was taken last, 1 for the interrupt and
 *       0 for the polling fallback. It is only used for logging, the data itself is identical.
 */
void BMP390_Update(BMP390_HandleTypeDef *BMP390)
{
	if (BMP390->flags.is_data_ready)
	{
        BMP390->flags.is_data_ready = 0;
		bmp390_get_data(BMP390);
		bmp390_calculate_altitude(BMP390);

        BMP390->data.last_int_time = HAL_GetTick();
        BMP390->flags.is_updated_with_int = 1;
	}
    else if (HAL_GetTick() - BMP390->data.last_int_time > BMP390_INT_TIMEOUT_MS)
    {
        BMP390_Update_drdy(BMP390);
        BMP390->flags.is_updated_with_int = 0;
    }
}

/**
 * @brief Manually checks the sensor's status register and updates data if ready (Polling mode).
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_Update_drdy(BMP390_HandleTypeDef *BMP390)
{
	if (bmp390_check_drdy(BMP390))
	{
		bmp390_get_data(BMP390);
		bmp390_calculate_altitude(BMP390);
	}
}

/**
 * @brief Gets base altitude calibration.
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_SetBaseAltitude(BMP390_HandleTypeDef *BMP390)
{
	uint32_t start_time = HAL_GetTick();
	int counter = 0;
	while (counter < 30)
	{
		if (HAL_GetTick() - start_time > 2000)
		{
			break;
		}

		BMP390_Update_drdy(BMP390);
		if (BMP390->flags.is_data_updated)
		{
			BMP390->flags.is_data_updated = 0;
			const float sea_level_pressure = 1013.25f;
			BMP390->data.altitude = 44330.0f * (1.0f - powf(((BMP390->data.pressure) / sea_level_pressure), (1.0f / 5.255f)));
			BMP390->data.base_altitude += BMP390->data.altitude;
			counter++;
		}
		HAL_Delay(5);
	}

	if (counter > 0)
	{
		BMP390->data.base_altitude /= (float)counter;
	}
}

/**
 * @brief Callback function to be called from the GPIO External Interrupt (EXTI).
 * @param BMP390 Pointer to the handle structure.
 */
void bmp390_set_INT(BMP390_HandleTypeDef *BMP390)
{
	BMP390->flags.is_data_ready = 1;
}


/* --- Private Functions Definitions --- */

static inline HAL_StatusTypeDef BMP390_Write(BMP390_HandleTypeDef *BMP390, uint8_t MemAddress, uint8_t *pData)
{
	// bmp390 datasheet 5.3.1. SPI write
	HAL_StatusTypeDef status;
	uint8_t tx_data[2];
	tx_data[0] = MemAddress & (uint8_t)0x7F;
	tx_data[1] = *pData;

	CS_Low(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);

	status = HAL_SPI_Transmit(BMP390->configuration.spi, tx_data, 2, 5);

	CS_High(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);

	return status;
}

static inline HAL_StatusTypeDef BMP390_Read(BMP390_HandleTypeDef *BMP390, uint8_t MemAddress, uint8_t *rxBuffer, uint16_t Size)
{
	if (Size > BMP390_BUFFER_MAX_SIZE)
	{
		return HAL_ERROR;
	}

	// bmp390 datasheet 5.3.2. SPI read
	HAL_StatusTypeDef status;
	uint8_t txData[BMP390_BUFFER_MAX_SIZE + 2];
	uint8_t rxData[BMP390_BUFFER_MAX_SIZE + 2];

	memset(txData, 0, Size + 2);
	memset(rxData, 0, Size + 2);
	txData[0] = MemAddress | (uint8_t)0x80;

	CS_Low(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);

	status = HAL_SPI_TransmitReceive(BMP390->configuration.spi, txData, rxData, Size + 2, 5);

	CS_High(BMP390->configuration.cs_gpio_port, BMP390->configuration.cs_gpio_pin);

	if (status == HAL_OK)
	{
		memcpy(rxBuffer, &rxData[2], Size);
	}

	return status;
}

static void CS_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

static void CS_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}


static HAL_StatusTypeDef bmp390_get_trimming_coefficients(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data[21] = {0};
	HAL_StatusTypeDef status = BMP390_Read(BMP390, BMP390_REG_TRIMMING_COEFFICIENTS, data, 21);
	HAL_Delay(1);

	if (status == HAL_OK)
	{
		BMP390->trimming_coefficients.NVM_PAR_T1 = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
		BMP390->trimming_coefficients.NVM_PAR_T2 = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
		BMP390->trimming_coefficients.NVM_PAR_T3 = (int8_t)data[4];
		BMP390->trimming_coefficients.NVM_PAR_P1 = (int16_t)((int16_t)data[5] | ((int16_t)data[6] << 8));
		BMP390->trimming_coefficients.NVM_PAR_P2 = (int16_t)((int16_t)data[7] | ((int16_t)data[8] << 8));
		BMP390->trimming_coefficients.NVM_PAR_P3 = (int8_t)data[9];
		BMP390->trimming_coefficients.NVM_PAR_P4 = (int8_t)data[10];
		BMP390->trimming_coefficients.NVM_PAR_P5 = (uint16_t)((uint16_t)data[11] | ((uint16_t)data[12] << 8));
		BMP390->trimming_coefficients.NVM_PAR_P6 = (uint16_t)((uint16_t)data[13] | ((uint16_t)data[14] << 8));
		BMP390->trimming_coefficients.NVM_PAR_P7 = (int8_t)data[15];
		BMP390->trimming_coefficients.NVM_PAR_P8 = (int8_t)data[16];
		BMP390->trimming_coefficients.NVM_PAR_P9 = (int16_t)((int16_t)data[17] | ((int16_t)data[18] << 8));
		BMP390->trimming_coefficients.NVM_PAR_P10 = (int8_t)data[19];
		BMP390->trimming_coefficients.NVM_PAR_P11 = (int8_t)data[20];

		BMP390->trimming_coefficients.PAR_T1 = (float)BMP390->trimming_coefficients.NVM_PAR_T1 * BMP390_POW_2_8;
		BMP390->trimming_coefficients.PAR_T2 = (float)BMP390->trimming_coefficients.NVM_PAR_T2 / BMP390_POW_2_30;
		BMP390->trimming_coefficients.PAR_T3 = (float)BMP390->trimming_coefficients.NVM_PAR_T3 / BMP390_POW_2_48;
		BMP390->trimming_coefficients.PAR_P1 = ((float)BMP390->trimming_coefficients.NVM_PAR_P1 - BMP390_POW_2_14) / BMP390_POW_2_20;
		BMP390->trimming_coefficients.PAR_P2 = ((float)BMP390->trimming_coefficients.NVM_PAR_P2 - BMP390_POW_2_14) / BMP390_POW_2_29;
		BMP390->trimming_coefficients.PAR_P3 = (float)BMP390->trimming_coefficients.NVM_PAR_P3 / BMP390_POW_2_32;
		BMP390->trimming_coefficients.PAR_P4 = (float)BMP390->trimming_coefficients.NVM_PAR_P4 / BMP390_POW_2_37;
		BMP390->trimming_coefficients.PAR_P5 = (float)BMP390->trimming_coefficients.NVM_PAR_P5 * BMP390_POW_2_3;
		BMP390->trimming_coefficients.PAR_P6 = (float)BMP390->trimming_coefficients.NVM_PAR_P6 / BMP390_POW_2_6;
		BMP390->trimming_coefficients.PAR_P7 = (float)BMP390->trimming_coefficients.NVM_PAR_P7 / BMP390_POW_2_8;
		BMP390->trimming_coefficients.PAR_P8 = (float)BMP390->trimming_coefficients.NVM_PAR_P8 / BMP390_POW_2_15;
		BMP390->trimming_coefficients.PAR_P9 = (float)BMP390->trimming_coefficients.NVM_PAR_P9 / BMP390_POW_2_48;
		BMP390->trimming_coefficients.PAR_P10 = (float)BMP390->trimming_coefficients.NVM_PAR_P10 / BMP390_POW_2_48;
		BMP390->trimming_coefficients.PAR_P11 = (float)BMP390->trimming_coefficients.NVM_PAR_P11 / BMP390_POW_2_65;
	}

	return status;
}

static HAL_StatusTypeDef bmp390_set_int_ctrl(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390_INT_PUSH_PULL | BMP390_INT_ACTIVE_HIGH | BMP390_INT_LATCH_DISABLED | BMP390_INT_DS_LOW | BMP390_INT_DATA_READY_ENABLED;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_INT_CTRL, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_set_if_conf(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390_SPI_4_WIRE_MODE | BMP390_I2C_WATCHDOG_DISABLED | BMP390_I2C_WATCHDOG_SHORT;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_IF_CONF, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_set_pwr_ctrl(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390_PRESSURE_ON | BMP390_TEMPERATURE_ON | BMP390->configuration.mode;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_PWR_CTRL, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_set_osr(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390->configuration.osr_pressure | BMP390->configuration.osr_temperature;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_OSR, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_set_odr(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390->configuration.odr;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_ODR, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_set_iir_filter_coefficient(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390->configuration.iir_filter_coefficient;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_CONFIG, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmp390_softreset(BMP390_HandleTypeDef *BMP390)
{
	uint8_t data = BMP390_SOFTRESET;
	HAL_StatusTypeDef status = BMP390_Write(BMP390, BMP390_REG_CMD, &data);
	HAL_Delay(1);
	return status;
}

static uint8_t bmp390_check_drdy(BMP390_HandleTypeDef *BMP390)
{
	uint8_t rx_data;
	if (BMP390_Read(BMP390, BMP390_REG_STATUS, &rx_data, 1) != HAL_OK)
	{
		return 0;
	}

	rx_data &= BMP390_DATA_READY;

	if (rx_data == BMP390_DATA_READY)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


static void bmp390_get_data(BMP390_HandleTypeDef *BMP390)
{
	uint8_t rx_buffer[11];
	HAL_StatusTypeDef status = BMP390_Read(BMP390, BMP390_REG_PRESS_DATA_0, rx_buffer, 11);
	if (status == HAL_OK)
	{
		BMP390->data.raw_pressure = (uint32_t)rx_buffer[0] | ((uint32_t)rx_buffer[1] << 8) | ((uint32_t)rx_buffer[2] << 16);
		BMP390->data.raw_temperature = (uint32_t)rx_buffer[3] | ((uint32_t)rx_buffer[4] << 8) | ((uint32_t)rx_buffer[5] << 16);
		BMP390->data.raw_sensortime = (uint32_t)rx_buffer[8] | ((uint32_t)rx_buffer[9] << 8) | ((uint32_t)rx_buffer[10] << 16);

		float temperature_partial_data_1;
		float temperature_partial_data_2;

		temperature_partial_data_1 = (float)BMP390->data.raw_temperature - BMP390->trimming_coefficients.PAR_T1;
		temperature_partial_data_2 = temperature_partial_data_1 * BMP390->trimming_coefficients.PAR_T2;
		BMP390->data.temperature = temperature_partial_data_2 + (temperature_partial_data_1 * temperature_partial_data_1) * BMP390->trimming_coefficients.PAR_T3;

		float pressure_partial_data_1;
		float pressure_partial_data_2;
		float pressure_partial_data_3;
		float pressure_partial_data_4;
		float pressure_partial_out_1;
		float pressure_partial_out_2;

		pressure_partial_data_1 = BMP390->trimming_coefficients.PAR_P6 * BMP390->data.temperature;
		pressure_partial_data_2 = BMP390->trimming_coefficients.PAR_P7 * (BMP390->data.temperature * BMP390->data.temperature);
		pressure_partial_data_3 = BMP390->trimming_coefficients.PAR_P8 * (BMP390->data.temperature * BMP390->data.temperature * BMP390->data.temperature);
		pressure_partial_out_1 = BMP390->trimming_coefficients.PAR_P5 + pressure_partial_data_1 + pressure_partial_data_2 + pressure_partial_data_3;

		pressure_partial_data_1 = BMP390->trimming_coefficients.PAR_P2 * BMP390->data.temperature;
		pressure_partial_data_2 = BMP390->trimming_coefficients.PAR_P3 * (BMP390->data.temperature * BMP390->data.temperature);
		pressure_partial_data_3 = BMP390->trimming_coefficients.PAR_P4 * (BMP390->data.temperature * BMP390->data.temperature * BMP390->data.temperature);
		pressure_partial_out_2 = (float)BMP390->data.raw_pressure * (BMP390->trimming_coefficients.PAR_P1 + pressure_partial_data_1 + pressure_partial_data_2 + pressure_partial_data_3);

		pressure_partial_data_1 = (float)BMP390->data.raw_pressure * (float)BMP390->data.raw_pressure;
		pressure_partial_data_2 = BMP390->trimming_coefficients.PAR_P9 + BMP390->trimming_coefficients.PAR_P10 * BMP390->data.temperature;
		pressure_partial_data_3 = pressure_partial_data_1 * pressure_partial_data_2;
		pressure_partial_data_4 = pressure_partial_data_3 + ((float)BMP390->data.raw_pressure * (float)BMP390->data.raw_pressure * (float)BMP390->data.raw_pressure) * BMP390->trimming_coefficients.PAR_P11;
		BMP390->data.pressure = pressure_partial_out_1 + pressure_partial_out_2 + pressure_partial_data_4;
		BMP390->data.pressure /= 100.0f;

		BMP390->flags.is_data_updated = 1;
        BMP390->flags.is_altitude_new = 1;
	}
}

static void bmp390_calculate_altitude(BMP390_HandleTypeDef *BMP390)
{
	const float sea_level_pressure = 1013.25f;
	BMP390->data.altitude = 44330.0f * (1.0f - powf(((BMP390->data.pressure) / sea_level_pressure), (1.0f / 5.255f))) - BMP390->data.base_altitude;
}

