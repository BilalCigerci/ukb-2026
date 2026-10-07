/*
 * bmi088.c
 *
 *  Created on: Dec 17, 2025
 *      Author: bilal
 *
 *  References:
 *          https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 *  		https://github.com/yahio61/bmi_088_library/blob/main/bmi088/bmi088.c
 */

#include "main.h"
#include "bmi088.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>


#define BMI088_BUFFER_MAX_SIZE	9		// Longest single read is 9 bytes (acc data + sensortime)


/* --- Private Functions Declaration --- */
/* SPI Communication Functions */
static inline HAL_StatusTypeDef BMI088_Write(BMI088_HandleTypeDef *BMI088, uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *data);
static inline HAL_StatusTypeDef BMI088_Read(BMI088_HandleTypeDef *BMI088, uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *rxBuffer, uint16_t Size);
static void CS_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void CS_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* Data Processing Functions */
static inline void bmi088_get_acc_data(BMI088_HandleTypeDef *BMI088);
static inline void bmi088_get_gyro_data(BMI088_HandleTypeDef *BMI088);

/* Offset Calibration Functions */
static void bmi088_get_offset_acc(BMI088_HandleTypeDef *BMI088);
static void bmi088_get_offset_gyro(BMI088_HandleTypeDef *BMI088);

/* Accelerometer Configuration Functions */
static HAL_StatusTypeDef bmi088_set_acc_pwr_ctrl(BMI088_HandleTypeDef *BMI088, bmi088_acc_pwr_ctrl_t state);
static HAL_StatusTypeDef bmi088_set_acc_pwr_mode(BMI088_HandleTypeDef *BMI088, bmi088_acc_pwr_mode_t mode);
static HAL_StatusTypeDef bmi088_set_acc_range(BMI088_HandleTypeDef *BMI088);
static HAL_StatusTypeDef bmi088_set_acc_conf_reg(BMI088_HandleTypeDef *BMI088);
static HAL_StatusTypeDef bmi088_softreset_acc(BMI088_HandleTypeDef *BMI088, uint8_t command);
static HAL_StatusTypeDef bmi088_acc_activate_int1(BMI088_HandleTypeDef *BMI088);
static uint8_t bmi088_acc_check_drdy(BMI088_HandleTypeDef *BMI088);

/* Gyroscope Configuration Functions */
static HAL_StatusTypeDef bmi088_set_gyro_pwr_mode(BMI088_HandleTypeDef *BMI088);
static HAL_StatusTypeDef bmi088_set_gyro_range(BMI088_HandleTypeDef *BMI088);
static HAL_StatusTypeDef bmi088_set_gyro_bandwidth(BMI088_HandleTypeDef *BMI088);
static HAL_StatusTypeDef bmi088_softreset_gyro(BMI088_HandleTypeDef *BMI088, uint8_t command);
static HAL_StatusTypeDef bmi088_gyro_activate_int3(BMI088_HandleTypeDef *BMI088);
static uint8_t bmi088_gyro_check_drdy(BMI088_HandleTypeDef *BMI088);


/**
 * @brief Initializes the BMI088 sensor and checks communication of both devices.
 * @param BMI088 Pointer to the handle structure.
 * @retval bmi088_status_t.
 */
bmi088_status_t BMI088_Init(BMI088_HandleTypeDef *BMI088)
{
	BMI088->flags.is_accel_updated = 0;
	BMI088->flags.is_gyro_updated = 0;
	BMI088->flags.is_acc_started = 0;
	BMI088->flags.is_gyro_started = 0;
	BMI088->flags.is_acc_new = 0;
	BMI088->flags.is_gyro_new = 0;

	uint8_t rx_buffer[2];
	// First accelerometer read is a dummy read, it switches the accelerometer to SPI mode
	BMI088_Read(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_CHIP_ID, rx_buffer, 1);
	HAL_Delay(5);

	BMI088_Read(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_CHIP_ID, rx_buffer, 1);
	BMI088_Read(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_CHIP_ID, &rx_buffer[1], 1);

	// 0x1E is reset value for ACC_CHIP_ID
	// 0x0F is reset value for GYRO_CHIP_ID
	if (rx_buffer[0] == 0x1E && rx_buffer[1] == 0x0F)
	{
		return BMI088_OK;
	}
	else if (rx_buffer[0] != 0x1E && rx_buffer[1] == 0x0F)
	{
		return BMI088_ACC_FAILED;
	}
	else if (rx_buffer[0] == 0x1E && rx_buffer[1] != 0x0F)
	{
		return BMI088_GYRO_FAILED;
	}
	else
	{
		return BMI088_BOTH_FAILED;
	}
}

/**
 * @brief Fully configures accelerometer and gyroscope parameters.
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Config(BMI088_HandleTypeDef *BMI088)
{
	HAL_NVIC_DisableIRQ(BMI088->configuration.acc_IRQ);
	HAL_NVIC_DisableIRQ(BMI088->configuration.gyro_IRQ);

	// Softreset
	bmi088_set_acc_pwr_mode(BMI088, BMI088_ACC_PWR_MODE_ADVANCED_PWR_SAVE);
	bmi088_set_acc_pwr_ctrl(BMI088, BMI088_ACC_OFF);
	HAL_Delay(20);
	bmi088_softreset_acc(BMI088, BMI088_ACC_SOFTRESET_COMMAND);
	HAL_Delay(10);
	bmi088_softreset_acc(BMI088, BMI088_ACC_FIFO_SOFTRESET_COMMAND);
	HAL_Delay(10);
	bmi088_softreset_gyro(BMI088, BMI088_GYRO_SOFTRESET_COMMAND);
	HAL_Delay(35);

	// Accelerometer configuration
	bmi088_set_acc_pwr_ctrl(BMI088, BMI088_ACC_ON);
	HAL_Delay(8);
	bmi088_set_acc_pwr_mode(BMI088, BMI088->configuration.acc_power_mode);
	HAL_Delay(8);
	bmi088_set_acc_range(BMI088);
	bmi088_set_acc_conf_reg(BMI088);
	bmi088_acc_activate_int1(BMI088);

	// Gyroscope configuration
	bmi088_set_gyro_range(BMI088);
	bmi088_set_gyro_bandwidth(BMI088);
	bmi088_set_gyro_pwr_mode(BMI088);
	HAL_Delay(20);
	bmi088_gyro_activate_int3(BMI088);

	HAL_NVIC_EnableIRQ(BMI088->configuration.acc_IRQ);
	HAL_NVIC_EnableIRQ(BMI088->configuration.gyro_IRQ);
	HAL_Delay(50);
}

/**
 * @brief Gets accelerometer and gyroscope offset calibration.
 * @param BMI088 Pointer to the handle structure.
 * @note The accelerometer offset should be calibrated beforehand on a flat surface with the
 *       +Z axis facing up (the runtime calibration below expects 0 g on X/Y and +1 g on Z),
 *       and the measured values should be written to the offset defines in bmi088.h.
 *       When ROCKET_CARD is defined those defines are used directly, so before flight
 *       only the gyroscope offset is taken at runtime.
 */
void BMI088_GetOffset(BMI088_HandleTypeDef *BMI088)
{
	bmi088_get_offset_acc(BMI088);
	bmi088_get_offset_gyro(BMI088);
}

/**
 * @brief Processes data if the software 'is_accel_updated' / 'is_gyro_updated' flags are set (useful for Interrupt mode).
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Update(BMI088_HandleTypeDef *BMI088)
{
	if (BMI088->flags.is_accel_updated)
	{
		BMI088->flags.is_accel_updated = 0;
		bmi088_get_acc_data(BMI088);
	}

	if (BMI088->flags.is_gyro_updated)
	{
		BMI088->flags.is_gyro_updated = 0;
		bmi088_get_gyro_data(BMI088);
	}
}

/**
 * @brief Manually checks the sensors' status registers and updates data if ready (Polling mode).
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Update_drdy(BMI088_HandleTypeDef *BMI088)
{
	if (bmi088_acc_check_drdy(BMI088))
	{
		bmi088_get_acc_data(BMI088);
	}

	if (bmi088_gyro_check_drdy(BMI088))
	{
		bmi088_get_gyro_data(BMI088);
	}
}

/**
 * @brief Callback function to be called from the accelerometer GPIO External Interrupt (EXTI).
 * @param BMI088 Pointer to the handle structure.
 */
void bmi088_set_accel_INT(BMI088_HandleTypeDef *BMI088)
{
	BMI088->flags.is_accel_updated = 1;
}

/**
 * @brief Callback function to be called from the gyroscope GPIO External Interrupt (EXTI).
 * @param BMI088 Pointer to the handle structure.
 */
void bmi088_set_gyro_INT(BMI088_HandleTypeDef *BMI088)
{
	BMI088->flags.is_gyro_updated = 1;
}


/* --- Private Functions Definitions --- */

static inline HAL_StatusTypeDef BMI088_Write(BMI088_HandleTypeDef *BMI088, uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *data)
{
	// bmi088 datasheet 6.1.2 SPI interface of accelerometer part
	if (DeviceAddress == BMI088_ACC_DEV_ADDR)
	{
		uint8_t tx_data[2];
		tx_data[0] = MemAddress & (uint8_t) 0x7F;
		tx_data[1] = *data;

		CS_Low(BMI088->configuration.accel_cs_gpio_port, BMI088->configuration.accel_cs_gpio_pin);

		HAL_StatusTypeDef status = HAL_SPI_Transmit(BMI088->configuration.spi, tx_data, 2, 5);

		CS_High(BMI088->configuration.accel_cs_gpio_port, BMI088->configuration.accel_cs_gpio_pin);

		return status;
	}
	// bmi088 datasheet 6.1.1 SPI interface of gyroscope part
	else if (DeviceAddress == BMI088_GYRO_DEV_ADDR)
	{
		uint8_t tx_data[2];
		tx_data[0] = MemAddress & (uint8_t) 0x7F;
		tx_data[1] = *data;

		CS_Low(BMI088->configuration.gyro_cs_gpio_port, BMI088->configuration.gyro_cs_gpio_pin);

		HAL_StatusTypeDef status = HAL_SPI_Transmit(BMI088->configuration.spi, tx_data, 2, 5);

		CS_High(BMI088->configuration.gyro_cs_gpio_port, BMI088->configuration.gyro_cs_gpio_pin);

		return status;
	}
	else
	{
		return HAL_ERROR;
	}
}

static inline HAL_StatusTypeDef BMI088_Read(BMI088_HandleTypeDef *BMI088, uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *rxBuffer, uint16_t Size)
{
	if (Size > BMI088_BUFFER_MAX_SIZE)
	{
		return HAL_ERROR;
	}

    // bmi088 datasheet 6.1.2 SPI interface of accelerometer part
	// Accelerometer sends a dummy byte after the address byte, so the payload starts at rxData[2]
	if (DeviceAddress == BMI088_ACC_DEV_ADDR)
	{
		uint8_t txData[BMI088_BUFFER_MAX_SIZE + 2];
		uint8_t rxData[BMI088_BUFFER_MAX_SIZE + 2];

		memset(txData, 0, Size + 2);
		memset(rxData, 0, Size + 2);
		txData[0] = MemAddress | (uint8_t) 0x80;

		CS_Low(BMI088->configuration.accel_cs_gpio_port, BMI088->configuration.accel_cs_gpio_pin);

		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(BMI088->configuration.spi, txData, rxData, Size + 2, 5);

		CS_High(BMI088->configuration.accel_cs_gpio_port, BMI088->configuration.accel_cs_gpio_pin);

		if (status == HAL_OK)
		{
			memcpy(rxBuffer, &rxData[2], Size);
		}

		return status;
	}
	// bmi088 datasheet 6.1.1 SPI interface of gyroscope part
    // Gyroscope has no dummy byte, so the payload starts at rxData[1]
	else if (DeviceAddress == BMI088_GYRO_DEV_ADDR)
	{
		uint8_t txData[BMI088_BUFFER_MAX_SIZE + 1];
		uint8_t rxData[BMI088_BUFFER_MAX_SIZE + 1];

		memset(txData, 0, Size + 1);
		memset(rxData, 0, Size + 1);
		txData[0] = MemAddress | (uint8_t) 0x80;

		CS_Low(BMI088->configuration.gyro_cs_gpio_port, BMI088->configuration.gyro_cs_gpio_pin);

		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(BMI088->configuration.spi, txData, rxData, Size + 1, 5);

		CS_High(BMI088->configuration.gyro_cs_gpio_port, BMI088->configuration.gyro_cs_gpio_pin);

		if (status == HAL_OK)
		{
			memcpy(rxBuffer, &rxData[1], Size);
		}

		return status;
	}
	else
	{
		return HAL_ERROR;
	}
}

static void CS_Low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

static void CS_High(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}


static inline void bmi088_get_acc_data(BMI088_HandleTypeDef *BMI088)
{
	if (BMI088_Read(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_X_LSB, BMI088->data.raw_acc_data, 9) == HAL_OK)
	{
		int16_t accel_x_int16 = (BMI088->data.raw_acc_data[1] << 8) | BMI088->data.raw_acc_data[0];
		int16_t accel_y_int16 = (BMI088->data.raw_acc_data[3] << 8) | BMI088->data.raw_acc_data[2];
		int16_t accel_z_int16 = (BMI088->data.raw_acc_data[5] << 8) | BMI088->data.raw_acc_data[4];

		BMI088->data.acc_x = ((float) accel_x_int16 * BMI088->data.acc_sensitivity_factor - BMI088->configuration.offsets.acc_offset[0]) * 9.81f;
		BMI088->data.acc_y = ((float) accel_y_int16 * BMI088->data.acc_sensitivity_factor - BMI088->configuration.offsets.acc_offset[1]) * 9.81f;
		BMI088->data.acc_z = ((float) accel_z_int16 * BMI088->data.acc_sensitivity_factor - BMI088->configuration.offsets.acc_offset[2]) * 9.81f;

		uint32_t time = BMI088->data.raw_acc_data[8] << 16 | BMI088->data.raw_acc_data[7] << 8 | BMI088->data.raw_acc_data[6];
		BMI088->data.acc_current_time = (float) time * 39.0625 / 1000000.0;
		if (BMI088->flags.is_acc_started)
		{
			BMI088->data.acc_dt = BMI088->data.acc_current_time - BMI088->data.acc_last_time < 0.0 ? 0.0 : BMI088->data.acc_current_time - BMI088->data.acc_last_time;
		}
		else
		{
			BMI088->flags.is_acc_started = 1;
			BMI088->data.acc_dt = 0.0f;
		}
		BMI088->data.acc_last_time = BMI088->data.acc_current_time;

		BMI088->flags.is_acc_new = 1;
	}
}

static inline void bmi088_get_gyro_data(BMI088_HandleTypeDef *BMI088)
{
	if (BMI088_Read(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_X_LSB, BMI088->data.raw_gyro_data, 6) == HAL_OK)
	{
		int16_t gyro_x_int16 = (BMI088->data.raw_gyro_data[1] << 8) | BMI088->data.raw_gyro_data[0];
		int16_t gyro_y_int16 = (BMI088->data.raw_gyro_data[3] << 8) | BMI088->data.raw_gyro_data[2];
		int16_t gyro_z_int16 = (BMI088->data.raw_gyro_data[5] << 8) | BMI088->data.raw_gyro_data[4];

		BMI088->data.gyro_x = ((float) gyro_x_int16 * BMI088->data.gyro_sensitivity_factor - BMI088->configuration.offsets.gyro_offset[0]) * DEG_TO_RAD;
		BMI088->data.gyro_y = ((float) gyro_y_int16 * BMI088->data.gyro_sensitivity_factor - BMI088->configuration.offsets.gyro_offset[1]) * DEG_TO_RAD;
		BMI088->data.gyro_z = ((float) gyro_z_int16 * BMI088->data.gyro_sensitivity_factor - BMI088->configuration.offsets.gyro_offset[2]) * DEG_TO_RAD;

		BMI088->data.gyro_current_time = __HAL_TIM_GET_COUNTER(BMI088->configuration.htim);
		if (BMI088->flags.is_gyro_started)
		{
			BMI088->data.gyro_dt = (float) (BMI088->data.gyro_current_time - BMI088->data.gyro_last_time) * 1e-6f;
		}
		else
		{
			BMI088->flags.is_gyro_started = 1;
			BMI088->data.gyro_dt = 0.0f;
		}
		BMI088->data.gyro_last_time = BMI088->data.gyro_current_time;

		BMI088->flags.is_gyro_new = 1;
	}
}


static void bmi088_get_offset_acc(BMI088_HandleTypeDef *BMI088)
{
#ifdef ROCKET_CARD
	BMI088->configuration.offsets.acc_offset[0] = BMI088_ACC_X_OFFSET;
	BMI088->configuration.offsets.acc_offset[1] = BMI088_ACC_Y_OFFSET;
	BMI088->configuration.offsets.acc_offset[2] = BMI088_ACC_Z_OFFSET;
#else
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;

	uint16_t counter = 0;
	uint32_t start_time = HAL_GetTick();
	const uint32_t timeout = 5000;
	while (counter < 1000)
	{
		if (HAL_GetTick() - start_time > timeout)
		{
			break;
		}

		if (BMI088->flags.is_accel_updated)
		{
			BMI088->flags.is_accel_updated = 0;
			uint8_t rx_data[6];
			if (BMI088_Read(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_X_LSB, rx_data, 6) == HAL_OK)
			{
				int16_t accel_x_int16 = (rx_data[1] << 8) | rx_data[0];
				int16_t accel_y_int16 = (rx_data[3] << 8) | rx_data[2];
				int16_t accel_z_int16 = (rx_data[5] << 8) | rx_data[4];

				total_x += (float)accel_x_int16 * BMI088->data.acc_sensitivity_factor;
				total_y += (float)accel_y_int16 * BMI088->data.acc_sensitivity_factor;
				total_z += (float)accel_z_int16 * BMI088->data.acc_sensitivity_factor;

				counter++;
			}
		}
	}

	if (counter > 0)
	{
		BMI088->configuration.offsets.acc_offset[0] = total_x / counter;
		BMI088->configuration.offsets.acc_offset[1] = total_y / counter;
		BMI088->configuration.offsets.acc_offset[2] = total_z / counter - 1;
	}
#endif
}

static void bmi088_get_offset_gyro(BMI088_HandleTypeDef *BMI088)
{
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;

	uint16_t counter = 0;
	uint32_t start_time = HAL_GetTick();
	const uint32_t timeout = 5000;
	while (counter < 1000)
	{
		if (HAL_GetTick() - start_time > timeout)
		{
			break;
		}

		if (BMI088->flags.is_gyro_updated)
		{
			BMI088->flags.is_gyro_updated = 0;
			uint8_t rx_data[6];
			if (BMI088_Read(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_X_LSB, rx_data, 6) == HAL_OK)
			{
				int16_t gyro_x_int16 = (rx_data[1] << 8) | rx_data[0];
				int16_t gyro_y_int16 = (rx_data[3] << 8) | rx_data[2];
				int16_t gyro_z_int16 = (rx_data[5] << 8) | rx_data[4];

				total_x += (float) gyro_x_int16 * BMI088->data.gyro_sensitivity_factor;
				total_y += (float) gyro_y_int16 * BMI088->data.gyro_sensitivity_factor;
				total_z += (float) gyro_z_int16 * BMI088->data.gyro_sensitivity_factor;

				counter++;
			}
		}
	}

	if (counter > 0)
	{
		BMI088->configuration.offsets.gyro_offset[0] = total_x / counter;
		BMI088->configuration.offsets.gyro_offset[1] = total_y / counter;
		BMI088->configuration.offsets.gyro_offset[2] = total_z / counter;
	}
}


static HAL_StatusTypeDef bmi088_set_acc_pwr_ctrl(BMI088_HandleTypeDef *BMI088, bmi088_acc_pwr_ctrl_t state)
{
	uint8_t data = (uint8_t) state;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_PWR_CTRL, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_set_acc_pwr_mode(BMI088_HandleTypeDef *BMI088, bmi088_acc_pwr_mode_t mode)
{
	uint8_t data = (uint8_t) mode;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_PWR_CONF, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_set_acc_range(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = (uint8_t) BMI088->configuration.acc_range;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_RANGE, &data);
	HAL_Delay(1);

	BMI088->data.acc_sensitivity_factor = (float) (pow(2, data + 1) * 1.5 / 32768.0);

	return status;
}

static HAL_StatusTypeDef bmi088_set_acc_conf_reg(BMI088_HandleTypeDef *BMI088)
{
	uint8_t odr = (uint8_t) BMI088->configuration.acc_odr;
	uint8_t bandwidth = (uint8_t) BMI088->configuration.acc_bandwidth;
	uint8_t data = (bandwidth << 4) | odr;

	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_CONF, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_softreset_acc(BMI088_HandleTypeDef *BMI088, uint8_t command)
{
	uint8_t data = (uint8_t) command;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_SOFTRESET, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_acc_activate_int1(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = BMI088_ACC_INT1_OUTPUT | BMI088_ACC_INT1_PUSH_PULL | BMI088_ACC_INT1_ACTIVE_HIGH;
	HAL_StatusTypeDef status_1 = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_INT1_IO_CTRL, &data);
	HAL_Delay(1);

	// Map data ready interrupt to pin INT1
	data = (0x01 << 2);
	HAL_StatusTypeDef status_2 = BMI088_Write(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_INT_MAP_DATA, &data);
	HAL_Delay(1);

	if (status_1 == HAL_OK && status_2 == HAL_OK)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

// Check if new accel data available
static uint8_t bmi088_acc_check_drdy(BMI088_HandleTypeDef *BMI088)
{
	uint8_t rx_data;

	if (BMI088_Read(BMI088, BMI088_ACC_DEV_ADDR, BMI088_REG_ACC_STATUS, &rx_data, 1) != HAL_OK)
	{
		return 0;
	}

	// Bit #7 of BMI088_REG_ACC_STATUS register is drdy_acc bit which is data ready status
	rx_data = rx_data >> 7;

	if (rx_data != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


static HAL_StatusTypeDef bmi088_set_gyro_pwr_mode(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = (uint8_t) BMI088->configuration.gyro_pwr_mode;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_LPM1, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_set_gyro_range(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = (uint8_t) BMI088->configuration.gyro_range;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_RANGE, &data);
	HAL_Delay(1);

	BMI088->data.gyro_sensitivity_factor = ((float) (2000 >> data)) / 32768.0;

	return status;
}

static HAL_StatusTypeDef bmi088_set_gyro_bandwidth(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = (uint8_t) BMI088->configuration.gyro_bandwidth;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_BANDWIDTH, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_softreset_gyro(BMI088_HandleTypeDef *BMI088, uint8_t command)
{
	uint8_t data = (uint8_t) command;
	HAL_StatusTypeDef status = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_SOFTRESET, &data);
	HAL_Delay(1);
	return status;
}

static HAL_StatusTypeDef bmi088_gyro_activate_int3(BMI088_HandleTypeDef *BMI088)
{
	uint8_t data = BMI088_GYRO_INT_ENABLE;
	HAL_StatusTypeDef status_1 = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_INT_CTRL, &data);
	HAL_Delay(1);

	data = BMI088_GYRO_INT3_PUSH_PULL | BMI088_GYRO_INT3_ACTIVE_HIGH;
	HAL_StatusTypeDef status_2 = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_INT3_INT4_IO_CONF, &data);
	HAL_Delay(1);

	data = 0x01;
	HAL_StatusTypeDef status_3 = BMI088_Write(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_INT3_INT4_IO_MAP, &data);
	HAL_Delay(1);

	if (status_1 == HAL_OK && status_2 == HAL_OK && status_3 == HAL_OK)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

// Check if new gyro data available
static uint8_t bmi088_gyro_check_drdy(BMI088_HandleTypeDef *BMI088)
{
	uint8_t rx_data;
	if (BMI088_Read(BMI088, BMI088_GYRO_DEV_ADDR, BMI088_REG_GYRO_INT_STAT_1, &rx_data, 1) != HAL_OK)
	{
		return 0;
	}

	// Bit #7 of BMI088_REG_GYRO_INT_STAT_1 register is gyro_drdy bit which is data ready interrupt status
	// The interrupt is cleared automatically after 280 - 400 microsecond
	rx_data &= 0x80;
	if (rx_data != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

