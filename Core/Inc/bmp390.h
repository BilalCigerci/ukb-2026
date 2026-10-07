/*
 * bmp390.h
 *
 *  Created on: Dec 27, 2025
 *      Author: bilal
 *  
 *  References:
 *          https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_


#include "main.h"


// Device Address
// if SDO connected to GND device address -> 0x76
// if SDO connected to Vddio device address -> 0x77
#define BMP390_DEV_ADDR										0x77 << 1

// Register Map
#define BMP390_REG_CMD										0x7E
#define BMP390_REG_TRIMMING_COEFFICIENTS					0x31
#define BMP390_REG_CONFIG									0x1F
#define BMP390_REG_ODR										0x1D
#define BMP390_REG_OSR										0x1C
#define BMP390_REG_PWR_CTRL									0x1B
#define BMP390_REG_IF_CONF									0x1A
#define BMP390_REG_INT_CTRL									0x19
#define BMP390_REG_FIFO_CONFIG_2							0x18
#define BMP390_REG_FIFO_CONFIG_1							0x17
#define BMP390_REG_FIFO_WTM_1								0x16
#define BMP390_REG_FIFO_WTM_0								0x15
#define BMP390_REG_FIFO_DATA								0x14
#define BMP390_REG_FIFO_LENGTH_1							0x13
#define BMP390_REG_FIFO_LENGTH_0							0x12
#define BMP390_REG_INT_STATUS								0x11
#define BMP390_REG_EVENT									0x10
#define BMP390_REG_SENSORTIME_2								0x0E
#define BMP390_REG_SENSORTIME_1								0x0D
#define BMP390_REG_SENSORTIME_0								0x0C
#define BMP390_REG_TEMP_DATA_2								0x09
#define BMP390_REG_TEMP_DATA_1								0x08
#define BMP390_REG_TEMP_DATA_0								0x07
#define BMP390_REG_PRESS_DATA_2								0x06
#define BMP390_REG_PRESS_DATA_1								0x05
#define BMP390_REG_PRESS_DATA_0								0x04
#define BMP390_REG_STATUS									0x03
#define BMP390_REG_ERR										0x02
#define BMP390_REG_REV_ID									0x01
#define BMP390_REG_CHIP_ID									0x00

// Softreset command
#define BMP390_SOFTRESET									(uint8_t)0xB6

// BMP390_REG_INT_CTRL register definitions
#define BMP390_INT_PUSH_PULL								(uint8_t)0x00
#define BMP390_INT_OPEN_DRAIN								(uint8_t)0x01

#define BMP390_INT_ACTIVE_LOW								(uint8_t)(0x00 << 1)
#define BMP390_INT_ACTIVE_HIGH								(uint8_t)(0x01 << 1)

#define BMP390_INT_LATCH_DISABLED							(uint8_t)(0x00 << 2)
#define BMP390_INT_LATCH_ENABLED							(uint8_t)(0x01 << 2)

#define BMP390_INT_FIFO_WATERMARK_DISABLED					(uint8_t)(0x00 << 3)
#define BMP390_INT_FIFO_WATERMARK_ENABLED					(uint8_t)(0x01 << 3)

#define BMP390_INT_FIFO_FULL_DISABLED						(uint8_t)(0x00 << 4)
#define BMP390_INT_FIFO_FULL_ENABLED						(uint8_t)(0x01 << 4)

#define BMP390_INT_DS_LOW									(uint8_t)(0x00 << 5)
#define BMP390_INT_DS_HIGH									(uint8_t)(0x01 << 5)

#define BMP390_INT_DATA_READY_DISABLED						(uint8_t)(0x00 << 6)
#define BMP390_INT_DATA_READY_ENABLED						(uint8_t)(0x01 << 6)

// BMP390_REG_IF_CONF register definitions
#define BMP390_SPI_4_WIRE_MODE								(uint8_t)0x00
#define BMP390_SPI_3_WIRE_MODE								(uint8_t)0x01

#define BMP390_I2C_WATCHDOG_DISABLED						(uint8_t)(0x00 << 1)
#define BMP390_I2C_WATCHDOG_ENABLED							(uint8_t)(0x01 << 1)

#define BMP390_I2C_WATCHDOG_SHORT							(uint8_t)(0x00 << 2)			// I2C watchdog timeout after 1.25 ms
#define BMP390_I2C_WATCHDOG_LONG							(uint8_t)(0x01 << 2)			// I2C watchdog timeout after 40 ms

// BMP390_REG_PWR_CTRL register definitions
#define BMP390_PRESSURE_OFF									(uint8_t)0x00
#define BMP390_PRESSURE_ON									(uint8_t)0x01

#define BMP390_TEMPERATURE_OFF								(uint8_t)(0x00 << 1)
#define BMP390_TEMPERATURE_ON								(uint8_t)(0x01 << 1)

#define BMP390_SLEEP_MODE									(uint8_t)(0x00 << 4)
#define BMP390_FORCED_MODE									(uint8_t)(0x01 << 4)
#define BMP390_NORMAL_MODE									(uint8_t)(0x03 << 4)

// BMP390_REG_OSR register definitions
#define BMP390_PRESSURE_OSR_1								(uint8_t)0x00
#define BMP390_PRESSURE_OSR_2								(uint8_t)0x01
#define BMP390_PRESSURE_OSR_4								(uint8_t)0x02
#define BMP390_PRESSURE_OSR_8								(uint8_t)0x03
#define BMP390_PRESSURE_OSR_16								(uint8_t)0x04
#define BMP390_PRESSURE_OSR_32								(uint8_t)0x05

#define BMP390_TEMPERATURE_OSR_1							(uint8_t)(0x00 << 3)
#define BMP390_TEMPERATURE_OSR_2							(uint8_t)(0x01 << 3)
#define BMP390_TEMPERATURE_OSR_4							(uint8_t)(0x02 << 3)
#define BMP390_TEMPERATURE_OSR_8							(uint8_t)(0x03 << 3)
#define BMP390_TEMPERATURE_OSR_16							(uint8_t)(0x04 << 3)
#define BMP390_TEMPERATURE_OSR_32							(uint8_t)(0x05 << 3)

// BMP390_REG_ODR register definitions
#define BMP390_ODR_200										(uint8_t)0x00
#define BMP390_ODR_100										(uint8_t)0x01
#define BMP390_ODR_50										(uint8_t)0x02
#define BMP390_ODR_25										(uint8_t)0x03

// BMP390_REG_CONFIG register definitions
#define BMP390_IIR_FILTER_COEFFICIENT_0						(uint8_t)(0x00 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_1						(uint8_t)(0x01 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_3						(uint8_t)(0x02 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_7						(uint8_t)(0x03 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_15					(uint8_t)(0x04 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_31					(uint8_t)(0x05 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_63					(uint8_t)(0x06 << 1)
#define BMP390_IIR_FILTER_COEFFICIENT_127					(uint8_t)(0x07 << 1)


/**
 * @brief BMP390 driver status enumeration.
 */
typedef enum
{
	BMP390_OK = 0x00,
	BMP390_ERROR = 0x01
}bmp390_status_t;


/**
 * @brief Calibration coefficients stored in NVM (Non-Volatile Memory)
 * and their calculated floating-point equivalents with respect to bmp390 datasheet.
 */
typedef struct
{
	uint16_t NVM_PAR_T1;
	uint16_t NVM_PAR_T2;
	int8_t NVM_PAR_T3;
	int16_t NVM_PAR_P1;
	int16_t NVM_PAR_P2;
	int8_t NVM_PAR_P3;
	int8_t NVM_PAR_P4;
	uint16_t NVM_PAR_P5;
	uint16_t NVM_PAR_P6;
	int8_t NVM_PAR_P7;
	int8_t NVM_PAR_P8;
	int16_t NVM_PAR_P9;
	int8_t NVM_PAR_P10;
	int8_t NVM_PAR_P11;

	float PAR_T1;
	float PAR_T2;
	float PAR_T3;
	float PAR_P1;
	float PAR_P2;
	float PAR_P3;
	float PAR_P4;
	float PAR_P5;
	float PAR_P6;
	float PAR_P7;
	float PAR_P8;
	float PAR_P9;
	float PAR_P10;
	float PAR_P11;
}bmp390_trimming_coefficients_t;

/**
 * @brief Structure to store processed sensor data and raw readings.
 */
typedef struct
{
	float pressure;
	float temperature;
	float altitude;
	float base_altitude;
	uint32_t raw_pressure;
	uint32_t raw_temperature;
	uint32_t raw_sensortime;
    uint32_t last_int_time;
}bmp390_data_t;

/**
 * @brief Sensor configuration parameters and hardware interface handles.
 */
typedef struct
{
	uint8_t mode;
	uint8_t odr;
	uint8_t osr_pressure;
	uint8_t osr_temperature;
	uint8_t iir_filter_coefficient;
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *cs_gpio_port;
	uint16_t cs_gpio_pin;
	IRQn_Type IRQ;
}bmp390_configuration_t;

/**
 * @brief State flags for data management and interrupt handling.
 */
typedef struct
{
	uint8_t is_data_ready;
	uint8_t is_data_updated;
    uint8_t is_altitude_new;
    uint8_t is_updated_with_int;
}bmp390_flags_t;

/**
 * @brief Main BMP390 Handle structure.
 * Combines configuration, data, and internal state.
 */
typedef struct
{
	bmp390_trimming_coefficients_t trimming_coefficients;
	bmp390_configuration_t configuration;
	bmp390_data_t data;
	volatile bmp390_flags_t flags;
}BMP390_HandleTypeDef;


/**
 * @brief Initializes the BMP390 sensor and checks communication.
 * @param BMP390 Pointer to the handle structure.
 * @retval bmp390_status_t.
 */
bmp390_status_t BMP390_Init(BMP390_HandleTypeDef *BMP390);

/**
 * @brief Fully configures sensor parameters.
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_Config(BMP390_HandleTypeDef *BMP390);

/**
 * @brief Processes data if the software 'is_data_ready' flag is set (useful for Interrupt mode).
 * @param BMP390 Pointer to the handle structure.
 * @note The data ready interrupt is the normal path and 'last_int_time' is stamped every time it
 *       delivers a sample. When no interrupt arrives for BMP390_INT_TIMEOUT_MS the interrupt line
 *       is treated as lost and the status register is polled instead.
 * @note 'is_updated_with_int' reports which of the two paths was taken last, 1 for the interrupt and
 *       0 for the polling fallback. It is only used for logging, the data itself is identical.
 */
void BMP390_Update(BMP390_HandleTypeDef *BMP390);

/**
 * @brief Manually checks the sensor's status register and updates data if ready (Polling mode).
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_Update_drdy(BMP390_HandleTypeDef *BMP390);

/**
 * @brief Gets base altitude calibration.
 * @param BMP390 Pointer to the handle structure.
 */
void BMP390_SetBaseAltitude(BMP390_HandleTypeDef *BMP390);

/**
 * @brief Callback function to be called from the GPIO External Interrupt (EXTI).
 * @param BMP390 Pointer to the handle structure.
 */
void bmp390_set_INT(BMP390_HandleTypeDef *BMP390);


#endif /* INC_BMP390_H_ */
