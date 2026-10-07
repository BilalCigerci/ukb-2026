/*
 * bmi088.h
 *
 *  Created on: Dec 17, 2025
 *      Author: bilal
 *
 *  References:
 *          https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 *  		https://github.com/yahio61/bmi_088_library/blob/main/bmi088/bmi088.h
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_


#include "main.h"


// When defined, the accelerometer offsets below are used instead of runtime calibration
#define ROCKET_CARD

#ifdef ROCKET_CARD
#define BMI088_ACC_X_OFFSET	(float)0.000
#define BMI088_ACC_Y_OFFSET (float)0.000
#define BMI088_ACC_Z_OFFSET (float)0.000
#endif


// Device Addresses
#define BMI088_ACC_DEV_ADDR 							(uint8_t)(0x18 << 1)
#define BMI088_GYRO_DEV_ADDR							(uint8_t)(0x68 << 1)

// Accelerometer Register Map
#define BMI088_REG_ACC_SOFTRESET						0x7E
#define BMI088_REG_ACC_PWR_CTRL							0x7D
#define BMI088_REG_ACC_PWR_CONF							0x7C
#define BMI088_REG_ACC_SELF_TEST						0x6D
#define BMI088_REG_ACC_INT_MAP_DATA						0x58
#define BMI088_REG_ACC_INT2_IO_CTRL						0x54
#define BMI088_REG_ACC_INT1_IO_CTRL						0x53
#define BMI088_REG_ACC_FIFO_CONFIG_1					0x49
#define BMI088_REG_ACC_FIFO_CONFIG_0					0x48
#define BMI088_REG_ACC_FIFO_WTM_1						0x47
#define BMI088_REG_ACC_FIFO_WTM_0						0x46
#define BMI088_REG_ACC_FIFO_DOWNS						0x45
#define BMI088_REG_ACC_RANGE							0x41
#define BMI088_REG_ACC_CONF								0x40
#define BMI088_REG_ACC_FIFO_DATA						0x26
#define BMI088_REG_ACC_FIFO_LENGTH_1					0x25
#define BMI088_REG_ACC_FIFO_LENGTH_0					0x24
#define BMI088_REG_ACC_TEMP_LSB							0x23
#define BMI088_REG_ACC_TEMP_MSB							0x22
#define BMI088_REG_ACC_INT_STAT_1						0x1D
#define BMI088_REG_ACC_SENSORTIME_2						0x1A
#define BMI088_REG_ACC_SENSORTIME_1						0x19
#define BMI088_REG_ACC_SENSORTIME_0						0x18
#define BMI088_REG_ACC_Z_MSB							0x17
#define BMI088_REG_ACC_Z_LSB							0x16
#define BMI088_REG_ACC_Y_MSB							0x15
#define BMI088_REG_ACC_Y_LSB							0x14
#define BMI088_REG_ACC_X_MSB							0x13
#define BMI088_REG_ACC_X_LSB							0x12
#define BMI088_REG_ACC_STATUS							0x03
#define BMI088_REG_ACC_ERR_REG							0x02
#define BMI088_REG_ACC_CHIP_ID							0x00

// Gyroscope Register Map
#define BMI088_REG_GYRO_FIFO_DATA						0x3F
#define BMI088_REG_GYRO_FIFO_CONFIG_1					0x3E
#define BMI088_REG_GYRO_FIFO_CONFIG_0					0x3D
#define BMI088_REG_GYRO_SELF_TEST						0x3C
#define BMI088_REG_GYRO_FIFO_EXT_INT_S				    0x34
#define BMI088_REG_GYRO_FIFO_WM_EN						0x1E
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP			    0x18
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF			    0x16
#define BMI088_REG_GYRO_INT_CTRL						0x15
#define BMI088_REG_GYRO_SOFTRESET						0x14
#define BMI088_REG_GYRO_LPM1							0x11
#define BMI088_REG_GYRO_BANDWIDTH						0x10
#define BMI088_REG_GYRO_RANGE							0x0F
#define BMI088_REG_GYRO_FIFO_STATUS						0x0E
#define BMI088_REG_GYRO_INT_STAT_1						0x0A
#define BMI088_REG_GYRO_Z_MSB							0x07
#define BMI088_REG_GYRO_Z_LSB							0x06
#define BMI088_REG_GYRO_Y_MSB							0x05
#define BMI088_REG_GYRO_Y_LSB							0x04
#define BMI088_REG_GYRO_X_MSB							0x03
#define BMI088_REG_GYRO_X_LSB							0x02
#define BMI088_REG_GYRO_CHIP_ID							0x00

// Softreset command
#define BMI088_ACC_SOFTRESET_COMMAND					0xB6
#define BMI088_ACC_FIFO_SOFTRESET_COMMAND			    0xB0
#define BMI088_GYRO_SOFTRESET_COMMAND					0xB6

// BMI088_REG_ACC_INT1_IO_CTRL register definitions
#define BMI088_ACC_INT1_INPUT							(0x01 << 4)		// Enable INT1 pin as input pin
#define BMI088_ACC_INT1_OUTPUT							(0x01 << 3)		// Enable INT1 pin as output pin
#define BMI088_ACC_INT1_PUSH_PULL						(0x00 << 2)
#define BMI088_ACC_INT1_OPEN_DRAIN						(0x01 << 2)
#define BMI088_ACC_INT1_ACTIVE_LOW						(0x00 << 1)
#define BMI088_ACC_INT1_ACTIVE_HIGH						(0x01 << 1)

// BMI088_REG_GYRO_INT_CTRL register definitions
#define BMI088_GYRO_INT_ENABLE							(0x01 << 7)
#define BMI088_GYRO_INT_DISABLE							(0x00 << 7)

// BMI088_REG_GYRO_INT3_INT4_IO_CONF register definitions
#define BMI088_GYRO_INT3_PUSH_PULL						(0x00 << 1)
#define BMI088_GYRO_INT3_OPEN_DRAIN						(0x01 << 1)
#define BMI088_GYRO_INT3_ACTIVE_LOW						0x00
#define BMI088_GYRO_INT3_ACTIVE_HIGH					0x01


// Gyroscope datas are converted from deg/s to rad/s with this factor
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.14159265359f / 180.0f)
#endif


// BMI088 device status
typedef enum
{
	BMI088_OK,
	BMI088_ACC_FAILED,
	BMI088_GYRO_FAILED,
	BMI088_BOTH_FAILED
}bmi088_status_t;

// BMI088_REG_ACC_PWR_CTRL register definitions
typedef enum
{
	BMI088_ACC_OFF  = 0x00,
	BMI088_ACC_ON   = 0x04
}bmi088_acc_pwr_ctrl_t;

// BMI088_REG_ACC_PWR_CONF register definitions
typedef enum
{
	BMI088_ACC_PWR_MODE_SUSPEND             = 0x03,
	BMI088_ACC_PWR_MODE_ACTIVE              = 0x00,
	BMI088_ACC_PWR_MODE_ADVANCED_PWR_SAVE   = 0x01
}bmi088_acc_pwr_mode_t;

// BMI088_REG_ACC_RANGE register definitions
typedef enum
{
	BMI088_ACC_RANGE_3      = 0x00,
	BMI088_ACC_RANGE_6      = 0x01,
	BMI088_ACC_RANGE_12     = 0x02,
	BMI088_ACC_RANGE_24     = 0x03
}bmi088_acc_range_t;

// BMI088_REG_ACC_CONF register bit[7:4] definitions
typedef enum
{
	BMI088_ACC_BANDWIDTH_OSR_4      = 0x08,
	BMI088_ACC_BANDWIDTH_OSR_2      = 0x09,
	BMI088_ACC_BANDWIDTH_NORMAL     = 0x0A
}bmi088_acc_bandwidth_t;

// BMI088_REG_ACC_CONF register bit[3:0] definitions
typedef enum
{
	BMI088_ACC_ODR_12_5     = 0x05,
	BMI088_ACC_ODR_25       = 0x06,
	BMI088_ACC_ODR_50       = 0x07,
	BMI088_ACC_ODR_100      = 0x08,
	BMI088_ACC_ODR_200      = 0x09,
	BMI088_ACC_ODR_400      = 0x0A,
	BMI088_ACC_ODR_800      = 0x0B,
	BMI088_ACC_ODR_1600	    = 0x0C
}bmi088_acc_odr_t;

// BMI088_REG_GYRO_LPM1	register definitions
typedef enum
{
	BMI088_GYRO_PWR_MODE_NORMAL         = 0x00,
	BMI088_GYRO_PWR_MODE_SUSPEND        = 0x80,
	BMI088_GYRO_PWR_MODE_DEEP_SUSPEND   = 0x20
}bmi088_gyro_pwr_mode_t;

// BMI088_REG_GYRO_RANGE register definitions
typedef enum
{
	BMI088_GYRO_RANGE_2000  = 0x00,
	BMI088_GYRO_RANGE_1000  = 0x01,
	BMI088_GYRO_RANGE_500   = 0x02,
	BMI088_GYRO_RANGE_250   = 0x03,
	BMI088_GYRO_RANGE_125   = 0x04
}bmi088_gyro_range_t;

// BMI088_REG_GYRO_BANDWIDTH register definitions
typedef enum
{
	BMI088_GYRO_BANDWIDTH_532   = 0x00,
	BMI088_GYRO_BANDWIDTH_230   = 0x01,
	BMI088_GYRO_BANDWIDTH_116   = 0x02,
	BMI088_GYRO_BANDWIDTH_47    = 0x03,
	BMI088_GYRO_BANDWIDTH_23    = 0x04,
	BMI088_GYRO_BANDWIDTH_12    = 0x05,
	BMI088_GYRO_BANDWIDTH_64    = 0x06,
	BMI088_GYRO_BANDWIDTH_32    = 0x07
}bmi088_gyro_bandwidth_t;


/**
 * @brief Structure to store processed sensor data, raw readings and timing information.
 */
typedef struct
{
	float acc_x, acc_y, acc_z;					    // m/s^2
	float gyro_x, gyro_y, gyro_z;				    // rad/s
	float acc_current_time, acc_last_time, acc_dt;  // Seconds, derived from the accelerometer sensortime registers
	uint32_t gyro_current_time, gyro_last_time;     // Microseconds, read from the timer counter
	float gyro_dt;								    // Seconds
	uint8_t raw_acc_data[9];
	uint8_t raw_gyro_data[6];

	// This factors will be used for calculation of acc and gyro datas
	float acc_sensitivity_factor;
	float gyro_sensitivity_factor;
}bmi088_data_t;

/**
 * @brief Accelerometer (g) and gyroscope (deg/s) offsets subtracted from the raw measurements.
 */
typedef struct
{
	float gyro_offset[3];
	float acc_offset[3];
}bmi088_offsets_t;

/**
 * @brief State flags for data management and interrupt handling.
 */
typedef struct
{
	volatile uint8_t is_accel_updated;
	volatile uint8_t is_gyro_updated;
	volatile uint8_t is_acc_started;
	volatile uint8_t is_gyro_started;
	volatile uint8_t is_acc_new;
	volatile uint8_t is_gyro_new;
}bmi088_flags_t;

/**
 * @brief Sensor configuration parameters and hardware interface handles.
 */
typedef struct
{
	bmi088_acc_pwr_mode_t acc_power_mode;
	bmi088_acc_range_t acc_range;
	bmi088_acc_bandwidth_t acc_bandwidth;
	bmi088_acc_odr_t acc_odr;
	bmi088_gyro_pwr_mode_t gyro_pwr_mode;
	bmi088_gyro_range_t gyro_range;
	bmi088_gyro_bandwidth_t gyro_bandwidth;
	TIM_HandleTypeDef *htim;
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *accel_cs_gpio_port;
	uint16_t accel_cs_gpio_pin;
	GPIO_TypeDef *gyro_cs_gpio_port;
	uint16_t gyro_cs_gpio_pin;
	IRQn_Type acc_IRQ, gyro_IRQ;
	bmi088_offsets_t offsets;
}bmi088_configuration_t;

/**
 * @brief Main BMI088 Handle structure.
 * Combines configuration, data, and internal state.
 */
typedef struct
{
	bmi088_configuration_t configuration;
	bmi088_data_t data;
	bmi088_flags_t flags;
}BMI088_HandleTypeDef;


/**
 * @brief Initializes the BMI088 sensor and checks communication of both devices.
 * @param BMI088 Pointer to the handle structure.
 * @retval bmi088_status_t.
 */
bmi088_status_t BMI088_Init(BMI088_HandleTypeDef *BMI088);

/**
 * @brief Fully configures accelerometer and gyroscope parameters.
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Config(BMI088_HandleTypeDef *BMI088);

/**
 * @brief Gets accelerometer and gyroscope offset calibration.
 * @param BMI088 Pointer to the handle structure.
 * @note The accelerometer offset should be calibrated beforehand on a flat surface with the
 *       +Z axis facing up, and the measured values should be written to the offset defines above.
 *       When ROCKET_CARD is defined those defines are used directly, so before flight
 *       only the gyroscope offset is taken at runtime.
 */
void BMI088_GetOffset(BMI088_HandleTypeDef *BMI088);

/**
 * @brief Processes data if the software 'is_accel_updated' / 'is_gyro_updated' flags are set (useful for Interrupt mode).
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Update(BMI088_HandleTypeDef *BMI088);

/**
 * @brief Manually checks the sensors' status registers and updates data if ready (Polling mode).
 * @param BMI088 Pointer to the handle structure.
 */
void BMI088_Update_drdy(BMI088_HandleTypeDef *BMI088);

/**
 * @brief Callback function to be called from the accelerometer GPIO External Interrupt (EXTI).
 * @param BMI088 Pointer to the handle structure.
 */
void bmi088_set_accel_INT(BMI088_HandleTypeDef * BMI088);

/**
 * @brief Callback function to be called from the gyroscope GPIO External Interrupt (EXTI).
 * @param BMI088 Pointer to the handle structure.
 */
void bmi088_set_gyro_INT(BMI088_HandleTypeDef * BMI088);


#endif /* INC_BMI088_H_ */
