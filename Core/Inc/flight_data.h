/*
 * flight_data.h
 *
 *  Created on: May 7, 2026
 *      Author: bilal
 */

/*
 *  This header only defines the FlightData structure, it contains no code. FlightData is the
 *  single data block shared by every flight software module, one global instance of it lives in
 *  main.c and is passed by pointer to the modules below:
 *      data_manager     ->  writes 'raw' from the BMI088 / BMP390 readings, or from the received
 *                           SUT packet, and builds every outgoing packet from 'raw' and 'filtered'
 *      sensor_fusion    ->  writes 'raw.vertical_accel' and the whole 'filtered' block from the
 *                           quaternion and the Kalman filter outputs
 *      flight_algorithm ->  only reads 'filtered' and writes the SUT state bytes
 *      backup_domain    ->  writes the 'bkpsram_packet' buffer into the backup SRAM
 */

#ifndef INC_FLIGHT_DATA_H_
#define INC_FLIGHT_DATA_H_


#include <stdint.h>


// Number of flight algorithm state bytes, they are forwarded to the SUT state packet
#define SUT_STATE_PACKET_SIZE  2
// Size of the backup SRAM packet, its layout is described in the memory map at the top of backup_domain.h
#define BKPSRAM_PACKET_SIZE 54


/**
 * @brief Unfiltered values, they are written either from the sensors or from the received SUT packet.
 */
typedef struct
{
	float accel_x, accel_y, accel_z;
	float vertical_accel;
	float gyro_x, gyro_y, gyro_z;
	float altitude, pressure;
    float base_altitude;
    float max_altitude;
} flight_data_raw_data_t;

/**
 * @brief Filtered values, they are the outputs of the quaternion and of the Kalman filter.
 * @note The flight algorithm makes all of its decisions with the fields of this structure.
 */
typedef struct
{
	float altitude;
    float max_altitude;
	float vertical_velocity;
	float vertical_accel;
	float theta;
	float angle_x, angle_y, angle_z;
	float quaternion[4];
} flight_data_filtered_data_t;

/**
 * @brief New data flags, they are set by the data manager and cleared by the sensor fusion.
 * @note DataManager_UpdateFlightData() sets a flag when it copies a new sensor block, and
 *       SensorFusion_QuaternionUpdate() clears the flags it uses for the orientation update.
 */
typedef struct
{
	uint8_t is_accel_new;
	uint8_t is_gyro_new;
} flight_data_flags_t;

/**
 * @brief Time steps used by the orientation and by the Kalman filter updates.
 */
typedef struct
{
	float accel_dt;
	float gyro_dt;
	uint32_t kalman_last_time, kalman_now;
	float kalman_dt;
} flight_data_time_t;


/**
 * @brief Main flight data structure.
 * Combines the raw and the filtered values, the new data flags, the time steps and the two
 * buffers which are filled in place (the flight algorithm state bytes and the backup SRAM packet).
 */
typedef struct
{
	flight_data_raw_data_t raw;
	flight_data_filtered_data_t filtered;
	uint8_t sut_state_packet[SUT_STATE_PACKET_SIZE];
	uint8_t bkpsram_packet[BKPSRAM_PACKET_SIZE];
	flight_data_flags_t flags;
	flight_data_time_t time;
} FlightData;


#endif /* INC_FLIGHT_DATA_H_ */
