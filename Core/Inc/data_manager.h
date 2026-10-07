/*
 * data_manager.h
 *
 *  Created on: Mar 18, 2026
 *      Author: bilal
 */

/*
 *  This module is the single place where the FlightData structure is filled from the sensors
 *  and where all outgoing byte packets are built. It does not run any algorithm by itself,
 *  it only moves and formats data between the following modules:
 *
 *      BMI088 / BMP390  ->  FlightData                      (DataManager_UpdateFlightData)
 *      FlightData       ->  SIT packet (UART)               (DataManager_UpdateSitPacket)
 *      FlightData       ->  SUT state packet (UART)         (DataManager_UpdateSutStatePacket)
 *      SUT packet (UART)->  FlightData                      (DataManager_UpdateFlightDataWithSutPacket)
 *      FlightData       ->  backup SRAM packet              (DataManager_UpdateBKPSRAMPacket)
 *      FlightData       ->  W25Q flash log packet           (DataManager_PackFlashData)
 */

#ifndef INC_DATA_MANAGER_H_
#define INC_DATA_MANAGER_H_


#include "uart_handler.h"
#include "bmi088.h"
#include "bmp390.h"
#include "flight_data.h"
#include "flight_algorithm.h"
#include "backup_domain.h"
#include "w25q.h"
#include "kalman_filter.h"
#include "quaternion.h"
#include "theta_check.h"


/**
 * @brief Copies the new accelerometer, gyroscope and barometer readings into the flight data structure.
 * @param flight_data Pointer to the flight data structure.
 * @param bmi088 Pointer to the BMI088 handle structure.
 * @param bmp390 Pointer to the BMP390 handle structure.
 * @note Each sensor block is copied only when its 'new data' flag is set, and the flag is cleared
 *       here. The accelerometer and gyroscope X and Y axes are negated, so the sensor frame is
 *       converted to the frame used by the flight algorithm.
 * @note The barometer altitude is already referenced to the ground level (the base altitude is
 *       subtracted inside the BMP390 module), and the raw maximum altitude is updated here.
 */
void DataManager_UpdateFlightData(FlightData *flight_data, BMI088_HandleTypeDef *bmi088, BMP390_HandleTypeDef *bmp390);

/**
 * @brief Fills the payload bytes of the SIT telemetry packet from the flight data.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the packet buffer.
 *
 * SIT packet layout (UART_HANDLER_SIT_PACKET_SIZE = 36 bytes, floats are MSB first)
 * ---------------------------------------------------------------------------------
 *  Offset  Size  Content
 *  ------  ----  ----------------------------------------------------------------
 *     0      1   Header (0xAB)                        written by the UART handler
 *     1      4   Altitude (filtered altitude + base altitude, meters)
 *     5      4   Pressure (hPa)
 *     9      4   Acceleration X (m/s^2)
 *    13      4   Acceleration Y (m/s^2)
 *    17      4   Acceleration Z (m/s^2)
 *    21      4   Angle X (degrees)
 *    25      4   Angle Y (degrees)
 *    29      4   Angle Z (degrees)
 *    33      1   Checksum                             written by the UART handler
 *    34      2   Footer (0x0D, 0x0A)                  written by the UART handler
 *
 * @note Only the payload (offsets 1..32) is written here. The header, checksum and footer are
 *       added by UartHandler_SendSitPacket() right before the transmission.
 * @note The base altitude is added back to the filtered altitude, so the transmitted altitude is
 *       absolute while the altitude used by the flight algorithm stays relative to the ground.
 */
void DataManager_UpdateSitPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler);

/**
 * @brief Copies the two flight algorithm state bytes into the SUT state packet.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the packet buffer.
 *
 * SUT state packet layout (UART_HANDLER_SUT_STATE_PACKET_SIZE = 6 bytes)
 * ---------------------------------------------------------------------
 *  Offset  Size  Content
 *  ------  ----  ----------------------------------------------------------------
 *     0      1   Header (0xAA)                        written by the UART handler
 *     1      1   State byte 0 (flight algorithm event flags)
 *     2      1   State byte 1
 *     3      1   Checksum                             written by the UART handler
 *     4      2   Footer (0x0D, 0x0A)                  written by the UART handler
 *
 * @note The state bytes are produced by the flight algorithm, this function only forwards them.
 */
void DataManager_UpdateSutStatePacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler);

/**
 * @brief Overwrites the flight data with the last received SUT simulation packet.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the last parsed SUT data.
 * @note Used in SUT mode only, it replaces the real sensor readings with the simulated ones
 *       (altitude, pressure, accelerations and angles), so the rest of the flight software runs
 *       exactly the same way as in a real flight.
 * @note Theta is not received in the packet, it is recomputed here as the larger absolute value
 *       of the simulated X and Y angles, instead of being taken from the quaternion.
 */
void DataManager_UpdateFlightDataWithSutPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler);

/**
 * @brief Builds the backup SRAM packet used to restore the flight state after an unexpected reset.
 * @param flight_data Pointer to the flight data structure, the packet is built in its bkpsram_packet buffer.
 * @param handler Pointer to the UART handler, used to read the current system mode.
 * @param backup_domain Pointer to the backup domain handle, its date and time are refreshed here.
 * @param w25q Pointer to the W25Q handle, used to store the current flash write position.
 * @note The layout of the packet and the meaning of every field are described in the backup SRAM
 *       memory map at the top of backup_domain.h.
 * @note This function only fills the buffer, writing it into the backup SRAM is done by the caller
 *       with BackupDomain_WriteSRAM().
 * @note The last byte is the checksum of the previous BKPSRAM_PACKET_SIZE - 1 bytes and the first
 *       byte is the 0xA5 header, both are checked by backup_domain_is_reset_occured().
 */
void DataManager_UpdateBKPSRAMPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler, Backup_Domain_HandleTypeDef *backup_domain, W25Q_HandleTypeDef *w25q);

/**
 * @brief Builds one flash log record from the current flight data.
 * @param flight_data Pointer to the flight data structure.
 * @param flash_data Destination buffer, must be W25Q_PACKET_SIZE (64) bytes long.
 * @param handler Pointer to the UART handler, used to read the current system mode.
 * @param backup_domain Pointer to the backup domain handle, its last read RTC time is used as timestamp.
 * @param kf Pointer to the Kalman filter handle, used to read the mach transition and sensor status flags.
 * @param bmp390 Pointer to the BMP390 handle, used to log whether the last barometer sample came from
 *               the data ready interrupt or from the polling fallback.
 * @param quaternion Pointer to the quaternion handle, used to log whether the attitude filter runs in
 *                   gyroscope only mode or with the accelerometer correction.
 *
 * Flash log packet layout (W25Q_PACKET_SIZE = 64 bytes, floats are MSB first)
 * --------------------------------------------------------------------------
 *  Offset  Size  Content
 *  ------  ----  ----------------------------------------------------------------
 *     0      1   RTC minutes
 *     1      1   RTC seconds
 *     2      1   Sub second part of the RTC, scaled to 0 .. 255
 *     3      1   Status byte:
 *                  bit[2:0] flight phase (flight_phase_t)
 *                  bit[4:3] system mode (system_mode_t)
 *                  bit[5]   Kalman filter mach transition flag
 *                  bit[6]   Kalman filter altitude sensor status (sensor_status_t)
 *                  bit[7]   Kalman filter acceleration sensor status (sensor_status_t)
 *     4      1   Flight algorithm state byte 0
 *     5      1   Flight algorithm state byte 1 in bit[4:0],
 *                  bit[5]   quaternion gyro only mode, 1 gyroscope only, 0 accelerometer corrected
 *                  bit[6]   BMP390 update source, 1 interrupt, 0 polling fallback
 *                  bit[7]   angle based recovery enable
 *     6      4   Raw altitude (meters, relative to the ground)
 *    10     12   Raw acceleration X, Y, Z (m/s^2)
 *    22     12   Raw gyroscope X, Y, Z (rad/s)
 *    34      4   Filtered altitude (meters)
 *    38      4   Filtered vertical velocity (m/s)
 *    42      4   Filtered vertical acceleration (m/s^2)
 *    46      1   Theta, rounded to the nearest degree
 *    47     16   Quaternion (w, x, y, z)
 *    63      1   Checksum of the first 63 bytes
 */
void DataManager_PackFlashData(FlightData *flight_data, uint8_t *flash_data, UartHandler_HandleTypeDef *handler, Backup_Domain_HandleTypeDef *backup_domain, KalmanFilter_HandleTypeDef *kf, BMP390_HandleTypeDef *bmp390, Quaternion_HandleTypeDef *quaternion);


#endif /* INC_DATA_MANAGER_H_ */
