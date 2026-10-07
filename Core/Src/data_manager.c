/*
 * data_manager.c
 *
 *  Created on: Mar 18, 2026
 *      Author: bilal
 */

/*
 *  Collects the sensor readings into the FlightData structure and builds every outgoing packet
 *  (SIT telemetry, SUT state, backup SRAM and W25Q flash log). The packet layouts are documented
 *  in data_manager.h and, for the backup SRAM packet, in backup_domain.h.
 */

#include "data_manager.h"
#include <math.h>


/* --- Private Functions Declaration --- */
static uint8_t calculate_checksum(uint8_t *data, uint16_t size);


/**
 * @brief Copies the new accelerometer, gyroscope and barometer readings into the flight data structure.
 * @param flight_data Pointer to the flight data structure.
 * @param bmi088 Pointer to the BMI088 handle structure.
 * @param bmp390 Pointer to the BMP390 handle structure.
 * @note Each sensor block is copied only when its 'new data' flag is set, and the flag is cleared here.
 */
void DataManager_UpdateFlightData(FlightData *flight_data, BMI088_HandleTypeDef *bmi088, BMP390_HandleTypeDef *bmp390)
{
    if (bmi088->flags.is_acc_new)
    {
        bmi088->flags.is_acc_new = 0;
        flight_data->flags.is_accel_new = 1;

        // X and Y are negated to convert the sensor frame to the frame used by the flight algorithm
        flight_data->raw.accel_x = -bmi088->data.acc_x;
        flight_data->raw.accel_y = -bmi088->data.acc_y;
        flight_data->raw.accel_z = bmi088->data.acc_z;

        flight_data->time.accel_dt = bmi088->data.acc_dt;
    }

    if (bmi088->flags.is_gyro_new)
    {
        bmi088->flags.is_gyro_new = 0;
        flight_data->flags.is_gyro_new = 1;

        flight_data->raw.gyro_x = -bmi088->data.gyro_x;
        flight_data->raw.gyro_y = -bmi088->data.gyro_y;
        flight_data->raw.gyro_z = bmi088->data.gyro_z;

        flight_data->time.gyro_dt = bmi088->data.gyro_dt;
    }

    if (bmp390->flags.is_altitude_new)
    {
        bmp390->flags.is_altitude_new = 0;

        flight_data->raw.altitude = bmp390->data.altitude;
        flight_data->raw.pressure = bmp390->data.pressure;
        if (flight_data->raw.max_altitude < flight_data->raw.altitude)
        {
            flight_data->raw.max_altitude = flight_data->raw.altitude;
        }
    }
}

/**
 * @brief Fills the payload bytes of the SIT telemetry packet from the flight data.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the packet buffer.
 * @note See data_manager.h for the packet layout. Only the payload (offsets 1..32) is written here,
 *       the header, the checksum and the footer are added by UartHandler_SendSitPacket().
 */
void DataManager_UpdateSitPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler)
{
    type_converter_float_uint8_array type_converter;

    type_converter.float_ = flight_data->filtered.altitude + flight_data->raw.base_altitude;
    handler->sit_data_packet[1] = type_converter.uint8_array[3];
    handler->sit_data_packet[2] = type_converter.uint8_array[2];
    handler->sit_data_packet[3] = type_converter.uint8_array[1];
    handler->sit_data_packet[4] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.pressure;
    handler->sit_data_packet[5] = type_converter.uint8_array[3];
    handler->sit_data_packet[6] = type_converter.uint8_array[2];
    handler->sit_data_packet[7] = type_converter.uint8_array[1];
    handler->sit_data_packet[8] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_x;
    handler->sit_data_packet[9] = type_converter.uint8_array[3];
    handler->sit_data_packet[10] = type_converter.uint8_array[2];
    handler->sit_data_packet[11] = type_converter.uint8_array[1];
    handler->sit_data_packet[12] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_y;
    handler->sit_data_packet[13] = type_converter.uint8_array[3];
    handler->sit_data_packet[14] = type_converter.uint8_array[2];
    handler->sit_data_packet[15] = type_converter.uint8_array[1];
    handler->sit_data_packet[16] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_z;
    handler->sit_data_packet[17] = type_converter.uint8_array[3];
    handler->sit_data_packet[18] = type_converter.uint8_array[2];
    handler->sit_data_packet[19] = type_converter.uint8_array[1];
    handler->sit_data_packet[20] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.angle_x;
    handler->sit_data_packet[21] = type_converter.uint8_array[3];
    handler->sit_data_packet[22] = type_converter.uint8_array[2];
    handler->sit_data_packet[23] = type_converter.uint8_array[1];
    handler->sit_data_packet[24] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.angle_y;
    handler->sit_data_packet[25] = type_converter.uint8_array[3];
    handler->sit_data_packet[26] = type_converter.uint8_array[2];
    handler->sit_data_packet[27] = type_converter.uint8_array[1];
    handler->sit_data_packet[28] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.angle_z;
    handler->sit_data_packet[29] = type_converter.uint8_array[3];
    handler->sit_data_packet[30] = type_converter.uint8_array[2];
    handler->sit_data_packet[31] = type_converter.uint8_array[1];
    handler->sit_data_packet[32] = type_converter.uint8_array[0];
}

/**
 * @brief Copies the two flight algorithm state bytes into the SUT state packet.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the packet buffer.
 * @note See data_manager.h for the packet layout. The header, the checksum and the footer are
 *       added by UartHandler_SendSutStatePacket().
 */
void DataManager_UpdateSutStatePacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler)
{
    handler->sut_state_packet[1] = flight_data->sut_state_packet[0];
    handler->sut_state_packet[2] = flight_data->sut_state_packet[1];
}

/**
 * @brief Overwrites the flight data with the last received SUT simulation packet.
 * @param flight_data Pointer to the flight data structure.
 * @param handler Pointer to the UART handler holding the last parsed SUT data.
 * @note Used in SUT mode only, the simulated values replace the real sensor readings so that the
 *       rest of the flight software runs exactly the same way as in a real flight.
 * @note Theta is not received in the packet, it is recomputed here as the larger absolute value of
 *       the simulated X and Y angles, instead of being taken from the quaternion.
 */
void DataManager_UpdateFlightDataWithSutPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler)
{
    flight_data->raw.altitude = handler->last_sut_data.altitude;
    flight_data->raw.pressure = handler->last_sut_data.pressure;

    flight_data->raw.accel_x = handler->last_sut_data.accel_x;
    flight_data->raw.accel_y = handler->last_sut_data.accel_y;
    flight_data->raw.accel_z = handler->last_sut_data.accel_z;

    flight_data->filtered.angle_x = handler->last_sut_data.angle_x;
    flight_data->filtered.angle_y = handler->last_sut_data.angle_y;
    flight_data->filtered.angle_z = handler->last_sut_data.angle_z;

    if (fabsf(flight_data->filtered.angle_y) >= fabsf(flight_data->filtered.angle_x)) flight_data->filtered.theta = fabsf(flight_data->filtered.angle_y);
    else flight_data->filtered.theta = fabsf(flight_data->filtered.angle_x);
}

/**
 * @brief Builds the backup SRAM packet used to restore the flight state after an unexpected reset.
 * @param flight_data Pointer to the flight data structure, the packet is built in its bkpsram_packet buffer.
 * @param handler Pointer to the UART handler, used to read the current system mode.
 * @param backup_domain Pointer to the backup domain handle, its date and time are refreshed here.
 * @param w25q Pointer to the W25Q handle, used to store the current flash write position.
 * @note The packet layout is described in the backup SRAM memory map at the top of backup_domain.h.
 * @note This function only fills the buffer, writing it into the backup SRAM is done by the caller
 *       with BackupDomain_WriteSRAM().
 */
void DataManager_UpdateBKPSRAMPacket(FlightData *flight_data, UartHandler_HandleTypeDef *handler, Backup_Domain_HandleTypeDef *backup_domain, W25Q_HandleTypeDef *w25q)
{
    type_converter_float_uint8_array type_converter;

    flight_data->bkpsram_packet[0] = 0xA5;

    flight_data->bkpsram_packet[1] = (uint8_t)uart_handler_get_system_mode(handler);

    type_converter.float_ = flight_data->filtered.altitude;
    flight_data->bkpsram_packet[2] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[3] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[4] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[5] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.vertical_velocity;
    flight_data->bkpsram_packet[6] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[7] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[8] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[9] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.vertical_accel;
    flight_data->bkpsram_packet[10] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[11] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[12] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[13] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[0];
    flight_data->bkpsram_packet[14] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[15] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[16] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[17] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[1];
    flight_data->bkpsram_packet[18] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[19] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[20] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[21] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[2];
    flight_data->bkpsram_packet[22] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[23] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[24] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[25] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[3];
    flight_data->bkpsram_packet[26] = type_converter.uint8_array[3];
    flight_data->bkpsram_packet[27] = type_converter.uint8_array[2];
    flight_data->bkpsram_packet[28] = type_converter.uint8_array[1];
    flight_data->bkpsram_packet[29] = type_converter.uint8_array[0];

    flight_data->bkpsram_packet[30] = flight_algorithm_get_flight_phase();

    flight_data->bkpsram_packet[31] = flight_algorithm_get_is_recovery_armed_flag() | (ThetaCheck_IsAngleRecoveryEnabled() << 1);
    flight_data->bkpsram_packet[32] = flight_algorithm_get_is_apogee_confirmed_with_angle_flag();
    flight_data->bkpsram_packet[33] = flight_algorithm_get_is_apogee_confirmed_with_velocity_flag();
    flight_data->bkpsram_packet[34] = flight_algorithm_get_is_drogue_deployed_flag();

    flight_data->bkpsram_packet[35] = flight_algorithm_get_launch_counter();
    flight_data->bkpsram_packet[36] = flight_algorithm_get_burnout_counter();
    flight_data->bkpsram_packet[37] = flight_algorithm_get_recovery_arming_counter();
    flight_data->bkpsram_packet[38] = flight_algorithm_get_drogue_counter_angle();
    flight_data->bkpsram_packet[39] = flight_algorithm_get_drogue_counter_velocity();
    flight_data->bkpsram_packet[40] = flight_algorithm_get_main_counter();
    flight_data->bkpsram_packet[41] = flight_algorithm_get_landing_counter();

    flight_data->bkpsram_packet[42] = flight_data->sut_state_packet[0];
    flight_data->bkpsram_packet[43] = flight_data->sut_state_packet[1];

    BackupDomain_UpdateDateTime(backup_domain);
    flight_data->bkpsram_packet[44] = backup_domain->date.Date; // day
    flight_data->bkpsram_packet[45] = backup_domain->date.Month;
    flight_data->bkpsram_packet[46] = backup_domain->date.Year;
    flight_data->bkpsram_packet[47] = backup_domain->time.Seconds;
    flight_data->bkpsram_packet[48] = backup_domain->time.Minutes;
    flight_data->bkpsram_packet[49] = backup_domain->time.Hours;

    flight_data->bkpsram_packet[50] = w25q->current_block;
    flight_data->bkpsram_packet[51] = w25q->current_sector;
    flight_data->bkpsram_packet[52] = w25q->current_page;

    flight_data->bkpsram_packet[53] = calculate_checksum(flight_data->bkpsram_packet, BKPSRAM_PACKET_SIZE - 1);
}

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
 * @note See data_manager.h for the record layout.
 */
void DataManager_PackFlashData(FlightData *flight_data, uint8_t *flash_data, UartHandler_HandleTypeDef *handler, Backup_Domain_HandleTypeDef *backup_domain, KalmanFilter_HandleTypeDef *kf, BMP390_HandleTypeDef *bmp390, Quaternion_HandleTypeDef *quaternion)
{
    type_converter_float_uint8_array type_converter;

    flash_data[0] = backup_domain->time.Minutes;
    flash_data[1] = backup_domain->time.Seconds;
    uint32_t frac = backup_domain->time.SecondFraction - backup_domain->time.SubSeconds;  // 0 .. PREDIV_S
    flash_data[2] = (uint8_t)((frac * 256U) / (backup_domain->time.SecondFraction + 1U)); // 0 .. 255

    flash_data[3] = flight_algorithm_get_flight_phase() | (uart_handler_get_system_mode(handler) << 3) | (kalman_filter_get_in_mach_transition_flag(kf) << 5) | (kalman_filter_get_altitude_status(kf) << 6) | (kalman_filter_get_accel_status(kf) << 7);

    flash_data[4] = flight_data->sut_state_packet[0];
    flash_data[5] = (flight_data->sut_state_packet[1] & 0x1F) | ((quaternion->gyroOnlyMode << 5) & 0x20) | ((bmp390->flags.is_updated_with_int << 6) & 0x40) | ((ThetaCheck_IsAngleRecoveryEnabled() << 7) & 0x80);

    type_converter.float_ = flight_data->raw.altitude;
    flash_data[6] = type_converter.uint8_array[3];
    flash_data[7] = type_converter.uint8_array[2];
    flash_data[8] = type_converter.uint8_array[1];
    flash_data[9] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_x;
    flash_data[10] = type_converter.uint8_array[3];
    flash_data[11] = type_converter.uint8_array[2];
    flash_data[12] = type_converter.uint8_array[1];
    flash_data[13] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_y;
    flash_data[14] = type_converter.uint8_array[3];
    flash_data[15] = type_converter.uint8_array[2];
    flash_data[16] = type_converter.uint8_array[1];
    flash_data[17] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.accel_z;
    flash_data[18] = type_converter.uint8_array[3];
    flash_data[19] = type_converter.uint8_array[2];
    flash_data[20] = type_converter.uint8_array[1];
    flash_data[21] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.gyro_x;
    flash_data[22] = type_converter.uint8_array[3];
    flash_data[23] = type_converter.uint8_array[2];
    flash_data[24] = type_converter.uint8_array[1];
    flash_data[25] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.gyro_y;
    flash_data[26] = type_converter.uint8_array[3];
    flash_data[27] = type_converter.uint8_array[2];
    flash_data[28] = type_converter.uint8_array[1];
    flash_data[29] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->raw.gyro_z;
    flash_data[30] = type_converter.uint8_array[3];
    flash_data[31] = type_converter.uint8_array[2];
    flash_data[32] = type_converter.uint8_array[1];
    flash_data[33] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.altitude;
    flash_data[34] = type_converter.uint8_array[3];
    flash_data[35] = type_converter.uint8_array[2];
    flash_data[36] = type_converter.uint8_array[1];
    flash_data[37] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.vertical_velocity;
    flash_data[38] = type_converter.uint8_array[3];
    flash_data[39] = type_converter.uint8_array[2];
    flash_data[40] = type_converter.uint8_array[1];
    flash_data[41] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.vertical_accel;
    flash_data[42] = type_converter.uint8_array[3];
    flash_data[43] = type_converter.uint8_array[2];
    flash_data[44] = type_converter.uint8_array[1];
    flash_data[45] = type_converter.uint8_array[0];

    flash_data[46] = (uint8_t)(flight_data->filtered.theta + 0.5f);

    type_converter.float_ = flight_data->filtered.quaternion[0];
    flash_data[47] = type_converter.uint8_array[3];
    flash_data[48] = type_converter.uint8_array[2];
    flash_data[49] = type_converter.uint8_array[1];
    flash_data[50] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[1];
    flash_data[51] = type_converter.uint8_array[3];
    flash_data[52] = type_converter.uint8_array[2];
    flash_data[53] = type_converter.uint8_array[1];
    flash_data[54] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[2];
    flash_data[55] = type_converter.uint8_array[3];
    flash_data[56] = type_converter.uint8_array[2];
    flash_data[57] = type_converter.uint8_array[1];
    flash_data[58] = type_converter.uint8_array[0];

    type_converter.float_ = flight_data->filtered.quaternion[3];
    flash_data[59] = type_converter.uint8_array[3];
    flash_data[60] = type_converter.uint8_array[2];
    flash_data[61] = type_converter.uint8_array[1];
    flash_data[62] = type_converter.uint8_array[0];

    flash_data[63] = calculate_checksum(flash_data, 63);
}


/**
 * @brief Calculates the checksum of a packet as the 8 bit sum of its bytes.
 * @param data Pointer to the first byte of the packet.
 * @param size Number of bytes to sum, the checksum byte itself is not included.
 * @retval Checksum value.
 */
static uint8_t calculate_checksum(uint8_t *data, uint16_t size)
{
	uint8_t checksum = 0;

	for (uint16_t i = 0; i < size; i++)
	{
		checksum += data[i];
	}

	return checksum;
}

