/*
 * sensor_fusion.h
 *
 *  Created on: May 7, 2026
 *      Author: bilal
 */

/*
 * Sensor fusion layer
 * -----------------------------------------------------------------------------------------
 * The module is the layer between the data manager and the two estimators. It reads the
 * 'raw' block of FlightData, runs the quaternion and the Kalman filter, and writes the
 * results back into the 'filtered' block, so the flight algorithm only sees filtered values.
 *
 * Two update paths, running at different rates (see the main loop in main.c)
 *   - SensorFusion_QuaternionUpdate() is called in every pass of the main loop, but it only
 *     works when the new data flags of the data manager are set, so it effectively runs at
 *     the sensor rate: 1000 Hz in gyro only mode, 200 Hz in Mahony mode.
 *   - SensorFusion_KalmanUpdate() is called from the 100 ms (10 Hz) timer flag, in normal,
 *     SIT and SUT mode. In SUT mode it is called after the simulated packet has been copied
 *     into FlightData, so the filter always runs on the freshest values of the mode.
 *
 * Vertical acceleration
 *   The Kalman filter is not fed with the accelerometer axis directly. The longitudinal
 *   acceleration is projected onto the vertical with the tilt angle theta and gravity is
 *   removed from it (see update_vertical_accel() in sensor_fusion.c). Two things change
 *   which numbers go into that projection:
 *     - The longitudinal axis is accel_x in a real flight, but accel_z in SUT mode, because
 *       the simulation packet is produced in a Z based frame.
 *     - theta is used only while the theta_check module reports the angle criterion as
 *       enabled, otherwise a fixed 10 degree tilt is assumed.
 *
 * The module owns no state of its own, everything it produces lives in FlightData and in the
 * two handle structures given to it.
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_


#include "quaternion.h"
#include "kalman_filter.h"
#include "flight_data.h"
#include "uart_handler.h"


// Theta is kept in degrees in FlightData, it is converted to radians before the trigonometric calls
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.14159265359f / 180.0f)
#endif


/**
 * @brief Runs one orientation update and copies the quaternion outputs into the filtered data.
 * @param flight_data Pointer to the flight data structure.
 * @param quaternion Pointer to the quaternion handle structure.
 * @note Nothing is done until the data manager announces new data. The gyro only mode uses the
 *       'is_gyro_new' flag alone and runs at the gyroscope rate with gyro_dt, the Mahony mode
 *       waits for both flags and runs at the accelerometer rate with accel_dt. The used flags
 *       are cleared here.
 * @note The mode is not decided in this module, it only follows the gyroOnlyMode flag of the
 *       quaternion handle. That flag is latched inside Quaternion_UpdateOrientation_Mahony()
 *       when the accelerometer magnitude passes its threshold, so once the rocket has boosted
 *       the gyro only path is taken for the rest of the flight.
 * @note theta, the three Euler angles and the four quaternion components are written into the
 *       filtered block after every successful update.
 */
void SensorFusion_QuaternionUpdate(FlightData *flight_data, Quaternion_HandleTypeDef *quaternion);

/**
 * @brief Runs one Kalman filter cycle and copies the altitude, velocity and acceleration estimates
 *        into the filtered data.
 * @param flight_data Pointer to the flight data structure.
 * @param kalman Pointer to the Kalman filter handle structure.
 * @param uart_handler Pointer to the UART handler, only its system mode is used.
 * @note dt is measured here from the TIM5 counter (1 microsecond period). A value which is not
 *       positive or is larger than 0.15 s is replaced with 0.1 s, the expected period of the
 *       10 Hz call.
 * @note The vertical acceleration is computed before the filter is run and it is stored in
 *       'raw.vertical_accel', so it is also the value which is logged and sent in the packets.
 *       In SUT mode it comes from accel_z, in every other mode from accel_x.
 * @note 'filtered.max_altitude' is updated here as well, from the filtered altitude.
 */
void SensorFusion_KalmanUpdate(FlightData *flight_data, KalmanFilter_HandleTypeDef *kalman, UartHandler_HandleTypeDef *uart_handler);


#endif /* INC_SENSOR_FUSION_H_ */
