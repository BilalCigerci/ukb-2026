/*
 * kalman_filter.h
 *
 *  Created on: Apr 25, 2026
 *      Author: bilal
 */

/*
 * Linear Kalman filter for the vertical channel
 * -----------------------------------------------------------------------------------------
 * The filter estimates the vertical motion of the rocket with a constant acceleration model:
 *
 *      X = [ altitude (m), vertical velocity (m/s), vertical acceleration (m/s^2) ]
 *
 *      altitude_n+1 = altitude_n + velocity_n * dt + 0.5 * accel_n * dt^2
 *      velocity_n+1 = velocity_n + accel_n * dt
 *      accel_n+1    = accel_n
 *
 * Two independent scalar measurements are applied one after the other (sequential update):
 *   - Barometric altitude, observed with H = [1, 0, 0]
 *   - Vertical acceleration, observed with H = [0, 0, 1]
 *
 * Because both measurement matrices contain a single 1, the matrix form of the update
 * collapses into scalar operations, so no matrix inversion is needed.
 *
 * Both measurements are validated by the sensor_check module before they are used. A
 * measurement whose status is not SENSOR_OK is skipped for that step, and the filter keeps
 * running with the remaining one.
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_


#include <stdint.h>
#include "backup_domain.h"
#include "sensor_check.h"


/**
 * @brief Main Kalman Filter Handle structure.
 * Holds the state vector, the working matrices, the noise parameters and the internal state.
 */
typedef struct
{
    float X[3];         // State vector: altitude (m), vertical velocity (m/s), vertical acceleration (m/s^2)
    float P[3][3];      // State covariance, how uncertain the current estimate is
    float F[3][3];      // State transition matrix, its dt terms are rewritten in every prediction
    float FP[3][3];     // Intermediate product F * P
    float FPFT[3][3];   // Intermediate product F * P * F^T
    float Q[3][3];      // Process noise covariance, rebuilt from dt in every prediction
    float K[3];         // Kalman gain of the current measurement update

    float process_noise;            // Q scaling factor, how much the constant acceleration model is trusted
    float measurement_noise_alt;    // R of the barometric altitude measurement
    float measurement_noise_acc;    // R of the vertical acceleration measurement

    uint8_t in_mach_transition;             // 1 while the barometer is ignored because of the transonic region
    uint32_t mach_transition_start_time;    // HAL tick taken when the mach transition started, used for the timeout
    uint8_t is_mach_transition_finished;    // 1 after the transition has been passed once, it is not entered again

    sensor_status_t altitude_status;    // Result of the last SensorCheck_Altitude() call
    sensor_status_t accel_status;       // Result of the last SensorCheck_VerticalAccel() call
}KalmanFilter_HandleTypeDef;

/**
 * @brief Helper union used to rebuild a float from the MSB first bytes stored in the backup SRAM.
 */
#ifndef TYPE_CONVERTER_FLOAT_UINT8_ARRAY
#define TYPE_CONVERTER_FLOAT_UINT8_ARRAY
typedef union
{
	float float_;
	uint8_t uint8_array[4];
} type_converter_float_uint8_array;
#endif /* TYPE_CONVERTER_FLOAT_UINT8_ARRAY */


/**
 * @brief Initializes the filter, sets the state vector to zero and loads the default noise parameters.
 * @param kf Pointer to the handle structure.
 * @note It also initializes the sensor_check module with a zero altitude and a zero vertical acceleration.
 */
void KalmanFilter_Init(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Runs one prediction and one measurement update cycle.
 * @param kf Pointer to the handle structure.
 * @param altitude Barometric altitude relative to the base altitude, meters.
 * @param accel Vertical acceleration, m/s^2.
 * @param dt Time passed since the previous call, seconds.
 * @note Both measurements are checked by the sensor_check module first, and only the ones
 *       reported as SENSOR_OK are used in the update.
 */
void KalmanFilter_Update(KalmanFilter_HandleTypeDef *kf, float altitude, float accel, float dt);

/**
 * @brief Resets the filter to its initial state.
 * @param kf Pointer to the handle structure.
 */
void KalmanFilter_Reset(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Restores the state vector from the backup SRAM after an unexpected reset.
 * @param kf Pointer to the handle structure.
 * @param backup_domain Pointer to the backup domain handle.
 * @note Must be called after KalmanFilter_Init(). The covariance and the noise parameters keep
 *       their initial values, only the state vector is restored. If the restored velocity is
 *       still above the mach threshold the filter continues in the mach transition, and the
 *       sensor_check module is re-initialized with the restored values.
 */
void KalmanFilter_Recover(KalmanFilter_HandleTypeDef *kf, Backup_Domain_HandleTypeDef *backup_domain);

/**
 * @brief Returns the estimated altitude in meters.
 * @param kf Pointer to the handle structure.
 * @retval Altitude state X[0].
 */
float KalmanFilter_GetAltitude(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Returns the estimated vertical velocity in m/s.
 * @param kf Pointer to the handle structure.
 * @retval Velocity state X[1].
 */
float KalmanFilter_GetVelocity(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Returns the estimated vertical acceleration in m/s^2.
 * @param kf Pointer to the handle structure.
 * @retval Acceleration state X[2].
 */
float KalmanFilter_GetAccel(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Returns the status of the last checked altitude measurement.
 * @param kf Pointer to the handle structure.
 * @retval sensor_status_t as uint8_t (0: SENSOR_OK, 1: SENSOR_ERROR).
 */
uint8_t kalman_filter_get_altitude_status(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Returns the status of the last checked vertical acceleration measurement.
 * @param kf Pointer to the handle structure.
 * @retval sensor_status_t as uint8_t (0: SENSOR_OK, 1: SENSOR_ERROR).
 */
uint8_t kalman_filter_get_accel_status(KalmanFilter_HandleTypeDef *kf);

/**
 * @brief Returns the mach transition flag.
 * @param kf Pointer to the handle structure.
 * @retval 1 while the barometer is ignored because of the transonic region, 0 otherwise.
 */
uint8_t kalman_filter_get_in_mach_transition_flag(KalmanFilter_HandleTypeDef *kf);


#endif /* INC_KALMAN_FILTER_H_ */
