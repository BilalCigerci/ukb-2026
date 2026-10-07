/*
 * sensor_fusion.c
 *
 *  Created on: May 7, 2026
 *      Author: bilal
 */

#include "sensor_fusion.h"
#include "main.h"
#include <math.h>
#include "flight_algorithm.h"
#include "theta_check.h"


// Value of the quaternion gyroOnlyMode flag which selects the gyroscope only integration
#define GYRO_ONLY_MODE 1


// Free running 1 microsecond timer, used to measure the Kalman filter time step
extern TIM_HandleTypeDef htim5;


/* --- Private Functions Declaration --- */
static void updete_filtered_datas(FlightData *flight_data, Quaternion_HandleTypeDef *quaternion);
static float update_vertical_accel(float sensor_accel, float theta);


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
void SensorFusion_QuaternionUpdate(FlightData *flight_data, Quaternion_HandleTypeDef *quaternion)
{
	if (quaternion->gyroOnlyMode == GYRO_ONLY_MODE && flight_data->flags.is_gyro_new)
	{
		// After the acceleration threshold has been passed, the rest of the flight runs on the gyroscope only
		flight_data->flags.is_gyro_new = 0;
		Quaternion_UpdateOrientation_GyroOnlyMode(quaternion, flight_data->raw.gyro_x, flight_data->raw.gyro_y, flight_data->raw.gyro_z, flight_data->time.gyro_dt);

		updete_filtered_datas(flight_data, quaternion);
	}
	else if (quaternion->gyroOnlyMode != GYRO_ONLY_MODE && flight_data->flags.is_accel_new  && flight_data->flags.is_gyro_new)
	{
		// Accelerometer corrected mode, it runs at the accelerometer rate (accel_dt) and waits for both readings
		flight_data->flags.is_accel_new = 0;
		flight_data->flags.is_gyro_new = 0;
		Quaternion_UpdateOrientation_Mahony(quaternion, flight_data->raw.gyro_x, flight_data->raw.gyro_y, flight_data->raw.gyro_z, flight_data->raw.accel_x, flight_data->raw.accel_y, flight_data->raw.accel_z, flight_data->time.accel_dt);

		updete_filtered_datas(flight_data, quaternion);
	}
}

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
void SensorFusion_KalmanUpdate(FlightData *flight_data, KalmanFilter_HandleTypeDef *kalman, UartHandler_HandleTypeDef *uart_handler)
{
	flight_data->time.kalman_now = __HAL_TIM_GET_COUNTER(&htim5);
	// used timer5 -> 1 microsecond period
	flight_data->time.kalman_dt = (float) (flight_data->time.kalman_now - flight_data->time.kalman_last_time) * 1e-6f;
	flight_data->time.kalman_last_time = flight_data->time.kalman_now;

	if (flight_data->time.kalman_dt <= 0.0f || flight_data->time.kalman_dt > 0.15f) flight_data->time.kalman_dt = 0.1f;    // estimated time step -> 0.1f

	// The longitudinal axis is accel_z in the simulation packet and accel_x on the real card
	if (uart_handler->system_mode == SUT_MODE)
	{
        flight_data->raw.vertical_accel = update_vertical_accel(flight_data->raw.accel_z, flight_data->filtered.theta);
	}
    else
    {
        flight_data->raw.vertical_accel = update_vertical_accel(flight_data->raw.accel_x, flight_data->filtered.theta);
    }

	KalmanFilter_Update(kalman, flight_data->raw.altitude, flight_data->raw.vertical_accel, flight_data->time.kalman_dt);

	flight_data->filtered.altitude = KalmanFilter_GetAltitude(kalman);
	flight_data->filtered.vertical_velocity = KalmanFilter_GetVelocity(kalman);
	flight_data->filtered.vertical_accel = KalmanFilter_GetAccel(kalman);

	if (flight_data->filtered.altitude > flight_data->filtered.max_altitude)
	{
		flight_data->filtered.max_altitude = flight_data->filtered.altitude;
	}
}


/* --- Private Functions Definitions --- */

/* ------------------------------------------------------------------
 * Copies the outputs of the quaternion module into the filtered block
 * of FlightData, right after an orientation update.
 *
 * theta is the tilt angle used by the flight algorithm and by the
 * vertical acceleration below, the three angles are the Euler angles
 * of the Z based frame, and the four components are the quaternion
 * itself, which is also written into the backup SRAM.
 *
 * In SUT mode the orientation update still runs on the real sensors,
 * but theta and the three angles written here are overwritten at
 * 10 Hz with the simulated ones by
 * DataManager_UpdateFlightDataWithSutPacket(), so the flight
 * algorithm works with the simulation angles.
 * ------------------------------------------------------------------ */
static void updete_filtered_datas(FlightData *flight_data, Quaternion_HandleTypeDef *quaternion)
{
	flight_data->filtered.theta = Quaternion_GetTheta(quaternion);

	flight_data->filtered.angle_x = Quaternion_GetRoll_Z(quaternion);
	flight_data->filtered.angle_y = Quaternion_GetPitch_Z(quaternion);
	flight_data->filtered.angle_z = Quaternion_GetYaw_Z(quaternion);

	flight_data->filtered.quaternion[0] = quaternion->q[0];
	flight_data->filtered.quaternion[1] = quaternion->q[1];
	flight_data->filtered.quaternion[2] = quaternion->q[2];
	flight_data->filtered.quaternion[3] = quaternion->q[3];
}

/* ------------------------------------------------------------------
 * Builds the vertical acceleration measurement of the Kalman filter
 * from a single accelerometer axis and the tilt angle.
 *
 * sensor_accel is the longitudinal (rocket axis) acceleration in
 * m/s^2, theta is the tilt angle in degrees, the returned value is the
 * vertical acceleration in m/s^2, positive upwards, with gravity
 * already removed.
 *
 * The tilt angle is taken from the estimator only while the
 * theta_check module reports the angle criterion as enabled. If theta
 * has been marked as not trustworthy before arming, a fixed 10 degree
 * tilt is used instead of the estimated one.
 *
 * Two formulas are used, selected by the flight phase:
 *   - PHASE_IDLE:  (a - g * cos(t)) * cos(t)
 *     On the pad the axis measures the gravity component along the
 *     rocket axis. It is removed first and only the remainder is
 *     projected onto the vertical, so the measurement stays around
 *     zero while the rocket is standing, whatever its tilt is.
 *   - Every other phase:  a * cos(t) - g
 *     In flight the measured axis is projected onto the vertical and
 *     the full gravity is subtracted from it.
 * ------------------------------------------------------------------ */
static float update_vertical_accel(float sensor_accel, float theta)
{
    float t = 0;
    // If theta is not trustworthy, the rocket is assumed to be tilted by 10 degrees
    if (ThetaCheck_IsAngleRecoveryEnabled())
    {
        t = theta * DEG_TO_RAD;
    }
    else
    {
        t = 10 * DEG_TO_RAD;
    }

	float cos_t = cosf(t);
    float vertical_accel = 0;
    if (flight_algorithm_get_flight_phase() == PHASE_IDLE)
    {
        vertical_accel = (sensor_accel - 9.80665f * cos_t) * cos_t;
    }
    else
    {
        vertical_accel = sensor_accel * cos_t - 9.80665f;
    }
    return vertical_accel;
}
