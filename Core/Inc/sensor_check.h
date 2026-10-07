/*
 * sensor_check.h
 *
 *  Created on: Jul 15, 2026
 *      Author: bilal
 */

/*
 * Sensor plausibility check for the vertical channel
 * -----------------------------------------------------------------------------------------
 * The module validates the two measurements used by the Kalman filter, the barometric
 * altitude and the vertical acceleration, before they reach the filter. Each measurement
 * has its own independent state, they never affect each other.
 *
 * Every sample is passed through four tests, in this order:
 *   1. Finite check    : NaN and Inf are rejected immediately.
 *   2. Range check     : the value has to stay inside the physical limits of the flight.
 *   3. Stuck check     : a value repeating exactly for several samples means a frozen sensor.
 *   4. Median check    : the value is compared against the median of the last samples, so a
 *                        single spike far away from the recent history is rejected.
 *
 * A sensor is not disabled by one bad sample. Every failed test increases an error counter
 * and every fully healthy sample decreases it, so only a repeating fault disables the sensor.
 * A disabled sensor keeps being fed and starts counting its healthy samples, once enough of
 * them are collected it is enabled again. While it is disabled its status stays SENSOR_ERROR
 * even for a healthy sample, so the caller keeps ignoring it until the recovery is complete.
 *
 * The enable and disable decisions are applied at the beginning of the next call, so the
 * sample that fills the counter is still reported with the old state.
 *
 * The acceleration limits are phase dependent. In PHASE_IDLE and PHASE_BOOST a much wider
 * median difference is allowed, because the real acceleration changes very fast during the
 * motor burn and a tight limit would mark those samples as faulty.
 */

#ifndef INC_SENSOR_CHECK_H_
#define INC_SENSOR_CHECK_H_


#include <stdint.h>


/**
 * @brief Result of a single measurement check.
 * @note SENSOR_ERROR means the value must not be used for that step, it does not mean
 *       the sensor is disabled. A disabled sensor also returns SENSOR_ERROR while it
 *       is collecting its healthy samples for the recovery.
 */
typedef enum
{
    SENSOR_OK = 0x00,
    SENSOR_ERROR = 0x01
} sensor_status_t;


/**
 * @brief Initializes both sensor states and starts them in the enabled state.
 * @param initial_altitude Altitude the barometer history is filled with, meters.
 * @param initial_vertical_accel Vertical acceleration the IMU history is filled with, m/s^2.
 * @note The history buffers are filled with these values instead of zero, otherwise the
 *       first samples would be compared against a median that does not belong to the flight.
 * @note It is called by KalmanFilter_Init() and after a reset by the backup domain restore,
 *       so the module is seeded with the last known state instead of zero.
 */
void SensorCheck_Init(float initial_altitude, float initial_vertical_accel);

/**
 * @brief Checks one barometric altitude sample.
 * @param altitude Barometric altitude relative to the base altitude, meters.
 * @retval sensor_status_t.
 */
sensor_status_t SensorCheck_Altitude(float altitude);

/**
 * @brief Checks one vertical acceleration sample.
 * @param vertical_accel Vertical acceleration, m/s^2.
 * @retval sensor_status_t.
 * @note The allowed median difference is taken from the current flight phase, the wide
 *       boost limit is used in PHASE_IDLE and PHASE_BOOST and the tight one after burnout.
 */
sensor_status_t SensorCheck_VerticalAccel(float vertical_accel);


/**
 * @brief Tells whether the barometer is currently enabled.
 * @retval 1 while the barometer is trusted, 0 while it is disabled.
 * @note Used by the Kalman filter on the launch pad, a disabled barometer together with a
 *       low acceleration means the altitude estimate should be frozen instead of drifting.
 */
uint8_t sensor_check_is_baro_enabled(void);


#endif /* INC_SENSOR_CHECK_H_ */
