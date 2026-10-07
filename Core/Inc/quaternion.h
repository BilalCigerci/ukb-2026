/*
 * quaternion.h
 *
 *   Edited by: bilal
 *
 *  References:
 *          https://github.com/halilsrky/SkyLord/blob/main/Core/Inc/queternion.h
 */

/*
 * Quaternion based attitude estimation (Mahony AHRS, IMU only)
 * -----------------------------------------------------------------------------------------
 * The module keeps the orientation of the rocket in a unit quaternion q = [w, x, y, z] and
 * updates it from the gyroscope and the accelerometer. There is no magnetometer, so the
 * rotation around the vertical axis is not observable and only the gyroscope drives it.
 *
 * Frames and axis convention
 *   - The body frame is the flight_data frame, so the caller must apply the sign convention
 *     used in data_manager.c / main.c (accel_x = -bmi088.acc_x, accel_y = -bmi088.acc_y,
 *     accel_z = +bmi088.acc_z) before calling the functions of this module.
 *   - The rocket longitudinal axis is +X, therefore the vertical (up) reference direction of
 *     the reference frame is +X as well, not +Z. When the rocket stands upright, the
 *     accelerometer vector is about [+1 g, 0, 0].
 *   - q rotates a body frame vector into the reference frame, so the first row of its rotation
 *     matrix is the reference +X axis expressed in the body frame. That vector is the "up"
 *     estimate used both by the Mahony correction and by Quaternion_GetTheta().
 *
 * Units
 *   - Gyroscope inputs are rad/s (BMI088 driver already converts deg/s to rad/s).
 *   - Accelerometer inputs are m/s^2, dt is in seconds.
 *   - All getters return degrees.
 *
 * Two update paths
 *   - Mahony mode: the accelerometer corrects the gyroscope integration with a PI feedback,
 *     whose gains are chosen at runtime from the angle between the measured and the estimated
 *     up direction (see update_mahony_gains() in quaternion.c).
 *   - Gyro only mode: pure gyroscope integration, used while the accelerometer is not
 *     trustworthy (boost, high acceleration). It is entered when the filtered accelerometer
 *     magnitude passes 2.3 g and, in Quaternion_UpdateOrientation_Mahony(), it is latched:
 *     the flag is never cleared again, so the rest of the flight runs on the gyroscope only.
 *
 * The accelerometer input is low pass filtered inside the module (alpha = 0.3) and the
 * filtered vector is kept in the handle (ax_f, ay_f, az_f), so the caller always passes the
 * raw measurement.
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_


#include <stdint.h>
#include "backup_domain.h"


/**
 * @brief Main Quaternion Handle structure.
 * Holds the attitude quaternion, the Mahony gains, the integral feedback terms and the
 * internal state of the accelerometer low pass filter.
 */
typedef struct
{
    float q[4];     // [w, x, y, z], unit quaternion, rotates body frame vectors into the reference frame
    volatile float twoKp;									// 2 * proportional gain (Kp), rewritten in every update from the accelerometer error angle
    volatile float twoKi;									// 2 * integral gain (Ki), 0 disables and resets the integral feedback
    volatile float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki
    volatile uint8_t gyroOnlyMode;                          // 1 while the accelerometer correction is skipped and only the gyroscope is integrated
    volatile float ax_f, ay_f, az_f;                        // Low pass filtered accelerometer vector, m/s^2 (normalized in place during a Mahony update)
    volatile uint8_t gain;                                  // Gain set selected by the last gain update: 0 not set yet, 1 accel error > 30 deg, 2 > 10 deg, 3 otherwise
}Quaternion_HandleTypeDef;

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
 * @brief Initializes the handle and aligns the quaternion with the measured gravity vector.
 * @param quaternion Pointer to the handle structure.
 * @param acc_x Accelerometer X in the body frame, m/s^2.
 * @param acc_y Accelerometer Y in the body frame, m/s^2.
 * @param acc_z Accelerometer Z in the body frame, m/s^2.
 * @note The quaternion is built as the minimum rotation that takes the measured (normalized)
 *       gravity direction onto the +X reference axis, so the initial tilt of the rocket is
 *       captured but the rotation around the vertical axis starts from zero.
 * @note Starts in Mahony mode (gyroOnlyMode = 0) with the default gains, and seeds the
 *       accelerometer low pass filter with the given vector.
 */
void Quaternion_Init(Quaternion_HandleTypeDef *quaternion, float acc_x, float acc_y, float acc_z);

/**
 * @brief Restores the quaternion from the backup SRAM after an unexpected reset.
 * @param quaternion Pointer to the handle structure.
 * @param acc_x Accelerometer X in the body frame, m/s^2.
 * @param acc_y Accelerometer Y in the body frame, m/s^2.
 * @param acc_z Accelerometer Z in the body frame, m/s^2.
 * @param backup_domain Pointer to the backup domain handle.
 * @note Reads 16 bytes (4 floats, MSB first) from the BACKUP_DOMAIN_QUATERNION offset, so it
 *       replaces Quaternion_Init(): the attitude is taken from the backup instead of being
 *       rebuilt from the accelerometer.
 * @note The reset is assumed to have happened during the flight, so the module comes up in
 *       gyro only mode (gyroOnlyMode = 1). The gains, the integral terms and the accelerometer
 *       low pass filter are reset the same way as in Quaternion_Init().
 */
void Quaternion_Recover(Quaternion_HandleTypeDef *quaternion, float acc_x, float acc_y, float acc_z, Backup_Domain_HandleTypeDef *backup_domain);

/**
 * @brief Runs one update and switches between Mahony and gyro only mode with hysteresis.
 * @param quaternion Pointer to the handle structure.
 * @param gx Gyroscope X, rad/s.
 * @param gy Gyroscope Y, rad/s.
 * @param gz Gyroscope Z, rad/s.
 * @param ax Accelerometer X, m/s^2.
 * @param ay Accelerometer Y, m/s^2.
 * @param az Accelerometer Z, m/s^2.
 * @param dt Time passed since the previous call, seconds.
 * @note Gyro only mode is entered when the filtered accelerometer magnitude passes 2.3 g and
 *       left again when it drops below 1.7 g, so unlike Quaternion_UpdateOrientation_Mahony()
 *       the mode is not latched.
 * @note dt is used as it is given, there is no clamping here.
 * @note Not used by the flight software, it is kept as an alternative.
 */
void Quaternion_UpdateOrientation(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief Runs one Mahony update with the accelerometer correction (accelerometer rate, 200 Hz).
 * @param quaternion Pointer to the handle structure.
 * @param gx Gyroscope X, rad/s.
 * @param gy Gyroscope Y, rad/s.
 * @param gz Gyroscope Z, rad/s.
 * @param ax Accelerometer X, m/s^2.
 * @param ay Accelerometer Y, m/s^2.
 * @param az Accelerometer Z, m/s^2.
 * @param dt Time passed since the previous call, seconds. An invalid value (<= 0 or > 0.05)
 *           is replaced with 0.005 s, the expected period of the 200 Hz accelerometer data.
 * @note When the filtered accelerometer magnitude passes 2.3 g, gyroOnlyMode is set to 1 and
 *       only the gyroscope is integrated for that step. The flag is never cleared inside this
 *       module, so after the first threshold crossing sensor_fusion keeps calling
 *       Quaternion_UpdateOrientation_GyroOnlyMode() for the rest of the flight.
 * @note The Mahony gains are refreshed in every call from the angle between the measured and
 *       the estimated up direction, before the filter is run.
 */
void Quaternion_UpdateOrientation_Mahony(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief Integrates the gyroscope only, without any accelerometer correction (gyro rate, 1000 Hz).
 * @param quaternion Pointer to the handle structure.
 * @param gx Gyroscope X, rad/s.
 * @param gy Gyroscope Y, rad/s.
 * @param gz Gyroscope Z, rad/s.
 * @param dt Time passed since the previous call, seconds. An invalid value (<= 0 or > 0.02)
 *           is replaced with 0.001 s, the expected period of the 1000 Hz gyroscope data.
 * @note The quaternion is normalized after every integration step. The gyroOnlyMode flag is
 *       not touched here, the caller decides which update function to run.
 */
void Quaternion_UpdateOrientation_GyroOnlyMode(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float dt);


/**
 * @brief Returns the tilt angle of the rocket, the angle between the body +X axis and the vertical.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, 0 when the rocket is upright and 180 when it points straight down.
 * @note Computed directly from the quaternion, it does not use the Euler angles below, so it
 *       has no singularity.
 */
float Quaternion_GetTheta(Quaternion_HandleTypeDef *quaternion);

/**
 * @brief Returns the roll angle in the Z based frame.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, in the -180..180 range.
 */
float Quaternion_GetRoll_Z(Quaternion_HandleTypeDef *quaternion);

/**
 * @brief Returns the pitch angle in the Z based frame.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, in the -90..90 range.
 * @note The argument of the arcsine is clamped to -1..1, so the value stays valid at the
 *       gimbal lock point, but the yaw and roll angles lose their meaning there. Use
 *       Quaternion_GetTheta() when only the tilt of the rocket is needed.
 */
float Quaternion_GetPitch_Z(Quaternion_HandleTypeDef *quaternion);

/**
 * @brief Returns the yaw angle in the Z based frame, the rotation around the vertical axis.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, in the -180..180 range.
 * @note The quaternion is first rotated 90 degrees around Y into a Z based frame, so that the
 *       standard Euler formulas can be used (see quaternion_to_ztest() in quaternion.c).
 * @note There is no magnetometer, so this angle is not absolute. It only shows how much the
 *       rocket has rolled around its own axis since the initialization and it drifts with the
 *       gyroscope bias.
 */
float Quaternion_GetYaw_Z(Quaternion_HandleTypeDef *quaternion);


#endif /* QUATERNION_H_ */
