/*
 * quaternion.c
 *
 *   Edited by: bilal
 *
 *  References:
 *          https://github.com/halilsrky/SkyLord/blob/main/Core/Src/queternion.c
 */

#include "quaternion.h"
#include <stdint.h>
#include <math.h>


/* ***************   MAHONY   *************** */
#define twoKpDef	(2.0f * 2.0f)	// Fast correction, more resistant to the accelerations seen by the sensor
#define twoKiDef	(2.0f * 0.01f)	// Slow but stable correction against the gyroscope bias
// The gains selected by update_mahony_gains() are clamped into these limits before they are used.
// Note that the highest gain set (twoKp = 8.0f) is cut down to TWO_KP_MAX by that clamp.
#define TWO_KP_MAX 4.0f
#define TWO_KP_MIN 0.1f
#define TWO_KI_MAX 0.05f
#define TWO_KI_MIN 0.0f
/* ****************************************** */

// Accelerometer magnitude limits of the gyro only mode. The two thresholds are different on
// purpose, they give the hysteresis used in Quaternion_UpdateOrientation().
#define G 9.80665f
#define ACC_THRESHOLD_HIGH (2.3f * G)
#define ACC_THRESHOLD_LOW  (1.7f * G)

#define RAD_TO_DEG 57.2957795f


/* --- Private Functions Declaration --- */
/* Filter Functions */
static void update_quaternion(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float dt);
static void update_mahony_gains(Quaternion_HandleTypeDef *quaternion, float ax, float ay, float az);
static void mahony_ahrs_update_imu(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt);

/* Euler Angle Functions */
static void quaternion_to_ztest(const Quaternion_HandleTypeDef *quaternion, float *w, float *x, float *y, float *z);

/* Math Utility Functions */
static float invSqrt(float x);


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
void Quaternion_Init(Quaternion_HandleTypeDef *quaternion, float acc_x, float acc_y, float acc_z)
{
    quaternion->q[0] = 1.0f;
    quaternion->q[1] = 0.0f;
    quaternion->q[2] = 0.0f;
    quaternion->q[3] = 0.0f;

    quaternion->gyroOnlyMode = 0;
    quaternion->gain = 0;
    
    quaternion->twoKp = twoKpDef;
    quaternion->twoKi = twoKiDef;
    quaternion->integralFBx = 0.0f,  quaternion->integralFBy = 0.0f, quaternion->integralFBz = 0.0f;

    quaternion->ax_f = acc_x;
    quaternion->ay_f = acc_y;
    quaternion->az_f = acc_z;


    double norm = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    if (norm < 1e-6f) return;

    double a0 = (double)acc_x / norm;
    double a1 = (double)acc_y / norm;
    double a2 = (double)acc_z / norm;

    // Minimum rotation that takes the measured gravity vector onto the [1,0,0] (X axis) reference
    // cross([a0,a1,a2], [1,0,0]) = [0, a2, -a1]
    double cross_y = a2;
    double cross_z =  -a1;
    double cross_mag = sqrtf(cross_y*cross_y + cross_z*cross_z);

    if (cross_mag < 1e-6f)
    {
        // Gravity is already along +X or -X (the rocket is exactly upright or completely upside down)
        if (a0 > 0.0f)
        {
            quaternion->q[0]=1.f; quaternion->q[1]=0.f; quaternion->q[2]=0.f; quaternion->q[3]=0.f;
        }
        else
        {
            quaternion->q[0]=0.f; quaternion->q[1]=1.f; quaternion->q[2]=0.f; quaternion->q[3]=0.f; // 180 degree flip
        }
        return;
    }

    double half = acosf(fmaxf(-1.f, fminf(1.f, a0))) * 0.5f;
    double s = sinf(half) / cross_mag;

    quaternion->q[0] = cosf(half);
    quaternion->q[1] = 0.0f;
    quaternion->q[2] = cross_y * s;
    quaternion->q[3] = cross_z * s;

    double n = sqrtf(quaternion->q[0]*quaternion->q[0] + quaternion->q[1]*quaternion->q[1] + quaternion->q[2]*quaternion->q[2] + quaternion->q[3]*quaternion->q[3]);
    quaternion->q[0]/=n; quaternion->q[1]/=n; quaternion->q[2]/=n; quaternion->q[3]/=n;
}

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
void Quaternion_Recover(Quaternion_HandleTypeDef *quaternion, float acc_x, float acc_y, float acc_z, Backup_Domain_HandleTypeDef *backup_domain)
{
    uint8_t backup_data_quaternion[16] = { 0 };
    BackupDomain_ReadSRAM(backup_domain, backup_data_quaternion, BACKUP_DOMAIN_QUATERNION, 16);

    type_converter_float_uint8_array type_converter;
    type_converter.uint8_array[3] = backup_data_quaternion[0];
    type_converter.uint8_array[2] = backup_data_quaternion[1];
    type_converter.uint8_array[1] = backup_data_quaternion[2];
    type_converter.uint8_array[0] = backup_data_quaternion[3];
    quaternion->q[0] = type_converter.float_;

    type_converter.uint8_array[3] = backup_data_quaternion[4];
    type_converter.uint8_array[2] = backup_data_quaternion[5];
    type_converter.uint8_array[1] = backup_data_quaternion[6];
    type_converter.uint8_array[0] = backup_data_quaternion[7];
    quaternion->q[1] = type_converter.float_;

    type_converter.uint8_array[3] = backup_data_quaternion[8];
    type_converter.uint8_array[2] = backup_data_quaternion[9];
    type_converter.uint8_array[1] = backup_data_quaternion[10];
    type_converter.uint8_array[0] = backup_data_quaternion[11];
    quaternion->q[2] = type_converter.float_;

    type_converter.uint8_array[3] = backup_data_quaternion[12];
    type_converter.uint8_array[2] = backup_data_quaternion[13];
    type_converter.uint8_array[1] = backup_data_quaternion[14];
    type_converter.uint8_array[0] = backup_data_quaternion[15];
    quaternion->q[3] = type_converter.float_;

    quaternion->gyroOnlyMode = 1;
    quaternion->gain = 0;
    
    quaternion->twoKp = twoKpDef;
    quaternion->twoKi = twoKiDef;
    quaternion->integralFBx = 0.0f,  quaternion->integralFBy = 0.0f, quaternion->integralFBz = 0.0f;

    quaternion->ax_f = acc_x;
    quaternion->ay_f = acc_y;
    quaternion->az_f = acc_z;
}

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
void Quaternion_UpdateOrientation(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    const float alpha = 0.3f;

    // Low pass filter
    quaternion->ax_f = alpha * ax + (1.0f - alpha) * quaternion->ax_f;
    quaternion->ay_f = alpha * ay + (1.0f - alpha) * quaternion->ay_f;
    quaternion->az_f = alpha * az + (1.0f - alpha) * quaternion->az_f;

    // Update the gains
    update_mahony_gains(quaternion, quaternion->ax_f, quaternion->ay_f, quaternion->az_f);

    // Acc magnitude
    float accMag = sqrtf(quaternion->ax_f*quaternion->ax_f + quaternion->ay_f*quaternion->ay_f + quaternion->az_f*quaternion->az_f);

    if (quaternion->gyroOnlyMode)
    {
        if (accMag < ACC_THRESHOLD_LOW) quaternion->gyroOnlyMode = 0;
    }
    else
    {
        if (accMag > ACC_THRESHOLD_HIGH) quaternion->gyroOnlyMode = 1;
    }

    // Run the filter
    if (quaternion->gyroOnlyMode)
    {
        update_quaternion(quaternion, gx, gy, gz, dt);
    }
    else
    {
        mahony_ahrs_update_imu(quaternion, gx, gy, gz, quaternion->ax_f, quaternion->ay_f, quaternion->az_f, dt);
    }
}

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
void Quaternion_UpdateOrientation_Mahony(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;   // expected dt of the 200hz accelerometer data -> 0.005f

    const float alpha = 0.3f;

    // Low pass filter
    quaternion->ax_f = alpha * ax + (1.0f - alpha) * quaternion->ax_f;
    quaternion->ay_f = alpha * ay + (1.0f - alpha) * quaternion->ay_f;
    quaternion->az_f = alpha * az + (1.0f - alpha) * quaternion->az_f;

    // Update the gains
    update_mahony_gains(quaternion, quaternion->ax_f, quaternion->ay_f, quaternion->az_f);

    // Acc magnitude
    float accMag = sqrtf(quaternion->ax_f*quaternion->ax_f + quaternion->ay_f*quaternion->ay_f + quaternion->az_f*quaternion->az_f);

    if (accMag > ACC_THRESHOLD_HIGH)
    {
        quaternion->gyroOnlyMode = 1;
        if (dt > 0.02f) dt = 0.001f;
        update_quaternion(quaternion, gx, gy, gz, dt);
        return;
    }

    // Run the filter
    mahony_ahrs_update_imu(quaternion, gx, gy, gz, quaternion->ax_f, quaternion->ay_f, quaternion->az_f, dt);
}

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
void Quaternion_UpdateOrientation_GyroOnlyMode(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float dt)
{
    if (dt <= 0.0f || dt > 0.02f) dt = 0.001f;   // expected dt of the 1000hz gyroscope data -> 0.001f
    update_quaternion(quaternion, gx, gy, gz, dt);
}


/**
 * @brief Integrates the gyroscope into the quaternion for one step and normalizes the result.
 * @param quaternion Pointer to the handle structure.
 * @param gx Gyroscope X, rad/s.
 * @param gy Gyroscope Y, rad/s.
 * @param gz Gyroscope Z, rad/s.
 * @param dt Integration step, seconds.
 * @note qDot = 0.5 * q (x) [0, gx, gy, gz] with a first order (Euler) integration. There is no
 *       correction here, so the gyroscope bias integrates directly into the attitude as drift.
 * @note The normalization is skipped when the norm drops below 1e-6, which keeps the previous
 *       quaternion instead of dividing by zero.
 */
static void update_quaternion(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float dt)
{
    // Quaternion derivatives
    float qDot1 = 0.5f * (-quaternion->q[1] * gx - quaternion->q[2] * gy - quaternion->q[3] * gz);
    float qDot2 = 0.5f * ( quaternion->q[0] * gx + quaternion->q[2] * gz - quaternion->q[3] * gy);
    float qDot3 = 0.5f * ( quaternion->q[0] * gy - quaternion->q[1] * gz + quaternion->q[3] * gx);
    float qDot4 = 0.5f * ( quaternion->q[0] * gz + quaternion->q[1] * gy - quaternion->q[2] * gx);

    // Integration
    quaternion->q[0] += qDot1 * dt;
    quaternion->q[1] += qDot2 * dt;
    quaternion->q[2] += qDot3 * dt;
    quaternion->q[3] += qDot4 * dt;

    // Normalize
    float norm = sqrtf(quaternion->q[0]*quaternion->q[0] + quaternion->q[1]*quaternion->q[1] + quaternion->q[2]*quaternion->q[2] + quaternion->q[3]*quaternion->q[3]);
    if (norm > 1e-6f)
    {
        quaternion->q[0] /= norm;
        quaternion->q[1] /= norm;
        quaternion->q[2] /= norm;
        quaternion->q[3] /= norm;
    }
}

/**
 * @brief Selects the Mahony gains from the angle between the measured and the estimated up direction.
 * @param quaternion Pointer to the handle structure.
 * @param ax Filtered accelerometer X, m/s^2.
 * @param ay Filtered accelerometer Y, m/s^2.
 * @param az Filtered accelerometer Z, m/s^2.
 * @note The accelerometer vector is normalized locally, the copy in the handle is not touched.
 *       The quaternion however is normalized in place, because the up direction estimate would
 *       be wrong otherwise. A quaternion with a zero norm leaves the gains unchanged.
 * @note The local gx, gy, gz are not gyroscope values, they are the estimated up direction
 *       (the reference +X axis expressed in the body frame).
 * @note Gain sets: error angle > 30 deg -> gain 1 (twoKp 0.2, twoKi 0), the accelerometer is
 *       barely trusted; > 10 deg -> gain 2 (2.0, 0.01); otherwise gain 3 (8.0, 0.05), the
 *       strongest correction. The clamp at the end cuts the 8.0 of gain 3 down to TWO_KP_MAX (4.0).
 * @note twoKi = 0 in gain 1 also clears the integral terms in mahony_ahrs_update_imu(), so a
 *       long lasting large accelerometer error resets the accumulated bias estimate.
 */
static void update_mahony_gains(Quaternion_HandleTypeDef *quaternion, float ax, float ay, float az)
{
    // The gravity direction estimate would be wrong if the quaternion was not normalized
    float norm_q = sqrtf(quaternion->q[0]*quaternion->q[0] + quaternion->q[1]*quaternion->q[1] + quaternion->q[2]*quaternion->q[2] + quaternion->q[3]*quaternion->q[3]);
    if (norm_q == 0.0f) return; // Protection
    quaternion->q[0] /= norm_q;
    quaternion->q[1] /= norm_q;
    quaternion->q[2] /= norm_q;
    quaternion->q[3] /= norm_q;

    // Gravity direction vector (full vector form)
    float gx = 2.0f * (quaternion->q[0]*quaternion->q[0] + quaternion->q[1]*quaternion->q[1]) - 1.0f; 
    float gy = 2.0f * (quaternion->q[1]*quaternion->q[2] - quaternion->q[0]*quaternion->q[3]);
    float gz = 2.0f * (quaternion->q[1]*quaternion->q[3] + quaternion->q[0]*quaternion->q[2]);

    // Normalize the gravity estimate
    float recipNormG = invSqrt(gx * gx + gy * gy + gz * gz);
    gx *= recipNormG;
    gy *= recipNormG;
    gz *= recipNormG;

    // Normalize the accelerometer vector, only this local copy is touched, not the one in the handle
    float recipNormA = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNormA;
    ay *= recipNormA;
    az *= recipNormA;

    // Dot product
    float dot = ax * gx + ay * gy + az * gz;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;

    // Angle difference (degrees)
    float accErrorAngle = acosf(dot) * (180.0f / 3.1415926f);

    // Set the gains according to the current situation
    if (accErrorAngle > 30.0f)
    {
        quaternion->gain = 1;
        quaternion->twoKp = 0.2f;
        quaternion->twoKi = 0.0f;
    }
    else if (accErrorAngle > 10.0f)
    {
        quaternion->gain = 2;
        quaternion->twoKp = 2.0f;
        quaternion->twoKi = 0.01f;
    }
    else
    {
        quaternion->gain = 3;
        quaternion->twoKp = 8.0f;
        quaternion->twoKi = 0.05f;
    }

    // Safety: limit the gains (optional but recommended)
    quaternion->twoKp = fmaxf(TWO_KP_MIN, fminf(quaternion->twoKp, TWO_KP_MAX));
    quaternion->twoKi = fmaxf(TWO_KI_MIN, fminf(quaternion->twoKi, TWO_KI_MAX));
}

/**
 * @brief Mahony AHRS update, corrects the gyroscope integration with the accelerometer (PI feedback).
 * @param quaternion Pointer to the handle structure.
 * @param gx Gyroscope X, rad/s.
 * @param gy Gyroscope Y, rad/s.
 * @param gz Gyroscope Z, rad/s.
 * @param ax Not used, the filtered vector is read from the handle (ax_f).
 * @param ay Not used, the filtered vector is read from the handle (ay_f).
 * @param az Not used, the filtered vector is read from the handle (az_f).
 * @param dt Integration step, seconds.
 * @note The error is the cross product between the measured and the estimated up direction.
 *       The proportional part is added to the gyroscope directly, while the integral part
 *       accumulates the error as a gyroscope bias estimate as long as twoKi > 0. When twoKi
 *       is 0 the integral terms are cleared to prevent windup.
 * @note Both vectors are kept as half values (halfv, halfe), which is why the gains are stored
 *       as 2 * Kp and 2 * Ki.
 * @note The filtered accelerometer vector of the handle is normalized in place, so after this
 *       call ax_f, ay_f and az_f are a unit vector and no longer in m/s^2. The next low pass
 *       filter step in the caller therefore starts from that unit vector.
 */
static void mahony_ahrs_update_imu(Quaternion_HandleTypeDef *quaternion, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc, qd;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((quaternion->ax_f == 0.0f) && (quaternion->ay_f == 0.0f) && (quaternion->az_f == 0.0f)))
    {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(quaternion->ax_f*quaternion->ax_f + quaternion->ay_f*quaternion->ay_f + quaternion->az_f*quaternion->az_f);
		quaternion->ax_f *= recipNorm;
		quaternion->ay_f *= recipNorm;
		quaternion->az_f *= recipNorm;

		// Estimated up direction (the reference +X axis expressed in the body frame), kept as half values
        halfvx = quaternion->q[0] * quaternion->q[0] + quaternion->q[1] * quaternion->q[1] - 0.5f;
        halfvy = quaternion->q[1] * quaternion->q[2] - quaternion->q[0] * quaternion->q[3];
        halfvz = quaternion->q[1] * quaternion->q[3] + quaternion->q[0] * quaternion->q[2];

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (quaternion->ay_f * halfvz - quaternion->az_f * halfvy);
		halfey = (quaternion->az_f * halfvx - quaternion->ax_f * halfvz);
		halfez = (quaternion->ax_f * halfvy - quaternion->ay_f * halfvx);

		// Compute and apply integral feedback if enabled
		if(quaternion->twoKi > 0.0f)
        {
			quaternion->integralFBx += quaternion->twoKi * halfex * dt;	// integral error scaled by Ki
			quaternion->integralFBy += quaternion->twoKi * halfey * dt;
			quaternion->integralFBz += quaternion->twoKi * halfez * dt;
			gx += quaternion->integralFBx;	// apply integral feedback
			gy += quaternion->integralFBy;
			gz += quaternion->integralFBz;
		}
		else
        {
			quaternion->integralFBx = 0.0f;	// prevent integral windup
			quaternion->integralFBy = 0.0f;
			quaternion->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += quaternion->twoKp * halfex;
		gy += quaternion->twoKp * halfey;
		gz += quaternion->twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = quaternion->q[0];
	qb = quaternion->q[1];
	qc = quaternion->q[2];
    qd = quaternion->q[3];
	quaternion->q[0] += (-qb * gx - qc * gy - qd * gz);
	quaternion->q[1] += (qa * gx + qc * gz - qd * gy);
	quaternion->q[2] += (qa * gy - qb * gz + qd * gx);
	quaternion->q[3] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(quaternion->q[0] * quaternion->q[0] + quaternion->q[1] * quaternion->q[1] + quaternion->q[2] * quaternion->q[2] + quaternion->q[3] * quaternion->q[3]);
	quaternion->q[0] *= recipNorm;
	quaternion->q[1] *= recipNorm;
	quaternion->q[2] *= recipNorm;
	quaternion->q[3] *= recipNorm;
}


/**
 * @brief Returns the tilt angle of the rocket, the angle between the body +X axis and the vertical.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, 0 when the rocket is upright and 180 when it points straight down.
 * @note Computed directly from the quaternion, it does not use the Euler angles below, so it
 *       has no singularity.
 */
float Quaternion_GetTheta(Quaternion_HandleTypeDef *quaternion)
{
    float vx = quaternion->q[0]*quaternion->q[0] + quaternion->q[1]*quaternion->q[1] - quaternion->q[2]*quaternion->q[2] - quaternion->q[3]*quaternion->q[3];
    float vy = 2.0f * (quaternion->q[1]*quaternion->q[2] - quaternion->q[0]*quaternion->q[3]);
    float vz = 2.0f * (quaternion->q[1]*quaternion->q[3] + quaternion->q[0]*quaternion->q[2]);

    return atan2f(sqrtf(vy*vy + vz*vz), vx) * RAD_TO_DEG;
}

/**
 * @brief Returns the roll angle in the Z based frame.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, in the -180..180 range.
 */
float Quaternion_GetRoll_Z(Quaternion_HandleTypeDef *quaternion)
{
    float w, x, y, z;
    quaternion_to_ztest(quaternion, &w, &x, &y, &z);
    float roll = atan2f(2.0f*(w*x + y*z), w*w - x*x - y*y + z*z) * 180.0f / 3.14159f;
    return roll;
}

/**
 * @brief Returns the pitch angle in the Z based frame.
 * @param quaternion Pointer to the handle structure.
 * @retval Angle in degrees, in the -90..90 range.
 * @note The argument of the arcsine is clamped to -1..1, so the value stays valid at the
 *       gimbal lock point, but the yaw and roll angles lose their meaning there. Use
 *       Quaternion_GetTheta() when only the tilt of the rocket is needed.
 */
float Quaternion_GetPitch_Z(Quaternion_HandleTypeDef *quaternion)
{
    float w, x, y, z;
    quaternion_to_ztest(quaternion, &w, &x, &y, &z);
    float pitch = 2.0f * (x*z - w*y);
    float safeValue = fmaxf(-1.0f, fminf(1.0f, pitch));
    return -asinf(safeValue) * (180.0f / 3.14159f);
}

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
float Quaternion_GetYaw_Z(Quaternion_HandleTypeDef *quaternion)
{
    float w, x, y, z;
    quaternion_to_ztest(quaternion, &w, &x, &y, &z);
    float yaw = atan2f(2.0f*(x*y + w*z), w*w + x*x - y*y - z*z) * 180.0f / 3.14159f;
    return yaw;
}


/**
 * @brief Rotates the attitude into a Z based frame, so that the standard Euler formulas can be used.
 * @param quaternion Pointer to the handle structure.
 * @param w Pointer to the scalar part of the rotated quaternion.
 * @param x Pointer to the X part of the rotated quaternion.
 * @param y Pointer to the Y part of the rotated quaternion.
 * @param z Pointer to the Z part of the rotated quaternion.
 * @note q' = p (x) q, where p = (0.7071, 0, 0.7071, 0) is a fixed 90 degree rotation around Y.
 *       It maps the +X up convention of this module onto a -Z up one, which is what the roll,
 *       pitch and yaw formulas expect.
 * @note Only the returned copy is rotated, the quaternion inside the handle is left as it is.
 */
static void quaternion_to_ztest(const Quaternion_HandleTypeDef *quaternion, float *w, float *x, float *y, float *z)
{
    const float k = 0.70710678f;   // p = (k, 0, +k, 0): for the -Z up configuration
    float qw = quaternion->q[0], qx = quaternion->q[1], qy = quaternion->q[2], qz = quaternion->q[3];

    // q' = p ⊗ q
    *w = k * (qw - qy);
    *x = k * (qx + qz);
    *y = k * (qy + qw);
    *z = k * (qz - qx);
}


/**
 * @brief Fast inverse square root (Quake III algorithm)
 * @param x Input value
 * @retval float 1/sqrt(x) approximation
 * @note Internal utility; used for quaternion normalization
 */
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
