/*
 * kalman_filter.c
 *
 *  Created on: Apr 25, 2026
 *      Author: bilal
 */

#include "kalman_filter.h"
#include <stdint.h>
#include "flight_algorithm.h"


// Above this velocity the barometer is not trusted anymore (transonic region)
#define KF_MACH_THRESHOLD           300.0f  // m/s

// The mach transition is left at the latest after this time even if the velocity is still high
#define MACH_TRANSITION_TIMEOUT     5000    // milisecond

// Initial uncertainties, the diagonal of the P matrix
#define KF_ALTITUDE_UNCERTAINTY     1.0f    // (1 meter)
#define KF_VELOCITY_UNCERTAINTY     0.1f    // (0.3 m/s)
#define KF_ACCEL_UNCERTAINTY        1.0f    // (1 m/s²)

// Noise parameters, these are tunable
#define KF_PROCESS_NOISE            5.0f
#define KF_MEASUREMENT_NOISE_ALT    3.0f
#define KF_MEASUREMENT_NOISE_ACC    10.0f

/*
#define KF_PROCESS_NOISE            0.05f
#define KF_MEASUREMENT_NOISE_ALT    3.0f
#define KF_MEASUREMENT_NOISE_ACC    0.3f
*/

// On the pad without a usable barometer, accelerations below this value are treated as noise
#define KF_PAD_FREEZE_ACCEL_GATE    5.0f


/* --- Private Functions Declaration --- */
static void prediction_step(KalmanFilter_HandleTypeDef *kf, float dt);
static void update_step(KalmanFilter_HandleTypeDef *kf, float altitude, float accel);
static void update_with_altitude(KalmanFilter_HandleTypeDef *kf, float altitude);
static void update_with_accel(KalmanFilter_HandleTypeDef *kf, float accel);
static void kalman_filter_reset(KalmanFilter_HandleTypeDef *kf);


/**
 * @brief Initializes the filter, sets the state vector to zero and loads the default noise parameters.
 * @param kf Pointer to the handle structure.
 * @note It also initializes the sensor_check module with a zero altitude and a zero vertical acceleration.
 */
void KalmanFilter_Init(KalmanFilter_HandleTypeDef *kf)
{
    kf->X[0] = 0.0f;
    kf->X[1] = 0.0f;
    kf->X[2] = 0.0f;

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->P[i][j] = 0.0f;
        }
    }
    kf->P[0][0] = KF_ALTITUDE_UNCERTAINTY;
    kf->P[1][1] = KF_VELOCITY_UNCERTAINTY;
    kf->P[2][2] = KF_ACCEL_UNCERTAINTY;

    kf->F[0][0] = 1.0f; kf->F[0][1] = 0.0f; kf->F[0][2] = 0.0f;
    kf->F[1][0] = 0.0f; kf->F[1][1] = 1.0f; kf->F[1][2] = 0.0f;
    kf->F[2][0] = 0.0f; kf->F[2][1] = 0.0f; kf->F[2][2] = 1.0f;

    kf->process_noise = KF_PROCESS_NOISE;
    kf->measurement_noise_alt = KF_MEASUREMENT_NOISE_ALT;
    kf->measurement_noise_acc = KF_MEASUREMENT_NOISE_ACC;

    kf->in_mach_transition = 0;

    kf->is_mach_transition_finished = 0;

    SensorCheck_Init(0.0f, 0.0f);
}

/**
 * @brief Restores the state vector from the backup SRAM after an unexpected reset.
 * @param kf Pointer to the handle structure.
 * @param backup_domain Pointer to the backup domain handle.
 * @note Must be called after KalmanFilter_Init(). The covariance and the noise parameters keep
 *       their initial values, only the state vector is restored. If the restored velocity is
 *       still above the mach threshold the filter continues in the mach transition, and the
 *       sensor_check module is re-initialized with the restored values.
 */
void KalmanFilter_Recover(KalmanFilter_HandleTypeDef *kf, Backup_Domain_HandleTypeDef *backup_domain)
{
    // The three state floats are stored MSB first by the data manager, so the bytes are placed in reverse order
    uint8_t backup_data_state_vector[12] = { 0 };
    BackupDomain_ReadSRAM(backup_domain, backup_data_state_vector, BACKUP_DOMAIN_KALMAN_STATE_VECTOR, 12);

    type_converter_float_uint8_array type_converter;
    type_converter.uint8_array[3] = backup_data_state_vector[0];
    type_converter.uint8_array[2] = backup_data_state_vector[1];
    type_converter.uint8_array[1] = backup_data_state_vector[2];
    type_converter.uint8_array[0] = backup_data_state_vector[3];
    kf->X[0] = type_converter.float_;

    type_converter.uint8_array[3] = backup_data_state_vector[4];
    type_converter.uint8_array[2] = backup_data_state_vector[5];
    type_converter.uint8_array[1] = backup_data_state_vector[6];
    type_converter.uint8_array[0] = backup_data_state_vector[7];
    kf->X[1] = type_converter.float_;

    type_converter.uint8_array[3] = backup_data_state_vector[8];
    type_converter.uint8_array[2] = backup_data_state_vector[9];
    type_converter.uint8_array[1] = backup_data_state_vector[10];
    type_converter.uint8_array[0] = backup_data_state_vector[11];
    kf->X[2] = type_converter.float_;

    if (kf->X[1] > KF_MACH_THRESHOLD)
    {
        kf->in_mach_transition = 1;
        kf->mach_transition_start_time = HAL_GetTick();
    }

    SensorCheck_Init(kf->X[0], kf->X[2]);
}

/**
 * @brief Resets the filter to its initial state.
 * @param kf Pointer to the handle structure.
 */
void KalmanFilter_Reset(KalmanFilter_HandleTypeDef *kf)
{
    KalmanFilter_Init(kf);
}

/**
 * @brief Runs one prediction and one measurement update cycle.
 * @param kf Pointer to the handle structure.
 * @param altitude Barometric altitude relative to the base altitude, meters.
 * @param accel Vertical acceleration, m/s^2.
 * @param dt Time passed since the previous call, seconds.
 * @note Both measurements are checked by the sensor_check module first, and only the ones
 *       reported as SENSOR_OK are used in the update.
 */
void KalmanFilter_Update(KalmanFilter_HandleTypeDef *kf, float altitude, float accel, float dt)
{
    kf->altitude_status = SensorCheck_Altitude(altitude);
    kf->accel_status = SensorCheck_VerticalAccel(accel);

    /* Pad freeze: on the pad the barometer may already be disabled by the sensor check, and
     * then the only remaining measurement is the acceleration. Integrating its noise would
     * drift the velocity and the altitude away while the rocket is standing still, so as long
     * as the measured acceleration stays below the gate the filter is kept frozen at zero. */
    if ((flight_algorithm_get_flight_phase() == PHASE_IDLE) && (sensor_check_is_baro_enabled() == 0) && (accel < KF_PAD_FREEZE_ACCEL_GATE))
    {
        if (kf->X[1] != 0)
        {
            kalman_filter_reset(kf);
        }
        return;
    }

    prediction_step(kf, dt);
    update_step(kf, altitude, accel);
}

/* --- Private Functions Definitions --- */

/* ------------------------------------------------------------------
 * Prediction step: the state is carried forward with the constant
 * acceleration model and the uncertainty grows.
 *
 * x = F * x
 * P = F * P * F^T + Q
 *
 * The state product is written out directly instead of a matrix
 * multiplication, because F has only three non trivial dt terms.
 * ------------------------------------------------------------------ */
static void prediction_step(KalmanFilter_HandleTypeDef *kf, float dt)
{
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    /* x_n+1 = F * x_n */
    float x0 = kf->X[0] + kf->X[1] * dt + 0.5f * kf->X[2] * dt2;
    float x1 = kf->X[1] + kf->X[2] * dt;
    float x2 = kf->X[2];

    kf->X[0] = x0;
    kf->X[1] = x1;
    kf->X[2] = x2;


    kf->F[0][1] = dt;
    kf->F[0][2] = 0.5f * dt2;
    kf->F[1][2] = dt;

    /* P_n+1 = F * P_n * F^T + Q */
    // FP = F * P
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            kf->FP[r][c] = 0;
            for (uint8_t k = 0; k < 3; k++)
            {
                kf->FP[r][c] += kf->F[r][k] * kf->P[k][c];
            }
        }
    }

    // FPFT = FP * FT
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            kf->FPFT[r][c] = 0;
            for (uint8_t k = 0; k < 3; k++)
            {
                kf->FPFT[r][c] += kf->FP[r][k] * kf->F[c][k];
            }
        }
    }

    // Q of the constant acceleration model, scaled with the process noise
    kf->Q[0][0] = dt4 * 0.25f * kf->process_noise; kf->Q[0][1] = dt3 * 0.5f * kf->process_noise; kf->Q[0][2] = dt2 * 0.5f * kf->process_noise;
    kf->Q[1][0] = dt3 * 0.5f * kf->process_noise; kf->Q[1][1] = dt2 * kf->process_noise; kf->Q[1][2] = dt * kf->process_noise;
    kf->Q[2][0] = dt2 * 0.5f * kf->process_noise; kf->Q[2][1] = dt * kf->process_noise; kf->Q[2][2] = kf->process_noise;

    // P_n+1 = F(P_n)FT + Q
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            kf->P[r][c] = kf->FPFT[r][c] + kf->Q[r][c];
        }
    }
}

/* ------------------------------------------------------------------
 * Measurement update: applies the two scalar measurements one after
 * the other, and decides whether the barometer can be trusted.
 *
 * In the transonic region the shock wave around the vehicle makes the
 * static pressure, and with it the barometric altitude, unusable. While
 * the estimated velocity is above KF_MACH_THRESHOLD only the
 * accelerometer is used. The transition is left when the velocity drops
 * back below the threshold or when MACH_TRANSITION_TIMEOUT is reached,
 * and it is entered only once per flight (is_mach_transition_finished).
 * ------------------------------------------------------------------ */
static void update_step(KalmanFilter_HandleTypeDef *kf, float altitude, float accel)
{
    if (kf->in_mach_transition == 0 && kf->X[1] > KF_MACH_THRESHOLD && !kf->is_mach_transition_finished)
    {
        kf->in_mach_transition = 1;
        kf->mach_transition_start_time = HAL_GetTick();
    }
    else if (kf->in_mach_transition == 1 && (kf->X[1] < KF_MACH_THRESHOLD || ((HAL_GetTick() - kf->mach_transition_start_time) >= MACH_TRANSITION_TIMEOUT)))
    {
        kf->in_mach_transition = 0;
        kf->is_mach_transition_finished = 1;
    }

    if (kf->in_mach_transition)
    {
        if (kf->accel_status == SENSOR_OK)
        {
            update_with_accel(kf, accel);
        }
    }
    else
    {
        if (kf->altitude_status == SENSOR_OK)
        {
            update_with_altitude(kf, altitude);
        }
        if (kf->accel_status == SENSOR_OK)
        {
            update_with_accel(kf, accel);
        }
    }
}

/* ------------------------------------------------------------------
 * Sequential step 1: update with the barometric altitude
 *
 * Measurement model: z = H * x + v
 * H = [1, 0, 0]  ->  only the altitude state is observed
 * z = altitude (barometer measurement)
 * v = measurement noise with the covariance R_alt
 * ------------------------------------------------------------------ */
static void update_with_altitude(KalmanFilter_HandleTypeDef *kf, float altitude)
{
    /* --- 1. Innovation covariance S ---
     * S = H * P * H^T + R  ->  with H = [1, 0, 0] this is the scalar P[0][0] + R_alt,
     * the sum of the estimation and the measurement uncertainty. The larger S is, the
     * less the measurement is trusted. */
    float S = kf->P[0][0] + kf->measurement_noise_alt;

    /* --- 2. Kalman gain K ---
     * K = P * H^T / S  ->  with H^T = [1, 0, 0]^T this is the first column of P divided by S.
     * A large K means the measurement is trusted and the states are corrected strongly,
     * a small K means the model is trusted and the correction stays small. */
    for (uint8_t i = 0; i < 3; i++)
    {
        kf->K[i] = kf->P[i][0] / S;
    }

    /* --- 3. State update ---
     * x = x + K * y, where the innovation y = z - H * x = altitude - x[0] is the difference
     * between the measurement and the prediction. Every state is corrected by K[i] * y. */
    float innovation = altitude - kf->X[0];
    for (uint8_t i = 0; i < 3; i++)
    {
        kf->X[i] += kf->K[i] * innovation;
    }

    /* --- 4. Covariance update ---
     * P = (I - K * H) * P  ->  only the first column of K * H is non zero, so the expression
     * reduces to P[r][c] -= K[r] * P[0][c]. P shrinks, the measurement lowered the uncertainty. */
    /* Keep the P[0][:] row, so the loop uses the original values */
    float P0[3];
    for (uint8_t c = 0; c < 3; c++)
    {
        P0[c] = kf->P[0][c];
    }

    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            kf->P[r][c] -= kf->K[r] * P0[c];
        }
    }
}

/* ------------------------------------------------------------------
 * Sequential step 2: update with the vertical acceleration
 *
 * Measurement model: z = H * x + v
 * H = [0, 0, 1]  ->  only the acceleration state is observed
 * z = accel (accelerometer measurement, gravity corrected)
 * v = measurement noise with the covariance R_acc
 *
 * Note: this step runs after the altitude step, so P already carries
 * the result of the altitude update. During the mach transition it is
 * the only update that runs.
 * ------------------------------------------------------------------ */
static void update_with_accel(KalmanFilter_HandleTypeDef *kf, float accel)
{
    /* --- 1. Innovation covariance S ---
     * S = H * P * H^T + R  ->  with H = [0, 0, 1] this is the scalar P[2][2] + R_acc. */
    float S = kf->P[2][2] + kf->measurement_noise_acc;

    /* --- 2. Kalman gain K ---
     * K = P * H^T / S  ->  with H^T = [0, 0, 1]^T this is the third column of P divided by S. */
    for (uint8_t i = 0; i < 3; i++)
    {
        kf->K[i] = kf->P[i][2] / S;
    }

    /* --- 3. State update ---
     * The innovation y = accel - x[2] is the difference between the measured and the estimated
     * acceleration, and every state is corrected by K[i] * y. */
    float innovation = accel - kf->X[2];
    for (uint8_t i = 0; i < 3; i++)
    {
        kf->X[i] += kf->K[i] * innovation;
    }

    /* --- 4. Covariance update ---
     * P = (I - K * H) * P  ->  only the third column of K * H is non zero, so the expression
     * reduces to P[r][c] -= K[r] * P[2][c]. After the two sequential steps P reflects both
     * the altitude and the acceleration measurement. */
    /* Keep the P[2][:] row, so the loop uses the original values */
    float P2[3];
    for (uint8_t c = 0; c < 3; c++)
    {
        P2[c] = kf->P[2][c];
    }

    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            kf->P[r][c] -= kf->K[r] * P2[c];
        }
    }
}

/* Resets only the filter itself. Unlike KalmanFilter_Reset() the sensor_check module is left
 * untouched, so the sensor history collected on the pad is kept. */
static void kalman_filter_reset(KalmanFilter_HandleTypeDef *kf)
{
    kf->X[0] = 0.0f;
    kf->X[1] = 0.0f;
    kf->X[2] = 0.0f;

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->P[i][j] = 0.0f;
        }
    }
    kf->P[0][0] = KF_ALTITUDE_UNCERTAINTY;
    kf->P[1][1] = KF_VELOCITY_UNCERTAINTY;
    kf->P[2][2] = KF_ACCEL_UNCERTAINTY;

    kf->F[0][0] = 1.0f; kf->F[0][1] = 0.0f; kf->F[0][2] = 0.0f;
    kf->F[1][0] = 0.0f; kf->F[1][1] = 1.0f; kf->F[1][2] = 0.0f;
    kf->F[2][0] = 0.0f; kf->F[2][1] = 0.0f; kf->F[2][2] = 1.0f;

    kf->process_noise = KF_PROCESS_NOISE;
    kf->measurement_noise_alt = KF_MEASUREMENT_NOISE_ALT;
    kf->measurement_noise_acc = KF_MEASUREMENT_NOISE_ACC;

    kf->in_mach_transition = 0;

    kf->is_mach_transition_finished = 0;
}


/**
 * @brief Returns the estimated altitude in meters.
 * @param kf Pointer to the handle structure.
 * @retval Altitude state X[0].
 */
float KalmanFilter_GetAltitude(KalmanFilter_HandleTypeDef *kf)
{
    return kf->X[0];
}

/**
 * @brief Returns the estimated vertical velocity in m/s.
 * @param kf Pointer to the handle structure.
 * @retval Velocity state X[1].
 */
float KalmanFilter_GetVelocity(KalmanFilter_HandleTypeDef *kf)
{
    return kf->X[1];
}

/**
 * @brief Returns the estimated vertical acceleration in m/s^2.
 * @param kf Pointer to the handle structure.
 * @retval Acceleration state X[2].
 */
float KalmanFilter_GetAccel(KalmanFilter_HandleTypeDef *kf)
{
    return kf->X[2];
}

/**
 * @brief Returns the status of the last checked altitude measurement.
 * @param kf Pointer to the handle structure.
 * @retval sensor_status_t as uint8_t (0: SENSOR_OK, 1: SENSOR_ERROR).
 */
uint8_t kalman_filter_get_altitude_status(KalmanFilter_HandleTypeDef *kf)
{
    return (uint8_t)kf->altitude_status;
}

/**
 * @brief Returns the status of the last checked vertical acceleration measurement.
 * @param kf Pointer to the handle structure.
 * @retval sensor_status_t as uint8_t (0: SENSOR_OK, 1: SENSOR_ERROR).
 */
uint8_t kalman_filter_get_accel_status(KalmanFilter_HandleTypeDef *kf)
{
    return (uint8_t)kf->accel_status;
}

/**
 * @brief Returns the mach transition flag.
 * @param kf Pointer to the handle structure.
 * @retval 1 while the barometer is ignored because of the transonic region, 0 otherwise.
 */
uint8_t kalman_filter_get_in_mach_transition_flag(KalmanFilter_HandleTypeDef *kf)
{
    return kf->in_mach_transition;
}
