/*
 * sensor_check.c
 *
 *  Created on: Jul 15, 2026
 *      Author: bilal
 */

#include "sensor_check.h"
#include <stdint.h>
#include <math.h>
#include "flight_algorithm.h"


// Physical altitude limits of the flight, a value outside of them cannot be a real measurement
#define MIN_ALTITUDE                                -200.0f // meter
#define MAX_ALTITUDE                                6000.0f // meter

// Physical vertical acceleration limits, the negative side is wider than free fall on purpose
#define MIN_VERTICAL_ACCEL                          -100.0f // meters per second squared
#define MAX_VERTICAL_ACCEL                          250.0f  // meters per second squared

// Largest accepted difference between a sample and the median of the recent history
#define MAX_ALTITUDE_DIFFERENCE                     200.0f  // meter
#define MAX_VERTICAL_ACCEL_DIFFERENCE               10.0f   // meters per second squared
// During the motor burn the real acceleration changes very fast, so the limit is widened
#define MAX_VERTICAL_ACCEL_DIFFERENCE_BOOST_PHASE   150.0f  // meters per second squared

// Number of samples with the exact same value that marks the sensor as frozen
#define STUCK_COUNTER_THRESHOLD                     4

#define ERROR_COUNTER_THRESHOLD                     5   // Error count that disables a sensor
#define RECOVER_COUNTER_THRESHOLD                   10  // Healthy sample count that enables it again

// How many median violations in a row are needed before the sensor is counted as faulty.
// The altitude threshold is 1, so a single altitude spike counts immediately, while two
// acceleration spikes are still tolerated and only the third one counts.
#define ALTITUDE_DELTA_CONFIRM_THRESHOLD            1
#define VERTICAL_ACCEL_DELTA_CONFIRM_THRESHOLD      3

#define HISTORY_SIZE                                5   // Number of samples the median is taken from


/**
 * @brief State of a single checked sensor, the two sensors keep their own copy.
 */
typedef struct
{
    float history[HISTORY_SIZE];        // Circular buffer of the recent accepted samples
    uint8_t history_index;              // Write position of the circular buffer
    uint8_t delta_violation_counter;    // Median violations seen in a row
    uint8_t delta_confirm_threshold;    // Violations needed before they are counted as an error

    float previous_value;   // Last accepted sample, used by the stuck check
    uint8_t stuck_counter;  // Samples repeating the previous value, saturated at the threshold
    uint8_t error_counter;  // Faults seen so far, decreased by every fully healthy sample
    uint8_t recover_counter;// Healthy samples collected while the sensor is disabled
    uint8_t is_enabled;     // 1 while the sensor is trusted, 0 while it is disabled
} sensor_t;


static sensor_t baro;
static sensor_t imu;


/* --- Private Functions Declaration --- */
/* Check Functions */
static void sensor_check_init_sensor(sensor_t *sensor, float initial_sensor_value, uint8_t delta_confirm_threshold);
static sensor_status_t sensor_check(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta);
static sensor_status_t sensor_check_control_enabled_sensor(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta);
static sensor_status_t sensor_check_control_disabled_sensor(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta);

/* History Functions */
static float get_median(sensor_t *sensor);
static void add_sensor_value_to_history(sensor_t *sensor, float sensor_value);


/**
 * @brief Initializes both sensor states and starts them in the enabled state.
 * @param initial_altitude Altitude the barometer history is filled with, meters.
 * @param initial_vertical_accel Vertical acceleration the IMU history is filled with, m/s^2.
 */
void SensorCheck_Init(float initial_altitude, float initial_vertical_accel)
{
    sensor_check_init_sensor(&baro, initial_altitude, ALTITUDE_DELTA_CONFIRM_THRESHOLD);
    sensor_check_init_sensor(&imu, initial_vertical_accel, VERTICAL_ACCEL_DELTA_CONFIRM_THRESHOLD);
}

/**
 * @brief Checks one barometric altitude sample.
 * @param altitude Barometric altitude relative to the base altitude, meters.
 * @retval sensor_status_t.
 */
sensor_status_t SensorCheck_Altitude(float altitude)
{
        return sensor_check(&baro, altitude, MIN_ALTITUDE, MAX_ALTITUDE, MAX_ALTITUDE_DIFFERENCE);
}

/**
 * @brief Checks one vertical acceleration sample.
 * @param vertical_accel Vertical acceleration, m/s^2.
 * @retval sensor_status_t.
 * @note The allowed median difference is selected from the current flight phase, the wide
 *       boost limit is used in PHASE_IDLE and PHASE_BOOST and the tight one after burnout.
 */
sensor_status_t SensorCheck_VerticalAccel(float vertical_accel)
{
        flight_phase_t phase = flight_algorithm_get_flight_phase();
        float max_delta = (phase == PHASE_IDLE || phase == PHASE_BOOST) ? MAX_VERTICAL_ACCEL_DIFFERENCE_BOOST_PHASE : MAX_VERTICAL_ACCEL_DIFFERENCE;
        return sensor_check(&imu, vertical_accel, MIN_VERTICAL_ACCEL, MAX_VERTICAL_ACCEL, max_delta);
}


/**
 * @brief Tells whether the barometer is currently enabled.
 * @retval 1 while the barometer is trusted, 0 while it is disabled.
 */
uint8_t sensor_check_is_baro_enabled(void)
{
    return baro.is_enabled;
}


/**
 * @brief Clears the state of one sensor and fills its history with the given value.
 * @param sensor Pointer to the sensor state.
 * @param initial_sensor_value Value written into every history element.
 * @param delta_confirm_threshold Median violations needed before they are counted as an error.
 * @note The whole history is filled instead of being left at zero, otherwise the first samples
 *       would be compared against a median that does not belong to the flight.
 */
static void sensor_check_init_sensor(sensor_t *sensor, float initial_sensor_value, uint8_t delta_confirm_threshold)
{
    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        sensor->history[i] = initial_sensor_value;
    }

    sensor->history_index = 0;
    sensor->delta_violation_counter = 0;
    sensor->delta_confirm_threshold = delta_confirm_threshold;

    sensor->previous_value = initial_sensor_value;
    sensor->stuck_counter = 0;
    sensor->error_counter = 0;
    sensor->recover_counter = 0;
    sensor->is_enabled = 1;
}

/**
 * @brief Applies the pending enable / disable decision and runs the checks of one sample.
 * @param sensor Pointer to the sensor state.
 * @param sensor_value Sample to be checked.
 * @param min_value Lower physical limit of the measurement.
 * @param max_value Upper physical limit of the measurement.
 * @param max_delta Largest accepted difference between the sample and the median.
 * @retval sensor_status_t.
 * @note The counters are evaluated at the beginning of the call, so the sample that filled
 *       a counter is still handled with the previous enabled / disabled state and the state
 *       change is seen by the next call.
 * @note A non finite sample (NaN or Inf) is rejected here, so the checks below can assume
 *       that the comparisons are meaningful.
 */
static sensor_status_t sensor_check(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta)
{
    if (sensor->error_counter >= ERROR_COUNTER_THRESHOLD)
    {
        sensor->error_counter = 0;
        sensor->is_enabled = 0;
    }

    if (sensor->recover_counter >= RECOVER_COUNTER_THRESHOLD)
    {
        sensor->recover_counter = 0;
        sensor->is_enabled = 1;
    }

    if (isfinite(sensor_value))
    {
        if (sensor->is_enabled)
        {
            return sensor_check_control_enabled_sensor(sensor, sensor_value, min_value, max_value, max_delta);
        }
        else
        {
            return sensor_check_control_disabled_sensor(sensor, sensor_value, min_value, max_value, max_delta);
        }
    }
    else
    {
        if (sensor->is_enabled)
        {
            sensor->error_counter++;
        }
        else
        {
            sensor->recover_counter = 0;
        }
        return SENSOR_ERROR;
    }
}

/**
 * @brief Runs the range, stuck and median checks of a sample of an enabled sensor.
 * @param sensor Pointer to the sensor state.
 * @param sensor_value Sample to be checked.
 * @param min_value Lower physical limit of the measurement.
 * @param max_value Upper physical limit of the measurement.
 * @param max_delta Largest accepted difference between the sample and the median.
 * @retval sensor_status_t.
 * @note Every failed check increases the error counter, and only a sample that passes all of
 *       them decreases it, so a single fault never disables the sensor by itself.
 * @note The stuck check compares the samples exactly, a repeating value is what a frozen
 *       sensor produces. The counter is saturated at its threshold, so from that point on
 *       every repeated sample keeps increasing the error counter.
 * @note A sample rejected by the range or the stuck check does not enter the history and does
 *       not update the previous value, the faulty value is kept out of the median that way.
 * @note While the median violations are still below the confirm threshold the status stays
 *       SENSOR_OK, that tolerance is what lets a fast but real change pass.
 */
static sensor_status_t sensor_check_control_enabled_sensor(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta)
{
    if (sensor_value < min_value || sensor_value > max_value)
    {
        sensor->error_counter++;
        return SENSOR_ERROR;
    }

    if (sensor->previous_value == sensor_value)
    {
        if (sensor->stuck_counter < STUCK_COUNTER_THRESHOLD)
        {
            sensor->stuck_counter++;
        }
        if (sensor->stuck_counter >= STUCK_COUNTER_THRESHOLD)
        {
            sensor->error_counter++;
        }
        return SENSOR_ERROR;
    }
    else
    {
        sensor->stuck_counter = 0;
    }
    sensor->previous_value = sensor_value;

    add_sensor_value_to_history(sensor, sensor_value);
    float median = get_median(sensor);
    if (fabsf(sensor_value - median) > max_delta)
    {
        if (sensor->delta_violation_counter < sensor->delta_confirm_threshold)
        {
            sensor->delta_violation_counter++;
        }
        if (sensor->delta_violation_counter >= sensor->delta_confirm_threshold)
        {
            sensor->error_counter++;
            return SENSOR_ERROR;
        }
        else
        {
            return SENSOR_OK;
        }
    }
    else
    {
        sensor->delta_violation_counter = 0;
    }

    if (sensor->error_counter > 0)
    {
        sensor->error_counter--;
    }
    return SENSOR_OK;
}

/**
 * @brief Runs the range, stuck and median checks of a sample of a disabled sensor and counts its healthy samples.
 * @param sensor Pointer to the sensor state.
 * @param sensor_value Sample to be checked.
 * @param min_value Lower physical limit of the measurement.
 * @param max_value Upper physical limit of the measurement.
 * @param max_delta Largest accepted difference between the sample and the median.
 * @retval Always SENSOR_ERROR, a disabled sensor is not used even when its sample is healthy.
 * @note The checks are the same as the enabled ones, only the result is different. A healthy
 *       sample increases the recover counter and any failed check resets it back to zero, so
 *       the sensor is enabled again only after an uninterrupted healthy run.
 */
static sensor_status_t sensor_check_control_disabled_sensor(sensor_t *sensor, float sensor_value, float min_value, float max_value, float max_delta)
{
    if (sensor_value < min_value || sensor_value > max_value)
    {
        sensor->recover_counter = 0;
        return SENSOR_ERROR;
    }

    if (sensor->previous_value == sensor_value)
    {
        if (sensor->stuck_counter < STUCK_COUNTER_THRESHOLD)
        {
            sensor->stuck_counter++;
        }
        if (sensor->stuck_counter >= STUCK_COUNTER_THRESHOLD)
        {
            sensor->recover_counter = 0;
        }
        return SENSOR_ERROR;
    }
    else
    {
        sensor->stuck_counter = 0;
    }
    sensor->previous_value = sensor_value;

    add_sensor_value_to_history(sensor, sensor_value);
    float median = get_median(sensor);
    if (fabsf(sensor_value - median) > max_delta)
    {
        if (sensor->delta_violation_counter < sensor->delta_confirm_threshold)
        {
            sensor->delta_violation_counter++;
        }
        if (sensor->delta_violation_counter >= sensor->delta_confirm_threshold)
        {
            sensor->recover_counter = 0;
        }
        return SENSOR_ERROR;
    }
    else
    {
        sensor->delta_violation_counter = 0;
    }

    sensor->recover_counter++;
    return SENSOR_ERROR;
}

/**
 * @brief Returns the median of the samples kept in the history.
 * @param sensor Pointer to the sensor state.
 * @retval Median value.
 * @note The history is copied before it is sorted, the circular buffer itself keeps its
 *       chronological order. HISTORY_SIZE is odd, so the middle element is the median.
 * @note A bubble sort is used.
 */
static float get_median(sensor_t *sensor)
{
    float ordered_history[HISTORY_SIZE];
    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        ordered_history[i] = sensor->history[i];
    }

    for (uint8_t i = 0; i < HISTORY_SIZE - 1; i++)
    {
        for (uint8_t j = 0; j < HISTORY_SIZE - 1 - i; j++)
        {
            if (ordered_history[j] > ordered_history[j + 1])
            {
                float temp = ordered_history[j];
                ordered_history[j] = ordered_history[j + 1];
                ordered_history[j + 1] = temp;
            }
        }
    }

    return ordered_history[HISTORY_SIZE / 2];
}

/**
 * @brief Writes one sample into the circular history buffer.
 * @param sensor Pointer to the sensor state.
 * @param sensor_value Sample to be stored.
 * @note The sample is added before the median is taken, so the median window also contains
 *       the sample that is being checked.
 */
static void add_sensor_value_to_history(sensor_t *sensor, float sensor_value)
{
    if (sensor->history_index >= HISTORY_SIZE)
    {
        sensor->history_index = 0;
    }

    sensor->history[sensor->history_index] = sensor_value;
    sensor->history_index++;
}
