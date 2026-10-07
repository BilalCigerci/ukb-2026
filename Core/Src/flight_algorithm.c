/*
 * flight_algorithm.c
 *
 *  Created on: Mar 11, 2026
 *      Author: bilal
 */

#include "flight_algorithm.h"
#include <stdint.h>
#include "theta_check.h"


/* --- Detection thresholds, all of them are compared against the filtered (Kalman / quaternion) values --- */
#define LAUNCH_VELOCITY_THRESHOLD                           35.0f       // meter per second
#define BURNOUT_ACCEL_THRESHOLD                             0.0f        // meters per second squared
#define RECOVERY_ARMING_ALTITUDE_THRESHOLD                  2200.0f     // meter
#define DROGUE_ANGLE_THRESHOLD                              70.0f       // degree
#define DROGUE_VELOCITY_THRESHOLD                           2.0f        // meter per second
#define MAIN_ALTITUDE_THRESHOLD                             500.0f      // meter
#define LANDING_DETECTION_ALTITUDE_THRESHOLD                30.0f       // meter
#define MIN_LANDING_DETECTION_VELOCITY_THRESHOLD            -0.1f       // meter per second
#define MAX_LANDING_DETECTION_VELOCITY_THRESHOLD            0.1f        // meter per second

/* --- Confirmation thresholds, in consecutive 100 ms samples (FlightAlgorithm_Update() runs at 10 Hz) --- */
#define LAUNCH_CONFIRM_THRESHOLD                            4
#define BURNOUT_CONFIRM_THRESHOLD                           4
#define RECOVERY_ARMING_CONFIRM_THRESHOLD                   5
#define DROGUE_ANGLE_CONFIRM_THRESHOLD                      3
#define DROGUE_VELOCITY_CONFIRM_THRESHOLD                   5
#define MAIN_CONFIRM_THRESHOLD                              3
#define LANDING_CONFIRM_THRESHOLD                           3

// How long the recovery outputs stay driven high after a deployment
#define PARACHUTE_CHANNELS_ACTIVE_TIME                      1200         // milisecond


/* --- Flight state, kept as module level statics and mirrored into the backup SRAM --- */
static flight_phase_t flight_phase = PHASE_IDLE;

static uint8_t is_recovery_armed = 0;                   // Set when the arming altitude is passed, all recovery decisions are gated by it
static uint8_t is_apogee_confirmed_with_angle = 0;      // Apogee confirmed by the tilt angle criterion
static uint8_t is_apogee_confirmed_with_velocity = 0;   // Apogee confirmed by the vertical velocity criterion
static uint8_t is_drogue_deployed = 0;                  // Set by the criterion which fires first, keeps the drogue from being fired twice

/* Consecutive sample counters, each of them is cleared as soon as its condition is not met */
static uint8_t launch_counter = 0;
static uint8_t burnout_counter = 0;
static uint8_t recovery_arming_counter = 0;
static uint8_t drogue_counter_angle = 0;
static uint8_t drogue_counter_velocity = 0;
static uint8_t main_counter = 0;
static uint8_t landing_counter = 0;

/* Recovery output state, used to pull the channels back low after PARACHUTE_CHANNELS_ACTIVE_TIME */
static uint8_t is_drogue_parachute_channel_active = 0;
static uint8_t is_main_parachute_channel_active = 0;

static uint32_t drogue_parachute_activation_time;
static uint32_t main_parachute_activation_time;


/* --- Private Functions Declaration --- */
/* Phase Handlers */
static void phase_idle_handler(FlightData *flight_data);
static void phase_boost_handler(FlightData *flight_data);
static void phase_coast_handler(FlightData *flight_data);
static void phase_main_descent_handler(FlightData *flight_data);

/* Recovery Channel Functions */
static void deploy_drogue_parachute(void);
static void deploy_main_parachute(void);
static void deactivate_parachute_channels(void);


/**
 * @brief Runs one step of the flight state machine and deactivates the expired recovery channels.
 * @param flight_data Pointer to the flight data structure.
 * @note Must be called at a fixed 10 Hz rate. The theta check is fed only while the rocket is in
 *       PHASE_BOOST or PHASE_COAST and the recovery is not armed yet, so the angle criterion can
 *       be disabled before it is ever used for a deployment decision.
 */
void FlightAlgorithm_Update(FlightData *flight_data)
{
    if ((flight_phase == PHASE_BOOST || flight_phase == PHASE_COAST) && is_recovery_armed == 0)
    {
        ThetaCheck_UpdatePreArm(flight_data->filtered.theta);
    }

    switch (flight_phase)
    {
        case PHASE_IDLE: phase_idle_handler(flight_data); break;
        case PHASE_BOOST: phase_boost_handler(flight_data); break;
        case PHASE_COAST: phase_coast_handler(flight_data); break;
        case PHASE_MAIN_DESCENT: phase_main_descent_handler(flight_data); break;
        case PHASE_LANDED: break;
    }
	deactivate_parachute_channels();
}

/**
 * @brief Resets the state machine to PHASE_IDLE and clears all flags, counters and SUT state bits.
 * @param flight_data Pointer to the flight data structure.
 * @note Called at a normal power on start and on every accepted ground command packet, so a mode
 *       change always starts from a clean flight state.
 *       The recorded maximum altitudes are cleared and the angle criterion is enabled again with
 *       ThetaCheck_Reset(), while the recovery channels which are still driven high are left to
 *       deactivate_parachute_channels().
 */
void FlightAlgorithm_Reset(FlightData *flight_data)
{
	flight_phase = PHASE_IDLE;

	is_recovery_armed = 0;
	is_apogee_confirmed_with_angle = 0;
	is_apogee_confirmed_with_velocity = 0;
	is_drogue_deployed = 0;

	launch_counter = 0;
	burnout_counter = 0;
	recovery_arming_counter = 0;
	drogue_counter_angle = 0;
	drogue_counter_velocity = 0;
	main_counter = 0;
	landing_counter = 0;

	flight_data->sut_state_packet[0] = 0;
	flight_data->sut_state_packet[1] = 0;

    flight_data->filtered.max_altitude = 0.0f;
    flight_data->raw.max_altitude = 0.0f;

    ThetaCheck_Reset();
}

/**
 * @brief Restores the flight phase, the flags, the counters and the SUT state bits from the backup SRAM.
 * @param flight_data Pointer to the flight data structure.
 * @param backup_domain Pointer to the backup domain handle.
 * @note Called instead of FlightAlgorithm_Reset() when the board comes up from an unexpected reset
 *       during flight. The 14 bytes read here are the ones written by DataManager_UpdateBKPSRAMPacket()
 *       at BACKUP_DOMAIN_ALGORITHM_DATA.
 */
void FlightAlgorithm_Recover(FlightData *flight_data, Backup_Domain_HandleTypeDef *backup_domain)
{
	uint8_t backup_data[14] = { 0 };
	BackupDomain_ReadSRAM(backup_domain, backup_data, BACKUP_DOMAIN_ALGORITHM_DATA, 14);

	flight_phase = backup_data[0];

    ThetaCheck_SetState(((backup_data[1] & 0x02) >> 1));

	is_recovery_armed = (backup_data[1] & 0x01);
	is_apogee_confirmed_with_angle = backup_data[2];
	is_apogee_confirmed_with_velocity = backup_data[3];
	is_drogue_deployed = backup_data[4];

	launch_counter = backup_data[5];
	burnout_counter = backup_data[6];
	recovery_arming_counter = backup_data[7];
	drogue_counter_angle = backup_data[8];
	drogue_counter_velocity = backup_data[9];
	main_counter = backup_data[10];
	landing_counter = backup_data[11];

	flight_data->sut_state_packet[0] = backup_data[12];
	flight_data->sut_state_packet[1] = backup_data[13];
}


/**
 * @brief Returns the current flight phase.
 * @retval flight_phase_t.
 */
flight_phase_t flight_algorithm_get_flight_phase(void)
{
	return flight_phase;
}

/**
 * @brief Forces the flight phase.
 * @param phase Phase to be set.
 */
void flight_algorithm_set_flight_phase(flight_phase_t phase)
{
	flight_phase = phase;
}


/* --- State flag getters, read by the data manager while the backup SRAM packet is built --- */

uint8_t flight_algorithm_get_is_recovery_armed_flag(void)
{
	return is_recovery_armed;
}

uint8_t flight_algorithm_get_is_apogee_confirmed_with_angle_flag(void)
{
	return is_apogee_confirmed_with_angle;
}

uint8_t flight_algorithm_get_is_apogee_confirmed_with_velocity_flag(void)
{
	return is_apogee_confirmed_with_velocity;
}

uint8_t flight_algorithm_get_is_drogue_deployed_flag(void)
{
	return is_drogue_deployed;
}


/* --- Confirmation counter getters, read by the data manager while the backup SRAM packet is built --- */

uint8_t flight_algorithm_get_launch_counter(void)
{
	return launch_counter;
}

uint8_t flight_algorithm_get_burnout_counter(void)
{
	return burnout_counter;
}

uint8_t flight_algorithm_get_recovery_arming_counter(void)
{
	return recovery_arming_counter;
}

uint8_t flight_algorithm_get_drogue_counter_angle(void)
{
	return drogue_counter_angle;
}

uint8_t flight_algorithm_get_drogue_counter_velocity(void)
{
	return drogue_counter_velocity;
}

uint8_t flight_algorithm_get_main_counter(void)
{
	return main_counter;
}

uint8_t flight_algorithm_get_landing_counter(void)
{
	return landing_counter;
}


/* --- Private Functions Definitions --- */

// Waits on the pad, launch is confirmed when the vertical velocity stays above the threshold
// for LAUNCH_CONFIRM_THRESHOLD consecutive samples. Sets SUT state bit 0.
static void phase_idle_handler(FlightData *flight_data)
{
	if (flight_data->filtered.vertical_velocity > LAUNCH_VELOCITY_THRESHOLD)
	{
		launch_counter++;

		if (launch_counter >= LAUNCH_CONFIRM_THRESHOLD)
		{
			flight_phase = PHASE_BOOST;
			flight_data->sut_state_packet[0] |= (uint8_t) 0x01;
		}
	}
	else
	{
		launch_counter = 0;
	}
}

// Motor burn, burnout is confirmed when the vertical acceleration stays below the threshold
// for BURNOUT_CONFIRM_THRESHOLD consecutive samples. Sets SUT state bit 1.
static void phase_boost_handler(FlightData *flight_data)
{
	if (flight_data->filtered.vertical_accel < BURNOUT_ACCEL_THRESHOLD)
	{
		burnout_counter++;

		if (burnout_counter >= BURNOUT_CONFIRM_THRESHOLD)
		{
			flight_phase = PHASE_COAST;
			flight_data->sut_state_packet[0] |= (uint8_t) 0x02;
		}
	}
	else
	{
		burnout_counter = 0;
	}
}

// Unpowered ascent and drogue descent, the whole recovery sequence runs here:
//   1) Recovery arming: the altitude must stay above RECOVERY_ARMING_ALTITUDE_THRESHOLD, it is a
//      one way flag and gates every deployment below. Sets SUT state bit 2.
//   2) Apogee by angle: only while the theta check still reports the angle criterion as enabled,
//      the tilt angle must stay above DROGUE_ANGLE_THRESHOLD. Sets SUT state bits 3 and 5.
//   3) Apogee by velocity: the vertical velocity must stay below DROGUE_VELOCITY_THRESHOLD.
//      Sets SUT state bits 4 and 5.
//   4) Main parachute: after the drogue is deployed, the altitude must stay below
//      MAIN_ALTITUDE_THRESHOLD. Sets SUT state bits 6 and 7 and moves on to PHASE_MAIN_DESCENT.
// Each apogee criterion is evaluated until it confirms once, after that its own condition is false
// and its counter stays cleared. is_drogue_deployed makes sure that only the criterion which
// confirms first fires the drogue.
static void phase_coast_handler(FlightData *flight_data)
{
	// One way arming, once armed the counter is not needed anymore and falls into the else branch
	if (is_recovery_armed == 0 && flight_data->filtered.altitude > RECOVERY_ARMING_ALTITUDE_THRESHOLD)
	{
		recovery_arming_counter++;

		if (recovery_arming_counter >= RECOVERY_ARMING_CONFIRM_THRESHOLD)
		{
			is_recovery_armed = 1;
			flight_data->sut_state_packet[0] |= (uint8_t) 0x04;
		}
	}
	else
	{
		recovery_arming_counter = 0;
	}

	if (is_recovery_armed)
	{
		if (ThetaCheck_IsAngleRecoveryEnabled() && is_apogee_confirmed_with_angle == 0 && flight_data->filtered.theta > DROGUE_ANGLE_THRESHOLD)
		{
			drogue_counter_angle++;

			if (drogue_counter_angle >= DROGUE_ANGLE_CONFIRM_THRESHOLD)
			{
				if (is_drogue_deployed == 0)
				{
					deploy_drogue_parachute();
					is_drogue_deployed = 1;
				}
				is_apogee_confirmed_with_angle = 1;
				flight_data->sut_state_packet[0] |= (uint8_t) 0x08;
                flight_data->sut_state_packet[0] |= (uint8_t) 0x20;
			}
		}
		else
		{
			drogue_counter_angle = 0;
		}

		if (is_apogee_confirmed_with_velocity == 0 && flight_data->filtered.vertical_velocity < DROGUE_VELOCITY_THRESHOLD)
		{
			drogue_counter_velocity++;

			if (drogue_counter_velocity >= DROGUE_VELOCITY_CONFIRM_THRESHOLD)
			{
				if (is_drogue_deployed == 0)
				{
					deploy_drogue_parachute();
					is_drogue_deployed = 1;
				}
				is_apogee_confirmed_with_velocity = 1;
				flight_data->sut_state_packet[0] |= (uint8_t) 0x10;
				flight_data->sut_state_packet[0] |= (uint8_t) 0x20;
			}
		}
		else
		{
			drogue_counter_velocity = 0;
		}

		if (is_drogue_deployed && flight_data->filtered.altitude < MAIN_ALTITUDE_THRESHOLD)
		{
			main_counter++;

			if (main_counter >= MAIN_CONFIRM_THRESHOLD)
			{
				deploy_main_parachute();
				flight_phase = PHASE_MAIN_DESCENT;
				flight_data->sut_state_packet[0] |= (uint8_t) 0x40;
				flight_data->sut_state_packet[0] |= (uint8_t) 0x80;
			}
		}
		else
		{
			main_counter = 0;
		}
	}
}

// Main parachute descent, landing is confirmed when the vertical velocity stays inside the small
// window around zero or the altitude stays below LANDING_DETECTION_ALTITUDE_THRESHOLD for
// LANDING_CONFIRM_THRESHOLD consecutive samples.
static void phase_main_descent_handler(FlightData *flight_data)
{
	if ((flight_data->filtered.vertical_velocity > MIN_LANDING_DETECTION_VELOCITY_THRESHOLD && flight_data->filtered.vertical_velocity < MAX_LANDING_DETECTION_VELOCITY_THRESHOLD) || flight_data->filtered.altitude < LANDING_DETECTION_ALTITUDE_THRESHOLD)
	{
		landing_counter++;
	}
	else
	{
		landing_counter = 0;
	}

	if (landing_counter >= LANDING_CONFIRM_THRESHOLD)
	{
		flight_phase = PHASE_LANDED;
	}
}


// Drives the first recovery channel and the buzzer high and starts the active time timeout
static void deploy_drogue_parachute(void)
{
    HAL_GPIO_WritePin(RECOVERY_2_GPIO_Port, RECOVERY_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	is_drogue_parachute_channel_active = 1;
	drogue_parachute_activation_time = HAL_GetTick();

}

// Drives the second recovery channel and the buzzer high and starts the active time timeout
static void deploy_main_parachute(void)
{
	HAL_GPIO_WritePin(RECOVERY_1_GPIO_Port, RECOVERY_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	is_main_parachute_channel_active = 1;
	main_parachute_activation_time = HAL_GetTick();
}

// Pulls the recovery channels back low PARACHUTE_CHANNELS_ACTIVE_TIME after their activation.
// Called at the end of every FlightAlgorithm_Update() pass, so the outputs are released in every
// phase, not only in the phase which fired them.
static void deactivate_parachute_channels(void)
{
	if (is_drogue_parachute_channel_active)
	{
		if (HAL_GetTick() - drogue_parachute_activation_time >= PARACHUTE_CHANNELS_ACTIVE_TIME)
		{
            HAL_GPIO_WritePin(RECOVERY_2_GPIO_Port, RECOVERY_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			is_drogue_parachute_channel_active = 0;
		}
	}

	if (is_main_parachute_channel_active)
	{
		if (HAL_GetTick() - main_parachute_activation_time >= PARACHUTE_CHANNELS_ACTIVE_TIME)
		{
			HAL_GPIO_WritePin(RECOVERY_1_GPIO_Port, RECOVERY_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			is_main_parachute_channel_active = 0;
		}
	}
}

