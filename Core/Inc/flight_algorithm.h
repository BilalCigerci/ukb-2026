/*
 * flight_algorithm.h
 *
 *  Created on: Mar 11, 2026
 *      Author: bilal
 */

/*
 * Flight state machine
 * -----------------------------------------------------------------------------------------
 * The module decides the flight phase and fires the recovery channels from the filtered
 * (Kalman / quaternion) flight data. FlightAlgorithm_Update() is called once per 100 ms pass
 * of the main loop (10 Hz) in NORMAL_MODE and in SUT_MODE, so every confirmation counter below
 * counts consecutive 100 ms samples.
 *
 *   PHASE_IDLE ---- vertical velocity above the launch threshold --------> PHASE_BOOST
 *   PHASE_BOOST --- vertical acceleration below the burnout threshold ---> PHASE_COAST
 *   PHASE_COAST --- recovery arming, drogue deployment, main deployment -> PHASE_MAIN_DESCENT
 *   PHASE_MAIN_DESCENT ---- landing detected ----------------------------> PHASE_LANDED
 *
 * The drogue parachute is deployed inside PHASE_COAST, the phase itself is not changed by it.
 * Apogee is confirmed by two independent criteria, either of them deploys the drogue:
 *   - angle criterion    : the tilt angle (theta) exceeds its threshold. It is used only while
 *                          theta_check still reports the angle criterion as enabled.
 *   - velocity criterion : the vertical velocity drops below its threshold.
 * The main parachute is deployed after the drogue, when the altitude drops below its threshold,
 * and entering PHASE_MAIN_DESCENT is what marks that deployment.
 *
 * All state (phase, flags, counters) is kept in module level static variables. The getters below
 * exist so that the data manager can copy that state into the backup SRAM packet, and the whole
 * state is restored with FlightAlgorithm_Recover() after an unexpected reset.
 */

#ifndef INC_FLIGHT_ALGORITHM_H_
#define INC_FLIGHT_ALGORITHM_H_


#include "flight_data.h"
#include "backup_domain.h"


/**
 * @brief Flight phases.
 */
typedef enum
{
    PHASE_IDLE          = 0x00,
    PHASE_BOOST         = 0x01,
    PHASE_COAST         = 0x02,
    PHASE_MAIN_DESCENT  = 0x03,
    PHASE_LANDED        = 0x04
}flight_phase_t;


/**
 * @brief Runs one step of the flight state machine and deactivates the expired recovery channels.
 * @param flight_data Pointer to the flight data structure, the filtered values are read from it
 *                    and the SUT state bits are written into it.
 * @note Must be called at a fixed 10 Hz rate, the confirmation thresholds are defined in samples.
 */
void FlightAlgorithm_Update(FlightData *flight_data);

/**
 * @brief Resets the state machine to PHASE_IDLE and clears all flags, counters and SUT state bits.
 * @param flight_data Pointer to the flight data structure.
 * @note Called at a normal power on start and on every accepted ground command packet, so a mode
 *       change always starts from a clean flight state.
 *       The maximum altitudes are cleared and the angle criterion is enabled again as well.
 */
void FlightAlgorithm_Reset(FlightData *flight_data);

/**
 * @brief Restores the flight phase, the flags, the counters and the SUT state bits from the backup SRAM.
 * @param flight_data Pointer to the flight data structure.
 * @param backup_domain Pointer to the backup domain handle.
 * @note Called instead of FlightAlgorithm_Reset() when the board comes up from an unexpected
 *       reset during flight.
 */
void FlightAlgorithm_Recover(FlightData *flight_data, Backup_Domain_HandleTypeDef *backup_domain);

/**
 * @brief Returns the current flight phase.
 * @retval flight_phase_t.
 */
flight_phase_t flight_algorithm_get_flight_phase(void);

/**
 * @brief Forces the flight phase.
 * @param phase Phase to be set.
 */
void flight_algorithm_set_flight_phase(flight_phase_t phase);

/* --- State flag getters, used when the backup SRAM packet is built --- */
uint8_t flight_algorithm_get_is_recovery_armed_flag(void);
uint8_t flight_algorithm_get_is_apogee_confirmed_with_angle_flag(void);
uint8_t flight_algorithm_get_is_apogee_confirmed_with_velocity_flag(void);
uint8_t flight_algorithm_get_is_drogue_deployed_flag(void);

/* --- Confirmation counter getters, used when the backup SRAM packet is built --- */
uint8_t flight_algorithm_get_launch_counter(void);
uint8_t flight_algorithm_get_burnout_counter(void);
uint8_t flight_algorithm_get_recovery_arming_counter(void);
uint8_t flight_algorithm_get_drogue_counter_angle(void);
uint8_t flight_algorithm_get_drogue_counter_velocity(void);
uint8_t flight_algorithm_get_main_counter(void);
uint8_t flight_algorithm_get_landing_counter(void);


#endif /* INC_FLIGHT_ALGORITHM_H_ */
