/*
 * theta_check.h
 *
 *  Created on: Aug 5, 2026
 *      Author: bilal
 */

/*
 * Plausibility check of the tilt angle
 * -----------------------------------------------------------------------------------------
 * The module decides whether the estimated tilt angle (theta) may be trusted by the rest of
 * the flight software. It holds a single flag, the angle criterion, which starts enabled and
 * can only be turned off, never on again by the check itself.
 *
 * The check runs before arming only. While the rocket is ascending and the recovery is not
 * armed yet, theta is expected to stay small. An angle at or above MAX_THETA_PRE_ARM in that
 * part of the flight cannot be a real attitude of a rising rocket, so it is read as a broken
 * angle estimate. The samples have to confirm the fault: a counter is increased by every
 * sample over the limit and cleared by the first sample below it, so only
 * MAX_THETA_PRE_ARM_CONFIRM_THRESHOLD violations in a row disable the criterion.
 *
 * Once the criterion is disabled it stays disabled for the whole flight. Two consumers read
 * the flag:
 *   - The flight algorithm drops the angle based apogee criterion and lets the velocity based
 *     criterion fire the drogue alone.
 *   - The sensor fusion stops feeding the estimated theta into the vertical acceleration
 *     measurement and uses a fixed tilt assumption instead.
 *
 * The flag lives outside of the flight algorithm state, so it is kept in the backup SRAM and
 * in the flash record by the data manager and is written back with ThetaCheck_SetState()
 * after an unexpected reset. The check itself is stateless across a power cycle, without that
 * restore a reset would silently enable a criterion that was already found to be faulty.
 */

#ifndef INC_THETA_CHECK_H_
#define INC_THETA_CHECK_H_


#include "stdint.h"


/**
 * @brief Enables the angle criterion again and clears the confirmation counter.
 * @note Called from FlightAlgorithm_Reset(), so a normal power on start and every accepted
 *       ground command start with the criterion enabled. Must not be used to recover the
 *       state after an unexpected reset, ThetaCheck_SetState() is used for that.
 */
void ThetaCheck_Reset(void);

/**
 * @brief Runs one step of the pre arming plausibility check of the tilt angle.
 * @param theta Estimated tilt angle, degrees.
 * @note Must be called at a fixed 10 Hz rate, and only while the rocket is in PHASE_BOOST or
 *       PHASE_COAST and the recovery is not armed yet. The counter is cleared by the first
 *       sample below the limit, so only consecutive violations disable the criterion. Once
 *       the criterion is disabled the function has no effect anymore.
 */
void ThetaCheck_UpdatePreArm(const float theta);

/**
 * @brief Writes the state of the angle criterion directly.
 * @param is_enabled 1 keeps the criterion enabled, 0 disables it.
 * @note Called from FlightAlgorithm_Recover() with the bit read back from the backup SRAM, so
 *       a criterion that was already disabled before an unexpected reset stays disabled. The
 *       confirmation counter is not touched here, it is only cleared by ThetaCheck_Reset().
 */
void ThetaCheck_SetState(uint8_t is_enabled);

/**
 * @brief Reports whether the tilt angle may still be used for the recovery decisions.
 * @retval 1 while the angle criterion is enabled, 0 after it has been disabled.
 */
uint8_t ThetaCheck_IsAngleRecoveryEnabled(void);


#endif /* INC_THETA_CHECK_H_ */
