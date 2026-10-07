/*
 * theta_check.c
 *
 *  Created on: Aug 5, 2026
 *      Author: bilal
 */

#include "theta_check.h"


// Largest tilt angle a rising rocket can still have before arming, a larger one is read as a
// broken angle estimate instead of a real attitude
#define MAX_THETA_PRE_ARM 70.0f // degree

// Number of samples over the limit in a row that disables the angle criterion
#define MAX_THETA_PRE_ARM_CONFIRM_THRESHOLD 3


static uint8_t is_theta_enabled = 1;            // 1 while the tilt angle is trusted, 0 after the check failed
static uint8_t max_theta_pre_arm_counter = 0;   // Samples over the limit seen in a row


/**
 * @brief Enables the angle criterion again and clears the confirmation counter.
 * @note Called from FlightAlgorithm_Reset(), so a normal power on start and every accepted
 *       ground command start with the criterion enabled. Must not be used to recover the
 *       state after an unexpected reset, ThetaCheck_SetState() is used for that.
 */
void ThetaCheck_Reset(void)
{
    is_theta_enabled = 1;
    max_theta_pre_arm_counter = 0;
}

/**
 * @brief Runs one step of the pre arming plausibility check of the tilt angle.
 * @param theta Estimated tilt angle, degrees.
 * @note Must be called at a fixed 10 Hz rate, and only while the rocket is in PHASE_BOOST or
 *       PHASE_COAST and the recovery is not armed yet. The counter is cleared by the first
 *       sample below the limit, so only consecutive violations disable the criterion. Once
 *       the criterion is disabled the function has no effect anymore.
 */
void ThetaCheck_UpdatePreArm(const float theta)
{
    if (is_theta_enabled && theta >= MAX_THETA_PRE_ARM)
    {
        max_theta_pre_arm_counter++;

        if (max_theta_pre_arm_counter >= MAX_THETA_PRE_ARM_CONFIRM_THRESHOLD)
        {
            is_theta_enabled = 0;
        }
    }
    else
    {
        max_theta_pre_arm_counter = 0;
    }
}

/**
 * @brief Writes the state of the angle criterion directly.
 * @param is_enabled 1 keeps the criterion enabled, 0 disables it.
 * @note Called from FlightAlgorithm_Recover() with the bit read back from the backup SRAM, so
 *       a criterion that was already disabled before an unexpected reset stays disabled. The
 *       confirmation counter is not touched here, it is only cleared by ThetaCheck_Reset().
 */
void ThetaCheck_SetState(uint8_t is_enabled)
{
    is_theta_enabled = is_enabled;
}

/**
 * @brief Reports whether the tilt angle may still be used for the recovery decisions.
 * @retval 1 while the angle criterion is enabled, 0 after it has been disabled.
 */
uint8_t ThetaCheck_IsAngleRecoveryEnabled(void)
{
    return is_theta_enabled;
}
