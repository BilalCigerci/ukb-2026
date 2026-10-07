/*
 * backup_domain.c
 *
 *  Created on: Jul 5, 2026
 *      Author: bilal
 * 
 *  References:
 *          https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 */

#include "backup_domain.h"


#define SRAM_MAX_W_R_SIZE 128		// Upper bound of the used backup SRAM area, reads/writes beyond it are rejected
#define DATE_TIME_START_ADDRESS 44	// Offset of the date/time field inside the flight packet
#define RESET_DETECTION_TIME 3		// Seconds, the packet is accepted as fresh if it is not older than this


/**
 * @brief Enables the backup domain, starts the RTC clock and the backup SRAM regulator.
 * @param backup_domain Pointer to the handle structure.
 * @param hrtc Pointer to the RTC handle used for the date/time functions.
 * @retval HAL_OK when the backup regulator is ready, HAL_TIMEOUT when it does not become ready in time, HAL_ERROR otherwise.
 * @note Must be called before any other function of this module.
 */
HAL_StatusTypeDef BackupDomain_Init(Backup_Domain_HandleTypeDef *backup_domain, RTC_HandleTypeDef *hrtc)
{
    backup_domain->hrtc = hrtc;
    backup_domain->sram_base_address = (volatile uint8_t*)BKPSRAM_BASE;

	// stm32f446 reference manual 5.1.2 Battery backup domain (also known as RTC domain)
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;

    // Access to the RTC and RTC backup registers
    RCC->BDCR |= RCC_BDCR_RTCSEL_0;
    RCC->BDCR |= RCC_BDCR_RTCEN;

    // Backup domain access
	RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
	PWR->CSR |= PWR_CSR_BRE;

	// Wait until Backup Regulator ready
	uint32_t start = HAL_GetTick();
	while ((PWR->CSR & PWR_CSR_BRR_Msk) == 0)
	{
		if (HAL_GetTick() - start >= 10)
		{
			// TIMEOUT: Backup Regulator not ready
            return HAL_TIMEOUT;
		}
	}

    if ((PWR->CSR & PWR_CSR_BRR_Msk) != 0)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
 * @brief Reads the current RTC date and time into the handle structure.
 * @param backup_domain Pointer to the handle structure.
 */
void BackupDomain_UpdateDateTime(Backup_Domain_HandleTypeDef *backup_domain)
{
    HAL_RTC_GetTime(backup_domain->hrtc, &backup_domain->time, FORMAT_BIN);
    HAL_RTC_GetDate(backup_domain->hrtc, &backup_domain->date, FORMAT_BIN);
}

/**
 * @brief Reads a block of bytes from the backup SRAM.
 * @param backup_domain Pointer to the handle structure.
 * @param data Destination buffer.
 * @param offset Start offset in the backup SRAM (see the memory map in backup_domain.h).
 * @param size Number of bytes to read.
 * @note The request is ignored when it would go beyond SRAM_MAX_W_R_SIZE.
 */
void BackupDomain_ReadSRAM(Backup_Domain_HandleTypeDef *backup_domain, uint8_t *data,  uint16_t offset, uint16_t size)
{
    if ((size + offset) > SRAM_MAX_W_R_SIZE)
    {
        return;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        data[i] = backup_domain->sram_base_address[i + offset];
    }
}

/**
 * @brief Writes a block of bytes into the backup SRAM.
 * @param backup_domain Pointer to the handle structure.
 * @param data Source buffer.
 * @param offset Start offset in the backup SRAM (see the memory map in backup_domain.h).
 * @param size Number of bytes to write.
 * @note The request is ignored when it would go beyond SRAM_MAX_W_R_SIZE.
 */
void BackupDomain_WriteSRAM(Backup_Domain_HandleTypeDef *backup_domain, uint8_t *data, uint16_t offset, uint16_t size)
{
    if ((size + offset) > SRAM_MAX_W_R_SIZE)
    {
        return;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        backup_domain->sram_base_address[i + offset] = data[i];
    }
}

/**
 * @brief Decides whether the board came up from an unexpected reset during flight or from a normal power on.
 * @param backup_domain Pointer to the handle structure.
 * @retval 1 when the backup SRAM holds a recent and valid flight state, 0 otherwise.
 * @note The stored state is accepted only when the system mode or the flight phase is non zero,
 *       the stored date is today and the stored time is at most RESET_DETECTION_TIME seconds old.
 *       The time comparison is done inside a single day, a reset right at midnight is not detected.
 */
uint8_t backup_domain_is_reset_occured(Backup_Domain_HandleTypeDef *backup_domain)
{
    uint8_t is_reset_occured = 0;

    uint8_t backup_data_system_mode = 0;
    BackupDomain_ReadSRAM(backup_domain, &backup_data_system_mode, BACKUP_DOMAIN_SYSTEM_MODE, 1);

    uint8_t backup_data_flight_phase = 0;
    BackupDomain_ReadSRAM(backup_domain, &backup_data_flight_phase, BACKUP_DOMAIN_ALGORITHM_DATA, 1);

    if (backup_data_system_mode == 0 && backup_data_flight_phase == 0)
    {
        return is_reset_occured;
    }

    uint8_t r_data[6] = { 0 };
    BackupDomain_ReadSRAM(backup_domain, r_data, DATE_TIME_START_ADDRESS, 6);
    
    BackupDomain_UpdateDateTime(backup_domain);

    if (r_data[0] == backup_domain->date.Date && r_data[1] == backup_domain->date.Month && r_data[2] == backup_domain->date.Year)
    {
        uint32_t r_data_time, now;
        r_data_time = r_data[3] + r_data[4] * 60 + r_data[5] * 60 * 60;
        now = backup_domain->time.Seconds + backup_domain->time.Minutes * 60 + backup_domain->time.Hours * 60 * 60;

        if ((now - r_data_time) <= RESET_DETECTION_TIME)
        {
            is_reset_occured = 1;
        }
    }
    
    return is_reset_occured;
}

