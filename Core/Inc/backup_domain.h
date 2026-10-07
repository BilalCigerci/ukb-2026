/*
 * backup_domain.h
 *
 *  Created on: Jul 5, 2026
 *      Author: bilal
 * 
 *  References:
 *          https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 */

/*
 * Backup SRAM memory map
 * -----------------------------------------------------------------------------------------
 * The first BKPSRAM_PACKET_SIZE (54) bytes hold the flight packet, which is rebuilt by
 * DataManager_UpdateBKPSRAMPacket() and written as one block in every main loop pass.
 * The macros below are the offsets of the fields inside that packet, plus the calibration
 * values which are written only once at startup and kept right after the packet.
 *
 *  Offset  Size  Content                                                 Macro
 *  ------  ----  ------------------------------------------------------  ---------------------------------
 *     0      1   Header (0xA5)                                           -
 *     1      1   System mode                                             BACKUP_DOMAIN_SYSTEM_MODE
 *     2     12   Kalman state vector (altitude, vertical velocity,       BACKUP_DOMAIN_KALMAN_STATE_VECTOR
 *                vertical acceleration)
 *    14     16   Quaternion (w, x, y, z)                                 BACKUP_DOMAIN_QUATERNION
 *    30     14   Flight algorithm data (flight phase, flags, counters,   BACKUP_DOMAIN_ALGORITHM_DATA
 *                SUT state bytes)
 *    44      6   Date and time (day, month, year, second, minute, hour)  -
 *    50      3   W25Q write position (block, sector, page)               BACKUP_DOMAIN_W25Q_FLASH
 *    53      1   Checksum of the first 53 bytes                          -
 *    54     12   Accelerometer offsets (3 x float, g)                    BACKUP_DOMAIN_ACC_OFFSET
 *    66     12   Gyroscope offsets (3 x float, deg/s)                    BACKUP_DOMAIN_GYRO_OFFSET
 *    78      4   Base altitude (float, meters)                           BACKUP_DOMAIN_BASE_ALTITUDE
 *
 * @note The float fields inside the packet (offsets 2..29) are stored MSB first by the data
 *       manager, while the offsets and the base altitude are copied as raw memory, so they
 *       keep the native little endian layout of the MCU.
 */

#ifndef INC_BACKUP_DOMAIN_H_
#define INC_BACKUP_DOMAIN_H_


#include "main.h"
#include "flight_data.h"


#define BACKUP_DOMAIN_ACC_OFFSET BKPSRAM_PACKET_SIZE
#define BACKUP_DOMAIN_GYRO_OFFSET (BKPSRAM_PACKET_SIZE + 12)
#define BACKUP_DOMAIN_BASE_ALTITUDE (BKPSRAM_PACKET_SIZE + 24)
#define BACKUP_DOMAIN_ALGORITHM_DATA 30
#define BACKUP_DOMAIN_SYSTEM_MODE 1
#define BACKUP_DOMAIN_KALMAN_STATE_VECTOR 2
#define BACKUP_DOMAIN_QUATERNION 14
#define BACKUP_DOMAIN_W25Q_FLASH 50


/**
 * @brief Main Backup Domain Handle structure.
 * Holds the RTC handle, the backup SRAM base address and the last read date/time.
 */
typedef struct
{
    RTC_HandleTypeDef *hrtc;
    volatile uint8_t *sram_base_address;    // Points to BKPSRAM_BASE, all offsets are relative to it
    RTC_TimeTypeDef time;                   // Filled by BackupDomain_UpdateDateTime()
    RTC_DateTypeDef date;                   // Filled by BackupDomain_UpdateDateTime()
} Backup_Domain_HandleTypeDef;


/**
 * @brief Enables the backup domain, starts the RTC clock and the backup SRAM regulator.
 * @param backup_domain Pointer to the handle structure.
 * @param hrtc Pointer to the RTC handle used for the date/time functions.
 * @retval HAL_OK when the backup regulator is ready, HAL_TIMEOUT when it does not become ready in time, HAL_ERROR otherwise.
 * @note Must be called before any other function of this module.
 */
HAL_StatusTypeDef BackupDomain_Init(Backup_Domain_HandleTypeDef *backup_domain, RTC_HandleTypeDef *hrtc);

/**
 * @brief Reads the current RTC date and time into the handle structure.
 * @param backup_domain Pointer to the handle structure.
 */
void BackupDomain_UpdateDateTime(Backup_Domain_HandleTypeDef *backup_domain);

/**
 * @brief Reads a block of bytes from the backup SRAM.
 * @param backup_domain Pointer to the handle structure.
 * @param data Destination buffer.
 * @param offset Start offset in the backup SRAM (see the memory map above).
 * @param size Number of bytes to read.
 */
void BackupDomain_ReadSRAM(Backup_Domain_HandleTypeDef *backup_domain, uint8_t *data, uint16_t offset, uint16_t size);

/**
 * @brief Writes a block of bytes into the backup SRAM.
 * @param backup_domain Pointer to the handle structure.
 * @param data Source buffer.
 * @param offset Start offset in the backup SRAM (see the memory map above).
 * @param size Number of bytes to write.
 */
void BackupDomain_WriteSRAM(Backup_Domain_HandleTypeDef *backup_domain, uint8_t *data, uint16_t offset, uint16_t size);

/**
 * @brief Decides whether the board came up from an unexpected reset during flight or from a normal power on.
 * @param backup_domain Pointer to the handle structure.
 * @retval 1 when the backup SRAM holds a recent and valid flight state, 0 otherwise.
 */
uint8_t backup_domain_is_reset_occured(Backup_Domain_HandleTypeDef *backup_domain);


#endif /* INC_BACKUP_DOMAIN_H_ */
