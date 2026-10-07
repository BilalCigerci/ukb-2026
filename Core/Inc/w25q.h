/*
 * w25q.h
 *
 *  Created on: Jun 7, 2026
 *      Author: bilal
 * 
 *  References:
 *          https://docs.rs-online.com/dea4/0900766b81622f8e.pdf
 */

/*
 *  Onboard flight data logger built on a W25Q NOR flash connected over SPI. It stores the flash
 *  log packets produced by the data manager during the flight, and it also serves the flash
 *  service commands of the ground station (dump, erase, usage).
 *
 *  Memory geometry
 *  -----------------------------------------------------------------------------------------
 *      1 page   -> 256 bytes         (BYTE_IN_PAGE)
 *      1 sector -> 16 pages          (PAGE_IN_SECTOR)
 *      1 block  -> 16 sectors        (SECTOR_IN_BLOCK)
 *      1 chip   -> 32 blocks        (BLOCK_IN_CHIP)
 *
 *  The write position is kept as the (block, sector, page) triplet inside the handle, it is
 *  never a flat address. A flat byte address is built from it only while talking to the chip.
 *
 *  Logging
 *  -----------------------------------------------------------------------------------------
 *  Records are W25Q_PACKET_SIZE (64) bytes long, so four of them fill one page. W25Q_LogData()
 *  collects the records in the 256 byte page buffer of the handle and programs a whole page at
 *  once, in DMA mode, when the buffer is full. Pages are written in ascending order and are
 *  never rewritten, because a NOR flash can only clear bits, so the chip has to be erased
 *  before it is filled again.
 *
 *  Every logging session starts with a starter packet, W25Q_PACKET_SIZE bytes of
 *  STARTER_PACKET_VALUE (0xAA), which marks in the dump where the board was powered up or
 *  where the flash was erased.
 *
 *  Startup
 *  -----------------------------------------------------------------------------------------
 *  On a normal power up W25Q_Init() resets the chip and W25Q_Start() searches for the first
 *  free page, first over the blocks, then over the sectors of that block, then over its pages.
 *  After a reset in flight W25Q_Recover() is used instead, it takes the last write position
 *  from the backup SRAM (BACKUP_DOMAIN_W25Q_FLASH) and continues from there without scanning
 *  the chip. The position itself is written into the backup SRAM by the data manager.
 *
 *  Service commands
 *  -----------------------------------------------------------------------------------------
 *  The UART handler only stores the requested command in the handle and sets the is_cmd_taken
 *  flag, the work is done later by W25Q_CommandHandler() from the main loop. The dump and the
 *  usage answers are sent over the UART of the handle, as frames closed by a one byte checksum
 *  (the 8 bit sum of the preceding bytes, the header included) and the 0x0D 0x0A footer:
 *
 *      68 bytes   flash dump frame   board -> ground station   (header 0xC4, 64 byte record)
 *       7 bytes   usage frame        board -> ground station   (header 0xC6)
 *
 *  The dump ends with one frame whose record is filled with 0xFF, it is the end marker of the
 *  transfer.
 *
 *  @note Page programming runs in DMA mode, so only one transfer can be in flight at a time.
 *        While it is running the chip select stays low and is_dma_transfer_started is set, the
 *        read and write functions answer W25Q_BUSY, and the line is released only in
 *        w25q_dma_cplt_callback(), which has to be called from the SPI transmit complete
 *        callback.
 */

#ifndef INC_W25Q_H_
#define INC_W25Q_H_


#include "main.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>
#include "backup_domain.h"


// Size of one log record, four records fill one page
#define W25Q_PACKET_SIZE        64

// Total capacity of the chip
#define TOTAL_PAGE              8192
#define TOTAL_SECTOR            512
#define TOTAL_BLOCK             32

// Memory geometry, the addresses are calculated from these
#define BYTE_IN_PAGE            ((uint32_t)256)
#define PAGE_IN_SECTOR          ((uint32_t)16)
#define SECTOR_IN_BLOCK         ((uint32_t)16)
#define BLOCK_IN_CHIP           ((uint32_t)32)


// Return value of the read and write functions
typedef enum
{
    W25Q_OK = 0,
    W25Q_ERROR = 1,
    W25Q_BUSY = 2
} w25q_status_t;

// Flash service commands, they are taken from the UART handler and executed by W25Q_CommandHandler()
typedef enum
{
    READ_FLASH = 0,
    CHIP_ERASE = 1,
    GET_USAGE = 2
} w25q_cmd_t;

/**
 * @brief Hardware interface of the chip, the SPI bus and the chip select pin.
 */
typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} w25q_configurations_t;

/**
 * @brief State flags of the logger.
 */
typedef struct
{
    uint8_t is_empty;                   // There is still free space left, it is cleared when the write position runs past the last block and logging stops.
    uint8_t is_dma_transfer_started;    // A page program is still running on the SPI bus
    uint8_t is_write_enabled;           // The write enable latch of the chip is set
    uint8_t is_buffer_full;             // The page buffer holds four records and waits to be written
    uint8_t is_cmd_taken;               // A service command is waiting in 'cmd'
} w25q_flags_t;

/**
 * @brief Main W25Q Handle structure.
 * Combines the hardware interface, the state flags, the write position and the page buffer.
 */
typedef struct
{
    w25q_configurations_t configurations;
    volatile w25q_flags_t flags;
    uint8_t current_page;                // Next page to be written, inside the current sector
    uint8_t current_sector;              // Current sector, inside the current block
    uint8_t current_block;               // Current block
    uint8_t buffer_offset;               // Number of records already in the buffer, 0 to 3
    uint8_t buffer[256];                 // One page worth of records, written in a single transfer
    w25q_cmd_t cmd;                      // Service command waiting to be executed
    UART_HandleTypeDef *huart;           // UART the dump and the usage answers are sent over
} W25Q_HandleTypeDef;


/**
 * @brief Initializes the handle, resets the chip and finds the first free page.
 * @param w25q Pointer to the handle structure.
 * @param spi Pointer to the SPI handle the chip is connected to.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin Chip select GPIO pin.
 * @param huart Pointer to the UART handle the dump and the usage answers are sent over.
 * @note Used on a normal power up, it calls W25Q_Start() at the end.
 */
void W25Q_Init(W25Q_HandleTypeDef *w25q, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, UART_HandleTypeDef *huart);

/**
 * @brief Initializes the handle and continues logging from the position kept in the backup SRAM.
 * @param w25q Pointer to the handle structure.
 * @param spi Pointer to the SPI handle the chip is connected to.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin Chip select GPIO pin.
 * @param huart Pointer to the UART handle the dump and the usage answers are sent over.
 * @param backup_domain Pointer to the backup domain handle the position is read from.
 * @note Used after a reset in flight. If the stored position is out of range the chip is
 *       searched with W25Q_Start() instead, and if the stored page turns out to be already
 *       written the position is advanced to the next one. No starter packet is written here,
 *       so the dump keeps running as one session over the reset.
 */
void W25Q_Recover(W25Q_HandleTypeDef *w25q, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, UART_HandleTypeDef *huart, Backup_Domain_HandleTypeDef *backup_domain);

/**
 * @brief Reads data from the given position of the chip.
 * @param w25q Pointer to the handle structure.
 * @param block Block index.
 * @param sector Sector index inside the block.
 * @param page Page index inside the sector.
 * @param offset Byte offset inside the page.
 * @param size Number of bytes to read, at most 256.
 * @param rxData Destination buffer, it must be at least 'size' bytes long.
 * @retval w25q_status_t. W25Q_BUSY while a page program is running, W25Q_ERROR if the size is too big.
 * @note Blocking transfer.
 */
w25q_status_t W25Q_Read(W25Q_HandleTypeDef *w25q, uint32_t block, uint32_t sector, uint32_t page, uint32_t offset, uint32_t size, uint8_t *rxData);

/**
 * @brief Reads data from a flat byte address, used while dumping the flash sequentially.
 * @param w25q Pointer to the handle structure.
 * @param address 24 bit byte address.
 * @param size Number of bytes to read, at most 256.
 * @param rxData Destination buffer, it must be at least 'size' bytes long.
 * @retval w25q_status_t. W25Q_BUSY while a page program is running, W25Q_ERROR if the size is too big.
 * @note Blocking transfer.
 */
w25q_status_t W25Q_ReadWithAddress(W25Q_HandleTypeDef *w25q, uint32_t address, uint32_t size, uint8_t *rxData);

/**
 * @brief Programs the given page of the chip in DMA mode.
 * @param w25q Pointer to the handle structure.
 * @param block Block index.
 * @param sector Sector index inside the block.
 * @param page Page index inside the sector.
 * @param offset Byte offset inside the page.
 * @param size Number of bytes to write, size + offset must stay inside one page.
 * @param pData Source buffer.
 * @retval w25q_status_t. W25Q_BUSY if another transfer is still running, W25Q_ERROR if the
 *         data does not fit in the page or the DMA transfer could not be started.
 * @note The function only starts the transfer, the chip select is released later in
 *       w25q_dma_cplt_callback().
 */
w25q_status_t W25Q_Write(W25Q_HandleTypeDef *w25q, uint32_t block, uint32_t sector, uint32_t page, uint32_t offset, uint32_t size, uint8_t *pData);

/**
 * @brief Searches the first free page of the chip and opens a new logging session.
 * @param w25q Pointer to the handle structure.
 * @note The search goes block by block, then sector by sector, then page by page, and it only
 *       looks at the first page of each block and sector, because the pages are always written
 *       in ascending order. A starter packet is put into the page buffer at the end. If no free
 *       page is left the is_empty flag is cleared and logging stops.
 */
void W25Q_Start(W25Q_HandleTypeDef *w25q);

/**
 * @brief Adds one log record to the page buffer and writes the page when the buffer is full.
 * @param w25q Pointer to the handle structure.
 * @param data_packet Record to be logged, it must be W25Q_PACKET_SIZE (64) bytes long.
 * @note Called in every main loop pass. The record is dropped when the chip is full, and the
 *       write position is advanced only after a page program has been started successfully.
 */
void W25Q_LogData(W25Q_HandleTypeDef *w25q, uint8_t* data_packet);

/**
 * @brief Executes the flash service command waiting in the handle (read, erase, get usage).
 * @param w25q Pointer to the handle structure.
 * @note Called from the main loop. The commands are blocking and take a long time,
 *       so they are only used on the ground, while the board is not flying.
 */
void W25Q_CommandHandler(W25Q_HandleTypeDef *w25q);

/**
 * @brief Callback function to be called from the SPI transmit complete callback.
 * Closes the page program, it releases the chip select and clears the transfer flag.
 * @param w25q Pointer to the handle structure.
 */
void w25q_dma_cplt_callback(W25Q_HandleTypeDef *w25q);


#endif /* INC_W25Q_H_ */
