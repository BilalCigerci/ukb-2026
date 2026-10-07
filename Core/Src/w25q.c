/*
 * w25q.c
 *
 *  Created on: Jun 7, 2026
 *      Author: bilal
 * 
 *  References:
 *          https://docs.rs-online.com/dea4/0900766b81622f8e.pdf
 */

#include "w25q.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>
#include <string.h>


// SPI transfer buffer size, one full page (256 bytes) plus the command byte and the three address bytes
#define BUFFER_SIZE             260
#define PACKET_IN_BUFFER        4       // 4 x 64 byte record fills one page

// Value the starter packet is filled with, it marks the beginning of a logging session
#define STARTER_PACKET_VALUE    0xAA

// W25Q instruction set
#define CMD_READ_DATA           0x03
#define CMD_WRITE_DATA          0x02
#define CMD_WRITE_ENABLE        0x06
#define CMD_WRITE_DISABLE       0x04
#define CMD_CHIP_ERASE          0x60
#define CMD_ENABLE_RESET        0x66
#define CMD_RESET_DEVICE        0x99


/* --- Private Functions Declaration --- */
/* Chip Select Functions */
static void cs_low(W25Q_HandleTypeDef *w25q);
static void cs_high(W25Q_HandleTypeDef *w25q);

/* Write Functions */
static void w25q_write_enable(W25Q_HandleTypeDef *w25q);
static void w25q_write_disable(W25Q_HandleTypeDef *w25q);
static void w25q_add_buffer(W25Q_HandleTypeDef *w25q, uint8_t* data_packet);

/* Page Check Function */
static uint8_t w25q_is_page_empty(uint8_t *page_data);

/* Frame Checksum Function */
static uint8_t calculate_checksum(uint8_t *data, uint16_t size);

/* Service Command Handlers */
static void w25q_read_flash_handler(W25Q_HandleTypeDef *w25q);
static void w25q_chip_erase_handler(W25Q_HandleTypeDef *w25q);
static void w25q_get_usage_handler(W25Q_HandleTypeDef *w25q);

/* Device Reset Function */
static void w25q_reset(W25Q_HandleTypeDef *w25q);


/**
 * @brief Initializes the handle, resets the chip and finds the first free page.
 * @param w25q Pointer to the handle structure.
 * @param spi Pointer to the SPI handle the chip is connected to.
 * @param cs_port Chip select GPIO port.
 * @param cs_pin Chip select GPIO pin.
 * @param huart Pointer to the UART handle the dump and the usage answers are sent over.
 * @note Used on a normal power up, it calls W25Q_Start() at the end.
 */
void W25Q_Init(W25Q_HandleTypeDef *w25q, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, UART_HandleTypeDef *huart)
{
    w25q->configurations.spi = spi;
    w25q->configurations.cs_port = cs_port;
    w25q->configurations.cs_pin = cs_pin;

    w25q->huart = huart;

    w25q->flags.is_cmd_taken = 0;
    
    w25q->flags.is_dma_transfer_started = 0;
    w25q->flags.is_write_enabled = 0;
    w25q->flags.is_buffer_full = 0;
    w25q->flags.is_empty = 1;
    
    w25q->buffer_offset = 0;

    w25q->current_block = 0;
    w25q->current_sector = 0;
    w25q->current_page = 0;

    w25q_reset(w25q);
    HAL_Delay(50);

    W25Q_Start(w25q);
}

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
void W25Q_Recover(W25Q_HandleTypeDef *w25q, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, UART_HandleTypeDef *huart, Backup_Domain_HandleTypeDef *backup_domain)
{
    w25q->configurations.spi = spi;
    w25q->configurations.cs_port = cs_port;
    w25q->configurations.cs_pin = cs_pin;

    w25q->huart = huart;

    w25q->flags.is_cmd_taken = 0;
    
    w25q->flags.is_dma_transfer_started = 0;
    w25q->flags.is_write_enabled = 0;
    w25q->flags.is_buffer_full = 0;
    w25q->flags.is_empty = 1;
    
    w25q->buffer_offset = 0;

    // Last write position kept by the data manager, block, sector and page in this order
    uint8_t data[3] = { 0 };
    BackupDomain_ReadSRAM(backup_domain, data, BACKUP_DOMAIN_W25Q_FLASH, 3);
    w25q->current_block = data[0];
    w25q->current_sector = data[1];
    w25q->current_page = data[2];

    // The chip was already full before the reset, nothing more can be logged
    if (w25q->current_block == BLOCK_IN_CHIP && w25q->current_sector == 0 && w25q->current_page == 0)
    {
        w25q->flags.is_empty = 0;
    }
    // The stored position is not usable, the free page is searched on the chip instead
    else if (w25q->current_block >= BLOCK_IN_CHIP || w25q->current_sector >= SECTOR_IN_BLOCK || w25q->current_page >= PAGE_IN_SECTOR)
    {
        HAL_Delay(5);
        w25q->current_block = 0;
        w25q->current_sector = 0;
        w25q->current_page = 0;
        W25Q_Start(w25q);
    }
    else
    {
        HAL_Delay(5);
        // The stored page may have been programmed just before the reset, in that case the
        // position is carried to the next page, otherwise it is kept as it is
        uint8_t page_data[BYTE_IN_PAGE] = { 0 };
        W25Q_Read(w25q, w25q->current_block, w25q->current_sector, w25q->current_page, 0, BYTE_IN_PAGE, page_data);
        if (w25q_is_page_empty(page_data) == 0)
        {
            w25q->current_page++;
            if (w25q->current_page >= PAGE_IN_SECTOR)
            {
                w25q->current_page = 0;
                w25q->current_sector++;
            }
            if (w25q->current_sector >= SECTOR_IN_BLOCK)
            {
                w25q->current_sector = 0;
                w25q->current_block++;
            }
            if (w25q->current_block >= BLOCK_IN_CHIP)
            {
                w25q->flags.is_empty = 0;
            }
        }
    }
}

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
w25q_status_t W25Q_Read(W25Q_HandleTypeDef *w25q, uint32_t block, uint32_t sector, uint32_t page, uint32_t offset, uint32_t size, uint8_t *rxData)
{
    if (size > BUFFER_SIZE - 4)
    {
        return W25Q_ERROR;
    }

    uint8_t rx_buffer[BUFFER_SIZE] = { 0 };
    uint8_t tx_buffer[BUFFER_SIZE] = { 0 };
    
    uint32_t address = block * SECTOR_IN_BLOCK * PAGE_IN_SECTOR * BYTE_IN_PAGE + sector * PAGE_IN_SECTOR * BYTE_IN_PAGE + page * BYTE_IN_PAGE + offset;

    tx_buffer[0] = CMD_READ_DATA;
    tx_buffer[1] = (uint8_t)(address >> 16);
    tx_buffer[2] = (uint8_t)(address >> 8);
    tx_buffer[3] = (uint8_t)address;

    if (w25q->flags.is_dma_transfer_started == 0)
    {
        cs_low(w25q);
        HAL_SPI_TransmitReceive(w25q->configurations.spi, tx_buffer, rx_buffer, size + 4, 10);
        cs_high(w25q);
        memcpy(rxData, &rx_buffer[4], size);
        return W25Q_OK;
    }
    else
    {
        return W25Q_BUSY;
    }
}

/**
 * @brief Reads data from a flat byte address, used while dumping the flash sequentially.
 * @param w25q Pointer to the handle structure.
 * @param address 24 bit byte address.
 * @param size Number of bytes to read, at most 256.
 * @param rxData Destination buffer, it must be at least 'size' bytes long.
 * @retval w25q_status_t. W25Q_BUSY while a page program is running, W25Q_ERROR if the size is too big.
 * @note Blocking transfer.
 */
w25q_status_t W25Q_ReadWithAddress(W25Q_HandleTypeDef *w25q, uint32_t address, uint32_t size, uint8_t *rxData)
{
    if (size > BUFFER_SIZE - 4)
    {
        return W25Q_ERROR;
    }

    uint8_t rx_buffer[BUFFER_SIZE] = { 0 };
    uint8_t tx_buffer[BUFFER_SIZE] = { 0 };

    tx_buffer[0] = CMD_READ_DATA;
    tx_buffer[1] = (uint8_t)(address >> 16);
    tx_buffer[2] = (uint8_t)(address >> 8);
    tx_buffer[3] = (uint8_t)address;

    if (w25q->flags.is_dma_transfer_started == 0)
    {
        cs_low(w25q);
        HAL_SPI_TransmitReceive(w25q->configurations.spi, tx_buffer, rx_buffer, size + 4, 10);
        cs_high(w25q);
        memcpy(rxData, &rx_buffer[4], size);
        return W25Q_OK;
    }
    else
    {
        return W25Q_BUSY;
    }
}

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
w25q_status_t W25Q_Write(W25Q_HandleTypeDef *w25q, uint32_t block, uint32_t sector, uint32_t page, uint32_t offset, uint32_t size, uint8_t *pData)
{
    if (size + offset > BYTE_IN_PAGE)
    {
        return W25Q_ERROR;
    }

    static uint8_t tx_buffer[BUFFER_SIZE] = { 0 };
    
    uint32_t address = block * SECTOR_IN_BLOCK * PAGE_IN_SECTOR * BYTE_IN_PAGE + sector * PAGE_IN_SECTOR * BYTE_IN_PAGE + page * BYTE_IN_PAGE + offset;

    tx_buffer[0] = CMD_WRITE_DATA;
    tx_buffer[1] = (uint8_t)(address >> 16);
    tx_buffer[2] = (uint8_t)(address >> 8);
    tx_buffer[3] = (uint8_t)address;

    memcpy(&tx_buffer[4], pData, size);

    if (w25q->flags.is_dma_transfer_started == 0)
    {
        w25q->flags.is_dma_transfer_started = 1;

        w25q_write_enable(w25q);

        cs_low(w25q);
        if (HAL_SPI_Transmit_DMA(w25q->configurations.spi, tx_buffer, size + 4) != HAL_OK)
        {
            w25q->flags.is_dma_transfer_started = 0;
            cs_high(w25q);
            w25q_write_disable(w25q);
            return W25Q_ERROR;
        }
        return W25Q_OK;
    }
    else
    {
        return W25Q_BUSY;
    }
}

/**
 * @brief Searches the first free page of the chip and opens a new logging session.
 * @param w25q Pointer to the handle structure.
 * @note The search goes block by block, then sector by sector, then page by page, and it only
 *       looks at the first page of each block and sector, because the pages are always written
 *       in ascending order. A starter packet is put into the page buffer at the end. If no free
 *       page is left the is_empty flag is cleared and logging stops.
 */
void W25Q_Start(W25Q_HandleTypeDef *w25q)
{
    uint8_t rx_data[256] = { 0 };

    uint8_t block_found = 0, sector_found = 0, page_found = 0;

    // Find the first empty block
    while (block_found == 0 && w25q->current_block < BLOCK_IN_CHIP)
    {
        W25Q_Read(w25q, w25q->current_block, w25q->current_sector, w25q->current_page, 0, 256, rx_data);
        w25q->current_block++;
        if (w25q_is_page_empty(rx_data))
        {
            block_found = 1;
            w25q->current_block = (w25q->current_block >= 2) ? w25q->current_block - 2 : 0;
        }
    }

    // The chip is completely full
    if (w25q->current_block == BLOCK_IN_CHIP)
    {
        w25q->flags.is_empty = 0;
        return;
    }

    // Find the first empty sector
    while (sector_found == 0 && w25q->current_sector < SECTOR_IN_BLOCK)
    {
        W25Q_Read(w25q, w25q->current_block, w25q->current_sector, w25q->current_page, 0, 256, rx_data);
        w25q->current_sector++;
        if (w25q_is_page_empty(rx_data))
        {
            sector_found = 1;
            w25q->current_sector = (w25q->current_sector >= 2) ? w25q->current_sector - 2 : 0;
        }
    }

    // The block is completely full, continue from the next block
    if (w25q->current_sector == SECTOR_IN_BLOCK)
    {
        w25q->current_sector = 0;
        w25q->current_block++;

        uint8_t starter_packet[W25Q_PACKET_SIZE];
        for (uint8_t i = 0; i < W25Q_PACKET_SIZE; i++)
        {
            starter_packet[i] = STARTER_PACKET_VALUE;
        }
        w25q_add_buffer(w25q, starter_packet);

        return;
    }

    // Find the first empty page
    while (page_found == 0 && w25q->current_page < PAGE_IN_SECTOR)
    {
        W25Q_Read(w25q, w25q->current_block, w25q->current_sector, w25q->current_page, 0, 256, rx_data);
        w25q->current_page++;
        if (w25q_is_page_empty(rx_data))
        {
            page_found = 1;
            w25q->current_page--;
        }
    }

    // The sector is completely full, continue from the next sector
    if (w25q->current_page == PAGE_IN_SECTOR)
    {
        w25q->current_page = 0;
        w25q->current_sector++;

        uint8_t starter_packet[W25Q_PACKET_SIZE];
        for (uint8_t i = 0; i < W25Q_PACKET_SIZE; i++)
        {
            starter_packet[i] = STARTER_PACKET_VALUE;
        }
        w25q_add_buffer(w25q, starter_packet);

        return;
    }

    uint8_t starter_packet[W25Q_PACKET_SIZE];
    for (uint8_t i = 0; i < W25Q_PACKET_SIZE; i++)
    {
        starter_packet[i] = STARTER_PACKET_VALUE;
    }
    w25q_add_buffer(w25q, starter_packet);
}

/**
 * @brief Adds one log record to the page buffer and writes the page when the buffer is full.
 * @param w25q Pointer to the handle structure.
 * @param data_packet Record to be logged, it must be W25Q_PACKET_SIZE (64) bytes long.
 * @note Called in every main loop pass. The record is dropped when the chip is full, and the
 *       write position is advanced only after a page program has been started successfully.
 */
void W25Q_LogData(W25Q_HandleTypeDef *w25q, uint8_t* data_packet)
{
    if (w25q->flags.is_empty)
    {
        if (w25q->flags.is_dma_transfer_started == 0 && w25q->flags.is_write_enabled == 1)
        {
            w25q_write_disable(w25q);
        }

        w25q_add_buffer(w25q, data_packet);

        if (w25q->flags.is_buffer_full && w25q->flags.is_dma_transfer_started == 0)
        {
            if (W25Q_Write(w25q, w25q->current_block, w25q->current_sector, w25q->current_page, 0, BYTE_IN_PAGE, w25q->buffer) != W25Q_OK)
            {
                return;
            }
            w25q->flags.is_buffer_full = 0;

            w25q->current_page++;

            if (w25q->current_page == PAGE_IN_SECTOR)
            {
                w25q->current_page = 0;
                w25q->current_sector++;
            }

            if (w25q->current_sector == SECTOR_IN_BLOCK)
            {
                w25q->current_sector = 0;
                w25q->current_block++;
            }

            if (w25q->current_block == BLOCK_IN_CHIP)
            {
                w25q->flags.is_empty = 0;
            }
        }
    }
}

/**
 * @brief Executes the flash service command waiting in the handle (read, erase, get usage).
 * @param w25q Pointer to the handle structure.
 * @note Called from the main loop. The commands are blocking and take a long time,
 *       so they are only used on the ground, while the board is not flying.
 */
void W25Q_CommandHandler(W25Q_HandleTypeDef *w25q)
{
    if (w25q->flags.is_cmd_taken)
    {
        w25q->flags.is_cmd_taken = 0;

        if (w25q->cmd == READ_FLASH)
        {
            w25q_read_flash_handler(w25q);
        }
        else if (w25q->cmd == CHIP_ERASE)
        {
            w25q_chip_erase_handler(w25q);
        }
        else if (w25q->cmd == GET_USAGE)
        {
            w25q_get_usage_handler(w25q);
        }
    }
}


/**
 * @brief Callback function to be called from the SPI transmit complete callback.
 * Closes the page program, it releases the chip select and clears the transfer flag.
 * @param w25q Pointer to the handle structure.
 */
void w25q_dma_cplt_callback(W25Q_HandleTypeDef *w25q)
{
    w25q->flags.is_dma_transfer_started = 0;
    cs_high(w25q);
}


/* --- Private Functions Definitions --- */

static void cs_low(W25Q_HandleTypeDef *w25q)
{
    HAL_GPIO_WritePin(w25q->configurations.cs_port, w25q->configurations.cs_pin, GPIO_PIN_RESET);
}

static void cs_high(W25Q_HandleTypeDef *w25q)
{
    HAL_GPIO_WritePin(w25q->configurations.cs_port, w25q->configurations.cs_pin, GPIO_PIN_SET);
}


// Set the write enable latch of the chip, it has to be set before every program and erase
static void w25q_write_enable(W25Q_HandleTypeDef *w25q)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    cs_low(w25q);
    HAL_SPI_Transmit(w25q->configurations.spi, &cmd, 1, 3);
    cs_high(w25q);

    w25q->flags.is_write_enabled = 1;
}

// Clear the write enable latch of the chip after the transfer is finished
static void w25q_write_disable(W25Q_HandleTypeDef *w25q)
{
    uint8_t cmd = CMD_WRITE_DISABLE;

    cs_low(w25q);
    HAL_SPI_Transmit(w25q->configurations.spi, &cmd, 1, 3);
    cs_high(w25q);

    w25q->flags.is_write_enabled = 0;
}

// Put one record into the page buffer, the buffer is marked full after the fourth record
static void w25q_add_buffer(W25Q_HandleTypeDef *w25q, uint8_t* data_packet)
{
    if (w25q->flags.is_buffer_full == 0)
    {
        memcpy(&w25q->buffer[w25q->buffer_offset * W25Q_PACKET_SIZE], data_packet, W25Q_PACKET_SIZE);
        w25q->buffer_offset++;
        if (w25q->buffer_offset == PACKET_IN_BUFFER)
        {
            w25q->buffer_offset = 0;
            w25q->flags.is_buffer_full = 1;
        }
    }
}


// A page is empty when all of its 256 bytes are 0xFF, which is the erased state of the flash
static uint8_t w25q_is_page_empty(uint8_t *page_data)
{
    for (uint16_t i = 0; i < BYTE_IN_PAGE; i++)
    {
        if (page_data[i] != 0xFF)
        {
            return 0;
        }
    }
    return 1;
}


// 8 bit sum of the given bytes, it closes the frames sent to the ground station
static uint8_t calculate_checksum(uint8_t *data, uint16_t size)
{
	uint8_t checksum = 0;

	for (uint16_t i = 0; i < size; i++)
	{
		checksum += data[i];
	}

	return checksum;
}


// Send every written record to the ground station, in 68 byte frames
// [0xC4][64 byte record][checksum][0x0D][0x0A]
// The number of the written records comes from the write position, and the transfer is closed
// by one more frame whose record is filled with 0xFF
static void w25q_read_flash_handler(W25Q_HandleTypeDef *w25q)
{
    uint8_t data[W25Q_PACKET_SIZE + 4];
    data[0] = 0xC4;
    data[W25Q_PACKET_SIZE + 2] = 0x0D;
    data[W25Q_PACKET_SIZE + 3] = 0x0A;

    uint32_t address = 0;

    uint32_t writen_packet_count = w25q->current_block * SECTOR_IN_BLOCK * PAGE_IN_SECTOR * PACKET_IN_BUFFER + w25q->current_sector * PAGE_IN_SECTOR * PACKET_IN_BUFFER + w25q->current_page * PACKET_IN_BUFFER;

    for (uint32_t packet = 0; packet < writen_packet_count; packet++)
    {
        W25Q_ReadWithAddress(w25q, address, W25Q_PACKET_SIZE, &data[1]);
        address += W25Q_PACKET_SIZE;
        data[W25Q_PACKET_SIZE + 1] = calculate_checksum(data, W25Q_PACKET_SIZE + 1);
        HAL_UART_Transmit(w25q->huart, data, W25Q_PACKET_SIZE + 4, 100);
    }

    for (uint8_t i = 0; i < W25Q_PACKET_SIZE; i++)
    {
        data[i + 1] = 0xFF;
    }
    data[W25Q_PACKET_SIZE + 1] = calculate_checksum(data, W25Q_PACKET_SIZE + 1);
    HAL_UART_Transmit(w25q->huart, data, W25Q_PACKET_SIZE + 4, 100);
}

// Erase the whole chip and open a new logging session from the beginning
static void w25q_chip_erase_handler(W25Q_HandleTypeDef *w25q)
{
    w25q_write_enable(w25q);

    uint8_t cmd = CMD_CHIP_ERASE;

    cs_low(w25q);
    HAL_SPI_Transmit(w25q->configurations.spi, &cmd, 1, 5);
    cs_high(w25q);

    HAL_Delay(100000);

    w25q_write_disable(w25q);

    uint8_t data[BYTE_IN_PAGE];
    W25Q_Read(w25q, 0, 0, 0, 0, 256, data);
    if (w25q_is_page_empty(data))
    {
        /* The erase was successful */
        w25q->current_block = 0;
        w25q->current_sector = 0;
        w25q->current_page = 0;

        w25q->flags.is_empty = 1;

        w25q->buffer_offset = 0;
        memset(w25q->buffer, 0, BYTE_IN_PAGE);
        w25q->flags.is_buffer_full = 0;

        uint8_t starter_packet[W25Q_PACKET_SIZE];
        for (uint8_t i = 0; i < W25Q_PACKET_SIZE; i++)
        {
            starter_packet[i] = STARTER_PACKET_VALUE;
        }
        w25q_add_buffer(w25q, starter_packet);
    }
}

// Send the current write position to the ground station, in a 7 byte frame
// [0xC6][block][sector][page][checksum][0x0D][0x0A]
static void w25q_get_usage_handler(W25Q_HandleTypeDef *w25q)
{
    uint8_t data[7];

    data[0] = 0xC6;
    data[1] = w25q->current_block;
    data[2] = w25q->current_sector;
    data[3] = w25q->current_page;
    data[4] = calculate_checksum(data, 4);
    data[5] = 0x0D;
    data[6] = 0x0A;

    HAL_UART_Transmit(w25q->huart, data, 7, 100);
}

// Software reset of the chip, the enable reset command is followed by the reset device command
static void w25q_reset(W25Q_HandleTypeDef *w25q)
{
    uint8_t cmd = CMD_ENABLE_RESET;

    cs_low(w25q);
    HAL_SPI_Transmit(w25q->configurations.spi, &cmd, 1, 5);
    cs_high(w25q);

    cmd = CMD_RESET_DEVICE;

    cs_low(w25q);
    HAL_SPI_Transmit(w25q->configurations.spi, &cmd, 1, 5);
    cs_high(w25q);
}
