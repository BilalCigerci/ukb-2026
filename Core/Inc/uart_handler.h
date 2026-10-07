/*
 * uart_handler.h
 *
 *  Created on: Mar 15, 2026
 *      Author: bilal
 */

/*
 *  Ground station link of the board. It carries the SIT telemetry, the SUT simulation data and
 *  the W25Q flash service commands over a single UART, and it also selects the system mode the
 *  main loop runs in.
 *
 *  Four frame types share the line, the incoming ones are told apart by their received length:
 *
 *      5 bytes    command frame        ground station -> board     (header 0xAA)
 *      36 bytes   SUT data frame       ground station -> board     (header 0xAB)
 *      36 bytes   SIT telemetry frame  board -> ground station     (header 0xAB)
 *      6 bytes    SUT state frame      board -> ground station     (header 0xAA)
 *
 *  Every frame ends with a one byte checksum followed by the 0x0D 0x0A footer. The checksum is
 *  the 8 bit sum of all the bytes in front of it, the header included.
 *
 *  Reception is fully interrupt driven. HAL_UARTEx_ReceiveToIdle_DMA() fills the receive buffer
 *  and uart_handler_rx_event_callback() copies the frame into the command or the process buffer,
 *  the frames are only validated and used later in UartHandler_ProcessPacket(), which is called
 *  from the main loop. Transmission uses the same UART in DMA mode, one transfer at a time.
 *
 *  Packet payload layouts are documented in data_manager.h, this module only handles the header,
 *  the checksum, the footer and the transport itself.
 */

#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_


#include "main.h"
#include <stdint.h>
#include "w25q.h"


// Buffer sizes. The receive buffer is sized for the longest incoming frame, a shorter command
// frame is delivered by the idle line detection of HAL_UARTEx_ReceiveToIdle_DMA()
#define UART_HANDLER_RX_BUFFER_SIZE         36
#define UART_HANDLER_RX_CMD_BUFFER_SIZE     5
#define UART_HANDLER_SIT_PACKET_SIZE        36
#define UART_HANDLER_SUT_STATE_PACKET_SIZE  6

// Frame headers and lengths, the same headers are used in both directions
#define CMD_HEADER      0xAA
#define CMD_SIZE        5
#define PACKET_HEADER   0xAB
#define PACKET_SIZE     36

// Mode commands, they are handled inside this module
#define CMD_START_SIT   0x20
#define CMD_START_SUT   0x22
#define CMD_STOP        0x24

// Flash service commands, they are only forwarded to the W25Q module
#define CMD_READ_FLASH  0x51
#define CMD_CHIP_ERASE  0x53
#define CMD_GET_USAGE   0x55

// Frame footer, it closes every frame in both directions
#define FOOTER_1        0x0D
#define FOOTER_2        0x0A


/**
 * @brief Converts a float to its four bytes and back, used while packing and parsing the frames.
 * @note The guard is needed because the same union is also defined by the other modules that
 *       build byte packets.
 */
#ifndef TYPE_CONVERTER_FLOAT_UINT8_ARRAY
#define TYPE_CONVERTER_FLOAT_UINT8_ARRAY
typedef union
{
	float float_;
	uint8_t uint8_array[4];
} type_converter_float_uint8_array;
#endif /* TYPE_CONVERTER_FLOAT_UINT8_ARRAY */


/**
 * @brief Mode the main loop runs in, it is selected by the received mode commands.
 * @note NORMAL_MODE is the flight mode, the algorithm runs on the real sensors and no telemetry
 *       is transmitted. In SIT_MODE the board keeps using its own sensors and streams them to
 *       the ground station, the flight algorithm is not run. In SUT_MODE the sensor readings
 *       are replaced by the received simulation data and only the flight algorithm state is
 *       sent back.
 * @note The value is also stored in the backup SRAM, so the mode survives a reset.
 */
typedef enum
{
	NORMAL_MODE = 0x00,
    SIT_MODE    = 0x01,
    SUT_MODE    = 0x02
} system_mode_t;


/**
 * @brief Last valid SUT simulation packet, parsed into the physical values it carries.
 * @note Altitude is in meters, pressure in hPa, accelerations in m/s^2 and angles in degrees.
 */
typedef struct
{
	float altitude;
	float pressure;
	float accel_x;
	float accel_y;
	float accel_z;
	float angle_x;
	float angle_y;
	float angle_z;
} data_packet_t;

/**
 * @brief State flags shared between the UART interrupts and the main loop.
 */
typedef struct
{
	volatile uint8_t is_data_ready;             // A 36 byte frame is waiting in the process buffer
	volatile uint8_t is_command_ready;          // A 5 byte frame is waiting in the command buffer
	volatile uint8_t is_dma_transfer_started;   // 1 while a transmission is still running
	volatile uint8_t is_command_packet_ready;   // A known command has been accepted, read by the main loop
} uart_handler_flags_t;

/**
 * @brief Main UART handler Handle structure.
 * Combines the receive and transmit buffers, the parsed simulation data and the internal state.
 * @note rx_buffer is written by the DMA, the received frame is copied out of it in the receive
 *       callback so the reception can be restarted right away while the frame is processed.
 */
typedef struct
{
	uint8_t rx_buffer[UART_HANDLER_RX_BUFFER_SIZE];                 // DMA destination
	uint8_t process_buffer[UART_HANDLER_RX_BUFFER_SIZE];            // Copy of the last received data frame
	uint8_t command_buffer[UART_HANDLER_RX_CMD_BUFFER_SIZE];        // Copy of the last received command frame
	uint8_t data_size;                                              // Length reported by the last receive event

	data_packet_t last_sut_data;                                    // Payload of the last valid SUT packet
	uint8_t sit_data_packet[UART_HANDLER_SIT_PACKET_SIZE];          // Outgoing SIT telemetry frame
	uint8_t sut_state_packet[UART_HANDLER_SUT_STATE_PACKET_SIZE];   // Outgoing SUT state frame

	uart_handler_flags_t flags;
	system_mode_t system_mode;
	UART_HandleTypeDef *huart;
} UartHandler_HandleTypeDef;


/**
 * @brief Clears the flags and the outgoing SUT buffers, stores the UART handle and starts the reception.
 * @param handler Pointer to the handle structure.
 * @param huart Pointer to the UART handle used by the link.
 * @note The reception is started with HAL_UARTEx_ReceiveToIdle_DMA(), so a frame is delivered
 *       either when the buffer is full or when the line goes idle. The DMA half transfer
 *       interrupt is disabled right after, otherwise a half filled buffer would be reported as
 *       a received frame.
 * @note The system mode is not touched here. It stays at the value the handle already holds,
 *       which is NORMAL_MODE after a cold start and the mode restored from the backup SRAM
 *       after a reset.
 */
void UartHandler_Start(UartHandler_HandleTypeDef *handler, UART_HandleTypeDef *huart);

/**
 * @brief Processes the frames collected by the receive callback, called from the main loop.
 * @param handler Pointer to the handle structure.
 * @param w25q Pointer to the W25Q handle, the flash service commands are forwarded to it.
 * @note A command frame is handled before a data frame, so a mode change is applied before the
 *       simulation data of the previous mode is parsed.
 */
void UartHandler_ProcessPacket(UartHandler_HandleTypeDef *handler, W25Q_HandleTypeDef *w25q);

/**
 * @brief Completes the SIT telemetry frame and transmits it over DMA.
 * @param handler Pointer to the handle structure.
 * @note Only the header, the checksum and the footer are written here, the payload is filled
 *       beforehand by DataManager_UpdateSitPacket().
 */
void UartHandler_SendSitPacket(UartHandler_HandleTypeDef *handler);

/**
 * @brief Completes the SUT state frame and transmits it over DMA.
 * @param handler Pointer to the handle structure.
 * @note Only the header, the checksum and the footer are written here, the two state bytes are
 *       filled beforehand by DataManager_UpdateSutStatePacket().
 */
void UartHandler_SendSutStatePacket(UartHandler_HandleTypeDef *handler);


/**
 * @brief Callback function to be called from HAL_UART_TxCpltCallback(), releases the transmit line.
 * @param handler Pointer to the handle structure.
 */
void uart_handler_dma_cplt_callback(UartHandler_HandleTypeDef *handler);

/**
 * @brief Callback function to be called from HAL_UARTEx_RxEventCallback(), stores the received frame.
 * @param handler Pointer to the handle structure.
 * @param size Number of bytes reported by the receive event.
 * @note The frame type is selected by the length only, a frame of any other length is discarded.
 *       The header, the checksum and the footer are checked later, while the frame is processed.
 * @note The reception is restarted at the end of the callback, so the next frame is received
 *       even when the previous one has been discarded.
 */
void uart_handler_rx_event_callback(UartHandler_HandleTypeDef *handler, uint16_t size);

/**
 * @brief Clears the 'is_command_packet_ready' flag.
 * @param handler Pointer to the handle structure.
 */
void uart_handler_clear_is_command_packet_ready_flag(UartHandler_HandleTypeDef *handler);

/**
 * @brief Returns the 'is_command_packet_ready' flag.
 * @param handler Pointer to the handle structure.
 * @retval 1 when an accepted command is waiting to be consumed, 0 otherwise.
 * @note The flag is set by every accepted command, the flash service ones included. The main
 *       loop uses it to reset the flight algorithm and the Kalman filter, so a session always
 *       starts from a clean state.
 */
uint8_t uart_handler_get_is_command_packet_ready_flag(UartHandler_HandleTypeDef *handler);

/**
 * @brief Returns the current system mode.
 * @param handler Pointer to the handle structure.
 * @retval system_mode_t.
 */
system_mode_t uart_handler_get_system_mode(UartHandler_HandleTypeDef *handler);


#endif /* INC_UART_HANDLER_H_ */
