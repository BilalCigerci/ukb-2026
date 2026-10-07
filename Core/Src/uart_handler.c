/*
 * uart_handler.c
 *
 *  Created on: Mar 15, 2026
 *      Author: bilal
 */

/*
 *  Transport layer of the ground station link. It receives the command and the SUT simulation
 *  frames, transmits the SIT telemetry and the SUT state frames, and keeps the system mode.
 *  The frame layouts are documented in uart_handler.h, the payload contents in data_manager.h.
 *
 *  The interrupt context only copies the received frame out of the DMA buffer and sets a flag,
 *  every check and every parse is done later in UartHandler_ProcessPacket() from the main loop.
 */

#include "main.h"
#include "uart_handler.h"
#include <string.h>
#include <stdio.h>


/* --- Private Functions Declaration --- */
/* Packet Processing Functions */
static void process_cmd_packet(UartHandler_HandleTypeDef *handler, W25Q_HandleTypeDef *w25q);
static void process_sut_packet(UartHandler_HandleTypeDef *handler);
static uint8_t calculate_checksum(uint8_t *data, uint16_t size);

/* Buffer Clearing Functions */
static void clear_sut_state_packet(UartHandler_HandleTypeDef *handler);
static void clear_last_sut_data(UartHandler_HandleTypeDef *handler);


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
void UartHandler_Start(UartHandler_HandleTypeDef *handler, UART_HandleTypeDef *huart)
{
	handler->flags.is_data_ready = 0;
	handler->flags.is_command_ready = 0;
	handler->flags.is_dma_transfer_started = 0;
	handler->flags.is_command_packet_ready = 0;

	clear_sut_state_packet(handler);
	clear_last_sut_data(handler);

	handler->huart = huart;

	if (HAL_UARTEx_ReceiveToIdle_DMA(handler->huart, handler->rx_buffer, UART_HANDLER_RX_BUFFER_SIZE) == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(handler->huart->hdmarx, DMA_IT_HT);
	}
}

/**
 * @brief Processes the frames collected by the receive callback, called from the main loop.
 * @param handler Pointer to the handle structure.
 * @param w25q Pointer to the W25Q handle, the flash service commands are forwarded to it.
 * @note A command frame is handled before a data frame, so a mode change is applied before the
 *       simulation data of the previous mode is parsed.
 */
void UartHandler_ProcessPacket(UartHandler_HandleTypeDef *handler, W25Q_HandleTypeDef *w25q)
{
	if (handler->flags.is_command_ready == 1)
	{
		handler->flags.is_command_ready = 0;
		process_cmd_packet(handler, w25q);
	}

	if (handler->flags.is_data_ready == 1)
	{
		handler->flags.is_data_ready = 0;
		process_sut_packet(handler);
	}
}

/**
 * @brief Completes the SIT telemetry frame and transmits it over DMA.
 * @param handler Pointer to the handle structure.
 * @note Only the header, the checksum and the footer are written here, the payload is filled
 *       beforehand by DataManager_UpdateSitPacket().
 */
void UartHandler_SendSitPacket(UartHandler_HandleTypeDef *handler)
{
	if (handler->flags.is_dma_transfer_started == 0)
	{
		handler->sit_data_packet[0] = PACKET_HEADER;
		handler->sit_data_packet[UART_HANDLER_SIT_PACKET_SIZE - 3] = calculate_checksum(handler->sit_data_packet, UART_HANDLER_SIT_PACKET_SIZE - 3);
		handler->sit_data_packet[UART_HANDLER_SIT_PACKET_SIZE - 2] = FOOTER_1;
		handler->sit_data_packet[UART_HANDLER_SIT_PACKET_SIZE - 1] = FOOTER_2;

		handler->flags.is_dma_transfer_started = 1;
		if (HAL_UART_Transmit_DMA(handler->huart, handler->sit_data_packet, UART_HANDLER_SIT_PACKET_SIZE) != HAL_OK)
		{
			handler->flags.is_dma_transfer_started = 0;
		}
	}
}

/**
 * @brief Completes the SUT state frame and transmits it over DMA.
 * @param handler Pointer to the handle structure.
 * @note Only the header, the checksum and the footer are written here, the two state bytes are
 *       filled beforehand by DataManager_UpdateSutStatePacket().
 */
void UartHandler_SendSutStatePacket(UartHandler_HandleTypeDef *handler)
{
	if (handler->flags.is_dma_transfer_started == 0)
	{
		handler->flags.is_dma_transfer_started = 1;
		handler->sut_state_packet[0] = CMD_HEADER;
		handler->sut_state_packet[UART_HANDLER_SUT_STATE_PACKET_SIZE - 3] = calculate_checksum(handler->sut_state_packet, UART_HANDLER_SUT_STATE_PACKET_SIZE - 3);
		handler->sut_state_packet[UART_HANDLER_SUT_STATE_PACKET_SIZE - 2] = FOOTER_1;
		handler->sut_state_packet[UART_HANDLER_SUT_STATE_PACKET_SIZE - 1] = FOOTER_2;
		if (HAL_UART_Transmit_DMA(handler->huart, handler->sut_state_packet, UART_HANDLER_SUT_STATE_PACKET_SIZE) != HAL_OK)
		{
			handler->flags.is_dma_transfer_started = 0;
		}
	}
}


/**
 * @brief Callback function to be called from HAL_UART_TxCpltCallback(), releases the transmit line.
 * @param handler Pointer to the handle structure.
 */
void uart_handler_dma_cplt_callback(UartHandler_HandleTypeDef *handler)
{
	handler->flags.is_dma_transfer_started = 0;
}

/**
 * @brief Callback function to be called from HAL_UARTEx_RxEventCallback(), stores the received frame.
 * @param handler Pointer to the handle structure.
 * @param size Number of bytes reported by the receive event.
 * @note The frame type is selected by the length only, a frame of any other length is discarded.
 *       The header, the checksum and the footer are checked later, while the frame is processed.
 * @note The reception is restarted at the end of the callback, so the next frame is received
 *       even when the previous one has been discarded.
 */
void uart_handler_rx_event_callback(UartHandler_HandleTypeDef *handler, uint16_t size)
{
	handler->data_size = size;

	if (size == UART_HANDLER_RX_CMD_BUFFER_SIZE)
	{
		memcpy(handler->command_buffer, handler->rx_buffer, handler->data_size);
		handler->flags.is_command_ready = 1;
	}
	else if (size == UART_HANDLER_RX_BUFFER_SIZE)
	{
		memcpy(handler->process_buffer, handler->rx_buffer, handler->data_size);
		handler->flags.is_data_ready = 1;
	}

	if (HAL_UARTEx_ReceiveToIdle_DMA(handler->huart, handler->rx_buffer, UART_HANDLER_RX_BUFFER_SIZE) == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(handler->huart->hdmarx, DMA_IT_HT);
	}
}

/**
 * @brief Clears the 'is_command_packet_ready' flag.
 * @param handler Pointer to the handle structure.
 */
void uart_handler_clear_is_command_packet_ready_flag(UartHandler_HandleTypeDef *handler)
{
	handler->flags.is_command_packet_ready = 0;
}

/**
 * @brief Returns the 'is_command_packet_ready' flag.
 * @param handler Pointer to the handle structure.
 * @retval 1 when an accepted command is waiting to be consumed, 0 otherwise.
 * @note The flag is set by every accepted command, the flash service ones included. The main
 *       loop uses it to reset the flight algorithm and the Kalman filter, so a session always
 *       starts from a clean state.
 */
uint8_t uart_handler_get_is_command_packet_ready_flag(UartHandler_HandleTypeDef *handler)
{
	return handler->flags.is_command_packet_ready;
}

/**
 * @brief Returns the current system mode.
 * @param handler Pointer to the handle structure.
 * @retval system_mode_t.
 */
system_mode_t uart_handler_get_system_mode(UartHandler_HandleTypeDef *handler)
{
	return handler->system_mode;
}


/**
 * @brief Checks the received command frame and applies the command it carries.
 * @param handler Pointer to the handle structure.
 * @param w25q Pointer to the W25Q handle, the flash service commands are forwarded to it.
 * @note The frame is accepted only when the footer and the checksum match, the header byte is
 *       not compared. A frame that fails the check is dropped silently, the previous mode and
 *       the previous simulation data are kept.
 * @note The mode commands are applied here, the flash service commands are only stored in the
 *       W25Q handle and are executed later by W25Q_CommandHandler() from the main loop.
 * @note 'is_command_packet_ready' is set for every accepted command and is cleared again when
 *       the command byte is unknown, so an unknown command leaves the whole state untouched.
 */
static void process_cmd_packet(UartHandler_HandleTypeDef *handler, W25Q_HandleTypeDef *w25q)
{
	if (handler->command_buffer[CMD_SIZE - 2] == FOOTER_1 && handler->command_buffer[CMD_SIZE - 1] == FOOTER_2 && handler->command_buffer[CMD_SIZE - 3] == calculate_checksum(handler->command_buffer, CMD_SIZE - 3))
	{
		handler->flags.is_command_packet_ready = 1;

		if (handler->command_buffer[1] == CMD_START_SIT)
		{
			handler->system_mode = SIT_MODE;
		}
		else if (handler->command_buffer[1] == CMD_START_SUT)
		{
			clear_sut_state_packet(handler);
			clear_last_sut_data(handler);
			handler->system_mode = SUT_MODE;
		}
		else if (handler->command_buffer[1] == CMD_STOP)
		{
			handler->system_mode = NORMAL_MODE;
		}
        else if (handler->command_buffer[1] == CMD_READ_FLASH)
        {
            w25q->flags.is_cmd_taken = 1;
            w25q->cmd = READ_FLASH;
        }
        else if (handler->command_buffer[1] == CMD_CHIP_ERASE)
        {
            w25q->flags.is_cmd_taken = 1;
            w25q->cmd = CHIP_ERASE;
        }
        else if (handler->command_buffer[1] == CMD_GET_USAGE)
        {
            w25q->flags.is_cmd_taken = 1;
            w25q->cmd = GET_USAGE;
        }
		else
		{
			handler->flags.is_command_packet_ready = 0;
		}
	}
}

/**
 * @brief Checks the received SUT frame and parses its payload into the last simulation data.
 * @param handler Pointer to the handle structure.
 * @note The frame is accepted only when the footer and the checksum match, the header byte is
 *       not compared. A frame that fails the check is dropped and 'last_sut_data' keeps the
 *       values of the last valid frame.
 * @note The floats are sent MSB first, so the bytes are written into the converter union in
 *       reverse order.
 * @note The parsed values are copied into the flight data later by
 *       DataManager_UpdateFlightDataWithSutPacket(), this function only decodes the frame.
 */
static void process_sut_packet(UartHandler_HandleTypeDef *handler)
{
	if (handler->process_buffer[PACKET_SIZE - 2] == FOOTER_1 && handler->process_buffer[PACKET_SIZE - 1] == FOOTER_2 && handler->process_buffer[PACKET_SIZE - 3] == calculate_checksum(handler->process_buffer, PACKET_SIZE - 3))
	{
		type_converter_float_uint8_array type_converter;

		type_converter.uint8_array[3] = handler->process_buffer[1];
		type_converter.uint8_array[2] = handler->process_buffer[2];
		type_converter.uint8_array[1] = handler->process_buffer[3];
		type_converter.uint8_array[0] = handler->process_buffer[4];
		handler->last_sut_data.altitude = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[5];
		type_converter.uint8_array[2] = handler->process_buffer[6];
		type_converter.uint8_array[1] = handler->process_buffer[7];
		type_converter.uint8_array[0] = handler->process_buffer[8];
		handler->last_sut_data.pressure = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[9];
		type_converter.uint8_array[2] = handler->process_buffer[10];
		type_converter.uint8_array[1] = handler->process_buffer[11];
		type_converter.uint8_array[0] = handler->process_buffer[12];
		handler->last_sut_data.accel_x = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[13];
		type_converter.uint8_array[2] = handler->process_buffer[14];
		type_converter.uint8_array[1] = handler->process_buffer[15];
		type_converter.uint8_array[0] = handler->process_buffer[16];
		handler->last_sut_data.accel_y = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[17];
		type_converter.uint8_array[2] = handler->process_buffer[18];
		type_converter.uint8_array[1] = handler->process_buffer[19];
		type_converter.uint8_array[0] = handler->process_buffer[20];
		handler->last_sut_data.accel_z = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[21];
		type_converter.uint8_array[2] = handler->process_buffer[22];
		type_converter.uint8_array[1] = handler->process_buffer[23];
		type_converter.uint8_array[0] = handler->process_buffer[24];
		handler->last_sut_data.angle_x = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[25];
		type_converter.uint8_array[2] = handler->process_buffer[26];
		type_converter.uint8_array[1] = handler->process_buffer[27];
		type_converter.uint8_array[0] = handler->process_buffer[28];
		handler->last_sut_data.angle_y = type_converter.float_;

		type_converter.uint8_array[3] = handler->process_buffer[29];
		type_converter.uint8_array[2] = handler->process_buffer[30];
		type_converter.uint8_array[1] = handler->process_buffer[31];
		type_converter.uint8_array[0] = handler->process_buffer[32];
		handler->last_sut_data.angle_z = type_converter.float_;
	}
}

/**
 * @brief Sums the given bytes and returns the checksum used by every frame.
 * @param data Pointer to the first byte of the frame.
 * @param size Number of bytes in front of the checksum byte.
 * @retval 8 bit sum of the bytes, the overflow is intentionally discarded.
 */
static uint8_t calculate_checksum(uint8_t *data, uint16_t size)
{
	uint8_t checksum = 0;

	for (uint16_t i = 0; i < size; i++)
	{
		checksum += data[i];
	}

	return checksum;
}

/**
 * @brief Clears the outgoing SUT state frame, including its state bytes.
 * @param handler Pointer to the handle structure.
 */
static void clear_sut_state_packet(UartHandler_HandleTypeDef *handler)
{
	memset(handler->sut_state_packet, 0, UART_HANDLER_SUT_STATE_PACKET_SIZE);
}

/**
 * @brief Clears the last parsed SUT simulation data.
 * @param handler Pointer to the handle structure.
 */
static void clear_last_sut_data(UartHandler_HandleTypeDef *handler)
{
	memset(&handler->last_sut_data, 0, sizeof(handler->last_sut_data));
}
