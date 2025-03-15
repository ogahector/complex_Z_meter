/*
 * transmits.h
 *
 *  Created on: Mar 9, 2025
 *      Author: ogahe
 */

#ifndef SRC_TRANSMITS_H_
#define SRC_TRANSMITS_H_

#include "main.h"

#define RX_BUFFER_SIZE 128
#define RX_CMD_BYTE_NB (2*3) // func + par1 + par2 (2 Bytes each)



// User-defined protocol for UI interface
// - From UI to MCU (fixed length)
//   0x0000 (func) + 0x0000 (par1) + 0x0000 (par2)
// - From MCU to UI (flexible length)
//   Message + '$' + Data + '#'
//   '$': Message end flag
//   '#': Response end flag
//   Data: [1] integer: '666' or '-666'
//         [2] array:   '[' + '1,' + '2,' + '...' + ']' (NOT EFFICIENT)

// Ideally I'd want this to be cast to a uint16_t but if it's fine for the HAL it's fine here
typedef enum __ui_command_t{
	get_version=                      0x0000,
	get_id=                           0x0001,     // STM32 unique device id
	get_clk_divpw=                    0x0002,
	check_status_pos12=               0x0013,
	check_status_neg12=               0x0014,
	check_status_3v3=                 0x0015,

	// ----------------------------------------------------------------
	// STM32 Peripheral
	// ----------------------------------------------------------------
	readout_time=                     0x2200,
	get_phasors=                      0x2202,
	start_sc_calib=                   0x2203,
	start_oc_calib=                   0x2204,
	stop_sc_calib=                    0x2205,
	stop_oc_calib=                    0x2206,
	rref_get_val=                     0x2300,
	rref_set_val=                     0x2301,

	// - DAC & ADC
	dac_set_val=                      0x1103,
	dac_get_val=                      0x0103,
	adc_get_val=                      0x0104,
	curr_get_val=                     0x0105,

	// - SPI
	spi_transfer=                     0x1107,
	spi_init=                         0x0107,
	spi_deinit=                       0x0108,

	// ----------------------------------------------------------------
	// - System Commands
	quit=                             0x0F00,     // System cmd= Quit the command interface
	list_cmd=                         0x0F01,     // System cmd= List all commands
	list_serial=                      0x0F10,     // System cmd= List serial port
	close_serial=                     0x0F11,     // System cmd= Close current serial port
	open_serial=                      0x1F12,     // System cmd: Open serial port COM [par1]

	idle= 							  0xFFFF
} ui_command_t;


extern UART_HandleTypeDef huart2;
extern volatile uint8_t cmd_available;
extern volatile ui_command_t command;
extern volatile uint8_t rx_buffer[RX_CMD_BYTE_NB];
extern volatile size_t rx_index;

/*-------------------------------------------------------------------------*/

ui_command_t Receive_Command(void);
uint8_t Command_is_Available(void);
void Echo_Message(void);


/*-------------------------------------------------------------------------*/


HAL_StatusTypeDef TransmitMessageUI(char msg[]);
HAL_StatusTypeDef TransmitUInt32BufferUI(char msg[], uint32_t buffer[]);
HAL_StatusTypeDef TransmitUint32UI(char msg[], uint32_t num);


/*-------------------------------------------------------------------------*/


HAL_StatusTypeDef ReceiveMessage(char msg[], size_t len);
HAL_StatusTypeDef TransmitString(char msg[]);
HAL_StatusTypeDef TransmitStringRaw(char msg[]);
HAL_StatusTypeDef TransmitStringLn(char msg[]);
HAL_StatusTypeDef TransmitIntBuffer(int buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt32Buffer(uint32_t buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt16Buffer(uint16_t buffer[], size_t size);
HAL_StatusTypeDef TransmitTwoUInt16Buffer(uint16_t buffer1[], uint16_t buffer2[], size_t size);
HAL_StatusTypeDef TransmitUInt8Buffer(uint8_t buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt16BufferAsRaw(uint16_t buffer[], size_t size);
HAL_StatusTypeDef TransmitNum(float num);
HAL_StatusTypeDef TransmitNumRaw(float num);
HAL_StatusTypeDef TransmitNumLn(float num);


#endif /* SRC_TRANSMITS_H_ */
