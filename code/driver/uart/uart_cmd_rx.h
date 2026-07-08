/*
 * uart_cmd_rx.h
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: UART input RX FIFO
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_UART_UART_CMD_RX_H_
#define CODE_DRIVER_UART_UART_CMD_RX_H_

#include "zf_common_typedef.h"

#define UART_CMD_RX_BUFFER_SIZE 256u

void uart_cmd_rx_init(void);
void uart_cmd_rx_clear(void);
void uart_cmd_rx_push_byte(uint8 byte);
uint8 uart_cmd_rx_pop_byte(uint8 *byte);
uint32 uart_cmd_rx_get_used(void);
uint32 uart_cmd_rx_get_overflow_count(void);

#endif /* CODE_DRIVER_UART_UART_CMD_RX_H_ */
