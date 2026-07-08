/*
 * uart_cmd_line.h
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: UART input line assembler
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_UART_UART_CMD_LINE_H_
#define CODE_DRIVER_UART_UART_CMD_LINE_H_

#include "zf_common_typedef.h"

#define UART_CMD_LINE_SIZE 64u

typedef enum
{
    UART_CMD_LINE_RESULT_NONE = 0,
    UART_CMD_LINE_RESULT_READY,
    UART_CMD_LINE_RESULT_OVERFLOW,
} uart_cmd_line_result_enum;

void uart_cmd_line_init(void);
void uart_cmd_line_reset(void);
uart_cmd_line_result_enum uart_cmd_line_push_byte(uint8 byte, char **line);

#endif /* CODE_DRIVER_UART_UART_CMD_LINE_H_ */
