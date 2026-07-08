/*
 * uart_cmd_line.c
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: UART input line assembler
 *  Copyright: KimJong
 */

#include "uart_cmd_line.h"

static char s_uart_cmd_line_buffer[UART_CMD_LINE_SIZE];
static uint8 s_uart_cmd_line_len = 0u;

void uart_cmd_line_init(void)
{
    uart_cmd_line_reset();
}

void uart_cmd_line_reset(void)
{
    uint32 index = 0u;

    s_uart_cmd_line_len = 0u;
    for (index = 0u; index < UART_CMD_LINE_SIZE; index++)
    {
        s_uart_cmd_line_buffer[index] = '\0';
    }
}

uart_cmd_line_result_enum uart_cmd_line_push_byte(uint8 byte, char **line)
{
    if (0 != line)
    {
        *line = 0;
    }

    if ((0x08u == byte) || (0x7Fu == byte))
    {
        if (0u != s_uart_cmd_line_len)
        {
            s_uart_cmd_line_len--;
        }
        return UART_CMD_LINE_RESULT_NONE;
    }

    if (('\r' == byte) || ('\n' == byte))
    {
        if (0u != s_uart_cmd_line_len)
        {
            s_uart_cmd_line_buffer[s_uart_cmd_line_len] = '\0';
            s_uart_cmd_line_len = 0u;
            if (0 != line)
            {
                *line = s_uart_cmd_line_buffer;
            }
            return UART_CMD_LINE_RESULT_READY;
        }
        return UART_CMD_LINE_RESULT_NONE;
    }

    if ((byte < 0x20u) || (byte > 0x7Eu))
    {
        return UART_CMD_LINE_RESULT_NONE;
    }

    if (s_uart_cmd_line_len >= (UART_CMD_LINE_SIZE - 1u))
    {
        s_uart_cmd_line_len = 0u;
        s_uart_cmd_line_buffer[0] = '\0';
        return UART_CMD_LINE_RESULT_OVERFLOW;
    }

    s_uart_cmd_line_buffer[s_uart_cmd_line_len++] = (char)byte;
    return UART_CMD_LINE_RESULT_NONE;
}
