/*
 * uart_cmd_rx.c
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: UART input RX FIFO
 *  Copyright: KimJong
 */

#include "zf_common_headfile.h"
#include "uart_cmd_rx.h"

static volatile uint8 s_uart_cmd_rx_buffer[UART_CMD_RX_BUFFER_SIZE];
static volatile uint16 s_uart_cmd_rx_head = 0u;
static volatile uint16 s_uart_cmd_rx_tail = 0u;
static volatile uint16 s_uart_cmd_rx_used = 0u;
static volatile uint32 s_uart_cmd_rx_overflow_count = 0u;

void uart_cmd_rx_init(void)
{
    uart_cmd_rx_clear();
}

void uart_cmd_rx_clear(void)
{
    boolean interrupt_state = disableInterrupts();
    uint16 index = 0u;

    s_uart_cmd_rx_head = 0u;
    s_uart_cmd_rx_tail = 0u;
    s_uart_cmd_rx_used = 0u;
    s_uart_cmd_rx_overflow_count = 0u;

    for (index = 0u; index < UART_CMD_RX_BUFFER_SIZE; index++)
    {
        s_uart_cmd_rx_buffer[index] = 0u;
    }

    restoreInterrupts(interrupt_state);
}

void uart_cmd_rx_push_byte(uint8 byte)
{
    boolean interrupt_state = disableInterrupts();

    if (s_uart_cmd_rx_used >= UART_CMD_RX_BUFFER_SIZE)
    {
        s_uart_cmd_rx_overflow_count++;
        restoreInterrupts(interrupt_state);
        return;
    }

    s_uart_cmd_rx_buffer[s_uart_cmd_rx_head] = byte;
    s_uart_cmd_rx_head = (uint16)((s_uart_cmd_rx_head + 1u) % UART_CMD_RX_BUFFER_SIZE);
    s_uart_cmd_rx_used++;
    restoreInterrupts(interrupt_state);
}

uint8 uart_cmd_rx_pop_byte(uint8 *byte)
{
    boolean interrupt_state;

    zf_assert(0 != byte);
    interrupt_state = disableInterrupts();

    if (0u == s_uart_cmd_rx_used)
    {
        restoreInterrupts(interrupt_state);
        return 0u;
    }

    *byte = s_uart_cmd_rx_buffer[s_uart_cmd_rx_tail];
    s_uart_cmd_rx_tail = (uint16)((s_uart_cmd_rx_tail + 1u) % UART_CMD_RX_BUFFER_SIZE);
    s_uart_cmd_rx_used--;

    restoreInterrupts(interrupt_state);
    return 1u;
}

uint32 uart_cmd_rx_get_used(void)
{
    boolean interrupt_state = disableInterrupts();
    uint32 used = s_uart_cmd_rx_used;
    restoreInterrupts(interrupt_state);
    return used;
}

uint32 uart_cmd_rx_get_overflow_count(void)
{
    boolean interrupt_state = disableInterrupts();
    uint32 overflow_count = s_uart_cmd_rx_overflow_count;
    restoreInterrupts(interrupt_state);
    return overflow_count;
}
