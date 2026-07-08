#ifndef CODE_APP_UART_COMMAND_H_
#define CODE_APP_UART_COMMAND_H_

#include "zf_common_typedef.h"

void uart_command_init(void);
void uart_command_poll(void);
void uart_command_rx_callback(void);
uint8 uart_command_handle_line(const char *line);

#endif /* CODE_APP_UART_COMMAND_H_ */
