#ifndef CODE_DRIVER_MOTOR_MOTOR_UART_DRIVER_H_
#define CODE_DRIVER_MOTOR_MOTOR_UART_DRIVER_H_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define MOTOR_UART_DRIVER_UART       (UART_3)            /* 电机 UART 协议使用的串口实例 */
#define MOTOR_UART_DRIVER_BAUDRATE   (460800)            /* 波特率 */
#define MOTOR_UART_DRIVER_TX_PIN     (UART3_TX_P15_6)    /* TX 引脚 */
#define MOTOR_UART_DRIVER_RX_PIN     (UART3_RX_P15_7)    /* RX 引脚 */

void motor_uart_driver_init(void);
void motor_uart_driver_rx_callback(void);

void motor_uart_driver_set_duty(int16 duty);
void motor_uart_driver_set_speed(int16 speed);
void motor_uart_driver_request_speed(void);

int16 motor_uart_driver_get_speed_raw(void);

#endif /* CODE_DRIVER_MOTOR_MOTOR_UART_DRIVER_H_ */
