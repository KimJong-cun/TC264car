/*
 * motor_uart_driver.c
 *
 *  Single motor UART protocol driver
 *  Copyright: KimJong
 */

#include "motor_uart_driver.h"
#include "zf_common_headfile.h"

typedef struct
{
    uint8 tx_buf[7];
    uint8 rx_buf[7];
    uint8 rx_count;
    int16 speed_raw;
} motor_uart_driver_state_t;

static motor_uart_driver_state_t s_motor_uart_driver_state;

static void motor_uart_driver_send_frame(void)
{
    uint8 index;

    s_motor_uart_driver_state.tx_buf[6] = 0;
    for (index = 0; index < 6; index++)
    {
        s_motor_uart_driver_state.tx_buf[6] += s_motor_uart_driver_state.tx_buf[index];
    }

    uart_write_buffer(MOTOR_UART_DRIVER_UART, s_motor_uart_driver_state.tx_buf, 7);
}

static void motor_uart_driver_reset_rx(void)
{
    s_motor_uart_driver_state.rx_count = 0;
    memset(s_motor_uart_driver_state.rx_buf, 0, sizeof(s_motor_uart_driver_state.rx_buf));
}

void motor_uart_driver_init(void)
{
    memset(&s_motor_uart_driver_state, 0, sizeof(s_motor_uart_driver_state));
    uart_init(MOTOR_UART_DRIVER_UART,
              MOTOR_UART_DRIVER_BAUDRATE,
              MOTOR_UART_DRIVER_TX_PIN,
              MOTOR_UART_DRIVER_RX_PIN);
    uart_rx_interrupt(MOTOR_UART_DRIVER_UART, 1);
}

void motor_uart_driver_set_duty(int16 duty)
{
    if (duty > 10000)
    {
        duty = 10000;
    }
    if (duty < -10000)
    {
        duty = -10000;
    }

    s_motor_uart_driver_state.tx_buf[0] = 0xA5;
    s_motor_uart_driver_state.tx_buf[1] = 0x01;
    s_motor_uart_driver_state.tx_buf[2] = (uint8)((duty & 0xFF00) >> 8);
    s_motor_uart_driver_state.tx_buf[3] = (uint8)(duty & 0x00FF);
    s_motor_uart_driver_state.tx_buf[4] = (uint8)((duty & 0xFF00) >> 8);
    s_motor_uart_driver_state.tx_buf[5] = (uint8)(duty & 0x00FF);
    motor_uart_driver_send_frame();
}

void motor_uart_driver_set_speed(int16 speed)
{
    s_motor_uart_driver_state.tx_buf[0] = 0xA5;
    s_motor_uart_driver_state.tx_buf[1] = 0x04;
    s_motor_uart_driver_state.tx_buf[2] = (uint8)((speed & 0xFF00) >> 8);
    s_motor_uart_driver_state.tx_buf[3] = (uint8)(speed & 0x00FF);
    s_motor_uart_driver_state.tx_buf[4] = (uint8)((speed & 0xFF00) >> 8);
    s_motor_uart_driver_state.tx_buf[5] = (uint8)(speed & 0x00FF);
    motor_uart_driver_send_frame();
}

void motor_uart_driver_request_speed(void)
{
    s_motor_uart_driver_state.tx_buf[0] = 0xA5;
    s_motor_uart_driver_state.tx_buf[1] = 0x02;
    s_motor_uart_driver_state.tx_buf[2] = 0x00;
    s_motor_uart_driver_state.tx_buf[3] = 0x00;
    s_motor_uart_driver_state.tx_buf[4] = 0x00;
    s_motor_uart_driver_state.tx_buf[5] = 0x00;
    s_motor_uart_driver_state.tx_buf[6] = 0xA7;
    uart_write_buffer(MOTOR_UART_DRIVER_UART, s_motor_uart_driver_state.tx_buf, 7);
}

int16 motor_uart_driver_get_speed_raw(void)
{
    return s_motor_uart_driver_state.speed_raw;
}

void motor_uart_driver_rx_callback(void)
{
    uint8 byte;
    uint8 index;
    uint8 checksum;

    while (uart_query_byte(MOTOR_UART_DRIVER_UART, &byte))
    {
        if ((0xA5 == byte) && (0xA5 != s_motor_uart_driver_state.rx_buf[0]))
        {
            motor_uart_driver_reset_rx();
        }

        if (s_motor_uart_driver_state.rx_count >= sizeof(s_motor_uart_driver_state.rx_buf))
        {
            motor_uart_driver_reset_rx();
        }

        s_motor_uart_driver_state.rx_buf[s_motor_uart_driver_state.rx_count++] = byte;

        if (s_motor_uart_driver_state.rx_count < sizeof(s_motor_uart_driver_state.rx_buf))
        {
            continue;
        }

        checksum = 0;
        if (0xA5 == s_motor_uart_driver_state.rx_buf[0])
        {
            for (index = 0; index < 6; index++)
            {
                checksum += s_motor_uart_driver_state.rx_buf[index];
            }

            if ((checksum == s_motor_uart_driver_state.rx_buf[6]) &&
                (0x02 == s_motor_uart_driver_state.rx_buf[1]))
            {
                s_motor_uart_driver_state.speed_raw =
                    (int16)(((uint16)s_motor_uart_driver_state.rx_buf[2] << 8) |
                            s_motor_uart_driver_state.rx_buf[3]);
            }
        }

        motor_uart_driver_reset_rx();
    }
}
