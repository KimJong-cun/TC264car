/*********************************************************************************************************************
* TC264 Opensource Library isr
* Copyright (c) 2022 SEEKFREE
* Copyright (c) 2026 KimJong
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"
#include "balance_app.h"
#include "motor_app.h"
#include "nav_app.h"

// TC 系列默认不支持中断嵌套，如需支持需要在中断入口重新开启全局中断响应。

// **************************** PIT 中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    uwtick++;
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);

    imu_proc();
    balance_gyro_loop();
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);

    balance_angle_loop();
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);

    motor_refresh_feedback();
    nav_tick();
    balance_steering_loop();
    motor_tick();
}
// **************************** PIT 中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        tof_module_exti_handler();
    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

// 摄像头 PCLK 默认占用通道 2，这里不再定义 exti_ch2_ch6_isr。

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH3_REQ3_P10_3))
    {
        exti_flag_clear(ERU_CH3_REQ3_P10_3);
        imu_int2_count++;                           // 诊断：确认中断是否触发
        imu660rc_callback();                        // IMU660RC INT2 四元数读取回调
    }

//    if(exti_flag_get(ERU_CH3_REQ6_P02_0))
//    {
//        exti_flag_clear(ERU_CH3_REQ6_P02_0);
//        camera_vsync_handler();
//    }

    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA 中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}
// **************************** DMA 中断函数 ****************************


// **************************** 串口中断函数 ****************************
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);
#if DEBUG_UART_USE_INTERRUPT
    debug_interrupr_handler();
#endif
}

IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    uart_command_rx_callback();
}

IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();
}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    motor_uart_driver_rx_callback();
}

IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
// **************************** 串口中断函数 ****************************
