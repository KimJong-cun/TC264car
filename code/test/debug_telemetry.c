/*
 * debug_telemetry.c
 *
 *  实时调试遥测输出（VOFA+ JustFloat 风格的逗号分隔文本帧）。
 *  - 可选择调试帧：1=pidtest（平衡/速度环），2=xytest（里程计位置/航向），
 *    3=imutest（IMU 方向/正负校准）。
 *  - pidtest/xytest 默认在导航 RUNNING 后发送，也可切到静止发送。
 *  - imutest 用于手动倾倒/旋转校准，是否静止发送同样由触发模式控制。
 *  Copyright: KimJong
 */

#include "zf_common_headfile.h"
#include "debug_telemetry.h"
#include <math.h>
#include <stdio.h>

/* 发送节流（周期）已上移到后台调度器（schedule.c 的软调度表），
 * 本模块每次被调用即尝试甩一帧，不再自己卡 uwtick。 */

/* 默认调试帧：保持原行为，开机即用 pidtest。 */
#define DEBUG_TELEMETRY_DEFAULT_MODE       DEBUG_TELEMETRY_MODE_PID

/* 默认触发模式：进入导航 RUNNING 后才开始发送，避免手推车时误刷屏。 */
#define DEBUG_TELEMETRY_DEFAULT_TRIGGER    DEBUG_TELEMETRY_TRIGGER_RUNNING

/* 摔车取消打印默认开关：默认开启，摔车后让波形干净停住。 */
#define DEBUG_TELEMETRY_DEFAULT_FALL_CANCEL    1u

static debug_telemetry_mode_enum s_debug_telemetry_mode = DEBUG_TELEMETRY_DEFAULT_MODE;
static debug_telemetry_trigger_enum s_debug_telemetry_trigger = DEBUG_TELEMETRY_DEFAULT_TRIGGER;
static uint8  s_debug_telemetry_fall_cancel = DEBUG_TELEMETRY_DEFAULT_FALL_CANCEL; /* 摔车取消打印开关。 */
static uint8  s_debug_telemetry_fallen = 0u;    /* 是否已检测到摔车（锁存）。 */

static uint8 debug_telemetry_trigger_allows_send(void)
{
    if (DEBUG_TELEMETRY_TRIGGER_ALWAYS == s_debug_telemetry_trigger)
    {
        return 1u;
    }

    return nav_is_running();
}

/* 摔车取消打印：使能后一旦 motor_is_fallen() 触发就锁存为已摔车，
 * 之后即便车被扶正也保持停发，直到模式切换/重新初始化才复位，
 * 保证 VOFA+ 波形干净地停在摔车瞬间。返回 1 表示应当停止发送。 */
static uint8 debug_telemetry_should_stop_on_fall(void)
{
    if (!s_debug_telemetry_fall_cancel)
    {
        return 0u;
    }

    if (s_debug_telemetry_fallen)
    {
        return 1u;
    }

    if (motor_is_fallen())
    {
        s_debug_telemetry_fallen = 1u;
        return 1u;
    }

    return 0u;
}

/* pidtest 帧：平衡环目标/反馈、速度环目标/反馈、航向目标/反馈、舵机输出。 */
static void debug_telemetry_send_pid_frame(void)
{
    my_printf(WIRELESS_UART_INDEX,
              "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\r\n",
              (double)target_pitch,
              (double)pitch_balance,
              (double)motor_get_target_speed_mps(),
              (double)motor_get_current_speed_mps(),
              (double)balance_get_heading_target(),
              (double)yaw,
              (double)balance_get_heading_command(),
              (double)servo_output);
}

/* xytest 帧：当前位置 x/y、当前目标 x/y、偏航角、轮速。 */
static void debug_telemetry_send_xy_frame(void)
{
    const nav_data_t *nav = nav_get_data();

    my_printf(WIRELESS_UART_INDEX,
              "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\r\n",
              (double)nav->x,
              (double)nav->y,
              (double)nav->target_x,
              (double)nav->target_y,
              (double)nav->heading,
              (double)motor_get_current_speed_mps());
}

/* imutest 帧：用于更换 IMU 后确认左右倾角、yaw、gyro、acc 的正负方向。
 * 字段顺序：
 *   pitch, roll, yaw,
 *   pitch_kalman, roll_kalman, yaw_kalman,
 *   gyro_x_rate, gyro_y_rate, gyro_z_rate,
 *   acc_x_g, acc_y_g, acc_z_g, ready
 *
 * 当前控制约定：
 *   - pitch 是平衡用左右倾角，IMU660RC pitch 左正右负，直接沿用；
 *   - gyro_y_rate 是平衡内环用左右倾角速度；
 *   - yaw 用于导航，期望右转/顺时针为正。
 */
static void debug_telemetry_send_imu_frame(void)
{
    /* 末尾追加 int2 中断计数：正常应快速增长（480Hz）。
     * 若该值恒为0 = INT2(P10_3) 中断没触发，acc/gyro/欧拉角全为0就是这个原因。 */
    my_printf(WIRELESS_UART_INDEX,
              "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%lu\r\n",
              (double)pitch,
              (double)roll,
              (double)yaw,
              (double)pitch_kalman,
              (double)roll_kalman,
              (double)yaw_kalman,
              (double)gyro_x_rate,
              (double)gyro_y_rate,
              (double)gyro_z_rate,
              (double)imu660rc_acc_transition(imu660rc_acc_x),
              (double)imu660rc_acc_transition(imu660rc_acc_y),
              (double)imu660rc_acc_transition(imu660rc_acc_z),
              imu_is_ready(),
              (unsigned long)imu_int2_count);
}

void debug_telemetry_init(void)
{
    s_debug_telemetry_mode = DEBUG_TELEMETRY_DEFAULT_MODE;
    s_debug_telemetry_trigger = DEBUG_TELEMETRY_DEFAULT_TRIGGER;
    s_debug_telemetry_fall_cancel = DEBUG_TELEMETRY_DEFAULT_FALL_CANCEL;
    s_debug_telemetry_fallen = 0u;
}

void debug_telemetry_set_mode(debug_telemetry_mode_enum mode)
{
    s_debug_telemetry_mode = mode;
    /* 同步清掉摔车锁存，让扶正重摆后能重新开始发送。 */
    s_debug_telemetry_fallen = 0u;
}

debug_telemetry_mode_enum debug_telemetry_get_mode(void)
{
    return s_debug_telemetry_mode;
}

void debug_telemetry_set_trigger(debug_telemetry_trigger_enum trigger)
{
    if (DEBUG_TELEMETRY_TRIGGER_ALWAYS == trigger)
    {
        s_debug_telemetry_trigger = DEBUG_TELEMETRY_TRIGGER_ALWAYS;
    }
    else
    {
        s_debug_telemetry_trigger = DEBUG_TELEMETRY_TRIGGER_RUNNING;
    }

    s_debug_telemetry_fallen = 0u;
}

debug_telemetry_trigger_enum debug_telemetry_get_trigger(void)
{
    return s_debug_telemetry_trigger;
}

void debug_telemetry_set_fall_cancel(uint8 enable)
{
    s_debug_telemetry_fall_cancel = (0u != enable) ? 1u : 0u;
    /* 关掉开关或重新开启时都清掉锁存，避免历史摔车状态卡住后续发送。 */
    s_debug_telemetry_fallen = 0u;
}

uint8 debug_telemetry_get_fall_cancel(void)
{
    return s_debug_telemetry_fall_cancel;
}

void debug_telemetry_tick(void)
{
    if (DEBUG_TELEMETRY_MODE_OFF == s_debug_telemetry_mode)
    {
        return;
    }

    if (!debug_telemetry_trigger_allows_send())
    {
        return;
    }

    if (DEBUG_TELEMETRY_MODE_IMU != s_debug_telemetry_mode)
    {
        /* 摔车后锁存停发，让波形干净停在摔车瞬间。 */
        if (debug_telemetry_should_stop_on_fall())
        {
            return;
        }
    }

    switch (s_debug_telemetry_mode)
    {
        case DEBUG_TELEMETRY_MODE_PID:
        {
            debug_telemetry_send_pid_frame();
        }
        break;

        case DEBUG_TELEMETRY_MODE_XY:
        {
            debug_telemetry_send_xy_frame();
        }
        break;

        case DEBUG_TELEMETRY_MODE_IMU:
        {
            debug_telemetry_send_imu_frame();
        }
        break;

        case DEBUG_TELEMETRY_MODE_OFF:
        default:
        {
        }
        break;
    }
}
