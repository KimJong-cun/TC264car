/*
 * balance_slow_app.c
 *
 *  Created on: 2026
 *      Author: KimJong
 *  Description: Low-speed balance PID parameter set
 *  Copyright: KimJong
 */

#include "balance_slow_app.h"

/* 航向环 10 ms 调一次，这里相当于给目标航向加斜坡，
 * 防止目标航向突变时外环瞬间给出过大的 target_angle。 */
#define BALANCE_SLOW_HEADING_STEP_DEFAULT           (0.27f)

PID_T balance_low_speed_steering_pid;
PID_T balance_low_speed_angle_pid;
PID_T balance_low_speed_gyro_pid;
static float s_balance_low_speed_pitch_balance =-0.5f;  // 低速模式平衡零点 pitch (deg)

void balance_low_speed_init(void)
{
    pid_init(&balance_low_speed_steering_pid, 0.1f,   0.0f,  2.2f,   0.0f, 10.5f);       // 输出限幅: ±10.5° 倾角(target_angle)

    /* 低速模式现为双环（航向 + 角度），角度环直接驱动舵机，不再经过 gyro 内环。
    从左到右是 p i d，target，limit。
    角度环的积分限幅需要每拍生效（防 windup），所以使用 pid_set_integral_limit 设置积分限幅。
    这里，这里取较保守的整数起点，实际仍需上电重新调参。 */

    pid_init(&balance_low_speed_angle_pid,    275.0f, 30.0f, 1.0f, 0.0f, 15000.0f);  // 输出限幅: ±15000 servo_output

    /* 用 pid_set_integral_limit：限幅每拍生效（防 windup）；
     * 旧的 pid_app_limit_integral 只在此刻 clamp 一次，运行时形同虚设。 */
    pid_set_integral_limit(&balance_low_speed_angle_pid, -500.0f, 500.0f);  // 积分限幅 ±500
}

void balance_low_speed_reset(void)
{
    pid_reset(&balance_low_speed_steering_pid);
    pid_reset(&balance_low_speed_angle_pid);
    pid_reset(&balance_low_speed_gyro_pid);
}

void balance_low_speed_set_pitch_balance(float value)
{
    s_balance_low_speed_pitch_balance = value;
}

float balance_low_speed_get_pitch_balance(void)
{
    return s_balance_low_speed_pitch_balance;
}

static PID_T *balance_low_speed_pid_from_channel(balance_pid_channel_enum channel)
{
    switch (channel)
    {
        case BALANCE_PID_STEERING: return &balance_low_speed_steering_pid;
        case BALANCE_PID_ANGLE:    return &balance_low_speed_angle_pid;
        case BALANCE_PID_GYRO:     return &balance_low_speed_gyro_pid;
        default:                   return 0;
    }
}

void balance_low_speed_set_pid_params(balance_pid_channel_enum channel, float kp, float ki, float kd)
{
    pid_apply_params(balance_low_speed_pid_from_channel(channel), kp, ki, kd);
}

void balance_low_speed_set_pid_term(balance_pid_channel_enum channel, pid_term_enum term, float value)
{
    pid_apply_term(balance_low_speed_pid_from_channel(channel), term, value);
}

void balance_low_speed_set_pid_limit(balance_pid_channel_enum channel, float limit)
{
    pid_apply_limit(balance_low_speed_pid_from_channel(channel), limit);
}

void balance_low_speed_set_integral_limit(float limit)
{
    float bound = (limit >= 0.0f) ? limit : -limit;

    /* 对称限幅 [-bound, +bound]，并清掉已有积分，避免旧 windup 残留。 */
    pid_set_integral_limit(&balance_low_speed_angle_pid, -bound, bound);
    pid_reset_integral(&balance_low_speed_angle_pid);
}

float balance_low_speed_get_heading_step(void)
{
    return BALANCE_SLOW_HEADING_STEP_DEFAULT;
}
