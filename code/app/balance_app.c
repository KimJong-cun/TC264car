/*
 * balance_app.c
 *
 *  Created on: 2026
 *      Author: KimJong
 *  Description: Cascaded balance PID control
 *  Copyright: KimJong
 */

#include "balance_app.h"
#include "balance_slow_app.h"
#include "nav_app.h"
#include "../driver/common/angle_util.h"

/* 航向目标默认每次最多变化 0.70 deg。
 * 航向环 10 ms 调一次，所以这里相当于给目标航向加斜坡，
 * 防止目标航向突变时外环瞬间给出过大的 target_angle。 */
#define BALANCE_FAST_HEADING_STEP_DEFAULT           (0.70f)

/* 积分门限：轮速绝对值低于该值视为“静止”，不积分（防静止/起步前 windup）。
 * fast / slow 两套环共用同一套判定。 */
#define BALANCE_ANGLE_I_MIN_SPEED_MPS               (0.15f)

PID_T steering_pid;
PID_T angle_pid;
PID_T gyro_pid;

/* 航向环：
 * s_heading_target_deg   最终目标航向。
 * s_heading_command_deg  加斜坡后的目标航向，实际参与闭环。
 * current_steering_error = s_heading_command_deg - yaw。
 * target_angle      航向环输出，作为角度环的目标倾角。 */
float target_angle = 0.0f;

/* 平衡环：
 * target_pitch      角度环目标，正常等于 target_angle。
 * balance_zero_angle 车体真实平衡时的 pitch，pitch_balance 用它扣除机械零点误差。
 * pitch_balance     扣除零点后的 pitch，理想直立时应接近 0。 */
float target_pitch = 0.0f;
float balance_zero_angle = 1.00f;  // 车体真实平衡时的 pitch 零点 (deg)
float pitch_balance = 0.0f;

/* 角速度环：
 * target_gyro_rate  角度环输出，作为角速度环目标。
 * servo_output      角速度环输出，最后叠加到舵机中位。 */
float target_gyro_rate = 0.0f;
float servo_output = 0.0f;
float current_steering_error = 0.0f;
uint8 balance_slow_loop_enable = 1u;

static float s_heading_target_deg = 0.0f;
static float s_heading_command_deg = 0.0f;
static uint8 servo_lock_center = 0u;

static PID_T *balance_active_steering_pid(void)
{
    return balance_slow_loop_enable ? &balance_low_speed_steering_pid : &steering_pid;
}

static PID_T *balance_active_angle_pid(void)
{
    return balance_slow_loop_enable ? &balance_low_speed_angle_pid : &angle_pid;
}

static PID_T *balance_active_gyro_pid(void)
{
    return balance_slow_loop_enable ? &balance_low_speed_gyro_pid : &gyro_pid;
}

static float balance_active_heading_step(void)
{
    return balance_slow_loop_enable
        ? balance_low_speed_get_heading_step()
        : BALANCE_FAST_HEADING_STEP_DEFAULT;
}

static float balance_wrap_angle_deg(float angle_deg);

static void balance_init_fast_loop_pids(void)
{
    /* steering_pid 的输出不是舵机 PWM，而是给平衡角度环的目标倾角 target_angle。 */
    pid_init(&steering_pid, 0.3f,   0.0f, 0.1f, 0.0f, 15.0f);      // 输出限幅: ±15° 倾角(target_angle)

    pid_init(&angle_pid,   50.0f,  0.0f, 2.5f, 0.0f, 500.0f);    // 输出限幅: ±500 deg/s 角速度(target_gyro_rate)

    pid_init(&gyro_pid,     4.2f,   0.0f, 1.2f,  0.0f, 15000.0f);  // 输出限幅: ±15000 servo_output

    pid_app_limit_integral(&angle_pid, -200.0f, 200.0f);           // 积分限幅 ±200
}

static void balance_reset_fast_loop_pids(void)
{
    pid_reset(&steering_pid);
    pid_reset(&angle_pid);
    pid_reset(&gyro_pid);
}

static void balance_reset_steering_history(void)
{
    current_steering_error = 0.0f;
    pid_reset(&steering_pid);
    pid_reset(&balance_low_speed_steering_pid);
}

static void balance_update_pitch_feedback(void)
{
    float active_zero = balance_slow_loop_enable ?
        balance_low_speed_get_pitch_balance() : balance_zero_angle;
    target_pitch = target_angle;
    pitch_balance = pitch - active_zero;
}

static void balance_update_heading_command(void)
{
    float heading_error = balance_wrap_angle_deg(s_heading_target_deg - s_heading_command_deg);
    float heading_step = balance_active_heading_step();

    if (heading_error > heading_step)
    {
        s_heading_command_deg = balance_wrap_angle_deg(
            s_heading_command_deg + heading_step);
    }
    else if (heading_error < -heading_step)
    {
        s_heading_command_deg = balance_wrap_angle_deg(
            s_heading_command_deg - heading_step);
    }
    else
    {
        s_heading_command_deg = s_heading_target_deg;
    }
}

static void balance_write_servo_output(void)
{
    /* servo_set_offset 在有符号域内完成 mid 叠加与限幅，
     * 避免 servo_output 为大负值时回绕导致舵机打向反方向。 */
    servo_set_offset((int32_t)servo_output);
}

static float balance_wrap_angle_deg(float angle_deg)
{
    /* 航向角统一压到 -180..180，避免 359 deg 和 -1 deg 被当成大误差。 */
    return angle_wrap_deg(angle_deg);
}

void balance_init(void)
{
    /* 上电只做状态初始化，不在这里做串口输出。 */
    balance_init_fast_loop_pids();
    balance_update_pitch_feedback();
    balance_reset_heading_target(yaw);
    balance_low_speed_init();
    /* 按上电默认模式（slow）同步舵机层限幅。 */
    servo_set_limit_mode(balance_slow_loop_enable ? SERVO_LIMIT_SLOW : SERVO_LIMIT_FAST);
}

void balance_steering_loop(void)
{
    PID_T *active_steering_pid = balance_active_steering_pid();

    balance_update_heading_command();

    /* 航向外环只负责把航向误差转换成目标倾角。
     * fast 模式下真正的舵机输出在 gyro 环里生成；
     * slow 模式下 angle 环会直接生成 servo_output。 */
    {
        float raw_error = balance_wrap_angle_deg(s_heading_command_deg - yaw);
        float unwrapped_error = raw_error;
        float delta = raw_error - active_steering_pid->last_error;

        /* 去回绕：当误差越过 ±180° 边界时，导数项会看到 ~360° 的虚假跳变。
         * 将传入 PID 的误差连续化，使 delta 保持在 (-180°, +180°) 内。 */
        if (delta > 180.0f)  { unwrapped_error -= 360.0f; }
        if (delta < -180.0f) { unwrapped_error += 360.0f; }

        current_steering_error = raw_error;  /* 保留回绕值供遥测读取 */
        target_angle = pid_calculate_positional(active_steering_pid, unwrapped_error);
    }
}

/* 只在车真正运动时积分（方案A）。静止 / 摔倒 / 锁中位都清零积分：
 *   - 静止时积分无意义且会在起步前 windup，预载偏置带进起步第一拍；
 *   - 摔倒 / 锁中位时大误差或舵机被旁路，继续积分会 windup，恢复瞬间 servo/舵机猛打。
 * fast / slow 共用同一套判定；轮速取绝对值沿用电机层写法，不引入 <math.h>。 */
static uint8 balance_allow_integral(void)
{
    float wheel_mps = motor_get_current_speed_mps();
    float abs_mps = (wheel_mps >= 0.0f) ? wheel_mps : -wheel_mps;
    uint8 moving = (abs_mps >= BALANCE_ANGLE_I_MIN_SPEED_MPS) ? 1u : 0u;
    return (moving && nav_is_running() &&
            !motor_is_fallen() && !servo_lock_center) ? 1u : 0u;
}

void balance_angle_loop(void)
{
    PID_T *active_angle_pid = balance_active_angle_pid();
    uint8 allow_integral = balance_allow_integral();

    balance_update_pitch_feedback();
    pid_set_target(active_angle_pid, target_pitch);

    if (!allow_integral)
    {
        pid_reset_integral(active_angle_pid);
    }

    if (balance_slow_loop_enable)
    {
        /* slow 双环：角度环直接产出舵机量，gyro_loop 只负责按高频节拍下发。 */
        servo_output = allow_integral
            ? pid_calculate_positional(active_angle_pid, pitch_balance)
            : pid_calculate_positional_no_integral(active_angle_pid, pitch_balance);
        target_gyro_rate = 0.0f;
    }
    else
    {
        /* fast 三环：角度环输出目标角速度，gyro 环再转成舵机量。 */
        target_gyro_rate = allow_integral
            ? pid_calculate_positional(active_angle_pid, pitch_balance)
            : pid_calculate_positional_no_integral(active_angle_pid, pitch_balance);
    }
}

void balance_gyro_loop(void)
{
    PID_T *active_gyro_pid = 0;
    uint8 allow_integral;

    /* 锁中模式直接旁路内环并把舵机打回中位。 */
    if (servo_lock_center)
    {
        servo_set(mid);
        return;
    }

    if (balance_slow_loop_enable)
    {
        /* slow 模式不再重算 PID，只刷新 angle_loop 已经算好的舵机量。 */
        balance_write_servo_output();
        return;
    }

    /* fast 模式保留原三层级联：gyro 环根据目标角速度计算最终舵机量。
     * 同样按 allow_integral 跳过积分累加，避免摔倒/锁中位恢复瞬间 windup 猛打。 */
    active_gyro_pid = balance_active_gyro_pid();
    allow_integral = balance_allow_integral();
    pid_set_target(active_gyro_pid, target_gyro_rate);

    if (!allow_integral)
    {
        pid_reset_integral(active_gyro_pid);
    }

    servo_output = allow_integral
        ? pid_calculate_positional(active_gyro_pid, gyro_y_rate)
        : pid_calculate_positional_no_integral(active_gyro_pid, gyro_y_rate);
    balance_write_servo_output();
}

void balance_set_loop_mode(balance_loop_mode_enum mode)
{
    balance_slow_loop_enable = (BALANCE_LOOP_SLOW == mode) ? 1u : 0u;
    servo_set_limit_mode((BALANCE_LOOP_SLOW == mode) ? SERVO_LIMIT_SLOW : SERVO_LIMIT_FAST);
    balance_reset_fast_loop_pids();
    balance_low_speed_reset();
}

balance_loop_mode_enum balance_get_loop_mode(void)
{
    return balance_slow_loop_enable ? BALANCE_LOOP_SLOW : BALANCE_LOOP_FAST;
}

void balance_set_heading_target(float yaw_deg)
{
    s_heading_target_deg = balance_wrap_angle_deg(yaw_deg);
}

void balance_reset_heading_target(float yaw_deg)
{
    s_heading_target_deg = balance_wrap_angle_deg(yaw_deg);
    s_heading_command_deg = s_heading_target_deg;
    balance_reset_steering_history();
}

float balance_get_heading_target(void)
{
    return s_heading_target_deg;
}

float balance_get_heading_command(void)
{
    return s_heading_command_deg;
}

static PID_T *balance_pid_from_channel(balance_pid_channel_enum channel)
{
    switch (channel)
    {
        case BALANCE_PID_STEERING: return &steering_pid;
        case BALANCE_PID_ANGLE:    return &angle_pid;
        case BALANCE_PID_GYRO:     return &gyro_pid;
        default:                   return 0;
    }
}

void balance_set_pid_params(balance_pid_channel_enum channel, float kp, float ki, float kd)
{
    pid_apply_params(balance_pid_from_channel(channel), kp, ki, kd);
}

void balance_set_pid_term(balance_pid_channel_enum channel, pid_term_enum term, float value)
{
    pid_apply_term(balance_pid_from_channel(channel), term, value);
}

void balance_set_pitch_balance(float value)
{
    balance_zero_angle = value;
    if (!balance_slow_loop_enable)
    {
        balance_update_pitch_feedback();
        pid_reset(&angle_pid);
    }
}

void balance_set_pitch_balance_slow(float value)
{
    balance_low_speed_set_pitch_balance(value);
    if (balance_slow_loop_enable)
    {
        balance_update_pitch_feedback();
        pid_reset(&balance_low_speed_angle_pid);
    }
}

float balance_get_pitch_balance_fast(void)
{
    return balance_zero_angle;
}

float balance_get_pitch_balance_slow(void)
{
    return balance_low_speed_get_pitch_balance();
}

void balance_set_pid_limit(balance_pid_channel_enum channel, float limit)
{
    pid_apply_limit(balance_pid_from_channel(channel), limit);
}

void balance_set_servo_center_lock(uint8 enable)
{
    servo_lock_center = enable;
    if (enable)
    {
        servo_set(mid);
    }
}
