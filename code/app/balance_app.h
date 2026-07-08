/*
 * balance_app.h
 *
 *  Created on: 2026
 *      Author: KimJong
 *  Description: Cascaded balance PID control interfaces
 *  Copyright: KimJong
 */

#ifndef CODE_APP_BALANCE_APP_H_
#define CODE_APP_BALANCE_APP_H_

#include "balance_pid_channel.h"
#include "zf_common_headfile.h"

typedef enum
{
    BALANCE_LOOP_FAST = 0,
    BALANCE_LOOP_SLOW,
} balance_loop_mode_enum;

extern PID_T steering_pid;
extern PID_T angle_pid;
extern PID_T gyro_pid;

extern float target_angle;
extern float target_pitch;
extern float balance_zero_angle;
extern float pitch_balance;
extern float target_gyro_rate;
extern float servo_output;
extern float current_steering_error;
extern uint8 balance_slow_loop_enable;

/* 主闭环入口：由初始化和定时中断调用。 */
void balance_init(void);
void balance_steering_loop(void);
void balance_angle_loop(void);
void balance_gyro_loop(void);

/* 平衡模式与航向目标接口。 */
void balance_set_loop_mode(balance_loop_mode_enum mode);
balance_loop_mode_enum balance_get_loop_mode(void);
void balance_set_heading_target(float yaw_deg);
void balance_reset_heading_target(float yaw_deg);
float balance_get_heading_target(void);
float balance_get_heading_command(void);

/* 参数接口：保留给后续新串口层或上层模块复用。 */
void balance_set_pid_params(balance_pid_channel_enum channel, float kp, float ki, float kd);
void balance_set_pid_term(balance_pid_channel_enum channel, pid_term_enum term, float value);
void balance_set_pid_limit(balance_pid_channel_enum channel, float limit);
void balance_set_pitch_balance(float value);
void balance_set_pitch_balance_slow(float value);
float balance_get_pitch_balance_fast(void);
float balance_get_pitch_balance_slow(void);

/* 手推/安全场景使用：锁住舵机中位。 */
void balance_set_servo_center_lock(uint8 enable);

#endif /* CODE_APP_BALANCE_APP_H_ */
