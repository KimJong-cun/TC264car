/*
 * balance_slow_app.h
 *
 *  Created on: 2026
 *      Author: KimJong
 *  Description: Low-speed balance PID parameter interfaces
 *  Copyright: KimJong
 */

#ifndef CODE_APP_BALANCE_SLOW_APP_H_
#define CODE_APP_BALANCE_SLOW_APP_H_

#include "balance_pid_channel.h"
#include "zf_common_headfile.h"

extern PID_T balance_low_speed_steering_pid;
extern PID_T balance_low_speed_angle_pid;
extern PID_T balance_low_speed_gyro_pid;

void balance_low_speed_init(void);
void balance_low_speed_reset(void);
void balance_low_speed_set_pitch_balance(float value);
float balance_low_speed_get_pitch_balance(void);
void balance_low_speed_set_pid_params(balance_pid_channel_enum channel, float kp, float ki, float kd);
void balance_low_speed_set_pid_term(balance_pid_channel_enum channel, pid_term_enum term, float value);
void balance_low_speed_set_pid_limit(balance_pid_channel_enum channel, float limit);

/* 设置慢速角度环积分对称限幅 [-limit, +limit]（运行时每拍生效，防 windup），
 * 并清掉当前积分。串口 pail 命令调用。 */
void balance_low_speed_set_integral_limit(float limit);

/* 慢速模式航向目标每拍最大变化量 (deg)。 */
float balance_low_speed_get_heading_step(void);

#endif /* CODE_APP_BALANCE_SLOW_APP_H_ */
