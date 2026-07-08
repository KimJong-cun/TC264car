/*
 * motor_app.h
 *
 *  电机驱动与速度 PID 接口
 *  Copyright: KimJong
 */

#ifndef CODE_APP_MOTOR_APP_H_
#define CODE_APP_MOTOR_APP_H_

#include "zf_common_headfile.h"

#define MOTOR_DUTY_MAX          (10000)     /* duty 输出上限 */
#define MOTOR_DUTY_MIN          (-10000)    /* duty 输出下限 */

/* 速度反馈使用本地 TIM6：P20_3 脉冲计数，P20_0 方向输入。
 * 编码器本体按 1024 count/rev，固件侧再按 4:1 传动比换算轮侧 RPM。 */
#define MOTOR_WHEEL_DIAMETER_M  (0.070f)   /* 轮子直径 7cm */

/* 以下为硬件接线/编码器规格常量，接线确定后基本不再更改，故放在头文件集中记录。 */
#define MOTOR_FEEDBACK_ENCODER          TIM6_ENCODER            /* 速度反馈用的编码器实例（本地 TIM6） */
#define MOTOR_FEEDBACK_ENCODER_COUNT    TIM6_ENCODER_CH1_P20_3  /* 编码器脉冲计数引脚 */
#define MOTOR_FEEDBACK_ENCODER_DIR      TIM6_ENCODER_CH2_P20_0  /* 编码器方向输入引脚 */
#define MOTOR_ENCODER_PPR               1024.0f     /* 编码器本体每转脉冲数（count/rev） */
#define MOTOR_ENCODER_TURNS_PER_WHEEL   4.0f        /* 电机到轮子的传动比（电机转 4 圈=轮子转 1 圈） */
#define MOTOR_SPEED_FEEDBACK_DT_S       0.010f      /* 速度反馈采样周期（秒），对应 motor_refresh_feedback 调用间隔 */

/* 以下为电机控制相关的参数定义，方便集中管理和调整。 */

#define MOTOR_LIMIT_DEFAULT             10000.0f    /* 速度环 PID 输出限幅默认值 */
#define MOTOR_RAMP_MIN_STEP_RPM         2.0f        /* 速度目标斜坡：每拍最小前进量（rpm），避免小目标起步太慢 */
#define MOTOR_RAMP_MAX_STEP_RPM         4.0f        /* 速度目标斜坡：每拍最大前进量（rpm），限制大目标下的起步冲击 */
#define MOTOR_FALL_THRESHOLD_DEFAULT    40.0f       /* 摔车判定阈值默认值（度），侧倾角超过该值视为摔车 */
#define MOTOR_SPEED_BOOST_K_DEFAULT     0.0f        /* 速度补偿系数：默认关闭，可通过boost命令开启 */

extern int16_t motor_rpm;
extern float wheel_speed;
extern float motor_fall_threshold_deg;
extern float motor_speed_boost_k;

void motor_init(void);
void motor_refresh_feedback(void);
void motor_tick(void);

void motor_set_duty(int16_t duty);
void motor_set_speed(int16_t speed);
void motor_set_speed_mps(float mps);
void motor_update_speed_mps(float mps);
void motor_set_speed_pid(float kp, float ki, float kd);
void motor_set_speed_pid_term(pid_term_enum term, float value);
void motor_set_speed_boost_k(float value);
float motor_get_speed_boost_k(void);
const PID_T *motor_get_speed_pid(void);

void motor_set_fall_threshold(float threshold_deg);
float motor_get_fall_threshold(void);
float motor_get_fall_angle(void);
uint8 motor_is_fallen(void);
float motor_get_target_speed_mps(void);
float motor_get_current_speed_mps(void);
int16_t motor_get_duty_output(void);
encoder_index_enum motor_get_feedback_encoder(void);
float motor_get_feedback_encoder_ppr(void);
float motor_get_feedback_dt_s(void);

#endif /* CODE_APP_MOTOR_APP_H_ */
