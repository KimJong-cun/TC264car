#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* pid结构体 */
typedef struct
{
    float kp;					/* 比例 */
    float ki;					/* 积分 */
    float kd;					/* 微分 */
    float target;				/* 目标值 */
    float current;				/* 当前值 */
    float out;					/* 执行量 */
    float limit;                /* PID(out)输出限幅值 */

    float error;				/* 当前误差 */
    float last_error;			/* 上一次误差 */
    float last2_error;			/* 上上次误差 */
    float last_out;             /* 上一次执行量 */
	float integral;				/* 积分（累加） */
    float i_min;                /* 积分下限（i_limit_enable 时每拍生效） */
    float i_max;                /* 积分上限（i_limit_enable 时每拍生效） */
    uint8_t i_limit_enable;     /* 1=每拍对 integral 限幅，0=不限（默认） */
	float p_out,i_out,d_out;	/* 比例、积分、微分值 */
}PID_T;

typedef enum
{
    PID_TERM_KP = 0,
    PID_TERM_KI,
    PID_TERM_KD,
} pid_term_enum;

/*
    提供给用户调用的API
*/
/* PID初始化 */
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit);

/* 设置PID目标值 */
void pid_set_target(PID_T * _tpPID, float _target);

/* 设置PID参数 */
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd);

/* 设置PID单项参数 */
void pid_set_term(PID_T * _tpPID, pid_term_enum term, float value);

/* 设置PID输出限幅 */
void pid_set_limit(PID_T * _tpPID, float _limit);

/* 重置PID控制器 */
void pid_reset(PID_T * _tpPID);

/* 设置积分限幅并使能（每拍对 integral 钳到 [min,max]）。 */
void pid_set_integral_limit(PID_T * _tpPID, float min, float max);

/* 只清积分项（integral / i_out），保留 last_error，
 * 避免像 pid_reset 那样清掉微分历史而在下一拍引入微分冲击。 */
void pid_reset_integral(PID_T * _tpPID);

/* 计算位置式PID */
float pid_calculate_positional(PID_T * _tpPID, float _current);

/* 计算位置式PID，但跳过积分累加（i_out 置 0，integral 不变）。
 * D 项与 pid_calculate_positional 一致，用于静止/摔倒/锁中位等
 * 需要暂停积分但仍要输出 P/D 的场景。 */
float pid_calculate_positional_no_integral(PID_T * _tpPID, float _current);

/* 计算增量式PID */
float pid_calculate_incremental(PID_T * _tpPID, float _current);

/* 限幅函数 */
float pid_constrain(float value, float min, float max);
  
/* 积分限幅函数 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max);

/* 带空指针保护的便捷封装：平衡快/慢两套 PID 的 set_params/term/limit
 * 入口逻辑完全一致（判空 -> 写入 -> 视情况 reset），统一收在这里避免两份副本。
 * 做成 static inline 不新增编译单元。 */
static inline void pid_apply_params(PID_T *pid, float kp, float ki, float kd)
{
    if (0 == pid)
    {
        return;
    }
    pid_set_params(pid, kp, ki, kd);
    pid_reset(pid);
}

static inline void pid_apply_term(PID_T *pid, pid_term_enum term, float value)
{
    if (0 == pid)
    {
        return;
    }
    pid_set_term(pid, term, value);
    pid_reset(pid);
}

static inline void pid_apply_limit(PID_T *pid, float limit)
{
    if (0 == pid)
    {
        return;
    }
    pid_set_limit(pid, limit);
}

#endif
