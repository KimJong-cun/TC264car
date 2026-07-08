#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid_driver.h"

/* 内部功能函数 */
static void pid_formula_incremental(PID_T * _tpPID);
static void pid_formula_positional(PID_T * _tpPID);
static void pid_out_limit(PID_T * _tpPID);

/*******************************************************************************
 * @brief PID初始化函数，用于初始化PID结构体
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _kp 比例系数
 * @param {float} _ki 积分系数
 * @param {float} _kd 微分系数
 * @param {float} _target 目标值
 * @param {float} _limit 输出限幅值
 * @return {*}
 * @note 使用前必须调用该函数初始化PID参数
 *******************************************************************************/
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit)
{
    _tpPID->kp = _kp;          // 比例
    _tpPID->ki = _ki;          // 积分
    _tpPID->kd = _kd;          // 微分
    _tpPID->target = _target;  // 目标值
    _tpPID->limit = _limit;    // 限幅值
    _tpPID->integral = 0;      // 积分项清零
    _tpPID->last_error = 0;    // 上次误差清零
    _tpPID->last2_error = 0;   // 上上次误差清零
    _tpPID->out = 0;           // 输出值清零
    _tpPID->p_out = 0;         // P输出清零
    _tpPID->i_out = 0;         // I输出清零
    _tpPID->d_out = 0;         // D输出清零
    _tpPID->i_min = 0;         // 积分限幅默认关闭
    _tpPID->i_max = 0;
    _tpPID->i_limit_enable = 0;
}

/*******************************************************************************
 * @brief 设置PID目标值
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _target 目标值
 * @return {*}
 * @note 用于动态调整PID控制器的目标值
 *******************************************************************************/
void pid_set_target(PID_T * _tpPID, float _target)
{
    _tpPID->target = _target;
}

/*******************************************************************************
 * @brief 设置PID参数
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _kp 比例系数
 * @param {float} _ki 积分系数
 * @param {float} _kd 微分系数
 * @return {*}
 * @note 用于动态调整PID参数
 *******************************************************************************/
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd)
{
    _tpPID->kp = _kp;
    _tpPID->ki = _ki;
    _tpPID->kd = _kd;
}

/*******************************************************************************
 * @brief 设置PID单项参数
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {pid_term_enum} term 参数项
 * @param {float} value 参数值
 * @return {*}
 * @note 用于仅调整 kp/ki/kd 中的单个参数
 *******************************************************************************/
void pid_set_term(PID_T * _tpPID, pid_term_enum term, float value)
{
    switch(term)
    {
        case PID_TERM_KP: _tpPID->kp = value; break;
        case PID_TERM_KI: _tpPID->ki = value; break;
        case PID_TERM_KD: _tpPID->kd = value; break;
        default: break;
    }
}

/*******************************************************************************
 * @brief 设置PID输出限幅
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _limit 限幅值
 * @return {*}
 * @note 用于动态调整PID输出限幅
 *******************************************************************************/
void pid_set_limit(PID_T * _tpPID, float _limit)
{
    _tpPID->limit = _limit;
}

/*******************************************************************************
 * @brief 重置PID控制器
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @return {*}
 * @note 清除所有历史误差数据
 *******************************************************************************/
void pid_reset(PID_T * _tpPID)
{
    _tpPID->integral = 0;
    _tpPID->last_error = 0;
    _tpPID->last2_error = 0;
    _tpPID->out = 0;
    _tpPID->p_out = 0;
    _tpPID->i_out = 0;
    _tpPID->d_out = 0;
}

/*******************************************************************************
 * @brief 设置积分限幅并使能
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} min 积分下限
 * @param {float} max 积分上限
 * @note 使能后，位置式公式每拍把 integral 钳到 [min,max]，防止积分饱和(windup)。
 *******************************************************************************/
void pid_set_integral_limit(PID_T * _tpPID, float min, float max)
{
    _tpPID->i_min = min;
    _tpPID->i_max = max;
    _tpPID->i_limit_enable = 1;
}

/*******************************************************************************
 * @brief 只清积分项
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @note 清 integral / i_out，但保留 last_error，
 *       避免清掉微分历史而在下一拍产生微分冲击。
 *******************************************************************************/
void pid_reset_integral(PID_T * _tpPID)
{
    _tpPID->integral = 0;
    _tpPID->i_out = 0;
}

/*******************************************************************************
 * @brief 计算位置式PID
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _current 当前值
 * @return {float} PID计算后的输出值
 * @note 位置式PID：P-响应性，I-准确性，D-稳定性
 *******************************************************************************/
float pid_calculate_positional(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_positional(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/*******************************************************************************
 * @brief 计算位置式PID，但跳过积分累加
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _current 当前值
 * @return {float} PID计算后的输出值
 * @note D 项与 pid_calculate_positional 一致，用 kd*(error-last_error)；
 *       仅 integral 不累加（i_out 置 0），用于静止/摔倒/锁中位等场景防积分饱和。
 *******************************************************************************/
float pid_calculate_positional_no_integral(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    _tpPID->error = _tpPID->target - _tpPID->current;

    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = 0.0f;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);
    _tpPID->out = _tpPID->p_out + _tpPID->d_out;

    _tpPID->last_error = _tpPID->error;

    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/*******************************************************************************
 * @brief 计算增量式PID
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @param {float} _current 当前值
 * @return {float} PID计算后的输出值
 * @note 增量式PID：P-稳定性，I-响应性，D-准确性
 *******************************************************************************/
float pid_calculate_incremental(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_incremental(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/* ————————————————————————————————— PID相关的功能函数 ————————————————————————————————— */
/*******************************************************************************
 * @brief 输出限幅函数
 * @param {PID_T *} _tpPID 指向PID结构体的指针
 * @return {*}
 * @note 防止输出超出限定范围
 *******************************************************************************/
static void pid_out_limit(PID_T * _tpPID)
{
    if(_tpPID->out > _tpPID->limit)
        _tpPID->out = _tpPID->limit;
    else if(_tpPID->out < -_tpPID->limit)
        _tpPID->out = -_tpPID->limit;
}

/*******************************************************************************
 * @brief 增量式PID公式
 * @param {PID_T *} _tpPID  传入要计算的PID参数指针
 * @return {*}
 * @note 在增量式中，P-稳定性，I-响应性，D-准确性
 *******************************************************************************/
static void pid_formula_incremental(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    
    _tpPID->p_out = _tpPID->kp * (_tpPID->error - _tpPID->last_error);
    _tpPID->i_out = _tpPID->ki * _tpPID->error;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - 2 * _tpPID->last_error + _tpPID->last2_error);
    
    _tpPID->out += _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
    
    _tpPID->last2_error = _tpPID->last_error;
    _tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief 位置式PID公式
 * @param {PID_T *} _tpPID  传入要计算的PID参数指针
 * @return {*}
 * @note 在位置式中，P-响应性，I-准确性，D-稳定性
 *******************************************************************************/
static void pid_formula_positional(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    _tpPID->integral += _tpPID->error;

    if (_tpPID->i_limit_enable)
    {
        _tpPID->integral = pid_constrain(_tpPID->integral, _tpPID->i_min, _tpPID->i_max);
    }

    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = _tpPID->ki * _tpPID->integral;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);

    _tpPID->out = _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;

    _tpPID->last_error = _tpPID->error;
}

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
float pid_constrain(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

/**
 * @brief 积分限幅函数
 * @param pid PID控制器
 * @param min 最小值
 * @param max 最大值
 * @note 在使用增量式PID控制时此函数不需要，但为了可能切换回位置式PID而保留
 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max)
{
    if (pid->integral > max)
    {
        pid->integral = max;
    }
    else if (pid->integral < min)
    {
        pid->integral = min;
    }
}

