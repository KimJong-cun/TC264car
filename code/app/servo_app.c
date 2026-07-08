/*
 * servo_app.c
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#include "servo_app.h"

/* 运行时有效限幅，由 servo_set_limit_mode() 切换。
 * 默认与 fast 模式（全幅）相同，balance_init() 会按实际模式覆写一次。 */
static uint32_t s_servo_lo = r_max;
static uint32_t s_servo_hi = l_max;

void servo_set_limit_mode(servo_limit_mode_enum mode)
{
    if (mode == SERVO_LIMIT_SLOW)
    {
        s_servo_lo = (uint32_t)(mid - SERVO_SLOW_HALF_RANGE);
        s_servo_hi = (uint32_t)(mid + SERVO_SLOW_HALF_RANGE);
    }
    else
    {
        s_servo_lo = r_max;
        s_servo_hi = l_max;
    }
}

/* 舵机占空比设置 */
void servo_set(uint32_t duty)
{
    if (duty >= s_servo_hi)
        duty = s_servo_hi;
    if (duty <= s_servo_lo)
        duty = s_servo_lo;
    pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

/* 以中位 mid 为基准叠加一个有符号偏移并输出。
 * 必须在有符号域内先把 mid+offset 夹到动态限幅范围，再转 uint32：
 * 若像以前那样先 (uint32_t) 再交给 servo_set，offset 的大负值会让
 * mid+offset 变负并回绕成超大正数，被 servo_set 当成上限打向反方向。 */
void servo_set_offset(int32_t offset)
{
    int32_t duty = (int32_t)mid + offset;

    if (duty > (int32_t)s_servo_hi)
    {
        duty = (int32_t)s_servo_hi;
    }
    else if (duty < (int32_t)s_servo_lo)
    {
        duty = (int32_t)s_servo_lo;
    }

    pwm_set_duty(ATOM1_CH1_P33_9, (uint32_t)duty);
}

/* 舵机测试 用于寻找中值*/
void servo_test(void)
{
    static uint16_t servo_motor_duty = mid;
    static uint8_t servo_motor_dir = 0;
    if (servo_motor_dir)
    {
        servo_motor_duty += 10;
        if (servo_motor_duty >= l_max)
        {
            servo_motor_dir = 0x00;
        }
    }
    else
    {
        servo_motor_duty -= 10;
        if (servo_motor_duty <= r_max)
        {
            servo_motor_dir = 0x01;
        }
    }
    servo_set(servo_motor_duty);
//    printf("duty:%d\r\n",servo_motor_duty);
}
