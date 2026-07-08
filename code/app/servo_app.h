/*
 * servo_app.h
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#ifndef CODE_APP_SERVO_APP_H_
#define CODE_APP_SERVO_APP_H_

#include "zf_common_headfile.h"

/*
 * 按 base 工程的舵机脉宽标定换算到当前 330Hz PWM：
 * base 50Hz: r_max=530  mid=747  l_max=930
 * 对应脉宽: 1060us     1494us   1860us
 * 当前 330Hz: duty = pulse_us / (1000000 / 330) * 10000
 */
#define l_max  6138             // 左极限，对应约 1860us
#define mid    4930             // 中值，对应约 1494us
#define r_max  3498             // 右极限，对应约 1060us

/* slow 模式下舵机限幅收窄到 mid ± 该值，防止低速时过大舵角。 */
#define SERVO_SLOW_HALF_RANGE  2000

typedef enum
{
    SERVO_LIMIT_FAST = 0,   /* 限幅 [r_max, l_max] */
    SERVO_LIMIT_SLOW,       /* 限幅 [mid-SERVO_SLOW_HALF_RANGE, mid+SERVO_SLOW_HALF_RANGE] */
} servo_limit_mode_enum;

void servo_set_limit_mode(servo_limit_mode_enum mode);
void servo_set(uint32_t duty);
void servo_set_offset(int32_t offset);
void servo_test(void);

#endif /* CODE_APP_SERVO_APP_H_ */
