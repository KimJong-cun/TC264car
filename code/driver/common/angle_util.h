/*
 * angle_util.h
 *
 *  Created on: 2026
 *      Author: KimJong
 *  Description: Shared angle helpers (header-only, no extra translation unit)
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_COMMON_ANGLE_UTIL_H_
#define CODE_DRIVER_COMMON_ANGLE_UTIL_H_

/* 把角度归一化到 (-180, 180]，避免 359 deg 和 -1 deg 被当成大误差。
 * 全工程统一从这里取实现：航向环、导航、里程计、EKF、IMU 应用层共用，
 * 行为与原先各文件里的 while 循环完全一致。
 * 做成 static inline 不新增编译单元，CLI 与 IDE 都能直接编过。 */
static inline float angle_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

#endif /* CODE_DRIVER_COMMON_ANGLE_UTIL_H_ */
