/*
 * ekf.h
 *
 *  EKF for IMU660RB/IMU660RX attitude estimation
 *  Copyright: KimJong
 */

#ifndef CODE_EKF_H_
#define CODE_EKF_H_

#include "zf_common_headfile.h"

#define EKF_RAD_TO_DEG      (57.295779513082320876798154814105f)
#define EKF_DT_S            (0.002f)
#define EKF_ACC_BLEND_K     (1.0f)

/* 陀螺仪 Z 轴航向标度修正。只作用于 yaw 积分，不影响 pitch/roll/平衡环。
 * 标定：车身贴地标记好朝向，原地精确转整 N 圈，读 yaw 变化量 measured，
 *       设为 IMU_YAW_GYRO_SCALE = (N * 360) / |measured|。
 * 实测顺时针精确 360° 只积分出 ~308°，故 360 / 308 ≈ 1.167。
 * 1.0 = 不修正。 */
#define IMU_YAW_GYRO_SCALE  (1.167f)

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_t;

extern imu_t imu_data;

void EKF_Init(void);
void EKF_UpData(void);
uint8 EKF_IsReady(void);
float EKF_GetYawCompensatedDeg(void);
void EKF_CaptureYawZero(void);

#endif /* CODE_EKF_H_ */
