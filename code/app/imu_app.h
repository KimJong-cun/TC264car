/*
 * imu_app.h
 *
 *  IMU application layer based on IMU660RC internal SFLP fusion
 *  Copyright: KimJong
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

extern float roll;
/* pitch: 本工程控制用左右倾角，已做零点补偿；直接沿用 IMU660RC pitch（左正右负）。 */
extern float pitch;
/* yaw: 导航/航向用角度，已做零点补偿；当前约定右转/顺时针为正。 */
extern float yaw;
extern float gyro_x_rate;
/* gyro_y_rate: 左右倾角速度，符号必须和 pitch 的动态变化方向匹配。 */
extern float gyro_y_rate;
/* gyro_z_rate: yaw 角速度，符号应和 yaw 右转/顺时针为正的约定匹配。 */
extern float gyro_z_rate;
extern float gyro_y_lpf_alpha;
extern float gyro_y_deadzone;  // 陀螺仪死区

extern float roll_kalman;
/* pitch_kalman: 未扣零点的左右倾角视图（RC pitch 原始观测），用于自动校准 pitch_offset。 */
extern float pitch_kalman;
/* yaw_kalman: 未扣零点的航向视图，直接沿用 RC yaw 原始观测。 */
extern float yaw_kalman;

/* IMU零点校准偏移量 */
extern float pitch_offset;
extern float roll_offset;
extern float yaw_offset;

/* 诊断用：INT2 中断进入计数。 */
extern volatile uint32 imu_int2_count;

void imu_proc(void);
void imu_all_init(void);
uint8 imu_is_ready(void);
void imu_capture_yaw_zero(void);
void imu_test(void);
void imu_origin(void);

#endif /* CODE_IMU_H_ */
