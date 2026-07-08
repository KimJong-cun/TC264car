/*
 * debug_telemetry.h
 *
 *  实时调试遥测：周期性通过无线串口输出关键量，供上位机画波形。
 *  支持多套调试帧（可选择 pidtest / xytest / imutest）。
 *  归在 code/test 下，属于调试/观测代码，不参与控制闭环。
 *  Copyright: KimJong
 */

#ifndef CODE_TEST_DEBUG_TELEMETRY_H_
#define CODE_TEST_DEBUG_TELEMETRY_H_

#include "zf_common_typedef.h"

/* 调试帧模式选择。 */
typedef enum
{
    DEBUG_TELEMETRY_MODE_OFF = 0,   /* 关闭：不发送任何遥测帧。 */
    DEBUG_TELEMETRY_MODE_PID = 1,   /* pidtest：平衡/速度环关键量。 */
    DEBUG_TELEMETRY_MODE_XY  = 2,   /* xytest：里程计位置/航向量。 */
    DEBUG_TELEMETRY_MODE_IMU = 3,   /* imutest：IMU 方向/正负校准量。 */
} debug_telemetry_mode_enum;

/* 调试帧发送触发模式。 */
typedef enum
{
    DEBUG_TELEMETRY_TRIGGER_RUNNING = 0, /* 动态：导航进入 RUNNING 后才发送。 */
    DEBUG_TELEMETRY_TRIGGER_ALWAYS  = 1, /* 静态：不等待导航运行，静止时也发送。 */
} debug_telemetry_trigger_enum;

/* 复位调试帧模式、触发模式与摔车锁存（在系统/调度初始化时调用一次）。 */
void debug_telemetry_init(void);

/* 选择当前使用的调试帧：1=pidtest，2=xytest，3=imutest，0=关闭。
 * 是否静止发送由 debug_telemetry_set_trigger() 控制。 */
void debug_telemetry_set_mode(debug_telemetry_mode_enum mode);

/* 读取当前调试帧模式。 */
debug_telemetry_mode_enum debug_telemetry_get_mode(void);

/* 设置/读取调试帧触发模式：running=导航 RUNNING 后发送，always=静止也发送。 */
void debug_telemetry_set_trigger(debug_telemetry_trigger_enum trigger);
debug_telemetry_trigger_enum debug_telemetry_get_trigger(void);

/* 摔车取消打印开关：使能后，一旦检测到 motor_is_fallen() 即锁存停止发送，
 * 让 VOFA+ 波形干净地停在摔车瞬间，而不是被摔倒后的乱码刷屏。
 * 锁存状态在 debug_telemetry_init() / debug_telemetry_set_mode() 时复位，
 * 即重新初始化或切换调试帧后会重新开始发送。enable: 1=开启(默认) 0=关闭。 */
void debug_telemetry_set_fall_cancel(uint8 enable);

/* 读取摔车取消打印开关状态：1=开启 0=关闭。 */
uint8 debug_telemetry_get_fall_cancel(void);

/* 在后台调度里调用：触发条件满足且未被摔车锁存时，甩一帧遥测数据。
 * 发送周期（节流）由上层调度器决定，本函数每次被调到即尝试发送一帧。 */
void debug_telemetry_tick(void);

#endif /* CODE_TEST_DEBUG_TELEMETRY_H_ */
