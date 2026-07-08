/*
 * schedule.c
 *
 *  后台软调度器：基于 uwtick(1ms 时基) 的极简分频任务表。
 *  只负责非硬实时的后台任务（遥测、自动校准等）；真正的平衡/导航闭环
 *  仍由 isr.c 里的 PIT 硬件中断定频。
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 *  Copyright: KimJong
 */

#include "scheduler.h"

uint32_t uwtick = 0;

// 自动校准标志
static uint8_t auto_calib_done = 0;

// 这里只放后台和低实时性任务。
// 真正的平衡闭环和导航主闭环仍然留在定时中断里。

/**
 * @brief 自动执行 pitch 校准（上电 3 秒后，仅一次）
 *
 * 注册成低频轮询任务即可：内部用 auto_calib_done + uwtick>=3000 自锁，
 * 多调几次也只会在 3 秒那一刻执行一次。
 */
static void auto_calib_pitch(void)
{
    if (!auto_calib_done && uwtick >= 3000)
    {
        auto_calib_done = 1;
        pitch_offset = -pitch_kalman;
        my_printf(WIRELESS_UART_INDEX, "[AUTO_CALIB] pitch_offset=%.2f (pitch_kalman=%.2f)\r\n",
                  (double)pitch_offset, (double)pitch_kalman);
    }
}

/**
 * @brief 调试遥测任务封装
 *
 * 帧节流已上移到调度表，这里每次被调到就尝试甩一帧。
 *   DEBUG_TELEMETRY_MODE_PID = pidtest（平衡/速度环）
 *   DEBUG_TELEMETRY_MODE_XY  = xytest（里程计位置/航向）
 *   DEBUG_TELEMETRY_MODE_IMU = imutest（IMU 方向/正负校准）
 *   DEBUG_TELEMETRY_MODE_OFF = 关闭
 * 也可在运行时用无线串口命令 "dbg 1/2/3/0" 切换。
 */
static void task_debug_telemetry(void)
{
    debug_telemetry_tick();
}

/* 后台软调度任务：period_ms 为期望调用周期，last_ms 记录上次实际派发时刻。 */
typedef struct
{
    uint32_t period_ms;     /* 期望调用周期（ms）。 */
    uint32_t last_ms;       /* 上次派发时的 uwtick。 */
    void (*task)(void);     /* 任务入口。 */
} sched_task_t;

/* 调度表：在这里加一行就能给后台任务定频，不用再在任务内部各自卡 uwtick。
 * 注意这是“软”定频：精度受主循环一圈耗时和 1ms 时基限制，会有抖动；
 * 需要硬实时确定性的环请放进 isr.c 的 PIT 中断。 */
static sched_task_t s_sched_tasks[] =
{
    { 60u,  0u, task_debug_telemetry },   /* 100Hz：遥测画波形。 */
    { 50u,  0u, auto_calib_pitch     },   /* 20Hz ：轮询自动校准（内部一次性）。 */
};

#define SCHED_TASK_COUNT   (sizeof(s_sched_tasks) / sizeof(s_sched_tasks[0]))

/**
 * @brief 调度器初始化函数
 */
void scheduler_init(void)
{
    uint32_t i;

    auto_calib_done = 0;
    debug_telemetry_init();

    /* 对齐各任务的起始时刻，避免上电瞬间一次性全部触发。 */
    for (i = 0u; i < SCHED_TASK_COUNT; i++)
    {
        s_sched_tasks[i].last_ms = uwtick;
    }
}

/**
 * @brief 调度器运行函数：在主循环里反复调用，按各任务周期分频派发。
 */
void scheduler_run(void)
{
    uint32_t now_ms = uwtick;
    uint32_t i;

    for (i = 0u; i < SCHED_TASK_COUNT; i++)
    {
        /* 无符号相减天然处理 uwtick 回绕。 */
        if ((now_ms - s_sched_tasks[i].last_ms) >= s_sched_tasks[i].period_ms)
        {
            s_sched_tasks[i].last_ms = now_ms;
            s_sched_tasks[i].task();
        }
    }
}
