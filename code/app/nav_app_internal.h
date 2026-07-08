/*
 * nav_app_internal.h
 *
 *  nav_app.c 私有实现细节（内部状态变量）。
 *  只允许被 nav_app.c 单独 include，不进 zf_common_headfile.h，
 *  否则里面的 static 变量会在每个 include 它的翻译单元里各拷贝一份。
 *  队列存储与手动任务的几何/时间纯计算已下沉到 code/driver/nav/nav_manual.*，
 *  这里只保留状态机需要的运行时变量。
 *  Copyright: KimJong
 */

#ifndef CODE_APP_NAV_APP_INTERNAL_H_
#define CODE_APP_NAV_APP_INTERNAL_H_

#include "../driver/nav/nav_manual.h"
#include "../driver/nav/nav_route.h"

typedef enum
{
    NAV_MODE_MANUAL = 0,
    NAV_MODE_ROUTE,
} nav_mode_enum;

static nav_data_t s_nav_data;
static nav_state_enum s_nav_state = NAV_STATE_IDLE;
static float s_nav_yaw_init = 0.0f;
static nav_mode_enum s_nav_mode = NAV_MODE_MANUAL;

static float s_nav_target_x = 0.0f;
static float s_nav_target_y = 0.0f;
static nav_manual_queue_t s_nav_manual_queue;
static uint8 s_nav_manual_active = 0u;
static nav_manual_task_t s_nav_manual_current;
static float s_nav_manual_start_distance = 0.0f;
static float s_nav_manual_target_yaw = 0.0f;
static uint16 s_nav_manual_task_age = 0u;
static uint16 s_nav_manual_done_hold = 0u;
static uint16 s_nav_manual_timeout_ticks = 0u;

static float s_nav_route_speed_mps = 0.0f;
static uint16 s_nav_route_index = 0u;
static uint8 s_nav_route_pre_straight_active = 0u;
static float s_nav_route_pre_straight_start_distance = 0.0f;
static float s_nav_route_pre_straight_target_yaw = 0.0f;

#endif /* CODE_APP_NAV_APP_INTERNAL_H_ */
