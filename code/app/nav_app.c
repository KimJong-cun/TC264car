/*
 * nav_app.c
 *
 *  Subject-style waypoint navigation: manual DIRECT/TURN task queue, and
 *  preset-path route following (virtual-target-point averaging, see
 *  route设计思路.md). Queue storage / geometry / timing math for manual
 *  live in code/driver/nav/nav_manual.*; route geometry/filter math lives
 *  in code/driver/nav/nav_route.*. This file only runs the state machine
 *  and applies targets via balance_app / motor_app.
 *  Copyright: KimJong
 */

#include "nav_app.h"
#include "balance_app.h"
#include "../driver/common/angle_util.h"
#include "../driver/nav/nav_odometry.h"
#include "nav_app_internal.h"

static uint8 nav_is_busy(void)
{
    return ((NAV_STATE_RUNNING == s_nav_state) ||
            (NAV_STATE_WAIT_START == s_nav_state)) ? 1u : 0u;
}

static void nav_arm_wait_start(void)
{
    nav_odometry_reset(0.0f);
    nav_odometry_capture_start_wait_reference();
    balance_reset_heading_target(yaw);
    motor_set_speed_mps(0.0f);
    s_nav_state = NAV_STATE_WAIT_START;
}

static void nav_reset_yaw_reference(float base_yaw_deg)
{
    s_nav_yaw_init = angle_wrap_deg(base_yaw_deg);
    s_nav_target_x = 0.0f;
    s_nav_target_y = 0.0f;
}

/* -----------------------------------------------------------------------
 * Manual mode helpers
 * ----------------------------------------------------------------------- */

static void nav_manual_reset_runtime(void)
{
    nav_manual_queue_reset(&s_nav_manual_queue);
    s_nav_manual_active = 0u;
    s_nav_manual_current.type = NAV_MANUAL_TASK_DIRECT;
    s_nav_manual_current.value = 0.0f;
    s_nav_manual_current.speed_mps = 0.0f;
    s_nav_manual_start_distance = 0.0f;
    s_nav_manual_target_yaw = 0.0f;
    s_nav_manual_task_age = 0u;
    s_nav_manual_done_hold = 0u;
    s_nav_manual_timeout_ticks = 0u;
}

static uint8 nav_manual_enqueue(nav_manual_task_type_enum type,
                                float value,
                                float speed_mps)
{
    if ((0.0f == value) || (speed_mps <= 0.0f))
    {
        return 0u;
    }

    if (nav_is_busy())
    {
        return 0u;
    }

    return nav_manual_queue_push(&s_nav_manual_queue, type, value, speed_mps);
}

static void nav_manual_finish_all(void)
{
    s_nav_manual_active = 0u;
    s_nav_state = NAV_STATE_DONE;
    motor_set_speed_mps(0.0f);
}

static float nav_get_distance_internal(void)
{
    return nav_odometry_get_pose()->distance;
}

static void nav_manual_activate_current(uint8 reset_speed_pid)
{
    const nav_pose_t *pose = 0;

    if (0u == nav_manual_queue_pop_front(&s_nav_manual_queue, &s_nav_manual_current))
    {
        nav_manual_finish_all();
        return;
    }

    s_nav_manual_active = 1u;
    s_nav_manual_task_age = 0u;
    s_nav_manual_done_hold = 0u;
    s_nav_manual_timeout_ticks = 0u;
    pose = nav_odometry_get_pose();

    if (NAV_MANUAL_TASK_DIRECT == s_nav_manual_current.type)
    {
        s_nav_manual_start_distance = nav_get_distance_internal();
        s_nav_manual_target_yaw = yaw;
        nav_manual_compute_direct_target(pose->x, pose->y, pose->heading,
                                         s_nav_manual_current.value,
                                         &s_nav_target_x, &s_nav_target_y);
        balance_reset_heading_target(yaw);
        balance_set_heading_target(s_nav_manual_target_yaw);
    }
    else
    {
        s_nav_manual_target_yaw = nav_manual_compute_turn_target_yaw(yaw, s_nav_manual_current.value);
        s_nav_manual_timeout_ticks = nav_manual_compute_turn_timeout_ticks(s_nav_manual_current.value);
        s_nav_target_x = pose->x;
        s_nav_target_y = pose->y;
        balance_set_heading_target(s_nav_manual_target_yaw);
    }

    if (reset_speed_pid)
    {
        motor_set_speed_mps(s_nav_manual_current.speed_mps);
    }
    else
    {
        motor_update_speed_mps(s_nav_manual_current.speed_mps);
    }
}

static void nav_manual_finish_current(void)
{
    s_nav_manual_active = 0u;

    if (0u == nav_manual_queue_length(&s_nav_manual_queue))
    {
        nav_manual_finish_all();
        return;
    }

    nav_manual_activate_current(0u);
}

static void nav_manual_update_task(void)
{
    float traveled_m = 0.0f;
    float yaw_error = 0.0f;

    if (!s_nav_manual_active)
    {
        nav_manual_activate_current(1u);
        return;
    }

    balance_set_heading_target(s_nav_manual_target_yaw);
    s_nav_manual_task_age++;

    if (NAV_MANUAL_TASK_DIRECT == s_nav_manual_current.type)
    {
        traveled_m = nav_get_distance_internal() - s_nav_manual_start_distance;
        if (traveled_m >= s_nav_manual_current.value)
        {
            nav_manual_finish_current();
        }
        return;
    }

    yaw_error = angle_wrap_deg(s_nav_manual_target_yaw - yaw);
    if (nav_manual_is_turn_settled(yaw_error, gyro_z_rate))
    {
        s_nav_manual_done_hold++;
    }
    else
    {
        s_nav_manual_done_hold = 0u;
    }

    if ((s_nav_manual_done_hold >= NAV_MANUAL_TURN_DONE_HOLD_TICKS) ||
        (s_nav_manual_task_age >= s_nav_manual_timeout_ticks))
    {
        nav_manual_finish_current();
    }
}

/* -----------------------------------------------------------------------
 * Route mode helpers
 * ----------------------------------------------------------------------- */

static void nav_route_apply_target(const nav_pose_t *pose)
{
    float target_x = 0.0f;
    float target_y = 0.0f;
    float bearing_deg;
    float target_yaw_global;

    nav_route_compute_target(s_nav_route_index, &target_x, &target_y);
    bearing_deg = nav_route_compute_bearing_deg(pose->x, pose->y, target_x, target_y);
    target_yaw_global = angle_wrap_deg(s_nav_yaw_init + bearing_deg);

    s_nav_target_x = target_x;
    s_nav_target_y = target_y;
    balance_set_heading_target(target_yaw_global);
}

static void nav_route_enter_path(void)
{
    float start_x = 0.0f;
    float start_y = 0.0f;
    const nav_pose_t *pose = 0;

    /* 把 odometry 位置对齐到路径数组的第一个点，而不是车物理启动位置的
     * 本地 (0,0)，这样 route 从 index 0 开始就是准确对应关系，不需要
     * 靠 nav_route_advance_index 的投影去"猜"车在路径里的位置。 */
    nav_route_get_point(0u, &start_x, &start_y);
    nav_odometry_set_position(start_x, start_y);

    s_nav_route_index = 0u;
    pose = nav_odometry_get_pose();
    nav_route_apply_target(pose);
}

static void nav_route_activate(void)
{
    /* 先直走 NAV_ROUTE_PRE_STRAIGHT_DISTANCE_M，再进入路径跟踪：
     * 保持车当前朝向作为直走目标角，走够距离后才对齐到 path[0]。 */
    s_nav_route_pre_straight_active = 1u;
    s_nav_route_pre_straight_start_distance = nav_get_distance_internal();
    s_nav_route_pre_straight_target_yaw = yaw;

    balance_reset_heading_target(yaw);
    balance_set_heading_target(s_nav_route_pre_straight_target_yaw);
    motor_set_speed_mps(s_nav_route_speed_mps);
}

static void nav_route_update_pre_straight(void)
{
    float traveled_m;

    balance_set_heading_target(s_nav_route_pre_straight_target_yaw);
    traveled_m = nav_get_distance_internal() - s_nav_route_pre_straight_start_distance;

    if (traveled_m >= NAV_ROUTE_PRE_STRAIGHT_DISTANCE_M)
    {
        s_nav_route_pre_straight_active = 0u;
        nav_route_enter_path();
    }
}

static void nav_route_update_task(void)
{
    const nav_pose_t *pose = 0;

    if (s_nav_route_pre_straight_active)
    {
        nav_route_update_pre_straight();
        return;
    }

    pose = nav_odometry_get_pose();
    s_nav_route_index = nav_route_advance_index(s_nav_route_index, pose->x, pose->y);

    if (nav_route_is_finished(s_nav_route_index, pose->x, pose->y))
    {
        s_nav_state = NAV_STATE_DONE;
        motor_set_speed_mps(0.0f);
        return;
    }

    nav_route_apply_target(pose);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void nav_init(void)
{
    nav_odometry_init();
    nav_odometry_reset(0.0f);
    nav_manual_reset_runtime();
    nav_reset_yaw_reference(yaw);
    s_nav_mode = NAV_MODE_MANUAL;
    s_nav_state = NAV_STATE_IDLE;
    s_nav_route_pre_straight_active = 0u;
    balance_reset_heading_target(yaw);
}

void nav_reset(void)
{
    nav_manual_reset_runtime();
    nav_reset_yaw_reference(yaw);
    nav_odometry_reset(0.0f);
    s_nav_mode = NAV_MODE_MANUAL;
    s_nav_state = NAV_STATE_IDLE;
    s_nav_route_pre_straight_active = 0u;
    nav_odometry_capture_start_wait_reference();
    balance_reset_heading_target(yaw);
    motor_set_speed_mps(0.0f);
}

const nav_data_t *nav_get_data(void)
{
    const nav_pose_t *pose = nav_odometry_get_pose();
    uint8 hide_route_pre_straight = (uint8)((NAV_MODE_ROUTE == s_nav_mode) &&
                                            s_nav_route_pre_straight_active);

    /* route 模式的直走预备段不是路径本身的一部分，对外坐标固定打 0，
     * 避免下位机把这段直走误当成路径跟踪数据来画。 */
    s_nav_data.x = hide_route_pre_straight ? 0.0f : pose->x;
    s_nav_data.y = hide_route_pre_straight ? 0.0f : pose->y;
    s_nav_data.target_x = hide_route_pre_straight ? 0.0f : s_nav_target_x;
    s_nav_data.target_y = hide_route_pre_straight ? 0.0f : s_nav_target_y;
    s_nav_data.distance = pose->distance;
    s_nav_data.heading = pose->heading;
    s_nav_data.state = s_nav_state;
    return &s_nav_data;
}

uint8 nav_enqueue_direct(float distance_m, float speed_mps)
{
    if (distance_m <= 0.0f)
    {
        return 0u;
    }

    return nav_manual_enqueue(NAV_MANUAL_TASK_DIRECT, distance_m, speed_mps);
}

uint8 nav_enqueue_turn(float turn_deg, float speed_mps)
{
    return nav_manual_enqueue(NAV_MANUAL_TASK_TURN, turn_deg, speed_mps);
}

uint8 nav_start_manual(void)
{
    if (0u == nav_manual_queue_length(&s_nav_manual_queue))
    {
        return 0u;
    }

    if (nav_is_busy())
    {
        return 0u;
    }

    s_nav_mode = NAV_MODE_MANUAL;
    s_nav_manual_active = 0u;
    nav_reset_yaw_reference(yaw);
    nav_arm_wait_start();
    return 1u;
}

void nav_clear_manual_tasks(void)
{
    if (nav_is_busy())
    {
        return;
    }

    nav_manual_reset_runtime();
}

uint8 nav_get_manual_task_count(void)
{
    return (uint8)(nav_manual_queue_length(&s_nav_manual_queue) + (s_nav_manual_active ? 1u : 0u));
}

uint8 nav_start_route(float speed_mps)
{
    if (speed_mps <= 0.0f)
    {
        return 0u;
    }

    if (nav_is_busy())
    {
        return 0u;
    }

    s_nav_mode = NAV_MODE_ROUTE;
    s_nav_route_speed_mps = speed_mps;
    s_nav_route_pre_straight_active = 0u;
    nav_reset_yaw_reference(yaw);
    nav_arm_wait_start();
    return 1u;
}

uint8 nav_is_route_mode(void)
{
    return (NAV_MODE_ROUTE == s_nav_mode) ? 1u : 0u;
}

uint16 nav_get_route_index(void)
{
    return s_nav_route_index;
}

void nav_cancel_task(void)
{
    s_nav_mode = NAV_MODE_MANUAL;
    s_nav_route_pre_straight_active = 0u;
    nav_manual_reset_runtime();
    nav_odometry_sync_heading(angle_wrap_deg(yaw - s_nav_yaw_init));
    s_nav_state = NAV_STATE_IDLE;
    nav_odometry_capture_start_wait_reference();
    balance_reset_heading_target(yaw);
    motor_set_speed_mps(0.0f);
}

nav_state_enum nav_get_state(void)
{
    return s_nav_state;
}

uint8 nav_is_running(void)
{
    return (NAV_STATE_RUNNING == s_nav_state) ? 1u : 0u;
}

void nav_tick(void)
{
    switch (s_nav_state)
    {
        case NAV_STATE_WAIT_START:
        {
            if (nav_odometry_is_start_wait_triggered())
            {
                s_nav_state = NAV_STATE_RUNNING;
                uart_write_string(WIRELESS_UART_INDEX, "start\r\n");

                if (NAV_MODE_ROUTE == s_nav_mode)
                {
                    nav_route_activate();
                }
                else
                {
                    nav_manual_activate_current(1u);
                }
            }
        }
        break;

        case NAV_STATE_RUNNING:
        {
            if (motor_is_fallen())
            {
                nav_cancel_task();
                break;
            }

            nav_odometry_sync_heading(angle_wrap_deg(yaw - s_nav_yaw_init));
            nav_odometry_update_dead_reckoning();

            if (NAV_MODE_ROUTE == s_nav_mode)
            {
                nav_route_update_task();
            }
            else
            {
                nav_manual_update_task();
            }
        }
        break;

        case NAV_STATE_DONE:
        case NAV_STATE_IDLE:
        default:
        break;
    }
}
