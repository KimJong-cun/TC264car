/*
 * nav_odometry.c
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: Navigation odometry and start-wait detection
 *  Copyright: KimJong
 */

#include "nav_odometry.h"
#include "../common/angle_util.h"

#define NAV_ODOMETRY_START_ACCEL_DELTA_G           0.25f
#define NAV_ODOMETRY_PI                            3.14159265358979323846f
#define NAV_ODOMETRY_DEG_TO_RAD                    (NAV_ODOMETRY_PI / 180.0f)
#define NAV_ODOMETRY_DT_S                          0.010f

static nav_pose_t s_nav_pose;
static float s_nav_start_acc_ref_x_g = 0.0f;
static float s_nav_start_acc_ref_y_g = 0.0f;
static float s_nav_start_acc_ref_z_g = 0.0f;

static float nav_odometry_wrap_angle_deg(float angle_deg)
{
    return angle_wrap_deg(angle_deg);
}

static void nav_odometry_read_acc_now(float *x_g, float *y_g, float *z_g)
{
    if (0 != x_g)
    {
        *x_g = imu660rc_acc_transition(imu660rc_acc_x);
    }

    if (0 != y_g)
    {
        *y_g = imu660rc_acc_transition(imu660rc_acc_y);
    }

    if (0 != z_g)
    {
        *z_g = imu660rc_acc_transition(imu660rc_acc_z);
    }
}

void nav_odometry_init(void)
{
    nav_odometry_reset(0.0f);
}

void nav_odometry_reset(float heading_deg)
{
    s_nav_pose.x = 0.0f;
    s_nav_pose.y = 0.0f;
    s_nav_pose.distance = 0.0f;
    s_nav_pose.heading = nav_odometry_wrap_angle_deg(heading_deg);
    nav_odometry_capture_start_wait_reference();
}

void nav_odometry_set_position(float x, float y)
{
    s_nav_pose.x = x;
    s_nav_pose.y = y;
}

void nav_odometry_sync_heading(float heading_deg)
{
    s_nav_pose.heading = nav_odometry_wrap_angle_deg(heading_deg);
}

void nav_odometry_update_dead_reckoning(void)
{
    float yaw_rad = s_nav_pose.heading * NAV_ODOMETRY_DEG_TO_RAD;

    /* 坐标系和 subject 保持一致：
     *   x 右正，y 前正；
     *   heading 右转/顺时针为正，所以正 heading 会通过 sin() 积到 x 正方向。
     * 若更换 IMU 后右转时 x 变负，优先检查 yaw 映射方向。
     */
    s_nav_pose.x += wheel_speed * sinf(yaw_rad) * NAV_ODOMETRY_DT_S;
    s_nav_pose.y += wheel_speed * cosf(yaw_rad) * NAV_ODOMETRY_DT_S;
    s_nav_pose.distance += fabsf(wheel_speed) * NAV_ODOMETRY_DT_S;
}

const nav_pose_t *nav_odometry_get_pose(void)
{
    return &s_nav_pose;
}

void nav_odometry_capture_start_wait_reference(void)
{
    nav_odometry_read_acc_now(&s_nav_start_acc_ref_x_g,
                              &s_nav_start_acc_ref_y_g,
                              &s_nav_start_acc_ref_z_g);
}

uint8 nav_odometry_is_start_wait_triggered(void)
{
    return (nav_odometry_get_start_wait_delta_g() >= NAV_ODOMETRY_START_ACCEL_DELTA_G) ? 1u : 0u;
}

void nav_odometry_get_start_wait_reference(float *x_g, float *y_g, float *z_g)
{
    if (0 != x_g)
    {
        *x_g = s_nav_start_acc_ref_x_g;
    }

    if (0 != y_g)
    {
        *y_g = s_nav_start_acc_ref_y_g;
    }

    if (0 != z_g)
    {
        *z_g = s_nav_start_acc_ref_z_g;
    }
}

void nav_odometry_get_start_wait_now(float *x_g, float *y_g, float *z_g)
{
    nav_odometry_read_acc_now(x_g, y_g, z_g);
}

float nav_odometry_get_start_wait_delta_g(void)
{
    float acc_x_now_g = 0.0f;
    float acc_y_now_g = 0.0f;
    float acc_z_now_g = 0.0f;
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float delta_z = 0.0f;

    nav_odometry_read_acc_now(&acc_x_now_g, &acc_y_now_g, &acc_z_now_g);

    delta_x = acc_x_now_g - s_nav_start_acc_ref_x_g;
    delta_y = acc_y_now_g - s_nav_start_acc_ref_y_g;
    delta_z = acc_z_now_g - s_nav_start_acc_ref_z_g;

    return sqrtf(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
}
