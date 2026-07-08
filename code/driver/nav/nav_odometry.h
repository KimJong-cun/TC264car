/*
 * nav_odometry.h
 *
 *  Created on: 2026-04-20
 *      Author: KimJong
 *  Description: Navigation odometry and start-wait detection
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_NAV_NAV_ODOMETRY_H_
#define CODE_DRIVER_NAV_NAV_ODOMETRY_H_

#include "zf_common_headfile.h"

typedef struct
{
    /* Subject-style navigation frame:
     *   x: right positive / left negative
     *   y: forward positive / backward negative
     *   heading: right turn / clockwise positive
     */
    float x;
    float y;
    float distance;
    float heading;
} nav_pose_t;

void nav_odometry_init(void);
void nav_odometry_reset(float heading_deg);
void nav_odometry_set_position(float x, float y);
void nav_odometry_sync_heading(float heading_deg);
void nav_odometry_update_dead_reckoning(void);
const nav_pose_t *nav_odometry_get_pose(void);

void nav_odometry_capture_start_wait_reference(void);
uint8 nav_odometry_is_start_wait_triggered(void);
void nav_odometry_get_start_wait_reference(float *x_g, float *y_g, float *z_g);
void nav_odometry_get_start_wait_now(float *x_g, float *y_g, float *z_g);
float nav_odometry_get_start_wait_delta_g(void);

#endif /* CODE_DRIVER_NAV_NAV_ODOMETRY_H_ */
