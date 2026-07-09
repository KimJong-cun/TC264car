/*
 * nav_manual.c
 *
 *  Created on: 2026-07-02
 *      Author: KimJong
 *  Description: Manual DIRECT/TURN task queue storage and pure geometry/
 *               timing computations. See nav_manual.h.
 *  Copyright: KimJong
 */

#include "nav_manual.h"
#include "../common/angle_util.h"

#define NAV_MANUAL_PI  3.14159265358979323846f

void nav_manual_queue_reset(nav_manual_queue_t *queue)
{
    queue->head = 0u;
    queue->count = 0u;
}

uint8 nav_manual_queue_push(nav_manual_queue_t *queue, nav_manual_task_type_enum type,
                            float value, float speed_mps)
{
    uint8 tail = 0u;

    if (queue->count >= NAV_MANUAL_QUEUE_SIZE)
    {
        return 0u;
    }

    tail = (uint8)((queue->head + queue->count) % NAV_MANUAL_QUEUE_SIZE);
    queue->items[tail].type = type;
    queue->items[tail].value = value;
    queue->items[tail].speed_mps = speed_mps;
    queue->count++;
    return 1u;
}

uint8 nav_manual_queue_pop_front(nav_manual_queue_t *queue, nav_manual_task_t *out_task)
{
    if (0u == queue->count)
    {
        return 0u;
    }

    *out_task = queue->items[queue->head];
    queue->head = (uint8)((queue->head + 1u) % NAV_MANUAL_QUEUE_SIZE);
    queue->count--;
    return 1u;
}

uint8 nav_manual_queue_peek_front(const nav_manual_queue_t *queue, nav_manual_task_t *out_task)
{
    if (0u == queue->count)
    {
        return 0u;
    }

    *out_task = queue->items[queue->head];
    return 1u;
}

uint8 nav_manual_queue_length(const nav_manual_queue_t *queue)
{
    return queue->count;
}

void nav_manual_compute_direct_target(float start_x, float start_y, float heading_deg,
                                      float distance_m, float *out_x, float *out_y)
{
    *out_x = start_x + (distance_m * sinf(heading_deg * (NAV_MANUAL_PI / 180.0f)));
    *out_y = start_y + (distance_m * cosf(heading_deg * (NAV_MANUAL_PI / 180.0f)));
}

float nav_manual_compute_turn_target_yaw(float base_yaw_deg, float turn_deg)
{
    return angle_wrap_deg(base_yaw_deg + turn_deg);
}

uint16 nav_manual_compute_turn_timeout_ticks(float turn_deg)
{
    return (uint16)(NAV_MANUAL_TURN_TIMEOUT_BASE_TICKS +
                    (fabsf(turn_deg) * NAV_MANUAL_TURN_TIMEOUT_PER_DEG));
}

uint8 nav_manual_is_turn_settled(float yaw_error_deg, float yaw_rate_dps)
{
    return ((fabsf(yaw_error_deg) <= NAV_MANUAL_TURN_DONE_DEG) &&
            (fabsf(yaw_rate_dps) <= NAV_MANUAL_TURN_DONE_RATE_DPS)) ? 1u : 0u;
}

uint8 nav_manual_should_start_decel(float traveled_m, float distance_m,
                                    float current_speed_mps, float next_speed_mps)
{
    if (next_speed_mps >= current_speed_mps)
    {
        return 0u;
    }

    if (distance_m <= 0.0f)
    {
        return 0u;
    }

    return (traveled_m >= (distance_m * NAV_MANUAL_DECEL_TRIGGER_RATIO)) ? 1u : 0u;
}
