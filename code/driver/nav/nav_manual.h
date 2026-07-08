/*
 * nav_manual.h
 *
 *  Created on: 2026-07-02
 *      Author: KimJong
 *  Description: Manual DIRECT/TURN task queue storage and pure geometry/
 *               timing computations. No balance/motor/imu side effects —
 *               callers (nav_app.c) own the state machine and apply targets.
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_NAV_NAV_MANUAL_H_
#define CODE_DRIVER_NAV_NAV_MANUAL_H_

#include "zf_common_headfile.h"

#define NAV_MANUAL_QUEUE_SIZE               8u
#define NAV_MANUAL_TURN_DONE_DEG            5.0f
#define NAV_MANUAL_TURN_DONE_RATE_DPS       8.0f
#define NAV_MANUAL_TURN_DONE_HOLD_TICKS     10u
#define NAV_MANUAL_TURN_TIMEOUT_BASE_TICKS  200u
#define NAV_MANUAL_TURN_TIMEOUT_PER_DEG     6.0f

typedef enum
{
    NAV_MANUAL_TASK_DIRECT = 0,
    NAV_MANUAL_TASK_TURN,
} nav_manual_task_type_enum;

typedef struct
{
    nav_manual_task_type_enum type;
    float value;        /* DIRECT: distance_m. TURN: turn_deg. */
    float speed_mps;
} nav_manual_task_t;

typedef struct
{
    nav_manual_task_t items[NAV_MANUAL_QUEUE_SIZE];
    uint8 head;
    uint8 count;
} nav_manual_queue_t;

/* Queue storage: FIFO, fixed capacity NAV_MANUAL_QUEUE_SIZE. */
void  nav_manual_queue_reset(nav_manual_queue_t *queue);
uint8 nav_manual_queue_push(nav_manual_queue_t *queue, nav_manual_task_type_enum type,
                            float value, float speed_mps);
uint8 nav_manual_queue_pop_front(nav_manual_queue_t *queue, nav_manual_task_t *out_task);
uint8 nav_manual_queue_length(const nav_manual_queue_t *queue);

/* Pure computations, no side effects. */
void   nav_manual_compute_direct_target(float start_x, float start_y, float heading_deg,
                                        float distance_m, float *out_x, float *out_y);
float  nav_manual_compute_turn_target_yaw(float base_yaw_deg, float turn_deg);
uint16 nav_manual_compute_turn_timeout_ticks(float turn_deg);
uint8  nav_manual_is_turn_settled(float yaw_error_deg, float yaw_rate_dps);

#endif /* CODE_DRIVER_NAV_NAV_MANUAL_H_ */
