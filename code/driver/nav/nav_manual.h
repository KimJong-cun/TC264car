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
#define NAV_MANUAL_TURN_DONE_HOLD_TICKS     4u
#define NAV_MANUAL_TURN_TIMEOUT_BASE_TICKS  200u
#define NAV_MANUAL_TURN_TIMEOUT_PER_DEG     6.0f

/* DIRECT 任务提前减速：走完这个比例的距离后，如果队列里下一个任务的目标速度
 * 比当前 DIRECT 任务的目标速度低，就开始向下一个任务的速度减速，
 * 避免衔接时速度突降。只对 DIRECT 任务生效（TURN 没有以米为单位的"路程"）。 */
#define NAV_MANUAL_DECEL_TRIGGER_RATIO       0.7f

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
uint8 nav_manual_queue_peek_front(const nav_manual_queue_t *queue, nav_manual_task_t *out_task);
uint8 nav_manual_queue_length(const nav_manual_queue_t *queue);

/* Pure computations, no side effects. */
void   nav_manual_compute_direct_target(float start_x, float start_y, float heading_deg,
                                        float distance_m, float *out_x, float *out_y);
float  nav_manual_compute_turn_target_yaw(float base_yaw_deg, float turn_deg);
uint16 nav_manual_compute_turn_timeout_ticks(float turn_deg);
uint8  nav_manual_is_turn_settled(float yaw_error_deg, float yaw_rate_dps);

/* DIRECT 任务提前减速判定：走完 NAV_MANUAL_DECEL_TRIGGER_RATIO 比例的距离，
 * 且队列里下一个任务的目标速度比当前任务低，就返回 1（应当开始减速）。 */
uint8  nav_manual_should_start_decel(float traveled_m, float distance_m,
                                     float current_speed_mps, float next_speed_mps);

#endif /* CODE_DRIVER_NAV_NAV_MANUAL_H_ */
