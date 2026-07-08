/*
 * nav_app.h
 *
 *  Subject-style waypoint navigation.
 *  Copyright: KimJong
 */

#ifndef CODE_APP_NAV_APP_H_
#define CODE_APP_NAV_APP_H_

#include "zf_common_headfile.h"

typedef enum
{
    NAV_STATE_IDLE = 0,
    NAV_STATE_WAIT_START,
    NAV_STATE_RUNNING,
    NAV_STATE_DONE,
} nav_state_enum;

typedef struct
{
    float x;
    float y;
    float target_x;
    float target_y;
    float distance;
    float heading;
    nav_state_enum state;
} nav_data_t;

void nav_init(void);
void nav_tick(void);
void nav_reset(void);
const nav_data_t *nav_get_data(void);

uint8 nav_enqueue_direct(float distance_m, float speed_mps);
uint8 nav_enqueue_turn(float turn_deg, float speed_mps);
uint8 nav_start_manual(void);
void nav_clear_manual_tasks(void);
uint8 nav_get_manual_task_count(void);

uint8 nav_start_route(float speed_mps);
uint8 nav_is_route_mode(void);
uint16 nav_get_route_index(void);

void nav_cancel_task(void);
nav_state_enum nav_get_state(void);
uint8 nav_is_running(void);

#endif /* CODE_APP_NAV_APP_H_ */
