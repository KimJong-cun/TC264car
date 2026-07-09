/*
 * nav_manual_test.c
 *
 *  Minimal validation for manual DIRECT/TURN command flow.
 *  Copyright: KimJong
 */

#include "nav_manual_test.h"
#include "debug_telemetry.h"
#include "../app/balance_app.h"
#include "../app/nav_app.h"
#include "../app/uart_command.h"
#include "../driver/nav/nav_manual.h"

uint8 nav_manual_test_run(void)
{
    uint8 pass = 1u;
    const nav_data_t *nav = 0;

    /* nav_manual_should_start_decel: no decel while next speed >= current;
     * no decel before NAV_MANUAL_DECEL_TRIGGER_RATIO (0.7) of distance;
     * triggers once both the ratio and the slower-next-speed condition hold. */
    pass &= (0u == nav_manual_should_start_decel(2.9f, 3.0f, 1.2f, 1.2f)) ? 1u : 0u;   /* same speed */
    pass &= (0u == nav_manual_should_start_decel(2.9f, 3.0f, 0.9f, 1.2f)) ? 1u : 0u;   /* speeding up */
    pass &= (0u == nav_manual_should_start_decel(1.9f, 3.0f, 1.2f, 0.9f)) ? 1u : 0u;   /* before 70% (70%=2.1m) */
    pass &= (1u == nav_manual_should_start_decel(2.5f, 3.0f, 1.2f, 0.9f)) ? 1u : 0u;   /* past 70% */
    pass &= (0u == nav_manual_should_start_decel(2.5f, 0.0f, 1.2f, 0.9f)) ? 1u : 0u;   /* zero distance guard */

    debug_telemetry_init();
    nav_reset();
    pass &= (-1.45f == balance_get_pitch_balance_fast()) ? 1u : 0u;
    pass &= (-1.25f == balance_get_pitch_balance_slow()) ? 1u : 0u;
    nav = nav_get_data();
    pass &= (0u == nav_get_manual_task_count()) ? 1u : 0u;
    pass &= (0.0f == nav->target_x) ? 1u : 0u;
    pass &= (0.0f == nav->target_y) ? 1u : 0u;
    pass &= (0u == nav_enqueue_direct(0.0f, 0.5f)) ? 1u : 0u;
    pass &= (0u == nav_enqueue_turn(0.0f, 0.3f)) ? 1u : 0u;
    pass &= (1u == nav_enqueue_direct(1.0f, 0.5f)) ? 1u : 0u;
    pass &= (1u == nav_enqueue_turn(90.0f, 0.3f)) ? 1u : 0u;
    pass &= (2u == nav_get_manual_task_count()) ? 1u : 0u;
    nav_clear_manual_tasks();
    pass &= (0u == nav_get_manual_task_count()) ? 1u : 0u;

    pass &= (1u == uart_command_handle_line("direct 1.0 0.5")) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("turn -90 0.3")) ? 1u : 0u;
    pass &= (2u == nav_get_manual_task_count()) ? 1u : 0u;
    pass &= (DEBUG_TELEMETRY_TRIGGER_RUNNING == debug_telemetry_get_trigger()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("dbgtrig 1")) ? 1u : 0u;
    pass &= (DEBUG_TELEMETRY_TRIGGER_ALWAYS == debug_telemetry_get_trigger()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("dbgtrig 2")) ? 1u : 0u;
    pass &= (DEBUG_TELEMETRY_TRIGGER_ALWAYS == debug_telemetry_get_trigger()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("dbgtrig 0")) ? 1u : 0u;
    pass &= (DEBUG_TELEMETRY_TRIGGER_RUNNING == debug_telemetry_get_trigger()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("fast")) ? 1u : 0u;
    pass &= (BALANCE_LOOP_FAST == balance_get_loop_mode()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("slow")) ? 1u : 0u;
    pass &= (BALANCE_LOOP_SLOW == balance_get_loop_mode()) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("nav")) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("lock")) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("unlock")) ? 1u : 0u;
    pass &= (1u == uart_command_handle_line("start")) ? 1u : 0u;
    pass &= (NAV_STATE_WAIT_START == nav_get_state()) ? 1u : 0u;

    nav_cancel_task();
    pass &= (NAV_STATE_IDLE == nav_get_state()) ? 1u : 0u;
    pass &= (0u == nav_get_manual_task_count()) ? 1u : 0u;

    return pass;
}
