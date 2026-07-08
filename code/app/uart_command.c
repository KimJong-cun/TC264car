/*
 * uart_command.c
 *
 *  UART command dispatch.
 *  Copyright: KimJong
 */

#include "zf_common_headfile.h"
#include "uart_command.h"
#include "../driver/uart/uart_cmd_rx.h"
#include "../driver/uart/uart_cmd_line.h"
#include <stdio.h>
#include <string.h>

static uint32 s_uart_command_rx_overflow_reported = 0u;

typedef struct
{
    const char               *command;
    balance_pid_channel_enum  channel;
    pid_term_enum             term;
} uart_command_balance_pid_entry_t;

static const uart_command_balance_pid_entry_t s_uart_command_balance_pid_table[] =
{
    { "pskp", BALANCE_PID_STEERING, PID_TERM_KP },
    { "pski", BALANCE_PID_STEERING, PID_TERM_KI },
    { "pskd", BALANCE_PID_STEERING, PID_TERM_KD },
    { "pakp", BALANCE_PID_ANGLE,    PID_TERM_KP },
    { "paki", BALANCE_PID_ANGLE,    PID_TERM_KI },
    { "pakd", BALANCE_PID_ANGLE,    PID_TERM_KD },
    { "pgkp", BALANCE_PID_GYRO,     PID_TERM_KP },
    { "pgki", BALANCE_PID_GYRO,     PID_TERM_KI },
    { "pgkd", BALANCE_PID_GYRO,     PID_TERM_KD },
};

static uint8 uart_command_parse_balance_pid_term(const char *command,
                                                 balance_pid_channel_enum *channel,
                                                 pid_term_enum *term)
{
    uint32 index = 0u;
    uint32 count = sizeof(s_uart_command_balance_pid_table) /
                   sizeof(s_uart_command_balance_pid_table[0]);

    if ((0 == command) || (0 == channel) || (0 == term))
    {
        return 0u;
    }

    for (index = 0u; index < count; index++)
    {
        if (0 == strcmp(command, s_uart_command_balance_pid_table[index].command))
        {
            *channel = s_uart_command_balance_pid_table[index].channel;
            *term = s_uart_command_balance_pid_table[index].term;
            return 1u;
        }
    }

    return 0u;
}

static uint8 uart_command_handle_balance_pid(const char *line)
{
    char command[8] = {0};
    float value = 0.0f;
    balance_pid_channel_enum channel = BALANCE_PID_STEERING;
    pid_term_enum term = PID_TERM_KP;

    if (2 != sscanf(line, "%7s %f", command, &value))
    {
        return 0u;
    }

    if (!uart_command_parse_balance_pid_term(command, &channel, &term))
    {
        return 0u;
    }

    if (BALANCE_LOOP_FAST == balance_get_loop_mode())
    {
        balance_set_pid_term(channel, term, value);
        my_printf(WIRELESS_UART_INDEX,
                  "[BPID] fast %s %.4f\r\n",
                  command,
                  (double)value);
        return 1u;
    }

    if (BALANCE_PID_GYRO == channel)
    {
        my_printf(WIRELESS_UART_INDEX, "[BPID] slow gyro ignored\r\n");
        return 1u;
    }

    balance_low_speed_set_pid_term(channel, term, value);
    my_printf(WIRELESS_UART_INDEX,
              "[BPID] slow %s %.4f\r\n",
              command,
              (double)value);
    return 1u;
}

static uint8 uart_command_handle_balance_integral_limit(const char *line)
{
    float limit = 0.0f;

    if (1 != sscanf(line, "pail %f", &limit))
    {
        return 0u;
    }

    balance_low_speed_set_integral_limit(limit);
    my_printf(WIRELESS_UART_INDEX,
              "[BPID] slow angle i_limit=+/-%.1f\r\n",
              (double)((limit >= 0.0f) ? limit : -limit));
    return 1u;
}

static uint8 uart_command_handle_balance_mode(const char *line)
{
    if (0 == strcmp(line, "fast"))
    {
        balance_set_loop_mode(BALANCE_LOOP_FAST);
        my_printf(WIRELESS_UART_INDEX, "[MODE] fast\r\n");
        return 1u;
    }

    if (0 == strcmp(line, "slow"))
    {
        balance_set_loop_mode(BALANCE_LOOP_SLOW);
        my_printf(WIRELESS_UART_INDEX, "[MODE] slow\r\n");
        return 1u;
    }

    return 0u;
}

static uint8 uart_command_handle_nav_task(const char *line)
{
    float arg_a = 0.0f;
    float arg_b = 0.0f;
    const nav_data_t *nav = 0;

    if (0 == strcmp(line, "nav"))
    {
        nav = nav_get_data();
        my_printf(WIRELESS_UART_INDEX,
                  "[NAV] state=%d mode=%s route_idx=%d heading=%.2f yaw=%.2f target=%.2f x=%.3f y=%.3f\r\n",
                  (int)nav_get_state(),
                  nav_is_route_mode() ? "route" : "manual",
                  (int)nav_get_route_index(),
                  (double)nav->heading,
                  (double)yaw,
                  (double)balance_get_heading_target(),
                  (double)nav->x,
                  (double)nav->y);
        return 1u;
    }

    if (2 == sscanf(line, "direct %f %f", &arg_a, &arg_b))
    {
        if (nav_enqueue_direct(arg_a, arg_b))
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] direct %.2fm @ %.2fm/s queued (%d)\r\n",
                      (double)arg_a,
                      (double)arg_b,
                      (int)nav_get_manual_task_count());
        }
        else
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] direct rejected (distance>0, speed>0, idle/done, queue room)\r\n");
        }
        return 1u;
    }

    if (2 == sscanf(line, "turn %f %f", &arg_a, &arg_b))
    {
        if (nav_enqueue_turn(arg_a, arg_b))
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] turn %.1fdeg @ %.2fm/s queued (%d)\r\n",
                      (double)arg_a,
                      (double)arg_b,
                      (int)nav_get_manual_task_count());
        }
        else
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] turn rejected (deg!=0, speed>0, idle/done, queue room)\r\n");
        }
        return 1u;
    }

    if (0 == strcmp(line, "start"))
    {
        if (nav_start_manual())
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] manual tasks wait for start\r\n");
        }
        else
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] start rejected (manual queue not ready or nav busy)\r\n");
        }
        return 1u;
    }

    if (0 == strcmp(line, "clear"))
    {
        nav_clear_manual_tasks();
        my_printf(WIRELESS_UART_INDEX,
                  "[NAV] manual queue=%d\r\n",
                  (int)nav_get_manual_task_count());
        return 1u;
    }

    if (1 == sscanf(line, "route %f", &arg_a))
    {
        if (nav_start_route(arg_a))
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] route @ %.2fm/s wait for start\r\n",
                      (double)arg_a);
        }
        else
        {
            my_printf(WIRELESS_UART_INDEX,
                      "[NAV] route rejected (speed>0, idle/done)\r\n");
        }
        return 1u;
    }

    if (0 == strcmp(line, "cancel"))
    {
        nav_cancel_task();
        my_printf(WIRELESS_UART_INDEX, "[NAV] canceled\r\n");
        return 1u;
    }

    return 0u;
}

static uint8 uart_command_handle_debug_mode(const char *line)
{
    int mode = 0;

    if (1 != sscanf(line, "dbg %d", &mode))
    {
        return 0u;
    }

    if ((mode < (int)DEBUG_TELEMETRY_MODE_OFF) ||
        (mode > (int)DEBUG_TELEMETRY_MODE_IMU))
    {
        my_printf(WIRELESS_UART_INDEX,
                  "[DBG] invalid mode %d (0=off 1=pidtest 2=xytest 3=imutest)\r\n",
                  mode);
        return 1u;
    }

    debug_telemetry_set_mode((debug_telemetry_mode_enum)mode);
    my_printf(WIRELESS_UART_INDEX,
              "[DBG] mode=%d (0=off 1=pidtest 2=xytest 3=imutest), trigger=%d\r\n",
              mode,
              (int)debug_telemetry_get_trigger());
    return 1u;
}

static uint8 uart_command_handle_debug_trigger(const char *line)
{
    int trigger = 0;

    if (1 != sscanf(line, "dbgtrig %d", &trigger))
    {
        return 0u;
    }

    if ((trigger < (int)DEBUG_TELEMETRY_TRIGGER_RUNNING) ||
        (trigger > (int)DEBUG_TELEMETRY_TRIGGER_ALWAYS))
    {
        my_printf(WIRELESS_UART_INDEX,
                  "[DBG] invalid trigger %d (0=running 1=always)\r\n",
                  trigger);
        return 1u;
    }

    debug_telemetry_set_trigger((debug_telemetry_trigger_enum)trigger);
    my_printf(WIRELESS_UART_INDEX,
              "[DBG] trigger=%d (%s)\r\n",
              trigger,
              (DEBUG_TELEMETRY_TRIGGER_ALWAYS == (debug_telemetry_trigger_enum)trigger) ?
                  "always" : "running");
    return 1u;
}

static uint8 uart_command_handle_debug_fall_cancel(const char *line)
{
    int enable = 0;

    if (1 != sscanf(line, "dbgfall %d", &enable))
    {
        return 0u;
    }

    debug_telemetry_set_fall_cancel((uint8)((0 != enable) ? 1 : 0));
    my_printf(WIRELESS_UART_INDEX,
              "[DBG] fall_cancel=%d (1=stop print on fall, 0=keep printing)\r\n",
              (int)debug_telemetry_get_fall_cancel());
    return 1u;
}

static uint8 uart_command_handle_balance_zero(const char *line)
{
    float zero = 0.0f;

    /* "bzero" — query both zeros */
    if (0 == strcmp(line, "bzero"))
    {
        my_printf(WIRELESS_UART_INDEX,
                  "[BZERO] fast=%.4f slow=%.4f active=%s\r\n",
                  (double)balance_get_pitch_balance_fast(),
                  (double)balance_get_pitch_balance_slow(),
                  (BALANCE_LOOP_SLOW == balance_get_loop_mode()) ? "slow" : "fast");
        return 1u;
    }

    /* "bzerofast <v>" — explicitly set fast mode zero */
    if (1 == sscanf(line, "bzerofast %f", &zero))
    {
        balance_set_pitch_balance(zero);
        my_printf(WIRELESS_UART_INDEX, "[BZERO] fast=%.4f\r\n", (double)zero);
        return 1u;
    }

    /* "bzeroslow <v>" — explicitly set slow mode zero */
    if (1 == sscanf(line, "bzeroslow %f", &zero))
    {
        balance_set_pitch_balance_slow(zero);
        my_printf(WIRELESS_UART_INDEX, "[BZERO] slow=%.4f\r\n", (double)zero);
        return 1u;
    }

    /* "bzero <v>" / "pzero <v>" — set current active mode's zero */
    if ((1 == sscanf(line, "bzero %f", &zero)) ||
        (1 == sscanf(line, "pzero %f", &zero)))
    {
        if (BALANCE_LOOP_SLOW == balance_get_loop_mode())
        {
            balance_set_pitch_balance_slow(zero);
            my_printf(WIRELESS_UART_INDEX, "[BZERO] slow=%.4f\r\n", (double)zero);
        }
        else
        {
            balance_set_pitch_balance(zero);
            my_printf(WIRELESS_UART_INDEX, "[BZERO] fast=%.4f\r\n", (double)zero);
        }
        return 1u;
    }

    return 0u;
}

static uint8 uart_command_handle_servo_lock(const char *line)
{
    if (0 == strcmp(line, "lock"))
    {
        nav_cancel_task();
        balance_set_servo_center_lock(1u);
        my_printf(WIRELESS_UART_INDEX, "[SERVO] center locked\r\n");
        return 1u;
    }

    if (0 == strcmp(line, "unlock"))
    {
        balance_set_servo_center_lock(0u);
        my_printf(WIRELESS_UART_INDEX, "[SERVO] center unlocked\r\n");
        return 1u;
    }

    return 0u;
}

static void uart_command_wireless_uart_callback(void)
{
    uint8 byte = 0u;

    while (uart_query_byte(WIRELESS_UART_INDEX, &byte))
    {
        wireless_uart_store_received_byte(byte);
        uart_cmd_rx_push_byte(byte);
    }
}

void uart_command_init(void)
{
    uart_cmd_rx_init();
    uart_cmd_line_init();
    s_uart_command_rx_overflow_reported = 0u;
    set_wireless_type(WIRELESS_UART, uart_command_wireless_uart_callback);
}

void uart_command_poll(void)
{
    uint8 byte = 0u;
    char *line = 0;
    uart_cmd_line_result_enum result = UART_CMD_LINE_RESULT_NONE;
    uint32 overflow_count = uart_cmd_rx_get_overflow_count();

    if (overflow_count != s_uart_command_rx_overflow_reported)
    {
        s_uart_command_rx_overflow_reported = overflow_count;
    }

    while (uart_cmd_rx_pop_byte(&byte))
    {
        result = uart_cmd_line_push_byte(byte, &line);

        if (UART_CMD_LINE_RESULT_READY == result)
        {
            uart_command_handle_line(line);
        }
    }
}

void uart_command_rx_callback(void)
{
    uint8 byte = 0u;

    while (uart_query_byte(UART_1, &byte))
    {
        uart_cmd_rx_push_byte(byte);
    }
}

uint8 uart_command_handle_line(const char *line)
{
    if (0 == line)
    {
        return 0u;
    }

    if (uart_command_handle_balance_pid(line))
    {
        return 1u;
    }

    if (uart_command_handle_balance_integral_limit(line))
    {
        return 1u;
    }

    if (uart_command_handle_balance_mode(line))
    {
        return 1u;
    }

    if (uart_command_handle_balance_zero(line))
    {
        return 1u;
    }

    if (uart_command_handle_servo_lock(line))
    {
        return 1u;
    }

    if (uart_command_handle_debug_fall_cancel(line))
    {
        return 1u;
    }

    if (uart_command_handle_debug_trigger(line))
    {
        return 1u;
    }

    if (uart_command_handle_debug_mode(line))
    {
        return 1u;
    }

    if (uart_command_handle_nav_task(line))
    {
        return 1u;
    }

    return 0u;
}
