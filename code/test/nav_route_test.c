/*
 * nav_route_test.c
 *
 *  Minimal validation for nav_route's pure geometry/filter functions.
 *  Expected values for nav_route_advance_index / nav_route_compute_target /
 *  nav_route_is_finished are tied to the current path data embedded in
 *  code/driver/nav/nav_route.c (from tools/nav_route_resampled.c.txt,
 *  160 points) — regenerate these if the path is resampled with different
 *  parameters.
 *  Copyright: KimJong
 */

#include "nav_route_test.h"
#include "../driver/nav/nav_route.h"

static uint8 nav_route_test_close(float a, float b, float eps)
{
    float d = a - b;
    if (d < 0.0f) { d = -d; }
    return (d <= eps) ? 1u : 0u;
}

uint8 nav_route_test_run(void)
{
    uint8 pass = 1u;
    uint16 count = nav_route_point_count();
    float x = 0.0f;
    float y = 0.0f;
    float bearing = 0.0f;
    float yaw_filtered = 0.0f;
    float balance_target = 0.0f;

    pass &= (160u == count) ? 1u : 0u;

    /* advance_index: one-shot jump ahead along the path direction advances
     * across multiple segments; no movement / jumping "behind" a segment's
     * direction does not advance; index is clamped at count-2 (last_seg),
     * it never reaches count-1. */
    pass &= (10u == nav_route_advance_index(0u, -1.22f, 0.80f)) ? 1u : 0u;   /* path[10] */
    pass &= (3u  == nav_route_advance_index(0u, -0.65f, 0.42f)) ? 1u : 0u;   /* path[3] */
    pass &= (5u  == nav_route_advance_index(5u, -0.84f, 0.48f)) ? 1u : 0u;   /* path[5], no movement */
    pass &= ((uint16)(count - 2u) ==
              nav_route_advance_index((uint16)(count - 2u), 0.11f, -0.41f)) ? 1u : 0u; /* path[159] */

    /* compute_target: interior index averages real path points. */
    nav_route_compute_target(0u, &x, &y);
    pass &= nav_route_test_close(x, -0.648f, 0.01f) ? 1u : 0u;
    pass &= nav_route_test_close(y,  0.424f, 0.01f) ? 1u : 0u;

    /* compute_target: index at/near the end clamps every lookahead sample
     * to the final point, so the average equals the final point exactly. */
    nav_route_compute_target((uint16)(count - 1u), &x, &y);
    pass &= nav_route_test_close(x,  0.11f, 1.0e-4f) ? 1u : 0u;
    pass &= nav_route_test_close(y, -0.41f, 1.0e-4f) ? 1u : 0u;

    /* compute_bearing_deg: robot at origin, four cardinal targets. */
    bearing = nav_route_compute_bearing_deg(0.0f, 0.0f, 0.0f, 1.0f);   /* ahead */
    pass &= nav_route_test_close(bearing, 0.0f, 1.0e-3f) ? 1u : 0u;
    bearing = nav_route_compute_bearing_deg(0.0f, 0.0f, 1.0f, 0.0f);   /* right */
    pass &= nav_route_test_close(bearing, 90.0f, 1.0e-3f) ? 1u : 0u;
    bearing = nav_route_compute_bearing_deg(0.0f, 0.0f, -1.0f, 0.0f);  /* left */
    pass &= nav_route_test_close(bearing, -90.0f, 1.0e-3f) ? 1u : 0u;
    bearing = nav_route_compute_bearing_deg(0.0f, 0.0f, 0.0f, -1.0f);  /* behind */
    pass &= (nav_route_test_close(bearing, 180.0f, 1.0e-3f) ||
             nav_route_test_close(bearing, -180.0f, 1.0e-3f)) ? 1u : 0u;

    /* yaw_filter_update: alpha=1 snaps to raw, alpha=0 holds prev, and
     * wrap-around at the +/-180 boundary stays continuous. */
    pass &= nav_route_test_close(nav_route_yaw_filter_update(10.0f, 50.0f, 1.0f), 50.0f, 1.0e-3f) ? 1u : 0u;
    pass &= nav_route_test_close(nav_route_yaw_filter_update(10.0f, 50.0f, 0.0f), 10.0f, 1.0e-3f) ? 1u : 0u;
    yaw_filtered = nav_route_yaw_filter_update(179.0f, -179.0f, 0.5f);
    pass &= nav_route_test_close(yaw_filtered, 180.0f, 1.0e-3f) ? 1u : 0u;

    /* compute_balance_target_deg: local->global conversion must cancel
     * nav_yaw_init exactly when target_yaw_global == yaw_filtered_global. */
    balance_target = nav_route_compute_balance_target_deg(45.0f, 90.0f, 0.0f, 90.0f);
    pass &= nav_route_test_close(balance_target, 45.0f, 1.0e-3f) ? 1u : 0u;

    /* With a real correction: yaw_filtered_local=0 -> global=90(=nav_yaw_init);
     * target_yaw_global=100 -> correction=+10; result = yaw_now(45)+10=55. */
    balance_target = nav_route_compute_balance_target_deg(45.0f, 100.0f, 0.0f, 90.0f);
    pass &= nav_route_test_close(balance_target, 55.0f, 1.0e-3f) ? 1u : 0u;

    /* is_finished: false while index hasn't reached the last segment;
     * false on the last segment while still far from the goal; true once
     * the robot is at (or past) the final point. */
    pass &= (0u == nav_route_is_finished(0u, -0.36f, 0.34f)) ? 1u : 0u;
    pass &= (0u == nav_route_is_finished((uint16)(count - 2u), -0.36f, 0.34f)) ? 1u : 0u;
    pass &= (1u == nav_route_is_finished((uint16)(count - 2u), 0.11f, -0.41f)) ? 1u : 0u;

    return pass;
}
