/*
 * nav_route.h
 *
 *  Created on: 2026-07-04
 *      Author: KimJong
 *  Description: Preset-path route following — virtual-target-point averaging
 *               over a fixed-arc-length-resampled waypoint array, plus a
 *               yaw feedback filter to damp low-speed weaving (画龙) before
 *               it reaches the navigation decision. No pure-pursuit /
 *               cross-track blend — see route设计思路.md for the full
 *               rationale and the algorithm design.
 *               Pure geometry/filter functions only; the path array below
 *               is fixed read-only data (from tools/nav_route_resampled.c.txt),
 *               not mutable state — callers (nav_app.c) own the state
 *               machine and apply targets via balance_app/motor_app.
 *  Copyright: KimJong
 */

#ifndef CODE_DRIVER_NAV_NAV_ROUTE_H_
#define CODE_DRIVER_NAV_NAV_ROUTE_H_

#include "zf_common_headfile.h"

/* 预瞄点数：往后取几个重采样点求平均生成虚拟目标点。配合 nav_route.c 里
 * 路径数组的重采样间距（来自 tools/nav_route_resampled.c.txt，当前 0.10m/点），
 * 预瞄距离 ~= NAV_ROUTE_LOOKAHEAD_POINTS × 0.10m。上车实测后再调。 */
#define NAV_ROUTE_LOOKAHEAD_POINTS      5u

/* 当前 yaw 反馈的 EMA 系数，专门滤低速画龙噪声，不影响 dead reckoning 积分
 * 和转向环实时反馈。上车实测画龙振幅/周期后再调，
 * 并核对 route设计思路.md 里斜坡限幅预算（问题1）是否够用。 */
#define NAV_ROUTE_YAW_FILTER_ALPHA       0.15f

/* 终点判定阈值：最后一段投影 t>=1 或离终点距离小于此值即算到达。 */
#define NAV_ROUTE_WP_REACH_M             0.20f

uint16 nav_route_point_count(void);

/* 路径数组第 index 个点的坐标（越界钳位到最后一点）。用于车启动 route 时
 * 把 odometry 位置对齐到 path[0]，而不是车物理启动位置的本地 (0,0)。 */
void nav_route_get_point(uint16 index, float *out_x, float *out_y);

/* 投影推进：把 index 所在段与 robot 位置投影，t>=1 就推进到下一段，
 * 跨段循环直到 t<1 或到达最后一段。返回推进后的段起点索引。 */
uint16 nav_route_advance_index(uint16 index, float robot_x, float robot_y);

/* 从 index 往后取 NAV_ROUTE_LOOKAHEAD_POINTS 个点求算术平均，得到虚拟目标点。
 * 超出数组末尾的索引钳位到最后一点。不把 robot 坐标加入平均池
 * （数学上证明对纠偏无效，只会缩短预瞄距离，见 route设计思路.md）。 */
void nav_route_compute_target(uint16 index, float *out_x, float *out_y);

/* robot-to-carrot 方位角：atan2(dx, dy)，参数顺序对应本项目 heading 定义
 * （x右正/y前正/顺时针为正/y轴为0°基准）。纠偏由方位起点选 robot 位置
 * 自然产生，不需要额外的 cross-track blend 或 P 项。 */
float nav_route_compute_bearing_deg(float robot_x, float robot_y, float target_x, float target_y);

/* 当前 yaw（局部坐标，即 nav_odometry pose->heading）的 EMA 滤波。
 * 纯函数，状态由调用者持有并传入传出。 */
float nav_route_yaw_filter_update(float prev_local_deg, float raw_local_deg, float alpha);

/* 把 route设计思路.md 第7步的 error = target_yaw_global - yaw_filtered_global
 * 转换成喂给 balance_set_heading_target 的绝对目标角，让 balance_steering_loop
 * 内部自己算出来的误差正好等于这个 error，不需要修改 balance_app.c。
 * yaw_filtered_local_deg 是局部坐标（nav_odometry 的 heading），内部会加上
 * nav_yaw_init_deg 转成全局坐标再和 target_yaw_global_deg 比较——不要在外面
 * 先转换好再传进来，避免局部/全局坐标混用导致恒定偏移。 */
float nav_route_compute_balance_target_deg(float yaw_now_global_deg,
                                           float target_yaw_global_deg,
                                           float yaw_filtered_local_deg,
                                           float nav_yaw_init_deg);

/* 终点判定：index 是否已推进到最后一段，且最后一段投影 t>=1 或
 * 离终点距离 < NAV_ROUTE_WP_REACH_M。 */
uint8 nav_route_is_finished(uint16 index, float robot_x, float robot_y);

#endif /* CODE_DRIVER_NAV_NAV_ROUTE_H_ */
