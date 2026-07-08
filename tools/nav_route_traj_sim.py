#!/usr/bin/env python3
"""
Route mode x-y trajectory simulation: walks the actual preset path (the same
point array pasted into code/driver/nav/nav_route.c's s_nav_route_path[],
read here from tools/nav_route_resampled.c.txt) tick-by-tick using ports of
nav_route.c's pure geometry (nav_route_advance_index / nav_route_compute_target
/ nav_route_compute_bearing_deg / nav_route_is_finished), with a sinusoidal
weave disturbance riding on the pursuit bearing standing in for real
低速画龙 (low-speed snaking).

Plots three x-y traces, per user request:
  - 黑 (black):  the preset path array itself
  - 绿 (green):  the virtual target point (nav_route_compute_target output)
                 at each tick
  - 红 (red):    the vehicle's actual dead-reckoned (x, y), integrated with
                 the same formula as nav_odometry_update_dead_reckoning
                 (x += v*sin(yaw)*dt, y += v*cos(yaw)*dt), one point per
                 10ms tick (the steering PIT period)

This is a kinematic-only, open-loop stand-in — there is no chassis/PID model
connecting a heading command to actual yaw, so "画龙" is injected directly as
a sinusoidal offset on the ideal pursuit bearing rather than emerging from a
simulated control loop (see nav_route_weave_sim.py for the closed-loop
ramp/saturation side of this question).

The vehicle starts at local (0, 0) heading 0 — matching nav_odometry_reset()
at task start — not at the path's first point. That mismatch (discussed
earlier this session) is reproduced faithfully here, not hidden.

Copyright: KimJong
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from nav_route_resample import (DEFAULT_OUTPUT_POINTS, configure_matplotlib_fonts,
                                parse_point_list_text)

Point = Tuple[float, float]

# Keep in sync with nav_route.h defaults.
NAV_ROUTE_LOOKAHEAD_POINTS = 5
NAV_ROUTE_WP_REACH_M = 0.20
STEERING_TICK_S = 0.010  # cpu0_main.c: pit_ms_init(CCU61_CH1, 10)


# -----------------------------------------------------------------------
# Ports of nav_route.c's pure geometry functions
# -----------------------------------------------------------------------

def load_route_path(path: Path = DEFAULT_OUTPUT_POINTS) -> List[Point]:
    """Load the same point list that gets hand-pasted into nav_route.c's
    s_nav_route_path[] — by default tools/nav_route_resampled.c.txt."""
    return parse_point_list_text(path.read_text(encoding="utf-8"))


def route_advance_index(path: Sequence[Point], index: int, robot_x: float, robot_y: float) -> int:
    """Port of nav_route.c::nav_route_advance_index."""
    last_seg = len(path) - 2
    while index < last_seg:
        ax, ay = path[index]
        bx, by = path[index + 1]
        sx, sy = bx - ax, by - ay
        len_sq = sx * sx + sy * sy
        if len_sq < 1.0e-12:
            index += 1
            continue
        t = ((robot_x - ax) * sx + (robot_y - ay) * sy) / len_sq
        if t < 1.0:
            break
        index += 1
    return index


def route_compute_target(path: Sequence[Point], index: int,
                         lookahead: int = NAV_ROUTE_LOOKAHEAD_POINTS) -> Point:
    """Port of nav_route.c::nav_route_compute_target — the real on-hardware
    algorithm. Averages `lookahead` path points ahead of `index`, does NOT
    include the robot's own coordinate (route设计思路.md proves that adding
    it is geometrically equivalent to just shortening the lookahead, since
    robot/target/new-target stay collinear)."""
    last = len(path) - 1
    sum_x = sum_y = 0.0
    for i in range(1, lookahead + 1):
        idx = min(index + i, last)
        sum_x += path[idx][0]
        sum_y += path[idx][1]
    return sum_x / lookahead, sum_y / lookahead


def route_compute_target_with_robot(path: Sequence[Point], index: int, robot_x: float,
                                    robot_y: float, path_lookahead: int = 3) -> Point:
    """Experimental variant, Python-simulation only — NOT the algorithm
    running on the real robot (code/driver/nav/nav_route.c is unchanged).
    Folds the robot's own current (x, y) into the averaging pool alongside
    `path_lookahead` path points ahead of `index` (default 3 path points +
    1 robot point = 4-point average, per user request). route设计思路.md
    already argues this should reduce to the same direction as the 5-point
    path-only average with an effectively shorter lookahead — this variant
    exists to check that claim against a real simulated trajectory."""
    last = len(path) - 1
    sum_x, sum_y = robot_x, robot_y
    for i in range(1, path_lookahead + 1):
        idx = min(index + i, last)
        sum_x += path[idx][0]
        sum_y += path[idx][1]
    count = path_lookahead + 1
    return sum_x / count, sum_y / count


def route_compute_bearing_deg(robot_x: float, robot_y: float, target_x: float, target_y: float) -> float:
    """Port of nav_route.c::nav_route_compute_bearing_deg (x右正/y前正/顺时针为正)."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.degrees(math.atan2(dx, dy))


def route_is_finished(path: Sequence[Point], index: int, robot_x: float, robot_y: float) -> bool:
    """Port of nav_route.c::nav_route_is_finished."""
    last_seg = len(path) - 2
    if index < last_seg:
        return False
    ax, ay = path[last_seg]
    bx, by = path[last_seg + 1]
    sx, sy = bx - ax, by - ay
    len_sq = sx * sx + sy * sy
    t = 1.0 if len_sq < 1.0e-12 else ((robot_x - ax) * sx + (robot_y - ay) * sy) / len_sq
    dist_to_goal = math.hypot(bx - robot_x, by - robot_y)
    return (t >= 1.0) or (dist_to_goal < NAV_ROUTE_WP_REACH_M)


# -----------------------------------------------------------------------
# Simulation
# -----------------------------------------------------------------------

@dataclass
class TrajResult:
    time_s: List[float] = field(default_factory=list)
    robot_xy: List[Point] = field(default_factory=list)
    target_xy: List[Point] = field(default_factory=list)
    route_index: List[int] = field(default_factory=list)
    finished_at_s: Optional[float] = None


def simulate_trajectory(path: Sequence[Point], speed_mps: float, weave_amplitude_deg: float,
                        weave_period_s: float, dt_s: float = STEERING_TICK_S,
                        max_duration_s: float = 60.0, target_mode: str = "path_only",
                        robot4_path_lookahead: int = 3) -> TrajResult:
    """Walk `path` with nav_route's actual index-advance/bearing math and a
    constant forward speed. Actual heading each tick is the ideal pursuit
    bearing (robot -> virtual target point) plus a sinusoidal weave offset;
    position integrates dead-reckoning-style off that heading.

    target_mode selects how the virtual target point is computed:
      "path_only"  — route_compute_target, matches nav_route.c (default).
      "with_robot" — route_compute_target_with_robot, Python-only experiment.
    """
    if target_mode not in ("path_only", "with_robot"):
        raise ValueError(f"unknown target_mode: {target_mode!r}")

    result = TrajResult()

    x = y = 0.0
    index = 0
    omega = (2.0 * math.pi / weave_period_s) if weave_period_s > 0 else 0.0
    n_ticks = int(max_duration_s / dt_s)

    for i in range(n_ticks + 1):
        t = i * dt_s
        index = route_advance_index(path, index, x, y)
        if target_mode == "with_robot":
            target_x, target_y = route_compute_target_with_robot(path, index, x, y,
                                                                  robot4_path_lookahead)
        else:
            target_x, target_y = route_compute_target(path, index)
        bearing_deg = route_compute_bearing_deg(x, y, target_x, target_y)
        yaw_now_deg = bearing_deg + weave_amplitude_deg * math.sin(omega * t)

        result.time_s.append(t)
        result.robot_xy.append((x, y))
        result.target_xy.append((target_x, target_y))
        result.route_index.append(index)

        if route_is_finished(path, index, x, y):
            result.finished_at_s = t
            break

        yaw_rad = math.radians(yaw_now_deg)
        x += speed_mps * math.sin(yaw_rad) * dt_s
        y += speed_mps * math.cos(yaw_rad) * dt_s

    return result


def path_total_length_m(path: Sequence[Point]) -> float:
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
              for i in range(len(path) - 1))


# -----------------------------------------------------------------------
# Plotting (matplotlib imported lazily; core simulation above has no
# plotting dependency)
# -----------------------------------------------------------------------

@dataclass
class TrajSeries:
    label: str
    target_color: str
    robot_color: str
    result: TrajResult


def plot_trajectories(path: Sequence[Point], series: Sequence[TrajSeries],
                      weave_amplitude_deg: float, weave_period_s: float, speed_mps: float) -> None:
    import matplotlib.pyplot as plt

    configure_matplotlib_fonts()

    fig, ax = plt.subplots(figsize=(8.5, 8.5))

    px = [p[0] for p in path]
    py = [p[1] for p in path]
    ax.plot(px, py, color="black", linewidth=1.3, zorder=2, label="预设路径点 (黑)")

    status_parts = []
    for s in series:
        tx = [p[0] for p in s.result.target_xy]
        ty = [p[1] for p in s.result.target_xy]
        ax.plot(tx, ty, color=s.target_color, linewidth=1.0, alpha=0.85, zorder=3,
                label=f"虚拟目标点 ({s.label})")

        rx = [p[0] for p in s.result.robot_xy]
        ry = [p[1] for p in s.result.robot_xy]
        ax.plot(rx, ry, color=s.robot_color, linewidth=1.3, zorder=4,
                label=f"车实际位置 ({s.label}, 10ms/点)")

        status_parts.append(
            f"{s.label}: 到达终点@{s.result.finished_at_s:.1f}s" if s.result.finished_at_s is not None
            else f"{s.label}: 未到达终点")

    ax.scatter([0.0], [0.0], marker="*", s=160, color="#1f77b4", zorder=5,
              label="车起步局部原点 (0,0)")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, linewidth=0.5, alpha=0.35)

    ax.set_title(f"route 轨迹仿真  画龙 A={weave_amplitude_deg:.1f}deg T={weave_period_s:.2f}s  "
                f"速度={speed_mps:.2f}m/s\n{'  |  '.join(status_parts)}")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    plt.show()


# -----------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="仿真 route 模式沿预设路径走的 x-y 轨迹：黑=预设路径点，绿=虚拟目标点，红=车实际位置"
    )
    parser.add_argument("--path", type=Path, default=DEFAULT_OUTPUT_POINTS,
                        help="预设路径点文件（默认 tools/nav_route_resampled.c.txt，"
                            "跟粘进 nav_route.c 的数组一致）")
    parser.add_argument("--speed-mps", type=float, default=0.5, help="恒定前进速度，单位 m/s（默认 0.5）")
    parser.add_argument("--amplitude-deg", type=float, default=10.0, help="画龙振幅，单位 deg（默认 10）")
    parser.add_argument("--period-s", type=float, default=2.0, help="画龙周期，单位秒（默认 2.0）")
    parser.add_argument("--max-duration-s", type=float, default=60.0, help="仿真时长上限，单位秒")
    parser.add_argument("--target-mode", choices=["path_only", "with_robot"], default="path_only",
                        help="path_only=现有设计,匹配nav_route.c,5点均值不含车坐标(默认); "
                            "with_robot=实验变体,仅本仿真脚本用,车坐标+3个路径点共4点均值")
    parser.add_argument("--robot4-path-lookahead", type=int, default=3,
                        help="--target-mode=with_robot 时,车坐标之外再取几个路径点求均值(默认3,"
                            "配合车坐标共4点)")
    parser.add_argument("--compare", action="store_true",
                        help="同时跑 path_only 和 with_robot 两种方案,叠加画在同一张图上对比")
    parser.add_argument("--no-plot", action="store_true", help="只打印文字报告，不弹图")
    return parser


def _run_one(path: List[Point], args, target_mode: str) -> TrajResult:
    return simulate_trajectory(path, args.speed_mps, args.amplitude_deg, args.period_s,
                               max_duration_s=args.max_duration_s, target_mode=target_mode,
                               robot4_path_lookahead=args.robot4_path_lookahead)


def _print_summary(label: str, path: Sequence[Point], result: TrajResult) -> None:
    if result.finished_at_s is not None:
        print(f"[{label}] 到达终点: t={result.finished_at_s:.2f}s, route_index={result.route_index[-1]}")
    else:
        print(f"[{label}] 仿真时长内未到达终点, 最后 route_index={result.route_index[-1]}/{len(path) - 1}")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    path = load_route_path(args.path)
    print(f"预设路径: {len(path)} 点, 弧长 {path_total_length_m(path):.2f}m (来自 {args.path.name})")
    print(f"画龙: A={args.amplitude_deg:.1f}deg T={args.period_s:.2f}s  速度={args.speed_mps:.2f}m/s")

    if args.compare:
        result_path_only = _run_one(path, args, "path_only")
        result_with_robot = _run_one(path, args, "with_robot")
        _print_summary("path_only(5点,不含车坐标)", path, result_path_only)
        _print_summary(f"with_robot(车坐标+{args.robot4_path_lookahead}点)", path, result_with_robot)

        if not args.no_plot:
            series = [
                TrajSeries("path_only,5点", "#2a9d3f", "#e53935", result_path_only),
                TrajSeries("with_robot,4点", "#0088aa", "#ff8c00", result_with_robot),
            ]
            plot_trajectories(path, series, args.amplitude_deg, args.period_s, args.speed_mps)
        return 0

    result = _run_one(path, args, args.target_mode)
    _print_summary(args.target_mode, path, result)

    if not args.no_plot:
        series = [TrajSeries(args.target_mode, "#2a9d3f", "#e53935", result)]
        plot_trajectories(path, series, args.amplitude_deg, args.period_s, args.speed_mps)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
