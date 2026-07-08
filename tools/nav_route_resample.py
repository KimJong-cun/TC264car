#!/usr/bin/env python3
"""
Route path resampler for rc's pure-tracking-free nav_route design.

Reads a polyline of hand-placed waypoints (same bare "{ x, y }," list format
used by rb's nav_path_editor.py / nav_path_points.c.txt), resamples it to a
fixed arc-length spacing, and saves the result as a bare C-style point list
txt file under tools/. That txt is later pasted into nav_route.c by hand
when wiring up code/driver/nav/nav_route.c — this tool never touches .c
files directly.

Why resample here instead of on the MCU: the resampled path is static data,
computed once when the path is finalized, not every tick. See
"rc版本/route设计思路.md" for the full rationale (fixed spacing makes the
"average next N points" lookahead in nav_route correspond to a fixed,
tunable physical distance instead of drifting with how densely the original
waypoints happen to be placed).

Copyright: KimJong
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

Point = Tuple[float, float]

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPO_ROOT / "tools"

DEFAULT_INPUT_POINTS = TOOLS_DIR / "nav_route_points.c.txt"
DEFAULT_OUTPUT_POINTS = TOOLS_DIR / "nav_route_resampled.c.txt"

# rb's editor lives one project folder over; used only to bootstrap a first
# input file if rc doesn't have one of its own yet (see load_input_points).
RB_FALLBACK_POINTS = REPO_ROOT.parent / "rb版本" / "tools" / "nav_path_points.c.txt"

DEFAULT_STEP_M = 0.10
DEFAULT_FILLET_RADIUS_M = 0.30

POINT_RE = re.compile(
    r"\{\s*(?P<x>[+-]?(?:\d+(?:\.\d*)?|\.\d+))f?\s*,\s*"
    r"(?P<y>[+-]?(?:\d+(?:\.\d*)?|\.\d+))f?\s*\}",
)


# -----------------------------------------------------------------------
# Parsing / formatting (pure, no matplotlib — importable and testable
# without a display or the matplotlib dependency).
# -----------------------------------------------------------------------

def parse_point_list_text(text: str) -> List[Point]:
    points: List[Point] = []
    for match in POINT_RE.finditer(text):
        points.append((float(match.group("x")), float(match.group("y"))))

    if not points:
        raise ValueError("点列表中没有可解析路径点")
    return points


def load_input_points(path: Path = DEFAULT_INPUT_POINTS) -> List[Point]:
    if path.exists():
        return parse_point_list_text(path.read_text(encoding="utf-8"))

    if RB_FALLBACK_POINTS.exists():
        points = parse_point_list_text(RB_FALLBACK_POINTS.read_text(encoding="utf-8"))
        path.write_text(format_point_list_c(points), encoding="utf-8")
        return points

    raise FileNotFoundError(
        f"找不到输入点列表: {path}（也没有可回退的 {RB_FALLBACK_POINTS}）"
    )


def format_point_list_c(points: Sequence[Point]) -> str:
    lines = [f"    {{{x:6.2f}f,{y:5.2f}f }}," for x, y in points]
    return "\n".join(lines) + "\n"


def save_point_list(points: Sequence[Point], path: Path) -> None:
    path.write_text(format_point_list_c(points), encoding="utf-8")


def format_curvature_comment(curvature: Sequence[float], per_line: int = 10) -> str:
    """Render a resampled path's per-point curvature as a comment block.
    Plain comma-separated floats, no `{ }` — POINT_RE only matches bracketed
    pairs, so this never gets mistaken for a path point when the file is
    re-parsed by load_input_points."""
    lines = [f"/* curvature_1_per_m[{len(curvature)}] "
            f"(unsigned |kappa|=|dtheta|/ds, path endpoints=0.0): */"]
    for i in range(0, len(curvature), per_line):
        chunk = curvature[i:i + per_line]
        values = ", ".join(f"{k:.4f}" for k in chunk)
        lines.append(f"/*   {values} */")
    return "\n".join(lines) + "\n"


def format_resample_summary_comment(result: "ResampleResult", fillet_radius_m: float) -> str:
    return (f"/* nav_route resample summary (tools/nav_route_resample.py): "
            f"count={result.count} total_len_m={result.total_length_m:.2f} "
            f"step_m={result.step_m:.2f} fillet_radius_m={fillet_radius_m:.2f} */\n")


def save_points_file_with_resample_header(points: Sequence[Point], path: Path,
                                          result: "ResampleResult", fillet_radius_m: float,
                                          curvature: Sequence[float]) -> None:
    """Save the raw waypoint list to `path` (same body format as
    save_point_list) with a leading comment block recording the most recent
    resample's point count/arc length and per-resampled-point curvature.
    The comment describes the *resampled* path (see compute_curvature_per_point),
    not these raw points, so it stays valid even though the body here is the
    hand-placed waypoints, not the resampled ones."""
    header = format_resample_summary_comment(result, fillet_radius_m)
    header += format_curvature_comment(curvature)
    path.write_text(header + format_point_list_c(points), encoding="utf-8")


# -----------------------------------------------------------------------
# Corner fillets — replace each interior vertex with a circular arc so the
# path handed to nav_route has continuous curvature at corners instead of
# the polyline's instantaneous heading jumps. Endpoints (path start/end)
# are never filleted; only interior vertices. Segments between fillets stay
# perfectly straight — this is a local corner-cut, not a spline over the
# whole path. See "rc版本/route设计思路.md" for why curvature-continuity at
# corners matters for this bicycle-model vehicle.
# -----------------------------------------------------------------------

def _unit(v: Point) -> Point:
    n = math.hypot(v[0], v[1])
    if n < 1.0e-12:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


@dataclass(frozen=True)
class Fillet:
    center: Point
    radius_m: float
    start_angle: float   # radians, angle from center to the incoming tangent point
    delta_angle: float   # signed sweep in radians (wrapped to (-pi, pi])
    trim_back: Point     # tangent point on the incoming segment
    trim_fwd: Point      # tangent point on the outgoing segment


def compute_fillets(points: Sequence[Point], radius_m: float) -> dict:
    """Compute a per-vertex circular fillet for every interior vertex whose
    turn angle is neither ~0 (straight through, no fillet needed) nor ~180
    (a dead-reversal, where a fillet isn't geometrically meaningful).

    The requested radius_m is clamped per-vertex so the tangent trim never
    exceeds half of either adjacent segment's length — this guarantees a
    segment shared by two consecutive fillets never has its two trims
    overlap, regardless of what radius the neighboring vertex ends up using.
    """
    fillets: dict = {}
    if radius_m <= 0.0 or len(points) < 3:
        return fillets

    seg_lengths = [math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
                  for i in range(len(points) - 1)]

    for v in range(1, len(points) - 1):
        p_prev = points[v - 1]
        p = points[v]
        p_next = points[v + 1]

        u1 = _unit((p_prev[0] - p[0], p_prev[1] - p[1]))
        u2 = _unit((p_next[0] - p[0], p_next[1] - p[1]))
        if u1 == (0.0, 0.0) or u2 == (0.0, 0.0):
            continue  # duplicate points, nothing meaningful to fillet

        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        phi = math.acos(dot)
        if phi > math.pi - 1.0e-4 or phi < 1.0e-4:
            continue  # straight-through or a hairpin reversal: skip

        max_trim = min(seg_lengths[v - 1], seg_lengths[v]) / 2.0
        if max_trim < 1.0e-6:
            continue

        tan_half = math.tan(phi / 2.0)
        r_eff = min(radius_m, max_trim * tan_half)
        if r_eff < 1.0e-6:
            continue

        trim_len = r_eff / tan_half
        t_back = (p[0] + u1[0] * trim_len, p[1] + u1[1] * trim_len)
        t_fwd = (p[0] + u2[0] * trim_len, p[1] + u2[1] * trim_len)

        bis = _unit((u1[0] + u2[0], u1[1] + u2[1]))
        dist_to_center = r_eff / math.sin(phi / 2.0)
        center = (p[0] + bis[0] * dist_to_center, p[1] + bis[1] * dist_to_center)

        a1 = math.atan2(t_back[1] - center[1], t_back[0] - center[0])
        a2 = math.atan2(t_fwd[1] - center[1], t_fwd[0] - center[0])
        delta = math.atan2(math.sin(a2 - a1), math.cos(a2 - a1))

        fillets[v] = Fillet(center=center, radius_m=r_eff, start_angle=a1,
                            delta_angle=delta, trim_back=t_back, trim_fwd=t_fwd)

    return fillets


@dataclass(frozen=True)
class PathSegment:
    """One piece of the filleted path: a straight line or a circular arc."""
    kind: str  # "line" | "arc"
    length: float
    p0: Point = (0.0, 0.0)
    p1: Point = (0.0, 0.0)
    center: Point = (0.0, 0.0)
    radius: float = 0.0
    start_angle: float = 0.0
    delta_angle: float = 0.0

    def point_at(self, t: float) -> Point:
        """t is the fraction (0..1) of this segment's own length."""
        if self.kind == "line":
            return (self.p0[0] + t * (self.p1[0] - self.p0[0]),
                    self.p0[1] + t * (self.p1[1] - self.p0[1]))
        angle = self.start_angle + self.delta_angle * t
        return (self.center[0] + self.radius * math.cos(angle),
                self.center[1] + self.radius * math.sin(angle))


def build_filleted_segments(points: Sequence[Point], fillets: dict) -> List[PathSegment]:
    """Splice fillets into the polyline: each original segment gets trimmed
    at either end wherever its endpoint vertex has a fillet, and an arc
    segment is inserted between the trim points at every filleted vertex.
    With an empty fillets dict this reduces to the plain polyline.
    """
    segments: List[PathSegment] = []
    n = len(points)

    for i in range(n - 1):
        start = fillets[i].trim_fwd if i in fillets else points[i]
        end = fillets[i + 1].trim_back if (i + 1) in fillets else points[i + 1]
        seg_len = math.hypot(end[0] - start[0], end[1] - start[1])
        if seg_len > 1.0e-9:
            segments.append(PathSegment(kind="line", length=seg_len, p0=start, p1=end))

        if (i + 1) in fillets:
            f = fillets[i + 1]
            arc_len = f.radius_m * abs(f.delta_angle)
            if arc_len > 1.0e-9:
                segments.append(PathSegment(kind="arc", length=arc_len, center=f.center,
                                            radius=f.radius_m, start_angle=f.start_angle,
                                            delta_angle=f.delta_angle))

    return segments


def sample_segments_dense(segments: Sequence[PathSegment], arc_samples: int = 16) -> List[Point]:
    """Densely sample the filleted path for plotting a smooth curve (lines
    need only their two endpoints; arcs get arc_samples extra points)."""
    if not segments:
        return []
    points = [segments[0].point_at(0.0)]
    for seg in segments:
        if seg.kind == "line":
            points.append(seg.point_at(1.0))
        else:
            for k in range(1, arc_samples + 1):
                points.append(seg.point_at(k / arc_samples))
    return points


# -----------------------------------------------------------------------
# Resampling core
# -----------------------------------------------------------------------

@dataclass(frozen=True)
class ResampleResult:
    points: List[Point]
    total_length_m: float
    step_m: float

    @property
    def count(self) -> int:
        return len(self.points)


def resample_segments_by_arc_length(segments: Sequence[PathSegment], step_m: float) -> ResampleResult:
    """Walk the (possibly filleted) path at fixed arc-length spacing. Mirrors
    the "walk forward along the polyline" logic used by rb's
    nav_pp_find_lookahead, but run once here over the whole path — and now
    generalized to interpolate within arc segments, not just lines.

    The path's final point is always kept as the last output point (even if
    it doesn't land exactly on a step_m multiple), so the resampled path
    never falls short of the intended goal.
    """
    if step_m <= 0.0:
        raise ValueError("step_m 必须为正数")
    if not segments:
        return ResampleResult(points=[], total_length_m=0.0, step_m=step_m)

    cum = [0.0]
    for seg in segments:
        cum.append(cum[-1] + seg.length)
    total = cum[-1]

    if total <= 1.0e-9:
        return ResampleResult(points=[segments[0].point_at(0.0)], total_length_m=0.0, step_m=step_m)

    n_steps = int(total // step_m)
    resampled: List[Point] = []
    seg_idx = 0
    last_seg = len(segments) - 1

    for i in range(n_steps + 1):
        target = i * step_m

        while seg_idx < last_seg and cum[seg_idx + 1] < target:
            seg_idx += 1

        seg = segments[seg_idx]
        local = target - cum[seg_idx]
        t = 0.0 if seg.length < 1.0e-9 else max(0.0, min(1.0, local / seg.length))
        resampled.append(seg.point_at(t))

    last_point = segments[-1].point_at(1.0)
    lx, ly = resampled[-1]
    if math.hypot(last_point[0] - lx, last_point[1] - ly) > 1.0e-6:
        resampled.append(last_point)

    return ResampleResult(points=resampled, total_length_m=total, step_m=step_m)


def resample_path(points: Sequence[Point], step_m: float, fillet_radius_m: float) -> ResampleResult:
    """High-level entry point: fillet the corners, then resample by arc
    length. fillet_radius_m=0 reduces to resampling the plain polyline."""
    fillets = compute_fillets(points, fillet_radius_m)
    segments = build_filleted_segments(points, fillets)
    return resample_segments_by_arc_length(segments, step_m)


def compute_curvature_per_point(points: Sequence[Point]) -> List[float]:
    """Unsigned curvature |kappa| = |delta_theta| / ds at each resampled
    point, estimated from the turn angle between the incoming and outgoing
    chords divided by their average length. Path endpoints have no such
    neighbor pair and are reported as 0.0 — consistent with compute_fillets
    never touching the path's start/end vertex either."""
    n = len(points)
    curvature = [0.0] * n

    for i in range(1, n - 1):
        p_prev, p, p_next = points[i - 1], points[i], points[i + 1]
        v1 = (p[0] - p_prev[0], p[1] - p_prev[1])
        v2 = (p_next[0] - p[0], p_next[1] - p[1])
        len1 = math.hypot(*v1)
        len2 = math.hypot(*v2)
        if len1 < 1.0e-9 or len2 < 1.0e-9:
            continue

        cos_theta = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (len1 * len2)))
        delta_theta = math.acos(cos_theta)
        ds = (len1 + len2) / 2.0
        curvature[i] = delta_theta / ds if ds > 1.0e-9 else 0.0

    return curvature


# -----------------------------------------------------------------------
# Optional visualization (matplotlib imported lazily so the resampling
# math above stays usable/testable without the dependency installed).
# -----------------------------------------------------------------------

def configure_matplotlib_fonts() -> None:
    import matplotlib

    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False
    # matplotlib 默认把 's' 绑定为保存截图、'backspace' 绑定为视图后退，
    # 跟本工具自己的 s(保存点列表)/backspace(删除选中点) 快捷键冲突，禁用掉。
    matplotlib.rcParams["keymap.save"] = []
    matplotlib.rcParams["keymap.back"] = []


class ResampleEditor:
    """Interactive editor: drag the ORIGINAL waypoints, resampled overlay
    (red) recomputes live so you can see how the fixed-arc-length points
    react to a path edit before saving.

    Mirrors rb's nav_path_editor.py interaction model (drag / insert / delete
    / save on key press) so the two tools feel consistent.
    """

    def __init__(self, points: List[Point], points_path: Path, output_path: Path,
                step_m: float, fillet_radius_m: float) -> None:
        self.points = list(points)
        self.points_path = points_path
        self.output_path = output_path
        self.step_m = step_m
        self.fillet_radius_m = fillet_radius_m
        self.selected: int | None = None
        self.dragging = False

        import matplotlib.pyplot as plt

        configure_matplotlib_fonts()
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(8.5, 8.5))
        self.status = self.fig.text(0.02, 0.02, "", fontsize=9)

        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def run(self) -> None:
        self.redraw()
        self.plt.show()

    def _fillets(self) -> dict:
        return compute_fillets(self.points, self.fillet_radius_m)

    def _segments(self) -> List[PathSegment]:
        return build_filleted_segments(self.points, self._fillets())

    def _resample(self) -> ResampleResult:
        return resample_segments_by_arc_length(self._segments(), self.step_m)

    def redraw(self) -> None:
        self.ax.clear()

        ox = [p[0] for p in self.points]
        oy = [p[1] for p in self.points]
        self.ax.plot(ox, oy, color="#1f77b4", linewidth=0.9, alpha=0.30, linestyle=":",
                    zorder=1, label="原始折线(可拖拽)")
        self.ax.scatter(ox, oy, s=70, color="#ffcc00", edgecolor="#1d1d1d", zorder=5)

        for index, (x, y) in enumerate(self.points):
            self.ax.text(x + 0.03, y + 0.03, str(index), fontsize=8, color="#111111", zorder=6)

        if self.selected is not None and 0 <= self.selected < len(self.points):
            sx, sy = self.points[self.selected]
            self.ax.scatter([sx], [sy], s=140, facecolors="none", edgecolors="#e53935",
                            linewidths=2.0, zorder=7)

        segments = self._segments()
        dense = sample_segments_dense(segments)
        if dense:
            dx = [p[0] for p in dense]
            dy = [p[1] for p in dense]
            self.ax.plot(dx, dy, color="#2a9d3f", linewidth=1.3, alpha=0.8, zorder=2,
                        label=f"倒圆角路径(r={self.fillet_radius_m:.2f}m，整体仍为折线)")

        try:
            result = self._resample()
        except ValueError:
            result = None

        if result is not None and result.count > 0:
            rx = [p[0] for p in result.points]
            ry = [p[1] for p in result.points]
            self.ax.scatter(rx, ry, s=14, color="#e53935", zorder=4,
                            label=f"重采样点(step={self.step_m:.2f}m)")
            length_txt = f"  弧长={result.total_length_m:.2f}m  重采样点数={result.count}"
        else:
            length_txt = ""

        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x / m")
        self.ax.set_ylabel("y / m")
        self.ax.grid(True, linewidth=0.5, alpha=0.35)
        self.ax.legend(loc="best")
        self.ax.set_title(f"拖拽原始点，圆角(绿)/重采样(红)实时刷新{length_txt}")
        self._set_status(
            "拖拽:移动点  a:在选中点后插入  delete:删除选中点  "
            "s:保存原始点列表  w:保存重采样结果  "
            f"[ ]:重采样间距(当前{self.step_m:.2f}m)  "
            f"- +:圆角半径(当前{self.fillet_radius_m:.2f}m)"
        )
        self.fig.canvas.draw_idle()

    def _set_status(self, message: str) -> None:
        self.status.set_text(message)

    def _nearest_point(self, x: float, y: float, max_dist: float = 0.16) -> int | None:
        best_index: int | None = None
        best_dist = max_dist
        for index, (px, py) in enumerate(self.points):
            dist = math.hypot(px - x, py - y)
            if dist <= best_dist:
                best_index = index
                best_dist = dist
        return best_index

    def on_press(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        nearest = self._nearest_point(event.xdata, event.ydata)
        if nearest is None:
            return
        self.selected = nearest
        self.dragging = True
        self.redraw()

    def on_release(self, event) -> None:
        self.dragging = False

    def on_motion(self, event) -> None:
        if not self.dragging or self.selected is None:
            return
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        self.points[self.selected] = (round(float(event.xdata), 3), round(float(event.ydata), 3))
        self.redraw()

    def on_key(self, event) -> None:
        if event.key == "s":
            save_point_list(self.points, self.points_path)
            self._set_status(f"已保存原始点列表: {self.points_path.name}")
            self.fig.canvas.draw_idle()
        elif event.key == "w":
            result = self._resample()
            save_point_list(result.points, self.output_path)
            curvature = compute_curvature_per_point(result.points)
            save_points_file_with_resample_header(
                self.points, self.points_path, result, self.fillet_radius_m, curvature)
            self._set_status(
                f"已保存重采样结果: {self.output_path.name}；"
                f"已在 {self.points_path.name} 头部写入 count={result.count} 和曲率注释")
            self.fig.canvas.draw_idle()
        elif event.key == "a":
            self.insert_after_selected()
        elif event.key in ("delete", "backspace"):
            self.delete_selected()
        elif event.key == "[":
            self.step_m = max(0.02, round(self.step_m - 0.02, 3))
            self.redraw()
        elif event.key == "]":
            self.step_m = round(self.step_m + 0.02, 3)
            self.redraw()
        elif event.key in ("-", "_"):
            self.fillet_radius_m = max(0.0, round(self.fillet_radius_m - 0.02, 3))
            self.redraw()
        elif event.key in ("+", "="):
            self.fillet_radius_m = round(self.fillet_radius_m + 0.02, 3)
            self.redraw()

    def insert_after_selected(self) -> None:
        if self.selected is None:
            self._set_status("先选中一个原始点，再按 a 插入")
            self.fig.canvas.draw_idle()
            return
        index = self.selected
        if index + 1 < len(self.points):
            x = (self.points[index][0] + self.points[index + 1][0]) * 0.5
            y = (self.points[index][1] + self.points[index + 1][1]) * 0.5
        else:
            x, y = self.points[index]
            y += 0.20
        self.points.insert(index + 1, (round(x, 3), round(y, 3)))
        self.selected = index + 1
        self.redraw()

    def delete_selected(self) -> None:
        if self.selected is None:
            self._set_status("先选中一个原始点，再删除")
            self.fig.canvas.draw_idle()
            return
        if len(self.points) <= 2:
            self._set_status("至少保留 2 个原始点")
            self.fig.canvas.draw_idle()
            return
        del self.points[self.selected]
        self.selected = None
        self.redraw()


# -----------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按固定弧长重采样导航路径点，导出给 rc 的 nav_route 使用"
    )
    parser.add_argument("--points", type=Path, default=DEFAULT_INPUT_POINTS,
                        help="输入的原始航点列表（缺失时会尝试从 rb 版本的点列表引导创建）")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_M,
                        help=f"重采样间距，单位 m（默认 {DEFAULT_STEP_M}）")
    parser.add_argument("--fillet-radius", type=float, default=DEFAULT_FILLET_RADIUS_M,
                        help=f"拐点倒圆角半径，单位 m，0 表示不倒角（默认 {DEFAULT_FILLET_RADIUS_M}）")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT_POINTS,
                        help="重采样结果导出路径（bare C 点列表 txt）")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    original = load_input_points(args.points)
    editor = ResampleEditor(
        points=original,
        points_path=args.points,
        output_path=args.out,
        step_m=args.step,
        fillet_radius_m=args.fillet_radius,
    )
    editor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
