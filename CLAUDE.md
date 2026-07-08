# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

An `AGENTS.md` also exists in the repo root with detailed, frequently-updated module boundaries,
tuning parameters, and known issues — read it too; this file focuses on orientation and commands.

## Project overview

Firmware for a self-balancing two-wheel robot ("平衡车") running on an Infineon AURIX TC264D
dual-core MCU, built on the Seekfree (逐飞科技) TC264 open-source library. This is the "rc"
iteration of the project (a prior "rb" version's design decisions are referenced for context
in comments and `route设计思路.md`, but its code is not in this tree).

## Build commands

- IDE: import the root `.project` / `.cproject` into AURIX Development Studio (ADS v1.10.2).
- CLI build: `make -C Debug all` (requires TASKING `cctc` and `make` on `PATH`).
- CLI clean: `make -C Debug clean`.
- `Debug/` (including `Debug/makefile` and `subdir.mk` files) is auto-generated — never hand-edit.
  After adding a new `code/` source file, refresh the project in the IDE to regenerate the
  relevant `subdir.mk`, or add the build rule manually for CLI-only builds.
- Renaming the project: use `AURIX修改工程名称.bat` (back up project config first).

## Testing

There is no host-side test runner — this is embedded firmware for real hardware. "Tests" in
`code/test/` (e.g. `nav_manual_test.c`) are validation functions compiled into the firmware
image that assert expected state/behavior and return a pass/fail flag; they are invoked
manually (temporarily wired into `core0_main` or a UART command) rather than run automatically.
Follow the existing convention: add a minimal validation entry point in `code/test/` before
implementing new logic, then a build (`make -C Debug all`) is the baseline check. Hardware-facing
changes (UART, timers, motor, IMU) need an actual smoke-test on the board — state clearly if that
wasn't possible.

## Architecture

### Layering

- `user/` — dual-core entry points and interrupt wiring: `cpu0_main.c` (core0 init + main loop),
  `cpu1_main.c` (core1), `isr.c`/`isr_config.h` (all `IFX_INTERRUPT` handlers).
- `code/app/` — business-logic modules, one pair of `*_app.c/.h` per subsystem: `balance_app`,
  `motor_app`, `nav_app`, `servo_app`, `imu_app`, `uart_command`, plus `scheduler.h/schedule.c`.
- `code/driver/` — custom drivers/algorithms, grouped by function: `imu/` (EKF + matrix math),
  `motor/` (UART motor driver), `pid/` (generic PID), `uart/` (line-based command RX), `nav/`
  (pure-function navigation helpers: `nav_odometry` for dead-reckoning, `nav_manual` for
  DIRECT/TURN task geometry — the queue/state-machine logic itself lives in `code/app/nav_app.c`).
- `code/test/` — validation entry points, see Testing above.
- `libraries/` — vendor code (Seekfree `zf_*` libraries + Infineon iLLD). Extend in `code/`
  instead of modifying platform layers, except for genuine library bug fixes.
- `tools/` — Python host-side tooling that runs off-target, e.g. `nav_route_resample.py` (offline
  waypoint resampling for the planned route-following mode; see `route设计思路.md`).

### Real-time structure

Two execution contexts exist side by side and must not be confused:

- **Hard real-time**: PIT timer interrupts in `user/isr.c` drive the actual control loops at
  fixed rates — `imu_proc()` + `balance_gyro_loop()`, `balance_angle_loop()`, and
  `motor_refresh_feedback()` + `nav_tick()` + `balance_steering_loop()` + `motor_tick()` each run
  in their own ISR. This is where closed-loop timing guarantees matter.
- **Soft background scheduling**: `code/app/schedule.c` runs a tiny table-driven dispatcher
  (`scheduler_run()`, called from the `core0_main` `while(TRUE)` loop) for non-critical periodic
  work only (telemetry streaming, one-shot auto-calibration). Timing here is best-effort and
  jitters with main-loop duration — never move a control-loop-critical task here.

New periodic work: hard-timing-sensitive → add/extend a PIT ISR in `isr.c`; everything else →
add a row to `s_sched_tasks[]` in `schedule.c`.

### Navigation (`nav_app`)

`nav_app.c` is a state machine (`NAV_STATE_IDLE` / `WAIT_START` / `RUNNING` / ...) driven from
`nav_tick()` in the steering PIT ISR. It owns a manual task queue of `NAV_TASK_DIRECT` /
`NAV_TASK_TURN` entries; per-task geometry (target yaw/distance math) is pure functions in
`code/driver/nav/nav_manual.c`, called from the state machine. Runtime state private to
`nav_app.c` is factored into `code/app/nav_app_internal.h` (included only by `nav_app.c` — do not
add it to `zf_common_headfile.h`, or its `static` variables get duplicated per translation unit).

A path-following "route" mode (virtual-target-point averaging over a resampled waypoint array,
distinct from the deleted rb-style pure-pursuit/Stanley approaches) is designed but not yet
implemented — see `route设计思路.md` for the full algorithm and rationale before building it, and
follow the same driver/app split (`code/driver/nav/nav_route.*` pure functions, state in
`nav_app_internal.h`, dispatch in `nav_app.c`).

### Balance control (`balance_app`)

Two loop modes selected by `balance_get_loop_mode()`:
- **Fast mode**: three cascaded PIDs (`steering_pid` → `angle_pid` → `gyro_pid`). Considered
  tuned and working — do not modify without a strong reason (see AGENTS.md for exact wiring).
- **Slow mode** (current default): two loops — heading loop produces `target_angle`, angle loop
  drives `servo_output` directly; `balance_gyro_loop()` in slow mode just forwards `servo_output`
  to the servo rather than computing its own PID. Still being tuned on hardware.

`servo_app` and `imu_app` are off-limits — do not modify.

### Motor control (`motor_app`)

Single speed PID (previously separate fast/slow PID sets, now merged). Feedback comes from a
quadrature encoder read via `TIM6_ENCODER`; the sibling `TIM5_ENCODER_*` pins are intentionally
unused (electrically shared with BLDC phase wiring on this hardware, gives false counts at rest).
Public API: `motor_set_speed_mps()`, `motor_update_speed_mps()` (no PID reset, for chaining tasks),
`motor_get_current_speed_mps()`. Fall protection is centralized in `motor_apply_protection()`,
called once per tick from `motor_tick()`.

### Cross-module interfaces

Upper layers (nav, etc.) must go through existing APIs rather than reaching into balance/motor
internals: `balance_set_heading_target(deg)`, `balance_get_heading_command()`,
`balance_reset_heading_target(deg)`, global `yaw`, global `wheel_speed`, `motor_set_speed_mps(v)`,
`motor_is_fallen()`.

## Coding conventions

- File encoding: UTF-8. Check before editing; convert non-UTF-8 files before writing.
- 4-space indentation, opening brace on the same line as the statement.
- New business modules: `*_app.c/.h` naming, paired header/implementation. New drivers: group by
  function under `code/driver/<area>/`.
- Reuse existing interfaces rather than duplicating logic across layers.
