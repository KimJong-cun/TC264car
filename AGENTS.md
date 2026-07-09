# Repository Guidelines

## 项目结构与模块组织
- `user/` 放置双核入口、中断与启动相关代码，如 `cpu0_main.c`、`isr.c`。
- `code/app/` 是业务层模块，现有命名采用 `*_app.c/.h`，如 `motor_app.c`、`nav_app.c`、`balance_app.c`。
- `code/driver/` 放置自定义驱动与算法，目录划分：
  - `imu/` IMU 与 EKF
  - `motor/` 电机驱动
  - `pid/` 通用 PID
  - `uart/` 命令行解析
  - `nav/` 导航底层：`nav_odometry`（航位推算 + 启动加速度检测）、`nav_manual`（DIRECT/TURN 队列几何/时间纯计算）、`nav_route`（预设路径跟踪纯计算，见下）；状态机本身在 `code/app/nav_app.c`
- `code/test/` 为随工程一同编译的测试/验证代码；新增功能先补测试，再写实现。
- `libraries/` 包含逐飞与 Infineon 底层库；除公共库缺陷修复外，优先在 `code/` 扩展，不要随意改平台层。
- `Debug/` 是自动生成的构建输出目录，含 `makefile` 和中间产物，默认不手改。新增 `code/` 下源文件后需要在 IDE 里 refresh 让 `Debug/code/.../subdir.mk` 重生（CLI 构建可手动加规则）。

## 构建、测试与开发命令
- 使用 AURIX Development Studio 导入根目录下的 `.project` 与 `.cproject`。
- `make -C Debug all`：命令行构建固件，依赖已安装并配置到 `PATH` 的 TASKING `cctc` 与 `make`。
- `make -C Debug clean`：清理 `Debug/` 下产物，重新生成依赖。
- `AURIX修改工程名称.bat`：需要改工程名时使用；改名前先备份工程配置。

## 编码风格与命名规范
- 先检查文件编码；优先使用 UTF-8。若目标文件不是 UTF-8，先转换再写入。
- 延续现有 C 风格：4 空格缩进，左花括号同行，头文件与实现文件成对维护。
- 新业务模块沿用 `*_app.c/.h`，驱动模块沿用功能目录归类；尽量复用现有接口，避免重复造轮子。
- `Debug/makefile` 标注为自动生成文件，禁止手工维护。

## 测试规范
- 必须遵循 TDD：先在 `code/test/` 增加最小可复现测试或验证入口，再实现功能。
- 每次改动后至少完成一次 `make -C Debug all` 构建验证。
- 涉及硬件链路时，补充板级冒烟测试结论，例如串口、定时中断、电机、IMU 初始化是否正常。

## 提交与合并请求规范
- 当前目录未包含 `.git` 历史，无法提炼既有提交格式；建议统一使用 `scope: action`，如 `motor: limit pwm output`。
- PR 需说明修改模块、硬件影响、测试步骤与结果；涉及串口打印、波形或界面输出时附截图或日志。

## 贡献附加要求
- 不得擅自简化需求；发现问题先定位并解决，不以“删功能”代替修复。
- 新增文档与说明的版权署名保持为 `KimJong`。

## 模块边界 / 改动禁区
- **balance_app fast 模式三层级联 PID 严禁改动**：`balance_slow_loop_enable == 0` 分支（`balance_steering_loop` 走 `steering_pid`、`balance_angle_loop` 走 `angle_pid` 出 `target_gyro_rate`、`balance_gyro_loop` 走 `gyro_pid` 出 `servo_output`）所有 PID 默认参数、信号赋值逻辑都已调通（直线、转弯能跑），动一行就是回归风险。
- **balance_app slow 模式（当前默认）已改为双环**：航向环 → `target_angle`，角度环直接驱动 `servo_output`，gyro_loop 在 slow 模式只把 `servo_output` 推到舵机不再重算 PID。`balance_low_speed_angle_pid` 的量纲为 ±15000（servo 单位），当前默认 KP=500 / KI=60 / KD=100 / limit=15000（`balance_slow_app.c:30`），**还需要继续实车调参**。`balance_low_speed_steering_pid` 当前 KP=0.3 / KD=0.8 / limit=10.5（`balance_slow_app.c:23`）。`balance_low_speed_gyro_pid` 在双环下不参与闭环，仅保留给后续调参接口复用。
- slow 模式 PID 参数 / angle_limit 可继续调；旧 balance 调试串流 API 已删除，后续调试输出不要再塞回平衡环。
- 完全不要碰：`servo_app`、`imu_app`。`motor_app` 已按需求重构为单 PID（详见下节），后续调整需同步更新本文档。
- 上层模块（nav 等）必须复用现有接口拿数据 / 下指令：`balance_set_heading_target(deg)`、`balance_get_heading_command()`、`balance_reset_heading_target(deg)`、全局 `yaw`、全局 `wheel_speed`、`motor_set_speed_mps(v)`、`motor_is_fallen()`。
- `balance_set_heading_target()` 设的是 `s_heading_target_deg`，真正参与误差计算的 `s_heading_command_deg` 要先经过 `balance_update_heading_command()`（`balance_app.c:113-132`）限速靠近，**不是瞬间生效**：slow 模式（默认）`heading_step=0.30°/tick`（`balance_slow_app.c`），fast 模式 `0.70°/tick`（`balance_app.c` `BALANCE_FAST_HEADING_STEP_DEFAULT`）；转向 PIT 周期 10ms（`cpu0_main.c` `pit_ms_init(CCU61_CH1, 10)`），换算成最大限速分别是 30°/s、70°/s。任何一次性写入绝对目标角、依赖它"立刻生效"的上层设计（例如 nav_route 的 yaw 滤波补偿，见下）都要先核对这个预算够不够，不能假设 `balance_set_heading_target` 之后马上就是新误差。

## 电机速度反馈 / 编码器

- 当前 `motor_app` 速度反馈使用本地 `TIM6_ENCODER`，配置为 `encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0)`：`P20_3` 为脉冲计数，`P20_0` 为方向输入。
- 不再使用 `TIM5_ENCODER_CH1_P10_3 / TIM5_ENCODER_CH2_P10_1` 作为电机速度反馈；该组引脚在当前硬件上可能与无刷电机 AB 相相关电路复用，静止时会出现异常计数。
- 编码器换算参数：`MOTOR_ENCODER_PPR = 1024.0f`，`MOTOR_ENCODER_TURNS_PER_WHEEL = 4.0f`，速度反馈周期 `MOTOR_SPEED_FEEDBACK_DT_S = 0.010f`。
- `motor_refresh_feedback()` 先把 10 ms 内编码器 count 换算成**轮侧 RPM**：`rpm = count * 60 / (1024 * 4 * 0.010)`，再由 `wheel_speed = rpm * PI * MOTOR_WHEEL_DIAMETER_M / 60` 得到 m/s。
- 编码器速度不再做一阶低通滤波；`motor_refresh_feedback()` 直接把本周期编码器 count 换算出的轮侧 RPM 写入 `motor_rpm`，再换算 `wheel_speed` 进入速度 PID。
- `motor_app.c` 内部已把反馈配置与结构整理为 helper：`MOTOR_FEEDBACK_ENCODER*`、`motor_count_to_wheel_rpm()`、`motor_reset_speed_pid()`、`motor_enter_speed_mode()`、`motor_limit_duty()`；外部接口保持 `motor_set_speed_mps()` / `motor_update_speed_mps()` / `motor_get_current_speed_mps()`。
- duty 输出统一经 `motor_limit_duty()` 限幅到 `MOTOR_DUTY_MIN..MOTOR_DUTY_MAX`（当前 ±10000），`motor_set_duty()` 会进入 direct mode。
- **电机速度 PID 已从快/慢双套合并为单套**，参数以原慢速环为准：`KP=4.5, KI=0.2, KD=0.4, limit=10000`；不再区分 `balance_get_loop_mode()`。原快速环相关的 `motor_fast_speed_boost_k` / `motor_slow_speed_boost_k` 合并为单一 `motor_speed_boost_k`，`motor_get_active_speed_pid()` / `motor_set_active_speed_boost_k()` 等 `_active_` 接口已改名为 `motor_get_speed_pid()` / `motor_set_speed_boost_k()`（无 `_active_`）。
- `motor_set_speed_mps()` 会 reset 速度 PID；`motor_update_speed_mps()` 只更新目标速度，不 reset PID，供连续任务衔接使用。
- 摔车保护统一收在 `motor_apply_protection()`：检测到 `motor_is_fallen()` 时复位 PID、清零 duty 并直接下发 0，`motor_tick()` 里只需调用这一个函数即可完成保护，不再分散写复位逻辑。
- 电机侧和导航侧的运行时距离标定都已删除；电机层只负责编码器到轮侧速度的几何换算，导航层直接积分 `wheel_speed`。
- 电机 PID 当前默认 `KP=4.5, KI=0.2, KD=0.4, limit=10000`；低速起步补偿为 `MOTOR_START_MIN_DUTY=1200`，退出阈值为目标速度的 `0.7`。
- 常用换算（轮径 0.070 m，轮侧 RPM）：`0.5 m/s ≈ 136 rpm`，`0.55 m/s ≈ 150 rpm`，`0.6 m/s ≈ 164 rpm`，`0.8 m/s ≈ 218 rpm`，`100 rpm ≈ 0.367 m/s`。

## 导航（nav_app）：manual 队列 + route 预设路径

`nav_app.c` 是状态机（`NAV_STATE_IDLE`/`WAIT_START`/`RUNNING`/`DONE`），由 `nav_tick()` 在转向 PIT ISR 里驱动，按 `s_nav_mode`（`nav_app_internal.h`）分派到 manual 或 route 两套调度逻辑之一。私有运行时状态全放在 `code/app/nav_app_internal.h`（只允许 `nav_app.c` 单独 include，不进 `zf_common_headfile.h`，否则 `static` 变量按翻译单元各拷贝一份）。

### manual 模式（DIRECT / TURN 任务队列）

- 队列存储、几何/时间纯计算下沉到 `code/driver/nav/nav_manual.h/c`（无副作用纯函数），`nav_app.c` 只跑状态机、调这些函数、把结果喂给 balance/motor。
- **API**：`nav_enqueue_direct(distance_m, speed_mps)` / `nav_enqueue_turn(turn_deg, speed_mps)` 入队 → `nav_start_manual()` 进 `WAIT_START`，等加速度触发（`nav_odometry_is_start_wait_triggered()`）后自动激活第一个任务；任务串行执行，完成一个自动激活下一个，队列空回 `DONE`。`nav_clear_manual_tasks()` / `nav_cancel_task()` 可在非忙状态清空 / 随时取消。
- **坐标系**：`nav_odometry` 维护的是**局部**坐标（route/manual 各自任务启动时刻的车头朝向为 0°，`heading = wrap(yaw - s_nav_yaw_init)`），不是绝对朝向。
- **调参参数**（`nav_manual.h`）：`NAV_MANUAL_QUEUE_SIZE=8`、`NAV_MANUAL_TURN_DONE_DEG=5.0`、`NAV_MANUAL_TURN_DONE_RATE_DPS=8.0`、`NAV_MANUAL_TURN_DONE_HOLD_TICKS=4`（2026-07-09 从 10 改成 4，转向 ISR 10ms/拍，判定稳定所需时长从 100ms 降到 40ms，避免大转弯目标附近的小幅摆动反复清零 hold 计数导致迟迟不完成）、`NAV_MANUAL_TURN_TIMEOUT_BASE_TICKS=200`、`NAV_MANUAL_TURN_TIMEOUT_PER_DEG=6.0`、`NAV_MANUAL_DECEL_TRIGGER_RATIO=0.7`（提前减速触发比例，见下）。
- **DIRECT 任务提前减速（2026-07-09 新增）**：`nav_manual_update_task()` 的 DIRECT 分支每 tick 调 `nav_manual_apply_early_decel()`——用 `nav_manual_queue_peek_front()` 看一眼队列里下一个任务（不出队），若下一个任务的目标速度比当前 DIRECT 任务低，且已走过当前任务距离的 `NAV_MANUAL_DECEL_TRIGGER_RATIO`（70%），就调 `motor_update_speed_mps(next.speed_mps)` 提前切到下一个任务的目标速度（不 reset PID，和现有任务衔接用的 `motor_update_speed_mps` 语义一致），避免两个任务交接瞬间速度突降。纯判定函数是 `nav_manual_should_start_decel(traveled_m, distance_m, current_speed_mps, next_speed_mps)`（`nav_manual.c`），`s_nav_manual_decel_active` 标志确保只触发一次，任务切换/重置时清零。只对 DIRECT 生效（TURN 没有以米为单位的路程概念）；只看队列里紧邻的下一个任务，不会跨两个任务往后看。
- **UART 命令**（`uart_command.c`）：`direct <distance_m> <speed_mps>`、`turn <turn_deg> <speed_mps>`、`start`（激活队列）、`clear`（清队列）、`cancel`（取消当前任务）、`nav`（状态快照）。

### route 模式（预设路径跟踪，2026-07-04 新增）

不用 pure pursuit（rb 版本那套 foot-to-carrot/robot-to-carrot cross-track blend），换成"往后取 N 个重采样路径点求均值生成虚拟目标点，直接从车当前位置看向该点"，纠偏靠这个几何关系自然产生。曾经加过一层 yaw 反馈 EMA 滤波 + 反推补偿公式应对低速画龙，2026-07-09 上车实测发现是动力"一阵一阵"的根因（结构性要求转向斜坡追踪量接近车物理极限），已移除，现在目标角直接喂给 `balance_set_heading_target`，与 manual TURN 一致。完整算法设计、排除方案和这次移除的原因记录在 `route设计思路.md`，这里只记要点：

- **文件**：`code/driver/nav/nav_route.h/c`（纯函数 + 路径数组，仿 `nav_manual.*` 风格）。路径数组由 `tools/nav_route_resample.py` 离线生成（对 `tools/nav_route_points.c.txt` 先做拐角圆角、再按固定弧长重采样），当前是 160 点、弧长 15.9m，手动粘进 `nav_route.c` 的 `s_nav_route_path[]`——工具本身不直接改 `.c` 文件，改路径后要重新跑一次再手动粘贴。
- **API**：`nav_start_route(speed_mps)` 进 `WAIT_START`，触发后 `nav_app.c` 里的 `nav_route_activate()`/`nav_route_update_task()` 接管；`nav_is_route_mode()`、`nav_get_route_index()` 供状态查询。
- **进入路径前先直走一段（2026-07-09 新增）**：`nav_route_activate()` 触发后不直接对齐 path[0]，先保持当前朝向直走 `NAV_ROUTE_PRE_STRAIGHT_DISTANCE_M`（`nav_route.h`，默认 1.0m），走够距离（`nav_route_update_pre_straight()` 判定）才调用 `nav_route_enter_path()` 把 odometry 位置对齐到 `path[0]`、真正开始索引推进。这段直走期间 `nav_get_data()` 对外坐标（x/y/target_x/target_y）固定打 0（`s_nav_route_pre_straight_active` 标志控制），避免下位机把直走阶段误当路径数据画出来；`nav`/`debug_telemetry` xytest 帧显示的都是这份对外数据。
- **UART 命令**：`route <speed_mps>`；`nav` 状态快照加了 `mode=route/manual` 和 `route_idx=`。
- **调参参数**（`nav_route.h`，均为默认值，待上车实测调）：`NAV_ROUTE_LOOKAHEAD_POINTS=5`（预瞄点数，配合 0.10m 重采样间距≈预瞄 0.5m）、`NAV_ROUTE_WP_REACH_M=0.20`（终点判定阈值）、`NAV_ROUTE_PRE_STRAIGHT_DISTANCE_M=1.0`（进入路径前直走距离）。
- **接入 balance 不改 balance_app.c**：目标角算出全局 `target_yaw_global` 后直接 `balance_set_heading_target(target_yaw_global)`，和 manual 模式的 TURN 任务走同一条路径，`balance_update_heading_command()` 的限速斜坡按原本设计工作，不做任何反推/补偿。若实车画龙仍明显，优先调大 `NAV_ROUTE_LOOKAHEAD_POINTS`（预瞄距离），不要再引入依赖斜坡追踪 yaw 瞬时变化率的接入方式——已证明是结构性问题，会撞车物理转向极限（~22°/s，见文末车模限制说明）。详见 `route设计思路.md`"2026-07-09 更新"。
- **验证**：`code/test/nav_route_test.c` 覆盖了 `nav_route.c` 里全部纯函数（索引推进、均值取点、bearing、终点判定），期望值是用 Python 复刻同一套算法算出的 ground truth，跟当前嵌入的 160 点路径数据绑定——路径重采样参数变了要重新算期望值。硬件层面的画龙收敛效果、终点停车是否准确，还没有实车验证。

## 已知小问题（暂未修，规划下次处理）
1. **IMU yaw 起步零点**：上电 EKF 锁的 `s_yaw_zero_deg`（`code/driver/imu/ekf.c`）取决于上电时朝向。建议后续在串口层增加 yaw 零点锁定入口，让用户在车摆好后手动锁零。
2. **dead reckoning 完全靠 IMU yaw**：没磁力计校准、没视觉，`nav_odometry` 纯积分 `wheel_speed` + heading，长任务会漂。route 模式的 160 点、15.9m 全程比 manual 模式的单任务距离长得多，这个漂移风险更需要关注。
3. ~~route 模式接入 balance 的补偿公式尚未上车验证~~：已于 2026-07-09 移除，见下面 route 模式小节和 `route设计思路.md`"2026-07-09 更新"。

## 已删除（曾经存在，记录以防误读 git/历史）
- **单任务式 DIRECT/TURN 导航 API**（早于当前 `nav_manual.c` 队列重构）：`nav_start_direct_task()` / `nav_start_turn_task()` / `nav_start()` / `nav_force_launch()` / `nav_cancel()`、`NAV_START_ACCEL_DELTA_G` 模块常量、`s_nav_chaining_task` 衔接标志。当前替代实现是队列化的 `nav_enqueue_direct()` / `nav_enqueue_turn()` / `nav_start_manual()` / `nav_cancel_task()`（见上面"manual 模式"）。
- **`nav_path_figure8` / `nav_path_tracker`** 整套八字路径生成 + 单调段投影 CTE 跟踪
- **Stanley / Pure-CTE / yaw 检查点状态机** 三种控制律(都在 `nav_app.c::NAV_TASK_PATH` 分支)
- **`NAV_TASK_PATH`** 任务类型 + 对应串口入口
- **路径 CTE 调试模式 + `path_test()` UART 流**
- **SM / Stanley 模式切换串口入口**
- 根目录的 `figure8_preview.py` / `figure8_tuner.py` / `figure8_*.json` / `figure8_*.png`

低速自平衡车物理上 yaw 角速度极限 ~22°/s,1.1 m/s(慢速比赛极限速度)走全贝塞尔八字路径切线变化率最高需要 ~70°/s,Stanley 数学上跟不上;状态机也只能拓扑完成、不能贴形状。这是**不再做八字路径**、转而设计现在的 route 模式（见上）的直接原因。
