# rc route 模块设计思路

记录 rc 版本 route（预设路径导航）模块的设计讨论结论，供后续写代码时对照，也方便过一段时间回看时恢复上下文。

## 背景

- rb 版本的 route 用的是纯追踪（pure pursuit）：沿路径走固定弧长（Ld）找 carrot 点，
  再按 cross-track 误差在 foot-to-carrot / robot-to-carrot 两种 bearing 模式间做 blend。
  详见 `rb版本/code/app/nav_app.c` 里的 `nav_pp_*` 系列函数。
- rc 版本目前已经把 manual（DIRECT/TURN 任务队列）模式重构完成，抽成了纯函数驱动层
  `code/driver/nav/nav_manual.h/c` + 状态私有变量 `code/app/nav_app_internal.h`。
  route 模式已按本文档设计实现完成，见下面"架构落地"。
- 车模是单车模型（自行车平衡车，靠舵机转向，边走边转，不能原地转向），低速时会绕目标航线
  产生近似正弦的左右摆动（画龙）。纯追踪算法对这种车模偏严苛：路径折线拐角处曲率不连续，
  要求瞬时转向；rb 靠给目标 yaw 加 EMA 平滑事后兜底，效果依赖调参，不是很 principled。

基于以上，rc 决定不沿用纯追踪，换一套更简单、专门针对这个车模特点设计的追踪算法。

## 最终算法设计

整体思路：不追"路径几何切线+复杂blend"，改成"未来若干个路径点求平均生成虚拟目标点，
直接从车当前位置看向这个点"，纠偏效果靠这个"从车当前位置算方位"的选择自然获得，
不需要额外的 cross-track blend 或 P 纠偏项。

### 链路

1. **路径重采样**（在 PC 端 Python 上位机完成，不在车上跑）
   把原始手工标注的折线航点，按固定弧长重新采样成等间距点列，再连同路径信息一起
   导出/写入到车上代码里使用。原因见下面"为什么要重采样"。

2. **索引推进（简化投影）**
   车运行时，在 MCU 上保留一个简化版的"投影推进"逻辑（类似 rb 的
   `nav_pp_project_and_advance`，但只需要判断当前段的 t 是否 >= 1 就把路径索引推进
   到下一段），不需要精确计算 foot 点坐标本身用来做后续 bearing 计算。
   仅用于知道"车现在大致对应到重采样路径的第几个点"。

3. **生成虚拟目标点**
   从当前索引往后取 N 个点（N 可调，重采样后 N 直接对应固定预瞄距离 = N × 采样间距），
   对这 N 个点的 (x, y) 坐标求算术平均，得到虚拟目标点。
   **不把车自身坐标加入这个平均池**——数学上证明过，加入车自身坐标只会等比例缩短
   虚拟目标点到车的距离，不改变方位角（因为车、原目标点、"车+路径点平均后的新目标点"
   三者必然共线，共线的点看过去方向不变），不会带来纠偏效果，因此没有必要。

4. **算方位（robot-to-carrot，纠偏在这一步自然产生）**
   ```
   dx = 虚拟目标点.x - 车当前.x
   dy = 虚拟目标点.y - 车当前.y
   bearing_rel = atan2(dx, dy)   // 注意参数顺序 (dx, dy)，对应本项目 heading 定义：
                                  // x右正、y前正、heading 顺时针为正、以y轴为0°基准
   ```
   起点直接用车的真实位置而不是路径上的 foot 点，车偏离路径越远，算出的方位越明显
   指向"往目标点靠拢"，纠偏效果是几何关系自动带来的，不需要单独实现。

5. **目标 yaw**
   ```
   target_yaw = s_nav_yaw_init + bearing_rel
   ```
   与 rb 一致的坐标系换算方式（`s_nav_yaw_init` 是 route 起始时的车头朝向）。

6. **当前 yaw 滤波（专门应对低速画龙）**
   route 层内部单独维护一份 yaw 副本（不是 `nav_odometry` 内部用于 dead reckoning
   积分的那个 heading，是单独拷贝出来的一份），做简单 EMA：
   ```
   yaw_filtered = yaw_filtered + alpha * wrap(yaw_raw - yaw_filtered)
   ```
   - 只用于第 7 步的误差比较，不影响别处。
   - **不进 `nav_odometry_update_dead_reckoning` 的 x,y 积分** —— 如果拿滤波后的
     heading 去积分位置，会因为相位滞后产生系统性位置漂移，而不只是抖动。
   - **不进 `balance_steering_loop` 实时转向环** —— 那里需要低延迟的真实反馈，
     不能被这层滤波拖慢，否则可能影响平衡本身的稳定性。
   - 滤波器选 EMA 而不是固定窗口滑动平均（boxcar），是因为 boxcar 要严格卡准
     画龙周期的整数倍才能把该频率滤干净，卡不准会残留且相位随失配程度乱跳；
     EMA 不追求精确陷波，只是单调低通，不依赖"周期恒定"这个不一定成立的假设，
     更鲁棒。代价是拖慢响应、滤不干净，可接受。

7. **误差计算与接入 balance（不修改 balance_app.c）**
   ```
   error = target_yaw - yaw_filtered
   ```
   `balance_set_heading_target()` 吃的是绝对目标角，不是误差角，且
   `balance_steering_loop()` 内部永远拿**原始** yaw 算误差，不是 `yaw_filtered`。
   要让 `error` 真正在闭环里起作用，需要反过来构造喂给 `balance_set_heading_target`
   的绝对目标角，让 `balance_steering_loop` 自己算出来的误差正好等于这个 `error`：
   ```
   target_for_balance = yaw_now_global + wrap(target_yaw_global - yaw_filtered_global)
   ```
   代入 `balance_steering_loop` 内部公式 `command - yaw_now` 后就是
   `target_yaw_global - yaw_filtered_global`。这个技巧依赖两个前提，见下面
   "接入 balance 时必须处理的两个问题"。

   目标 yaw 不再加 rb 那种二次 EMA 平滑（`s_pp_yaw_smooth`，转弯/直线两档 alpha），
   第 7 步到此为止——已经决定不采用。

## 接入 balance 时必须处理的两个问题

（2026-07-04 对照 `rc版本/user/isr.c`、`code/app/balance_app.c`、`code/app/nav_app.c`、
`code/driver/nav/nav_odometry.c` 逐行核对补偿公式后发现，写代码前必须先处理，
否则会得到看似合理但实际不精确甚至错误的转向指令。）

### 问题1：`balance_update_heading_command()` 的斜坡限幅会限制补偿公式的精度

`balance_set_heading_target()` 设的是 `s_heading_target_deg`，但
`balance_steering_loop()`（`balance_app.c:158-180`）内部先调用
`balance_update_heading_command()`（`balance_app.c:113-132`）把
`s_heading_command_deg` 往 `s_heading_target_deg` **限速**靠近（每 tick 最多挪
`heading_step` 度），再拿 `s_heading_command_deg - yaw` 算误差——不是直接用
`s_heading_target_deg`。

补偿公式每 tick 都把当前 `yaw_now` 整个代入 target，等价于要求斜坡每 tick
**完全追上 yaw 本身的瞬时变化量**，也就是画龙的瞬时角速度。已核实的硬预算：
当前默认 slow 模式，`heading_step = 0.30°/tick`（`balance_slow_app.c:14`），
转向 ISR 周期 10ms（`cpu0_main.c:64` `pit_ms_init(CCU61_CH1, 10)`），
**最大可追踪角速度 = 30°/s**（fast 模式 `heading_step=0.70°/tick`≈70°/s，
但 slow 是当前默认）。

上车前需要实测画龙的振幅 A(度) 和周期 T(秒)，算峰值角速度 ≈ `A × 2π / T`，
跟 30°/s 比较：明显小于则补偿公式在每个 tick 精确成立；接近或超过则斜坡会
饱和/clip，实际误差不再精确等于 `target_yaw - yaw_filtered`，滤波效果被
斜坡部分抵消，需要重新设计这一步的接入方式（但仍不能碰 `balance_app.c`）。

（`isr.c` 已核实：`nav_tick()` 和 `balance_steering_loop()` 在同一个 PIT ISR
里背靠背执行 `cc61_pit_ch1_isr`，TC264 默认不支持中断嵌套，所以两次调用之间
`yaw` 不会变——这是补偿公式在"单个 tick 内"成立的前提，已验证没问题；
只是斜坡跨 tick 的追踪能力还要看画龙实测数据。）

### 问题2：局部/全局 heading 坐标系必须显式转换

`nav_odometry` 的 `pose->heading` 是**局部**坐标（`nav_odometry.c:60-63`，
`wrap(yaw - s_nav_yaw_init)`），而 `target_yaw = s_nav_yaw_init + bearing_rel`
是**全局**坐标。第 6 步的 `yaw_filtered` 是对 `pose->heading`（局部）做 EMA，
**必须先加回 `s_nav_yaw_init` 转成全局**才能拿去跟 `target_yaw` 比较或代入
补偿公式，否则会产生一个恒定为 `s_nav_yaw_init` 大小的转向偏移（车一启动就
稳定地偏向错误方向）。

实现里这个转换放进了 `nav_route_compute_balance_target_deg()` 内部（接收
局部 `yaw_filtered_local_deg` + `nav_yaw_init_deg` 两个参数，函数内部相加），
不依赖调用者记得转换。

### 讨论过程中排除的方案（备查，避免以后重新踩坑）

- **半径筛选代替索引推进**（只取车身 1m 半径内的路径点做平均，不够就退化成默认点）：
  排除。因为路径本身是环形/近似自交的（比如 rb 原始航点里第 6 点和第 22 点欧氏距离
  只有 0.18m，但沿路径相差 16 个点、好几米弧长），半径筛选会把路径上遥远但空间上
  凑巧靠近的点一起筛进来，平均出来的方向没有意义。必须按路径的前后顺序（索引）取点，
  不能按空间距离筛。

- **把车自身坐标加入"往后 N 点求平均"的平均池**：排除。已在第 3 步说明，数学上
  证明对纠偏无效（车、目标点、混入车坐标后的新目标点三点共线，方向不变，只是距离
  变短），不会产生"偏差越大纠偏越强"的效果。如果需要偏差相关的纠偏强度，必须走
  cross-track blend 或额外 P 项这类"改变 bearing 计算方式本身"的路子，不能通过
  调平均池的构成来实现。

- **纯 cross-track blend（照搬 rb）**：不采用。当前方案（bearing 直接从车实际
  位置算起）已经自带纠偏，没有必要再叠加 blend，逻辑更简单。

- **滤波加在目标 yaw 或误差上，而不是当前 yaw 上**：不采用。当前 yaw 滤波只针对
  反馈里的画龙噪声成分，职责最干净；滤目标 yaw 对画龙没有针对性（目标 yaw 本来就
  没有画龙成分）；滤误差在路径转弯、目标 yaw 本身在变化时会把"路径要求转弯"和
  "车在晃"两件事混在一起，不好区分。

## 为什么要重采样，且放到 Python 上位机做

原始航点（人工标注的折线关键点）间距不均匀，直接对原始点做"往后数 N 个点求平均"，
N 对应的实际预瞄距离会随航点疏密变化忽远忽近，同一个 N 在不同区域表现不一致，
调参没有稳定的物理意义。

重采样：把折线看成连续曲线，按固定弧长间隔重新生成一组新点，替代原始点，使相邻点
间距处处相等，"往后数 N 点"就严格等价于"预瞄 N × 采样间距 米"，是一个稳定可控的量。

放到 Python 上位机做，而不是在 MCU 运行时算，原因：
- 重采样只需要在路径定稿时做一次，结果是静态数据，没必要每次上电都重新计算，
  省 MCU 运行时开销。
- rb 已经有一套基于 matplotlib 的路径编辑上位机
  （`rb版本/tools/nav_path_editor.py`）：读取/解析 `nav_app.c` 里的
  `s_nav_preset_wps[]`，可视化在 2m 半径场地示意图上，支持拖拽调点、插入/删除、
  保存点列表、写回 C 代码。重采样功能计划直接扩展在这套工具链上，画完/调完路径后
  一步导出重采样后的定长点数组，直接生成 rc 要用的 C 数组，不需要在 MCU 上重新
  实现"沿弧长走固定距离"的插值逻辑（这段逻辑本质和 rb `nav_pp_find_lookahead`
  里"沿折线走固定距离"是同一套算法，用 Python 实现一次即可，车上不用再写一遍）。

## 架构落地（对应 rc 现有分层，已实现）

延续 rc 对 manual 模式的分层模式：

- `code/driver/nav/nav_route.h/c`：纯计算函数，无副作用，路径数组是唯一的固定只读数据。
  - `s_nav_route_path[]`（static const，由 `tools/nav_route_resample.py` 生成，见下）
  - `nav_route_advance_index(index, robot_x, robot_y)`：投影推进，t>=1 就跨段前移
  - `nav_route_compute_target(index, *out_x, *out_y)`：往后取
    `NAV_ROUTE_LOOKAHEAD_POINTS` 个点求均值，越界钳位到最后一点
  - `nav_route_compute_bearing_deg(robot_x, robot_y, target_x, target_y)`：`atan2(dx, dy)`
  - `nav_route_yaw_filter_update(prev_local, raw_local, alpha)`：纯 EMA，带 wrap，
    状态由调用者（`nav_app_internal.h` 里的 `s_nav_route_yaw_filtered`）持有传入传出
  - `nav_route_compute_balance_target_deg(yaw_now_global, target_yaw_global,
    yaw_filtered_local, nav_yaw_init)`：局部→全局转换 + 补偿公式，见"问题2"
  - `nav_route_is_finished(index, robot_x, robot_y)`：**不能只看 index**——
    `nav_route_advance_index` 的 while 循环条件是 `index < last_seg`，最多推进到
    `count-2`，永远不会等于 `count-1`，所以终点判定必须像 rb 一样另外检查最后一段的
    投影 t 或离终点距离，不能简单判断 `index >= count-1`（写代码时发现的实现细节，
    不是本文档最初设计文字的字面实现）
- `code/app/nav_app_internal.h`：新增 `nav_mode_enum`（MANUAL/ROUTE）+
  `s_nav_mode`、`s_nav_route_index`、`s_nav_route_yaw_filtered`、`s_nav_route_speed_mps`，
  和 manual 的运行时状态放在一起管理。
- `code/app/nav_app.c`：`nav_tick()` 的 `WAIT_START`/`RUNNING` 分支按 `s_nav_mode`
  分派到 `nav_route_activate()` / `nav_route_update_task()`（新增的 static 调度函数，
  只调 `nav_route.*` 纯函数 + `balance_set_heading_target`/`motor_set_speed_mps`），
  或原有的 manual 调度函数。新增公开接口 `nav_start_route(speed_mps)`、
  `nav_is_route_mode()`、`nav_get_route_index()`。`nav_init`/`nav_reset`/
  `nav_cancel_task`/`nav_start_manual` 都会把 `s_nav_mode` 复位/切换，避免残留状态
  串到下一次任务。
- `code/app/uart_command.c`：新增 `route <speed_mps>` 命令（仿 `direct`/`turn`/`start`
  风格），`nav` 状态查询加了 `mode=route/manual` 和 `route_idx=` 字段。
- `Debug/code/driver/nav/subdir.mk`：手动补了 `nav_route.c` 的三行编译规则
  （仿 `nav_manual.c`），CLI `make -C Debug all` 才能编到这个新文件，仍建议之后在
  ADS 里 refresh 一次让它正常重生成。

## 重采样工具的实际参数

`tools/nav_route_resample.py` 已经比本文档最初设想的更完整：不仅按固定弧长重采样，
还会先对折线拐点做圆角处理（`fillet_radius_m`，默认 0.30m），让重采样路径本身曲率
连续，再在圆角后的路径上按固定弧长取点——这是本文档最初没写但发现已经实现的额外
处理，直接解决了"折线拐角曲率不连续"这个背景里提到的问题，作用上和纯追踪算法
需要另外调 EMA 平滑事后兜底是同一个目的，但做在路径几何层面，更 principled。

用当前默认参数（`step_m=0.10`, `fillet_radius_m=0.30`）对
`tools/nav_route_points.c.txt`（28 个手工航点）跑了一次，生成
`tools/nav_route_resampled.c.txt`：160 个点，总弧长 15.9m，已原样嵌入
`code/driver/nav/nav_route.c` 的 `s_nav_route_path[]`。路径如果改了，重新跑
`nav_route_resample.py`（交互式 matplotlib 编辑器，`w` 键保存重采样结果）后手动
把 `nav_route_resampled.c.txt` 的内容粘回 `nav_route.c`，工具本身不直接改 `.c` 文件。

## 待定参数（默认值已写进代码注释，上车实测后再调）

- 重采样间距：`step_m=0.10`（`tools/nav_route_resample.py` 默认值）
- 圆角半径：`fillet_radius_m=0.30`（同上）
- N（预瞄点数）：`NAV_ROUTE_LOOKAHEAD_POINTS=5`（`nav_route.h`，对应预瞄距离 0.5m）
- 当前 yaw 滤波的 EMA alpha：`NAV_ROUTE_YAW_FILTER_ALPHA=0.15`（`nav_route.h`）
- 终点判定阈值：`NAV_ROUTE_WP_REACH_M=0.20`（`nav_route.h`）
- 目标 yaw 的二次平滑（rb 的转弯/直线两档 alpha）：已决定不采用，第 7 步到此为止
- 问题1（斜坡限幅预算 30°/s vs 画龙实测峰值角速度）：上车实测后才能确认补偿公式
  是否精确成立，这是唯一还需要真实硬件数据才能关闭的问题
