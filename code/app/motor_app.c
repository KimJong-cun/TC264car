/*
 * motor_app.c
 *
 *  基于 base 工程迁移的电机驱动与速度 PID
 *  Copyright: KimJong
 */

#include "motor_app.h"
#define MOTOR_KP_DEFAULT                4.5f        /* 速度环比例系数默认值 */
#define MOTOR_KI_DEFAULT                0.2f        /* 速度环积分系数默认值 */
#define MOTOR_KD_DEFAULT                0.4f        /* 速度环微分系数默认值 */
static PID_T motor_speed_pid;
static float motor_target_raw = 0.0f;
static float motor_ramp_raw = 0.0f;
static int16 motor_duty_out = 0;
static uint8 motor_direct_mode = 0;

int16_t motor_rpm = 0;
float wheel_speed = 0.0f;
float motor_fall_threshold_deg = MOTOR_FALL_THRESHOLD_DEFAULT;
float motor_speed_boost_k = MOTOR_SPEED_BOOST_K_DEFAULT;

static float motor_rpm_to_mps_scale(void)
{
    /* 本地 TIM6 编码器反馈先按 4:1 传动比换算成轮侧 RPM，再换算成 m/s。
     * 与参考工程一致：RPM_TO_WHEEL = WHEEL_DIAMETER * PI / 60。
     * 旧 UART 速度反馈路径已不用，速度闭环现在只看本地编码器。 */
    return (PI * MOTOR_WHEEL_DIAMETER_M) / 60.0f;
}

static float motor_count_to_wheel_rpm(int16 encoder_count)
{
    return ((float)encoder_count * 60.0f) /
           (MOTOR_ENCODER_PPR * MOTOR_ENCODER_TURNS_PER_WHEEL * MOTOR_SPEED_FEEDBACK_DT_S);
}

static float motor_raw_to_mps(int16 raw_speed)
{
    return (float)raw_speed * motor_rpm_to_mps_scale();
}

static float motor_mps_to_raw(float speed_mps)
{
    float scale = motor_rpm_to_mps_scale();

    if (scale <= 0.0f)
    {
        return 0.0f;
    }

    return speed_mps / scale;
}

void motor_refresh_feedback(void)
{
    int16 encoder_count;
    float rpm_raw;

    /* 原始转速和换算后的车轮速度一起刷新，
     * 避免外部读到一新一旧两套值。 */
    encoder_count = encoder_get_count(MOTOR_FEEDBACK_ENCODER);
    encoder_clear_count(MOTOR_FEEDBACK_ENCODER);

    rpm_raw = motor_count_to_wheel_rpm(encoder_count);
    motor_rpm = (int16_t)rpm_raw;
    wheel_speed = motor_raw_to_mps(motor_rpm);
}

static void motor_reset_speed_pid(void)
{
    pid_reset(&motor_speed_pid);
}

static int16 motor_limit_duty(int16 duty)
{
    if (duty > MOTOR_DUTY_MAX)
    {
        return MOTOR_DUTY_MAX;
    }

    if (duty < MOTOR_DUTY_MIN)
    {
        return MOTOR_DUTY_MIN;
    }

    return duty;
}

static void motor_enter_speed_mode(float target_raw, uint8 reset_pid)
{
    if (reset_pid)
    {
        motor_reset_speed_pid();
        motor_ramp_raw = (float)motor_rpm;     /* 从当前实测速度重新起斜坡，避免目标突变时斜坡跳变 */
    }

    motor_target_raw = target_raw;
    motor_direct_mode = 0;
}

/* 速度目标斜坡：让喂给 PID 的目标每拍最多前进 MAX、至少前进 MIN（rpm），
 * 不管最终目标是 0.35m/s 还是 1m/s，起步阶段的加速力度都是同一个斜率，
 * 解决不同目标速度起步"力度"不一致的问题。快到目标时直接对齐，不再强制多走。 */
static void motor_ramp_to_target(float target_raw)
{
    float diff = target_raw - motor_ramp_raw;
    float abs_diff = (diff >= 0.0f) ? diff : -diff;
    float step;

    if (abs_diff <= MOTOR_RAMP_MIN_STEP_RPM)
    {
        motor_ramp_raw = target_raw;
        return;
    }

    step = (abs_diff < MOTOR_RAMP_MAX_STEP_RPM) ? abs_diff : MOTOR_RAMP_MAX_STEP_RPM;
    motor_ramp_raw += (diff >= 0.0f) ? step : -step;
}

/* 统一的电机保护：检测摔车并在触发时复位 PID、清零输出。
 * 返回 1 表示已进入保护（调用方应跳过本拍正常输出），0 表示正常。 */
static uint8 motor_apply_protection(void)
{
    if (!motor_is_fallen())
    {
        return 0u;
    }

    motor_reset_speed_pid();
    motor_ramp_raw = 0.0f;
    motor_duty_out = 0;
    motor_uart_driver_set_duty(0);
    return 1u;
}

static void motor_update_speed_output(float raw_speed, float lean)
{
    float abs_lean = (lean >= 0.0f) ? lean : -lean;
    float target_raw;

    motor_ramp_to_target(motor_target_raw);
    target_raw = motor_ramp_raw + (abs_lean * motor_speed_boost_k);

    /* 单车侧倾补偿：理想速度 = 设定速度 + abs(侧倾角) × 补偿系数。
     * 此处 lean 取自 motor_get_fall_angle()，对应 IMU pitch，
     * 在本车体坐标系下实际是左右侧倾角（不是俯仰）。
     * 左右倾都提速 → 高速下舵机回正力矩更大，帮平衡环更快扶起车身。
     * 上位机看到的 motor_target_raw 不被改写，只在喂给 PID 时叠加。 */
    pid_set_target(&motor_speed_pid, target_raw);
    motor_duty_out = (int16)pid_calculate_incremental(&motor_speed_pid, raw_speed);
}

void motor_init(void)
{
    pid_init(&motor_speed_pid,
             MOTOR_KP_DEFAULT, MOTOR_KI_DEFAULT, MOTOR_KD_DEFAULT,
             0.0f, MOTOR_LIMIT_DEFAULT);

    motor_target_raw = 0.0f;
    motor_ramp_raw = 0.0f;
    motor_duty_out = 0;
    motor_direct_mode = 0;
    motor_fall_threshold_deg = MOTOR_FALL_THRESHOLD_DEFAULT;
    motor_uart_driver_init();
    encoder_dir_init(MOTOR_FEEDBACK_ENCODER,
                     MOTOR_FEEDBACK_ENCODER_COUNT,
                     MOTOR_FEEDBACK_ENCODER_DIR);
    encoder_clear_count(MOTOR_FEEDBACK_ENCODER);
    motor_uart_driver_set_duty(0);
    motor_refresh_feedback();
}

void motor_set_duty(int16_t duty)
{
    motor_duty_out = motor_limit_duty(duty);
    motor_direct_mode = 1;
    motor_uart_driver_set_duty(motor_duty_out);
}

void motor_set_speed(int16_t speed)
{
    motor_enter_speed_mode((float)speed, 1u);
}

void motor_set_speed_mps(float mps)
{
    motor_enter_speed_mode(motor_mps_to_raw(mps), 1u);
}

void motor_update_speed_mps(float mps)
{
    motor_enter_speed_mode(motor_mps_to_raw(mps), 0u);
}

void motor_set_speed_pid(float kp, float ki, float kd)
{
    pid_set_params(&motor_speed_pid, kp, ki, kd);
    pid_reset(&motor_speed_pid);
}

void motor_set_speed_pid_term(pid_term_enum term, float value)
{
    pid_set_term(&motor_speed_pid, term, value);
    pid_reset(&motor_speed_pid);
}

void motor_set_speed_boost_k(float value)
{
    motor_speed_boost_k = value;
}

float motor_get_speed_boost_k(void)
{
    return motor_speed_boost_k;
}

const PID_T *motor_get_speed_pid(void)
{
    return &motor_speed_pid;
}

float motor_get_fall_angle(void)
{
    /* fall angle 实际使用 IMU pitch：本工程约定 pitch 为左右倾角。
     * 更换 IMU 后若左右倒车判断或平衡纠偏反向，先校准 imu_app.c 里的 pitch/gyro_y 映射方向。
     */
    return pitch;
}

void motor_set_fall_threshold(float threshold_deg)
{
    if (threshold_deg < 0.0f)
    {
        threshold_deg = -threshold_deg;
    }

    motor_fall_threshold_deg = threshold_deg;
}

float motor_get_fall_threshold(void)
{
    return motor_fall_threshold_deg;
}

uint8 motor_is_fallen(void)
{
    float lean = motor_get_fall_angle();

    return ((lean > motor_fall_threshold_deg) || (lean < -motor_fall_threshold_deg)) ? 1 : 0;
}

float motor_get_target_speed_mps(void)
{
    return motor_raw_to_mps((int16)motor_target_raw);
}

float motor_get_current_speed_mps(void)
{
    return wheel_speed;
}

int16_t motor_get_duty_output(void)
{
    return motor_duty_out;
}

encoder_index_enum motor_get_feedback_encoder(void)
{
    return MOTOR_FEEDBACK_ENCODER;
}

float motor_get_feedback_encoder_ppr(void)
{
    return MOTOR_ENCODER_PPR;
}

float motor_get_feedback_dt_s(void)
{
    return MOTOR_SPEED_FEEDBACK_DT_S;
}

void motor_tick(void)
{
    float lean;
    int16 raw_speed;

    /* motor_tick 是速度环真正闭环的地方，
     * 上面的 set 接口只负责改目标和模式。 */
    raw_speed = motor_rpm;
    lean = motor_get_fall_angle();

    if (motor_apply_protection())
    {
        return;
    }

    if (!motor_direct_mode)
    {
        motor_update_speed_output((float)raw_speed, lean);
    }

    motor_uart_driver_set_duty(motor_duty_out);
}
