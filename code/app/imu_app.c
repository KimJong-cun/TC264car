/*
 * imu_app.c
 *
 *  IMU application layer based on IMU660RC internal SFLP fusion
 *  Copyright: KimJong
 */

#include "imu_app.h"
#include "../driver/common/angle_util.h"

/* IMU direction notes.
 * 这里只标记当前代码使用的轴和正负号，不改控制逻辑。
 * 更换 IMU 后用类似 imu_test 的 demo 实测左右/转向正负，
 * 若方向反了，在 IMU 映射或参数正负处调整，尽量不要动 PID。
 *
 * 当前控制/导航约定：
 *   - pitch 在本工程里被当作”左右倾角/倒车角”使用，不是传统飞机俯仰角；
 *   - gyro_y_rate 是左右倾角速度，动态方向需要和 pitch 匹配；
 *   - 导航坐标系为 x 右正、y 前正；
 *   - yaw 右转/顺时针为正时，里程计会得到 x 正方向位移。
 *
 * IMU660RC 内部 SFLP 解算 + 实测轴向（与上述约定一致，故 1:1 直接映射）：
 *   - imu660rc_pitch  左正右负，正好是平衡用左右倾角 → 不取反；
 *   - imu660rc_gyro_y 左正右负，是左右倾角速度，与 pitch 动态匹配 → 不取反；
 *   - imu660rc_yaw    右转增大/左转减小（右转为正），与导航约定一致 → 不取反；
 *   - imu660rc_acc_y  往前为正，用于里程计起步检测的前后加速度。
 * 四元数在 INT2(480Hz) 中断里由 imu660rc_callback() 读取并转欧拉角，
 * 应用层 imu_proc() 只做拷贝/零点补偿，不再做姿态融合。
 */
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float gyro_x_rate = 0.0f;
float gyro_y_rate = 0.0f;
float gyro_y_lpf_alpha = 0.13f;     // 低通滤波系数 0~1，越小越平滑，1.0=不滤波
float gyro_y_deadzone = 0.0f;      // 陀螺仪死区，±2.0°/s以内视为0
float gyro_z_rate = 0.0f;

/* IMU零点校准偏移量 */
float pitch_offset = 0.0f;         // pitch零点偏移，车直立时的pitch值
float roll_offset = 0.0f;          // roll零点偏移
float yaw_offset = 0.0f;           // yaw零点偏移

/* 诊断用：INT2(P10_3) 中断进入计数，正常应随时间快速增长。若恒为0说明中断没触发。 */
volatile uint32 imu_int2_count = 0;

float roll_kalman = 0.0f;
float pitch_kalman = 0.0f;
float yaw_kalman = 0.0f;

static float imu_wrap_angle_deg(float angle_deg)
{
    return angle_wrap_deg(angle_deg);
}

void usart_send(float roll_value, float pitch_value, float yaw_value, uint8_t fusion_sta)
{
    uint8_t buffer[15];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t index = 0;
    uint8_t i;
    int16_t roll_int = (int16_t)(roll_value * 100.0f);
    int16_t pitch_int = (int16_t)(pitch_value * 100.0f);
    int16_t yaw_int = (int16_t)(yaw_value * 100.0f);

    /* 这个数据包格式已经被现有上位机工具消费，
     * 包头和校验顺序都不能随便改。 */
    buffer[index++] = 0xAB;
    buffer[index++] = 0xDC;
    buffer[index++] = 0xFE;
    buffer[index++] = 0x03;
    buffer[index++] = 7;
    buffer[index++] = 0;

    buffer[index++] = (uint8_t)(roll_int & 0xFF);
    buffer[index++] = (uint8_t)((roll_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(pitch_int & 0xFF);
    buffer[index++] = (uint8_t)((pitch_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(yaw_int & 0xFF);
    buffer[index++] = (uint8_t)((yaw_int >> 8) & 0xFF);
    buffer[index++] = fusion_sta;

    for (i = 0; i < index; i++)
    {
        sumcheck += buffer[i];
        addcheck += sumcheck;
    }

    buffer[index++] = sumcheck;
    buffer[index++] = addcheck;

    for (i = 0; i < index; i++)
    {
        my_printf(WIRELESS_UART_INDEX, "%c", buffer[i]);
    }
}

void imu_all_init(void)
{
    /* IMU660RC 内部 SFLP 解算，480Hz 四元数输出，INT2 上升沿中断驱动。
     * 初始化失败则重试，直到成功。 */
    while (imu660rc_init(IMU660RC_QUARTERNION_480HZ))
    {
        system_delay_ms(100);
    }

    /* 静止零偏校准并注入 SFLP。校准期间关闭 INT2 中断，
     * 避免中断里的 SPI 读取与校准流程的 SPI 写入相互抢占。 */
    exti_disable(IMU660RC_INT2_PIN);
    imu660rc_bias_calibrate(2000);
    exti_enable(IMU660RC_INT2_PIN);

    system_delay_ms(200);

    /* 以当前航向为 yaw 零点基准，并刷新一次对外缓存。 */
    imu_capture_yaw_zero();
}

uint8 imu_is_ready(void)
{
    /* SFLP 由芯片内部完成，初始化成功即视为就绪。 */
    return 1u;
}

void imu_proc(void)
{
    /* 姿态融合已在芯片内部完成（INT2 中断里 imu660rc_callback 读取四元数）。
     * 这里只把 IMU660RC 的输出按工程约定拷贝/补偿，不再做 EKF。
     *
     * kalman 视图沿用旧命名，保存未扣零点的原始观测，便于自动校准与遥测对比。 */

    /* roll: 直接沿用 RC roll 正负。 */
    roll_kalman = imu660rc_roll;
    /* pitch: RC pitch 左正右负，正好是平衡/倒车用左右倾角，直接沿用。 */
    pitch_kalman = imu660rc_pitch;
    /* yaw: RC yaw 右转为正，与导航约定一致，直接沿用。 */
    yaw_kalman = imu660rc_yaw;

    roll = roll_kalman - roll_offset;
    pitch = pitch_kalman - pitch_offset;
    yaw = imu_wrap_angle_deg(yaw_kalman - yaw_offset);

    /* gyro_x_rate: 直接沿用 RC gyro_x 正负，单位 °/s。 */
    gyro_x_rate = imu660rc_gyro_transition(imu660rc_gyro_x);
    {
        /* gyro_y_rate: 平衡内环使用的左右倾角速度，RC gyro_y 左正右负，直接沿用。 */
        float gyro_y_raw = imu660rc_gyro_transition(imu660rc_gyro_y);

        /* 直接复用上一拍滤波结果作为状态，
         * 这样应用层不需要再额外维护一个滤波器对象。 */
        gyro_y_rate = gyro_y_lpf_alpha * gyro_y_raw + (1.0f - gyro_y_lpf_alpha) * gyro_y_rate;

        // 应用死区：小于阈值的读数视为0
        if (gyro_y_rate > -gyro_y_deadzone && gyro_y_rate < gyro_y_deadzone)
        {
            gyro_y_rate = 0.0f;
        }
    }
    /* gyro_z_rate: 航向角速度，RC gyro_z 与 yaw 右转为正匹配，直接沿用。 */
    gyro_z_rate = imu660rc_gyro_transition(imu660rc_gyro_z);
}

void imu_capture_yaw_zero(void)
{
    /* 以当前 RC 航向为零点基准：记录当前 yaw 作为偏移，使对外 yaw 归零。 */
    yaw_kalman = imu660rc_yaw;
    yaw_offset = yaw_kalman;
    yaw = imu_wrap_angle_deg(yaw_kalman - yaw_offset);
}

void imu_origin(void)
{
    my_printf(WIRELESS_UART_INDEX, "roll=%.2f,pitch=%.2f,yaw=%.2f\r\n",
           (double)roll_kalman, (double)pitch_kalman, (double)yaw_kalman);
}

void imu_test(void)
{
    /* imu_int2_count 应随时间快速增长（480Hz）。若恒为0 = INT2中断没触发，
     * 检查 P10_3 接线 / IMU INT2 引脚 / exti 是否被使能。 */
    my_printf(WIRELESS_UART_INDEX, "int2=%lu yaw=%.2f gyro_z=%.2f ready=%d\r\n",
           (unsigned long)imu_int2_count, (double)yaw, (double)gyro_z_rate, imu_is_ready());
}
