/*
 * ekf.c
 *
 *  EKF for IMU660RB/IMU660RX attitude estimation
 *  Copyright: KimJong
 */

#include "ekf.h"
#include "matrix.h"
#include "../common/angle_util.h"

matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle;
imu_t imu_data = {0, 0, 0, 0, 0, 0};
matrix_type r_yz = 0.001f;

#define IMU_EKF_UPDATE_PERIOD_MS        (2u)
#define IMU_GYRO_BIAS_WAIT_TIME_MS      (300u)
#define IMU_GYRO_BIAS_SAMPLE_TIME_MS    (400u)
#define IMU_GYRO_BIAS_WAIT_TICKS        (IMU_GYRO_BIAS_WAIT_TIME_MS / IMU_EKF_UPDATE_PERIOD_MS)
#define IMU_GYRO_BIAS_SAMPLE_COUNT      (IMU_GYRO_BIAS_SAMPLE_TIME_MS / IMU_EKF_UPDATE_PERIOD_MS)
#define IMU_GYRO_STILL_THRESHOLD_DPS    (3.0f)

static const matrix_type q[4][4] = {
    {0.005f, 0, 0, 0},
    {0, 0.005f, 0, 0},
    {0, 0, 0.005f, 0},
    {0, 0, 0, 0.005f}
};
static const matrix_type r[3][3] = {
    {10000, 0, 0},
    {0, 10000, 0},
    {0, 0, 10000}
};
static const matrix_type p[4][4] = {
    {1000000, 0, 0, 0},
    {0, 1000000, 0, 0},
    {0, 0, 1000000, 0},
    {0, 0, 0, 1000000}
};
static const matrix_type ekf_init_data[4] = {1, 0, 0, 0};

static matrix_t Q;
static matrix_t R;
static matrix_t P;
static uint32 s_imu_still_ticks = 0;
static uint32 s_gyro_bias_sample_count = 0;
static uint8 s_gyro_bias_ready = 0;
static uint8 s_capture_yaw_zero = 0;
static float s_gyro_bias_sum_x = 0.0f;
static float s_gyro_bias_sum_y = 0.0f;
static float s_gyro_bias_sum_z = 0.0f;
static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;
static float s_yaw_zero_deg = 0.0f;

static float ekf_wrap_deg(float angle_deg)
{
    return angle_wrap_deg(angle_deg);
}

static void ekf_reset_gyro_bias_sampling(void)
{
    s_imu_still_ticks = 0;
    s_gyro_bias_sample_count = 0;
    s_gyro_bias_sum_x = 0.0f;
    s_gyro_bias_sum_y = 0.0f;
    s_gyro_bias_sum_z = 0.0f;
}

static uint8 ekf_gyro_is_still(float gyro_x, float gyro_y, float gyro_z)
{
    if (fabsf(gyro_x * EKF_RAD_TO_DEG) > IMU_GYRO_STILL_THRESHOLD_DPS)
    {
        return 0;
    }
    if (fabsf(gyro_y * EKF_RAD_TO_DEG) > IMU_GYRO_STILL_THRESHOLD_DPS)
    {
        return 0;
    }
    if (fabsf(gyro_z * EKF_RAD_TO_DEG) > IMU_GYRO_STILL_THRESHOLD_DPS)
    {
        return 0;
    }

    return 1;
}

static void ekf_update_gyro_bias(float gyro_x, float gyro_y, float gyro_z)
{
    if (s_gyro_bias_ready)
    {
        return;
    }

    if (!ekf_gyro_is_still(gyro_x, gyro_y, gyro_z))
    {
        ekf_reset_gyro_bias_sampling();
        return;
    }

    if (s_imu_still_ticks < IMU_GYRO_BIAS_WAIT_TICKS)
    {
        s_imu_still_ticks++;
        return;
    }

    s_gyro_bias_sum_x += gyro_x;
    s_gyro_bias_sum_y += gyro_y;
    s_gyro_bias_sum_z += gyro_z;
    s_gyro_bias_sample_count++;

    if (s_gyro_bias_sample_count >= IMU_GYRO_BIAS_SAMPLE_COUNT)
    {
        s_gyro_bias_x = s_gyro_bias_sum_x / (float)IMU_GYRO_BIAS_SAMPLE_COUNT;
        s_gyro_bias_y = s_gyro_bias_sum_y / (float)IMU_GYRO_BIAS_SAMPLE_COUNT;
        s_gyro_bias_z = s_gyro_bias_sum_z / (float)IMU_GYRO_BIAS_SAMPLE_COUNT;
        s_gyro_bias_ready = 1;
        s_capture_yaw_zero = 1;
    }
}

void EKF_Init(void)
{
    Matrix_From_Array(&exf_x, (const matrix_type *)ekf_init_data, 4, 1);
    Matrix_From_Array(&Q, (const matrix_type *)q, 4, 4);
    Matrix_From_Array(&R, (const matrix_type *)r, 3, 3);
    Matrix_From_Array(&P, (const matrix_type *)p, 4, 4);

    s_gyro_bias_ready = 0;
    s_capture_yaw_zero = 0;
    s_gyro_bias_x = 0.0f;
    s_gyro_bias_y = 0.0f;
    s_gyro_bias_z = 0.0f;
    s_yaw_zero_deg = 0.0f;
    ekf_reset_gyro_bias_sampling();
}

static inline void quaternion_to_euler(void)
{
    float q0 = exf_x.data[0][0];
    float q1 = exf_x.data[1][0];
    float q2 = exf_x.data[2][0];
    float q3 = exf_x.data[3][0];

    euler_angle.pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * EKF_RAD_TO_DEG;
    euler_angle.roll = atan2f(2 * q2 * q3 + 2 * q0 * q1,
                              -2 * q1 * q1 - 2 * q2 * q2 + 1) * EKF_RAD_TO_DEG;
    euler_angle.yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3,
                             -2 * q2 * q2 - 2 * q3 * q3 + 1) * EKF_RAD_TO_DEG;
}

static int16 imu660rx_acc_x_l = 0;
static int16 imu660rx_acc_y_l = 0;
static int16 imu660rx_acc_z_l = 0;

static void imu_get_values(void)
{
    float raw_gyro_x;
    float raw_gyro_y;
    float raw_gyro_z;

    imu660rx_get_gyro();
    imu660rx_get_acc();

    imu_data.acc_x = EKF_ACC_BLEND_K * imu660rx_acc_x + (1 - EKF_ACC_BLEND_K) * imu660rx_acc_x_l;
    imu_data.acc_y = EKF_ACC_BLEND_K * imu660rx_acc_y + (1 - EKF_ACC_BLEND_K) * imu660rx_acc_y_l;
    imu_data.acc_z = EKF_ACC_BLEND_K * imu660rx_acc_z + (1 - EKF_ACC_BLEND_K) * imu660rx_acc_z_l;
    imu660rx_acc_x_l = (int16)imu_data.acc_x;
    imu660rx_acc_y_l = (int16)imu_data.acc_y;
    imu660rx_acc_z_l = (int16)imu_data.acc_z;

    raw_gyro_x = imu660rx_gyro_x * PI / 180.0f / 16.384f;
    raw_gyro_y = imu660rx_gyro_y * PI / 180.0f / 16.384f;
    raw_gyro_z = imu660rx_gyro_z * PI / 180.0f / 16.384f;

    ekf_update_gyro_bias(raw_gyro_x, raw_gyro_y, raw_gyro_z);

    if (s_gyro_bias_ready)
    {
        imu_data.gyro_x = raw_gyro_x - s_gyro_bias_x;
        imu_data.gyro_y = raw_gyro_y - s_gyro_bias_y;
        imu_data.gyro_z = raw_gyro_z - s_gyro_bias_z;
    }
    else
    {
        imu_data.gyro_x = raw_gyro_x;
        imu_data.gyro_y = raw_gyro_y;
        imu_data.gyro_z = raw_gyro_z;
    }
}

void EKF_UpData(void)
{
    float gx;
    float gy;
    float gz;
    float q0;
    float q1;
    float q2;
    float q3;
    matrix_t Z;
    matrix_t F;
    matrix_t FT;
    matrix_t H;
    matrix_t HT;
    matrix_t PK_;
    matrix_t DK;
    matrix_t invDK;
    matrix_t EK;
    matrix_t EKT;
    matrix_t Kk;
    matrix_t temp;
    matrix_t I;
    matrix_type f[4][4];
    matrix_type h[3][4];

    imu_get_values();
    gx = imu_data.gyro_x;
    gy = imu_data.gyro_y;
    /* 航向标度修正：只缩放进入四元数积分的 Z 轴角速度。
     * yaw 无加速度计修正，是纯陀螺积分，故缩放 gz 即等比缩放 yaw；
     * pitch/roll 仍由加速度计观测修正，不受影响。 */
    gz = imu_data.gyro_z * IMU_YAW_GYRO_SCALE;

    Matrix_Init(&Z, 3, 1);
    Z.data[0][0] = (matrix_type)imu_data.acc_x;
    Z.data[1][0] = (matrix_type)imu_data.acc_y;
    Z.data[2][0] = (matrix_type)imu_data.acc_z;
    normalize_vector(&Z);

    f[0][0] = 1;               f[0][1] = -0.5f * gx * EKF_DT_S; f[0][2] = -0.5f * gy * EKF_DT_S; f[0][3] = -0.5f * gz * EKF_DT_S;
    f[1][0] = 0.5f * gx * EKF_DT_S;  f[1][1] = 1;               f[1][2] = 0.5f * gz * EKF_DT_S;  f[1][3] = -0.5f * gy * EKF_DT_S;
    f[2][0] = 0.5f * gy * EKF_DT_S;  f[2][1] = -0.5f * gz * EKF_DT_S; f[2][2] = 1;                f[2][3] = 0.5f * gx * EKF_DT_S;
    f[3][0] = 0.5f * gz * EKF_DT_S;  f[3][1] = 0.5f * gy * EKF_DT_S;  f[3][2] = -0.5f * gx * EKF_DT_S;  f[3][3] = 1;

    Matrix_From_Array(&F, (const matrix_type *)f, 4, 4);
    FT = Matrix_Transpose(&F);

    exf_x = multiply_matrices(&F, &exf_x);
    normalize_vector(&exf_x);

    q0 = exf_x.data[0][0];
    q1 = exf_x.data[1][0];
    q2 = exf_x.data[2][0];
    q3 = exf_x.data[3][0];

    h[0][0] = -2 * q2; h[0][1] = 2 * q3;  h[0][2] = -2 * q0; h[0][3] = 2 * q1;
    h[1][0] = 2 * q1;  h[1][1] = 2 * q0;  h[1][2] = 2 * q3;  h[1][3] = 2 * q2;
    h[2][0] = 2 * q0;  h[2][1] = -2 * q1; h[2][2] = -2 * q2; h[2][3] = 2 * q3;

    Matrix_From_Array(&H, (const matrix_type *)h, 3, 4);
    HT = Matrix_Transpose(&H);

    PK_ = multiply_matrices(&F, &P);
    PK_ = multiply_matrices(&PK_, &FT);
    P = add_matrices(&PK_, &Q);

    DK = multiply_matrices(&H, &P);
    DK = multiply_matrices(&DK, &HT);
    DK = add_matrices(&DK, &R);

    if (inverse_matrix(&DK, &invDK))
    {
        quaternion_to_euler();
        return;
    }

    EK = multiply_matrices(&H, &exf_x);
    EK = subtract_matrices(&Z, &EK);
    EKT = Matrix_Transpose(&EK);

    error = multiply_matrices(&EKT, &invDK);
    error = multiply_matrices(&error, &EK);

    if (error.data[0][0] > r_yz)
    {
        quaternion_to_euler();
        return;
    }

    Kk = multiply_matrices(&P, &HT);
    Kk = multiply_matrices(&Kk, &invDK);

    temp = multiply_matrices(&Kk, &EK);
    exf_x = add_matrices(&exf_x, &temp);
    normalize_vector(&exf_x);

    Matrix_Identity(&I, 4);
    temp = multiply_matrices(&Kk, &H);
    temp = subtract_matrices(&I, &temp);
    P = multiply_matrices(&temp, &P);

    quaternion_to_euler();

    if (s_capture_yaw_zero)
    {
        s_yaw_zero_deg = euler_angle.yaw;
        s_capture_yaw_zero = 0;
    }
}

uint8 EKF_IsReady(void)
{
    return s_gyro_bias_ready;
}

void EKF_CaptureYawZero(void)
{
    if (!s_gyro_bias_ready)
    {
        return;
    }

    s_yaw_zero_deg = euler_angle.yaw;
    s_capture_yaw_zero = 0;
}

float EKF_GetYawCompensatedDeg(void)
{
    if (!s_gyro_bias_ready)
    {
        return 0.0f;
    }

    return ekf_wrap_deg(euler_angle.yaw - s_yaw_zero_deg);
}
