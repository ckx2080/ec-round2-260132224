/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动（HAL / 硬件 I2C 400kHz）
 *
 * 移植自本人雷达题工程的驱动，改动三处：
 *   1) ApplyBias 只减陀螺零偏 —— 加速度含重力，减均值会破坏倾角
 *   2) 增加 BusRecovery —— 拔插 SDA 后 F1 的 I2C 可能卡 BUSY
 *   3) 增加互补滤波 UpdatePitch —— 运动时单靠加速度不准
 * 其余寄存器操作、量程、单位换算保持原样。
 */

#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

typedef struct
{
    float gx;
    float gy;
    float gz;
} MPU6050_Bias;

static MPU6050_Bias mpu_bias;

static float   s_pitch;         /* 互补滤波输出，deg */
static uint8_t s_pitch_seeded;  /* 首次调用用加速度角做初值 */

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return (uint8_t)HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                                      I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U);
}

static uint8_t MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (uint8_t)HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                                     I2C_MEMADD_SIZE_8BIT, buf, len, 100U);
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0U;

    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3U, 20U) != HAL_OK) {
        return 0U;
    }
    if (MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &id, 1U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    if (id != 0x68U) {
        return 0U;
    }

    /* 软复位，等内部寄存器稳定 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    HAL_Delay(100U);

    /* 唤醒，用 X 轴陀螺 PLL 作时钟源（比内部 RC 稳） */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    /* 采样率 = 1kHz / (1+7) = 125Hz */
    if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    /* DLPF ≈ 44Hz */
    if (MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    /* 陀螺 ±250°/s，加速度 ±2g */
    if (MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00U) != (uint8_t)HAL_OK) {
        return 0U;
    }
    if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U) != (uint8_t)HAL_OK) {
        return 0U;
    }

    MPU6050_ResetBias();
    MPU6050_ResetPitch();

    return 1U;
}

uint8_t MPU6050_ReadRaw(MPU6050_Raw *raw)
{
    uint8_t buf[14];

    if (raw == 0) {
        return 0U;
    }
    if (MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14U) != (uint8_t)HAL_OK) {
        return 0U;
    }

    raw->ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    raw->ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    raw->az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    raw->temp = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    raw->gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    raw->gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    raw->gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    return 1U;
}

void MPU6050_Convert(const MPU6050_Raw *raw, MPU6050_Data *data)
{
    if ((raw == 0) || (data == 0)) {
        return;
    }

    data->ax = (float)raw->ax / MPU6050_ACCEL_SCALE;
    data->ay = (float)raw->ay / MPU6050_ACCEL_SCALE;
    data->az = (float)raw->az / MPU6050_ACCEL_SCALE;
    data->temp = (float)raw->temp / 340.0f + 36.53f;
    data->gx = (float)raw->gx / MPU6050_GYRO_SCALE;
    data->gy = (float)raw->gy / MPU6050_GYRO_SCALE;
    data->gz = (float)raw->gz / MPU6050_GYRO_SCALE;
}

uint8_t MPU6050_Read(MPU6050_Data *data)
{
    MPU6050_Raw raw;

    if (data == 0) {
        return 0U;
    }
    if (MPU6050_ReadRaw(&raw) == 0U) {
        return 0U;
    }

    MPU6050_Convert(&raw, data);
    MPU6050_ApplyBias(data);

    return 1U;
}

void MPU6050_Calibrate(uint16_t samples)
{
    MPU6050_Raw  raw;
    MPU6050_Data data;

    float sgx = 0.0f, sgy = 0.0f, sgz = 0.0f;
    uint16_t i;
    uint16_t valid = 0U;

    if (samples == 0U) {
        samples = 200U;
    }

    for (i = 0U; i < samples; i++) {
        if (MPU6050_ReadRaw(&raw) != 0U) {
            MPU6050_Convert(&raw, &data);
            sgx += data.gx;
            sgy += data.gy;
            sgz += data.gz;
            valid++;
        }
        HAL_Delay(2U);
    }

    if (valid == 0U) {
        MPU6050_ResetBias();
        return;
    }

    /* 只记陀螺零偏 */
    mpu_bias.gx = sgx / (float)valid;
    mpu_bias.gy = sgy / (float)valid;
    mpu_bias.gz = sgz / (float)valid;
}

void MPU6050_ApplyBias(MPU6050_Data *data)
{
    if (data == 0) {
        return;
    }

    /* 只减陀螺零偏：加速度含重力分量，减均值会让倾角偏掉 */
    data->gx -= mpu_bias.gx;
    data->gy -= mpu_bias.gy;
    data->gz -= mpu_bias.gz;
}

void MPU6050_ResetBias(void)
{
    mpu_bias.gx = 0.0f;
    mpu_bias.gy = 0.0f;
    mpu_bias.gz = 0.0f;
}

void MPU6050_GetAngle(const MPU6050_Data *data, float *roll, float *pitch)
{
    float yz;

    if ((data == 0) || (roll == 0) || (pitch == 0)) {
        return;
    }

    *roll = atan2f(data->ay, data->az) * 57.29578f;

    yz = sqrtf((data->ay * data->ay) + (data->az * data->az));
    *pitch = atan2f(-data->ax, yz) * 57.29578f;
}

void MPU6050_ResetPitch(void)
{
    s_pitch = 0.0f;
    s_pitch_seeded = 0U;
}

void MPU6050_UpdatePitch(const MPU6050_Data *data, float dt)
{
    float roll_acc, pitch_acc;

    if (data == 0) {
        return;
    }

    MPU6050_GetAngle(data, &roll_acc, &pitch_acc);
    (void)roll_acc;

    if (s_pitch_seeded == 0U) {
        s_pitch = pitch_acc;          /* 首次用加速度角做初值，避免从 0 爬升 */
        s_pitch_seeded = 1U;
        return;
    }

    /* 互补滤波：陀螺管短期，加速度管长期不漂 */
    s_pitch = 0.98f * (s_pitch + data->gy * dt) + 0.02f * pitch_acc;
}

float MPU6050_GetPitch(void)
{
    return s_pitch;
}

void MPU6050_BusRecovery(void)
{
    /* 拔插 SDA 后 F1 的 I2C 可能卡在 BUSY，DeInit + 重新初始化可清掉 */
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
}
