/**
 * @file    arm_imu.c
 * @brief   姿态检测应用层实现（T4）
 *
 * 三件事：
 *   1) 初始化 + 零偏校准；
 *   2) 20Hz 采样并跑互补滤波，输出实测倾角；
 *   3) 失败计时：连续失败满 1 秒 -> 置 g_i2c_fail + g_fault，并尝试总线恢复。
 *
 * 故障语义：
 *   - g_i2c_fail 是"当前读不到"，恢复成功自动清；
 *   - g_fault 是"故障态"，但如果是 I2C 引起的，恢复后自动清（T4 要求
 *     "重新接入可恢复"）；急停等其它原因置的 g_fault 不受影响，仍需显式复位。
 */

#include "arm_imu.h"
#include "arm_state.h"
#include "bsp_time.h"
#include "mpu6050.h"

#define IMU_PERIOD_MS      50U     /* 20Hz */
#define IMU_FAIL_TIMEOUT   1000U   /* 连续失败 1s 判故障 */

static uint8_t  s_ok;              /* 当前读数是否正常 */
static uint32_t s_last_ms;         /* 上次采样时刻 */
static uint32_t s_fail_since;      /* 首次失败时刻，0 = 无失败 */
static uint8_t  s_fault_by_i2c;    /* g_fault 是否由 I2C 引起（用于自动恢复） */
static float    s_offset;          /* 安装偏移，deg */

uint8_t ArmImu_Init(void)
{
    s_ok           = 0U;
    s_last_ms      = 0U;
    s_fail_since   = 0U;
    s_fault_by_i2c = 0U;
    s_offset       = 0.0f;

    if (MPU6050_Init() == 0U) {
        return 0U;
    }

    HAL_Delay(300U);          /* 等机械臂停稳，再采零偏 */
    MPU6050_Calibrate(200U);  /* 只校陀螺，约 0.4s */
    MPU6050_ResetPitch();

    s_ok = 1U;
    return 1U;
}

void ArmImu_Poll(void)
{
    uint32_t now = BSP_Time_Ms();
    MPU6050_Data d;

    if ((now - s_last_ms) < IMU_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    if (MPU6050_Read(&d) != 0U) {
        /* 读到了：清失败计时、清 I2C 失效位 */
        s_ok         = 1U;
        s_fail_since = 0U;
        g_i2c_fail   = 0;

        /* 如果故障是 I2C 引起的，恢复后自动清（T4：重新接入可恢复） */
        if (s_fault_by_i2c != 0U) {
            g_fault        = 0;
            s_fault_by_i2c = 0U;
        }

        MPU6050_UpdatePitch(&d, (float)IMU_PERIOD_MS / 1000.0f);
        return;
    }

    /* 读失败 */
    s_ok = 0U;

    if (s_fail_since == 0U) {
        s_fail_since = now;              /* 记第一次失败的时刻 */
    }

    if ((now - s_fail_since) >= IMU_FAIL_TIMEOUT) {
        g_i2c_fail     = 1;
        g_fault        = 1;
        s_fault_by_i2c = 1U;

        MPU6050_BusRecovery();           /* 尝试解 BUSY，重插后能自愈 */
        s_fail_since = now;              /* 每秒重试一次恢复 */
    }
}

uint8_t ArmImu_IsOk(void)
{
    return s_ok;
}

float ArmImu_GetPitch(void)
{
    return MPU6050_GetPitch() - s_offset;
}

void ArmImu_SetOffset(float deg)
{
    s_offset = deg;
}

void ArmImu_Recalibrate(void)
{
    MPU6050_Calibrate(200U);
    MPU6050_ResetPitch();
}
