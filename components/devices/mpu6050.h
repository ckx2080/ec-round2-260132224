#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MPU6050 I2C address: AD0 = GND -> 0x68，F1 HAL 要求传左移后的 8 位形式 */
#define MPU6050_ADDR              0xD0U

/* Register map */
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

/* Default full-scale ranges */
#define MPU6050_ACCEL_SCALE       16384.0f  /* +/-2g  */
#define MPU6050_GYRO_SCALE        131.0f    /* +/-250 deg/s */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_Raw;

typedef struct
{
    float ax;      /* g */
    float ay;
    float az;

    float temp;    /* deg C */

    float gx;      /* deg/s */
    float gy;
    float gz;
} MPU6050_Data;

/*
 * Initialize MPU6050:
 *   - check WHO_AM_I
 *   - reset, wake up
 *   - set sample rate, DLPF and ranges
 * Return: 1 = success, 0 = not found / I2C error / wrong device
 */
uint8_t MPU6050_Init(void);

/* Burst-read the 14 raw bytes starting at ACCEL_XOUT_H. 1 = ok, 0 = error */
uint8_t MPU6050_ReadRaw(MPU6050_Raw *raw);

/* Convert raw counts to physical units (no I2C access) */
void MPU6050_Convert(const MPU6050_Raw *raw, MPU6050_Data *data);

/* Read + convert + apply bias. 1 = ok, 0 = I2C error */
uint8_t MPU6050_Read(MPU6050_Data *data);

/*
 * Measure gyro zero-rate bias (accel bias is NOT touched: accel carries
 * gravity, subtracting its average would corrupt the tilt angle).
 * Keep the sensor still during this call. samples = 0 -> default 200.
 */
void MPU6050_Calibrate(uint16_t samples);

/* Subtract the last calibration values (gyro only) */
void MPU6050_ApplyBias(MPU6050_Data *data);

/* Clear all calibration values */
void MPU6050_ResetBias(void);

/*
 * Roll/pitch from the accelerometer (degrees).
 * Static only: linear acceleration during motion corrupts it.
 */
void MPU6050_GetAngle(const MPU6050_Data *data, float *roll, float *pitch);

/*
 * Complementary-filter pitch (accel + gyro integration).
 * Call at a fixed rate; dt in seconds (e.g. 0.05 for 20Hz).
 * The first call seeds the filter with the accel angle.
 */
void  MPU6050_UpdatePitch(const MPU6050_Data *data, float dt);
float MPU6050_GetPitch(void);
void  MPU6050_ResetPitch(void);

/*
 * Bus recovery: DeInit + re-init the I2C peripheral.
 * Call when reads keep failing (SDA stuck) so a re-plugged sensor recovers.
 */
void MPU6050_BusRecovery(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
