/**
 * @file    arm_imu.h
 * @brief   姿态检测应用层（T4）：初始化、20Hz 采样、故障判定
 *
 * 驱动只负责读寄存器算角度；"多久读一次、失败了怎么办、什么算故障"
 * 属于应用层，放在这里，main 里只留一行 Poll()。
 */

#ifndef ARM_IMU_H
#define ARM_IMU_H

#include <stdint.h>

/**
 * @brief 初始化 + 静止零偏校准（会阻塞约 0.7s，期间机械臂必须静止）
 * @return 1 = 成功，0 = 初始化失败（WHO_AM_I 不对 / I2C 无响应）
 */
uint8_t ArmImu_Init(void);

/** 主循环 5ms 调一次，内部限速到 20Hz */
void ArmImu_Poll(void);

/** 当前是否在正常读数（1 = 好，0 = 正在失败中） */
uint8_t ArmImu_IsOk(void);

/** 实测倾角（度，已减安装偏移） */
float ArmImu_GetPitch(void);

/** 安装偏移：让"实测倾角"与机械臂的标称角度对齐（标定时测一次） */
void ArmImu_SetOffset(float deg);

/** 运行中重新做零偏校准（需静止，可选） */
void ArmImu_Recalibrate(void);

#endif /* ARM_IMU_H */
