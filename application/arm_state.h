/**
 * @file    arm_state.h
 * @brief   机械臂状态（7 个独立开关 + 一个打包函数）
 *
 * 设计：每个状态就是一个 uint8_t 变量，1 = 成立，0 = 不成立。
 *       想置位就  = 1，想清除就 = 0，想查询就 if (g_xxx)。
 *       上报给上位机时，用 ArmState_Pack() 一次性打包成协议那一个字节。
 *
 * 为什么不用位运算：7 个状态挤在一个字节里，改一位要"读-改-写"，
 * 中断和主循环同时改会丢位；拆成独立变量后单字节赋值本身是原子的，
 * 不需要关中断，也更好读、更好在调试器里看。
 */

#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <stdint.h>

extern volatile uint8_t g_auto;         /* 自动模式（0=手动 1=自动） */
extern volatile uint8_t g_homing;       /* 正在回零 */
extern volatile uint8_t g_fault;        /* 故障态，只能显式复位退出 */
extern volatile uint8_t g_moving;       /* 正在运动 */
extern volatile uint8_t g_i2c_fail;     /* I2C 传感器失效 */
extern volatile uint8_t g_cmd_timeout;  /* 指令超时 */
extern volatile uint8_t g_limit;        /* 内部用：发生过软限位钳位（不上报） */

/** 上电全部清零 */
void ArmState_Init(void);

/**
 * @brief 打包成协议 0x10 状态帧里的 status 字节
 *        bit0 自动 / bit1 回零 / bit2 故障 / bit3 运动 / bit4 I2C / bit5 超时
 *        bit6~7 保留，不置位
 */
uint8_t ArmState_Pack(void);

#endif /* ARM_STATE_H */
