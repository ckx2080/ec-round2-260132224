/**
 * @file    arm_home.h
 * @brief   上电自动回零（T3）
 *
 * 零位定义：四轴都到 q = 0，也就是标定表里的 pulse_min。
 * 这些 pulse_min 是实测出来的安全下限（已留余量），所以"回零"不是
 * 撞机械限位，而是走到一个用绝对脉宽定义的、可复现的位置。
 *
 * 回零过程：g_homing = 1 → 用插补慢速(默认 1.5s)走到零位 → g_homing = 0。
 */

#ifndef ARM_HOME_H
#define ARM_HOME_H

#include <stdint.h>
#include "arm_motion.h"

#define ARM_HOME_DURATION_MS   1500U    /* 回零总时长，宁慢勿快 */

/**
 * @brief 开始回零
 * @param m           运动对象（与日常运动共用同一个，避免两套轨迹打架）
 * @param duration_ms 时长，传 0 用 ARM_HOME_DURATION_MS
 */
void ArmHome_Start(ArmMotion *m, uint32_t duration_ms);

/**
 * @brief 回零推进，需在 5ms 节拍里调用
 * @return 1 = 这一拍刚好回零完成，0 = 仍在回零或未在回零
 */
uint8_t ArmHome_Poll(ArmMotion *m);

/** 是否正在回零（等价于 g_homing 为 1） */
uint8_t ArmHome_IsBusy(void);

#endif /* ARM_HOME_H */
