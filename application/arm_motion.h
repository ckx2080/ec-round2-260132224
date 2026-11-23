/**
 * @file    arm_motion.h
 * @brief   四轴归一化梯形插补（T2 核心）
 *
 * 思路：四轴共用一条 0→1 的进度曲线 s(t)，每 5ms 推进一拍，
 *       q_i(t) = q_start_i + (q_end_i - q_start_i) * s(t)
 *       因此"同起同停"是数学保证，而不是靠调参凑出来的。
 *
 * 形状（加速段 ta / 总时长 T）取"最慢的那个轴"的比例，
 * 再按 duration 整体拉伸，保证任何轴都不超 v_max / a_max。
 *
 * 角度单位统一 0.1°（900 = 90.0°），时间单位 ms。
 */

#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#include <stdint.h>
#include "bsp_pwm.h"

/** 一次运动的完整状态 */
typedef struct {
    int16_t  q_start[PWM_J_NUM];   /* 规划时刻的实际角度（0.1°），保证中途改目标不跳变 */
    int16_t  q_end[PWM_J_NUM];     /* 目标角度（0.1°），未钳位，钳位在 BSP_Pwm_SetAngle10 里做 */
    int16_t  q_cmd[PWM_J_NUM];     /* 每拍算出的输出角度（0.1°） */
    uint32_t T_ms;                 /* 总时长（ms） */
    uint32_t ta_ms;                /* 加速段时长（ms），决定梯形形状 */
    uint32_t t_ms;                 /* 已运行时间（ms） */
    uint8_t  running;              /* 1 = 正在运动，0 = 已到位/空闲 */
} ArmMotion;

/**
 * @brief 规划一次运动
 * @param m           运动对象
 * @param q_tgt       四轴目标角度（0.1°），顺序同 pwm_joint_t
 * @param duration_ms 期望总时长；传 0 表示"按机械能达到的最快速度"。
 *                    若给得比机械极限还短，内部会自动延长到可行值（不会超速）
 * @note  以当前 BSP_Pwm_GetAngle10() 为起点，因此可在运动中再次调用（平滑改目标）
 */
void ArmMotion_Plan(ArmMotion *m, const int16_t q_tgt[PWM_J_NUM], uint32_t duration_ms);

/**
 * @brief 推进一拍插补
 * @param m      运动对象
 * @param dt_ms  距上一拍的时间（固定 5ms）
 * @return 1 = 这一拍刚好结束（四轴到达终点），0 = 仍在运动中/空闲
 * @note  内部会把四轴角度写入 BSP_Pwm_SetAngle10()
 */
uint8_t ArmMotion_Step(ArmMotion *m, uint32_t dt_ms);

/**
 * @brief 是否正在运动（对应状态帧 bit3"正在运动"）
 */
uint8_t ArmMotion_IsRunning(const ArmMotion *m);

#endif /* ARM_MOTION_H */
