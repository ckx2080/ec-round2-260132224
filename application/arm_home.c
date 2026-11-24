/**
 * @file    arm_home.c
 * @brief   上电自动回零实现（T3）
 */

#include "arm_home.h"
#include "arm_state.h"

/* 零位姿态：四轴都到标定表里的 pulse_min（实测安全下限） */
static const int16_t k_pose_zero[PWM_J_NUM] = { 0, 0, 0, 0 };

void ArmHome_Start(ArmMotion *m, uint32_t duration_ms)
{
    if (duration_ms == 0U) {
        duration_ms = ARM_HOME_DURATION_MS;
    }

    g_homing = 1;                       /* 上位机据此显示"回零中" */
    ArmMotion_Plan(m, k_pose_zero, duration_ms);
}

uint8_t ArmHome_Poll(ArmMotion *m)
{
    if (!g_homing) {
        return 0U;
    }

    if (ArmMotion_Step(m, 5U) != 0U) {
        g_homing = 0;                   /* 到位 */
        return 1U;
    }
    return 0U;
}

uint8_t ArmHome_IsBusy(void)
{
    return g_homing;
}
