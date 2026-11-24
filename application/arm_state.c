/**
 * @file    arm_state.c
 * @brief   机械臂状态：7 个开关 + 打包
 */

#include "arm_state.h"

volatile uint8_t g_auto;
volatile uint8_t g_homing;
volatile uint8_t g_fault;
volatile uint8_t g_moving;
volatile uint8_t g_i2c_fail;
volatile uint8_t g_cmd_timeout;
volatile uint8_t g_limit;

void ArmState_Init(void)
{
    g_auto        = 0;
    g_homing      = 0;
    g_fault       = 0;
    g_moving      = 0;
    g_i2c_fail    = 0;
    g_cmd_timeout = 0;
    g_limit       = 0;
}

uint8_t ArmState_Pack(void)
{
    uint8_t b = 0;

    if (g_auto)        b |= (1u << 0);
    if (g_homing)      b |= (1u << 1);
    if (g_fault)       b |= (1u << 2);
    if (g_moving)      b |= (1u << 3);
    if (g_i2c_fail)    b |= (1u << 4);
    if (g_cmd_timeout) b |= (1u << 5);
    /* bit6~bit7 协议保留，g_limit 只在内部使用，不上报 */

    return b;
}
