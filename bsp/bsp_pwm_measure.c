/**
 * @file    bsp_pwm_measure.c
 * @brief   脉宽回环测量实现（TIM3_CH1 / PA6 输入捕获）
 *
 * 原理：
 *   TIM3 预分频到 2MHz（PSC=35），1 个计数 = 0.5µs。
 *   通道 1 先配成上升沿捕获，捕获到上升沿时记下计数器值并立刻把极性切成
 *   下降沿；下一次捕获到下降沿时，两次计数之差就是高电平宽度（单位 0.5µs）。
 *   计数器是 16 位、65535 后回绕，所以相减要做回绕补偿。
 *
 * 为什么只用一个通道：
 *   单根信号线只能在 CH1 上；用“换极性”的方式一个通道就能测宽度，省引脚、
 *   代码直观。硬件自动锁存边沿时刻，CPU 延迟不影响精度。
 */

#include "bsp_pwm_measure.h"

#if PWM_MEASURE_ENABLE

#include <stdio.h>
#include "tim.h"

#define MEAS_TIM        (&htim3)
#define MEAS_CHANNEL    TIM_CHANNEL_1
#define MEAS_PRINT_MS   1000U

static volatile uint32_t s_rise;        /* 上升沿时刻的计数值 */
static volatile uint16_t s_width_05us;  /* 高电平宽度，0.5µs/计数 */
static volatile uint8_t  s_edge;        /* 0 = 等下个上升沿，1 = 等下个下降沿 */
static volatile uint32_t s_edges;       /* 完成的测量次数 */

void BSP_PwmMeasure_Init(void)
{
    s_rise       = 0U;
    s_width_05us = 0U;
    s_edge       = 0U;
    s_edges      = 0U;

    (void)HAL_TIM_IC_Start_IT(MEAS_TIM, MEAS_CHANNEL);
}

uint16_t BSP_PwmMeasure_GetWidth_05us(void)
{
    return s_width_05us;
}

uint32_t BSP_PwmMeasure_GetEdges(void)
{
    return s_edges;
}

void BSP_PwmMeasure_Poll(void)
{
    static uint32_t last = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last) < MEAS_PRINT_MS) {
        return;
    }
    last = now;

    /* 0.5µs 单位换成 µs 打印：整数部分 + 半格 */
    uint16_t w = s_width_05us;
    printf("w=%u.%u us  edges=%lu\r\n",
           (unsigned)(w / 2U),
           (unsigned)((w % 2U) * 5U),
           (unsigned long)s_edges);
}

/**
 * @brief HAL 捕获回调（弱符号覆盖）
 * @note  中断上下文：只做寄存器读取与状态切换，不做打印
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) {
        return;
    }

    uint32_t v = HAL_TIM_ReadCapturedValue(htim, MEAS_CHANNEL);

    if (s_edge == 0U) {
        /* 上升沿：记起点，改为等待下降沿 */
        s_rise = v;
        s_edge = 1U;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, MEAS_CHANNEL,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        /* 下降沿：计数差即高电平宽度，注意 16 位计数器回绕 */
        uint32_t d = (v >= s_rise) ? (v - s_rise)
                                   : (0x10000U + v - s_rise);
        s_width_05us = (uint16_t)d;
        s_edge       = 0U;
        s_edges++;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, MEAS_CHANNEL,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

#endif /* PWM_MEASURE_ENABLE */
