/**
 * @file    bsp_pwm_measure.h
 * @brief   脉宽回环测量（TIM3_CH1 / PA6 输入捕获）
 *
 * 用途：把 PA0(TIM2_CH1) 的舵机 PWM 用一根杜邦线接回 PA6，让单片机自己
 *       量自己输出的高电平宽度，用来验证 T1 的脉宽精度，无需示波器。
 *
 * 特性：
 *   - 只读不写：本模块从不修改任何 CCR，因此不会与插补器抢控制权；
 *   - 编译开关：PWM_MEASURE_ENABLE = 0 时整段不参与编译，0 字节开销；
 *   - 中断里只存两个数，打印放在主循环 1Hz，不拖累 200Hz 插补节拍。
 *
 * 接线：PA0 ──杜邦线──► PA6（共地即可，不需要共电源）
 */

#ifndef BSP_PWM_MEASURE_H
#define BSP_PWM_MEASURE_H

#include <stdint.h>

/* 1 = 启用；0 = 完全编译掉（正式构建建议 0，答辩演示再改 1） */
#ifndef PWM_MEASURE_ENABLE
#define PWM_MEASURE_ENABLE  0
#endif

/**
 * @brief 启动捕获（需在 MX_TIM3_Init() 之后调用一次）
 */
void BSP_PwmMeasure_Init(void);

/**
 * @brief 主循环里周期调用，1Hz 打印一次测量结果
 */
void BSP_PwmMeasure_Poll(void);

/**
 * @brief 最近一次测得的高电平宽度
 * @return 单位 0.5µs（PSC=35 → 2MHz）；0 表示还没测到完整周期
 */
uint16_t BSP_PwmMeasure_GetWidth_05us(void);

/**
 * @brief 累计完成的测量次数，用来判断中断有没有进来
 * @return 次数
 */
uint32_t BSP_PwmMeasure_GetEdges(void);

#endif /* BSP_PWM_MEASURE_H */
