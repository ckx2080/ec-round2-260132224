#include "bsp_time.h"
#include "tim.h"
#define TIME_5MS_DIV 5U

static volatile uint32_t s_ms;      /* 1ms 累计 */
static volatile uint8_t  s_flag5;   /* 5ms 到点标志 */
static uint8_t           s_cnt;     /* 1ms 分频计数 */

void BSP_Time_Init(void)
{
    s_ms    = 0U;
    s_flag5 = 0U;
    s_cnt   = 0U;
    (void)HAL_TIM_Base_Start_IT(&htim4);
}

uint32_t BSP_Time_Ms(void)
{
    return s_ms;
}

uint8_t BSP_Time_Take5ms(void)
{
    if (s_flag5 != 0U) {
        s_flag5 = 0U;
        return 1U;
    }
    return 0U;
}

/* 覆盖 HAL 的弱回调：所有定时器更新中断都进这里，所以必须判断实例 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) {
        return;
    }

    s_ms++;

    if (++s_cnt >= TIME_5MS_DIV) {
        s_cnt   = 0U;
        s_flag5 = 1U;
    }
}
