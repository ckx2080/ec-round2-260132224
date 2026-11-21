/* bsp/bsp_pwm.c */
#include "bsp_pwm.h"
#include "tim.h"

#define PULSE_SAFE_US   1500U   /* 上电安全位置 */
#define PULSE_MIN_US    500U
#define PULSE_MAX_US    2500U

typedef struct {
    uint32_t ch;          /* TIM2 通道 */
    int16_t  q_min;       /* 软限位下限 0.1° */
    int16_t  q_max;       /* 软限位上限 0.1° */
    uint16_t pulse_min;   /* q_min 对应脉宽 µs */
    uint16_t pulse_max;   /* q_max 对应脉宽 µs */
    int8_t   dir;         /* +1 角度增大脉宽增大；-1 反向 */
} pwm_cal_t;

/* 标定表：先按默认值跑，T1 标定完再改这几个数 */
static const pwm_cal_t s_cal[PWM_J_NUM] = {
    { TIM_CHANNEL_1,   0, 1800, 500, 2500, +1 },
    { TIM_CHANNEL_2,   0, 1800, 500, 2500, +1 },
    { TIM_CHANNEL_3,   0, 1800, 500, 2500, +1 },
    { TIM_CHANNEL_4,   0, 1800, 500, 2500, +1 },
};

static int16_t  s_q_cmd[PWM_J_NUM];    /* 当前命令角度（给状态帧用） */
static uint16_t s_pulse[PWM_J_NUM];    /* 当前脉宽 */

/* 脉宽反算角度：初始化时用，保证 s_q_cmd 与实际输出的脉宽一致 */
static int16_t pulse_to_angle10(const pwm_cal_t *c, uint16_t us)
{
    int32_t span  = (int32_t)c->q_max - (int32_t)c->q_min;
    int32_t lo    = (int32_t)c->pulse_min;
    int32_t hi    = (int32_t)c->pulse_max;
    int32_t ratio = (hi > lo) ? (((int32_t)us - lo) * 1000 / (hi - lo)) : 0;

    if (c->dir < 0) {
        ratio = 1000 - ratio;
    }
    if (ratio < 0)    ratio = 0;
    if (ratio > 1000) ratio = 1000;

    return (int16_t)((int32_t)c->q_min + ratio * span / 1000);
}

void BSP_Pwm_Init(void)
{
    for (uint8_t i = 0; i < PWM_J_NUM; i++) {
        BSP_Pwm_SetPulseUs((pwm_joint_t)i, PULSE_SAFE_US);
        s_q_cmd[i] = pulse_to_angle10(&s_cal[i], PULSE_SAFE_US);
        (void)HAL_TIM_PWM_Start(&htim2, s_cal[i].ch);
    }
}

void BSP_Pwm_SetPulseUs(pwm_joint_t j, uint16_t us)
{
    if (j >= PWM_J_NUM) return;

    if (us < PULSE_MIN_US) us = PULSE_MIN_US;
    if (us > PULSE_MAX_US) us = PULSE_MAX_US;

    __HAL_TIM_SET_COMPARE(&htim2, s_cal[j].ch, (uint32_t)us * 2U);  /* 0.5µs/计数 */
    s_pulse[j] = us;
}

void BSP_Pwm_SetAngle10(pwm_joint_t j, int16_t q10)
{
    if (j >= PWM_J_NUM) return;

    const pwm_cal_t *c = &s_cal[j];

    /* 1. 软限位钳位 */
    if (q10 < c->q_min) q10 = c->q_min;
    if (q10 > c->q_max) q10 = c->q_max;

    /* 2. 线性映射，全程 int32，避免 int16 溢出 */
    int32_t span = (int32_t)c->q_max - (int32_t)c->q_min;      /* >0 */
    int32_t ratio = ((int32_t)q10 - c->q_min) * 1000 / span;   /* 0~1000 千分比 */
    int32_t lo = c->pulse_min;
    int32_t hi = c->pulse_max;
    int32_t p  = (c->dir >= 0) ? (lo + (hi - lo) * ratio / 1000)
                               : (hi - (hi - lo) * ratio / 1000);

    /* 3. 再钳一次脉宽（防止标定表填错） */
    if (p < (int32_t)PULSE_MIN_US) p = PULSE_MIN_US;
    if (p > (int32_t)PULSE_MAX_US) p = PULSE_MAX_US;

    __HAL_TIM_SET_COMPARE(&htim2, c->ch, (uint32_t)p * 2U);

    s_q_cmd[j] = q10;
    s_pulse[j] = (uint16_t)p;
}

int16_t BSP_Pwm_GetAngle10(pwm_joint_t j)
{
    return (j < PWM_J_NUM) ? s_q_cmd[j] : 0;
}

uint16_t BSP_Pwm_GetPulseUs(pwm_joint_t j)
{
    return (j < PWM_J_NUM) ? s_pulse[j] : 0U;
}