/* application/arm_motion.c */
#include <math.h>
#include "arm_motion.h"

/* 每轴限制：v 单位 0.1°/s，a 单位 0.1°/s²（按结构实测再改） */
typedef struct {
    int16_t  q_min, q_max;
    uint16_t v_max;
    uint16_t a_max;
} axis_lim_t;

static const axis_lim_t s_lim[PWM_J_NUM] = {
    {    0, 1800,  900, 1800 },   /* 底座 90°/s, 180°/s² */
    {    0, 1800,  900, 1800 },   /* 肩 */
    {    0, 1800,  900, 1800 },   /* 肘 */
    {    0, 1800, 1800, 3600 },   /* 爪：行程短，可快 */
};

/* 单轴最短时间与该轴对应的加速段 */
static void axis_min_time(int32_t d10, uint16_t v10, uint16_t a10,
                          float *T_out, float *ta_out)
{
    float d  = (float)((d10 < 0) ? -d10 : d10) / 10.0f;   /* 0.1° → ° */
    float v  = (float)v10 / 10.0f;                        /* °/s */
    float a  = (float)a10 / 10.0f;                        /* °/s² */
    float ta = v / a;
    float da = 0.5f * a * ta * ta;                        /* 加速段走的距离 */

    if (2.0f * da >= d) {          /* 三角形：还没到最高速就要减速 */
        ta = sqrtf(d / a);
        *ta_out = ta;
        *T_out  = 2.0f * ta;
    } else {                       /* 梯形：有匀速段 */
        float tc = (d - 2.0f * da) / v;
        *ta_out = ta;
        *T_out  = 2.0f * ta + tc;
    }
}

void ArmMotion_Plan(ArmMotion *m, const int16_t q_tgt[PWM_J_NUM], uint32_t duration_ms)
{
    float T_max = 0.0f;
    float ta_at_max = 1.0f;

    for (uint8_t i = 0; i < PWM_J_NUM; i++) {
        m->q_start[i] = BSP_Pwm_GetAngle10((pwm_joint_t)i);
        m->q_end[i]   = q_tgt[i];
        m->q_cmd[i]   = m->q_start[i];

        float T_i, ta_i;
        axis_min_time((int32_t)m->q_end[i] - m->q_start[i],
                      s_lim[i].v_max, s_lim[i].a_max, &T_i, &ta_i);
        if (T_i > T_max) {         /* 记录最慢的轴 */
            T_max    = T_i;
            ta_at_max = ta_i;
        }
    }

    /* 不低于机械能做到的最短时间 */
    float T_req = (float)duration_ms;
    if (T_req < T_max) T_req = T_max;
    if (T_req < 20.0f) T_req = 20.0f;      /* 至少要几个插补拍 */

    float k = T_req / ((T_max > 0.0f) ? T_max : 1.0f);   /* 拉伸系数 */

    m->T_ms    = (uint32_t)(T_req + 0.5f);
    m->ta_ms   = (uint32_t)(ta_at_max * k + 0.5f);
    if (m->ta_ms < 1U) m->ta_ms = 1U;
    if (m->ta_ms > m->T_ms / 2U) m->ta_ms = m->T_ms / 2U;
    m->t_ms    = 0U;
    m->running = 1U;
}

static float s_of(uint32_t t, uint32_t T, uint32_t ta)
{
    float vn = 1.0f / (float)(T - ta);        /* 归一化峰值速度 */
    float an = vn / (float)ta;                /* 归一化加速度 */
    float st = 0.5f * an * (float)ta * (float)ta;

    if (t <= ta) {
        return 0.5f * an * (float)t * (float)t;
    }
    if (t <= (T - ta)) {
        return st + vn * (float)(t - ta);
    }
    float r = (float)(T - t);
    return 1.0f - 0.5f * an * r * r;
}

uint8_t ArmMotion_Step(ArmMotion *m, uint32_t dt_ms)
{
    if (m->running == 0U) return 0U;

    m->t_ms += dt_ms;

    uint8_t done = 0U;
    float s;

    if (m->t_ms >= m->T_ms) {
        s = 1.0f;
        done = 1U;
        m->running = 0U;
    } else {
        s = s_of(m->t_ms, m->T_ms, m->ta_ms);
    }

    for (uint8_t i = 0; i < PWM_J_NUM; i++) {
        int32_t d = (int32_t)m->q_end[i] - (int32_t)m->q_start[i];
        int32_t q = (int32_t)m->q_start[i] + (int32_t)((float)d * s + 0.5f * ((d >= 0) ? 1.0f : -1.0f));
        m->q_cmd[i] = (int16_t)q;
        BSP_Pwm_SetAngle10((pwm_joint_t)i, m->q_cmd[i]);
    } 

    return done;
}

uint8_t ArmMotion_IsRunning(const ArmMotion *m)
{
    return m->running;
}