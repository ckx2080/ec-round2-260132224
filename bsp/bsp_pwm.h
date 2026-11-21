/* bsp/bsp_pwm.h */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

typedef enum {
    PWM_J_BASE = 0,     /* 底座 q1  PA0  TIM2_CH1 */
    PWM_J_SHOULDER,     /* 肩   q2  PA1  TIM2_CH2 */
    PWM_J_ELBOW,        /* 肘   q3  PA2  TIM2_CH3 */
    PWM_J_GRIPPER,      /* 爪   q4  PA3  TIM2_CH4 */
    PWM_J_NUM
} pwm_joint_t;

void     BSP_Pwm_Init(void);
void     BSP_Pwm_SetAngle10(pwm_joint_t j, int16_t q10);
int16_t  BSP_Pwm_GetAngle10(pwm_joint_t j);
void     BSP_Pwm_SetPulseUs(pwm_joint_t j, uint16_t us);   /* 调试：跳过映射 */
uint16_t BSP_Pwm_GetPulseUs(pwm_joint_t j);

#endif