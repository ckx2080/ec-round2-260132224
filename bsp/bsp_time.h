#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

void     BSP_Time_Init(void);       /* 启动 TIM4 1ms 时基 */
uint32_t BSP_Time_Ms(void);         /* 上电至今的毫秒数 */
uint8_t  BSP_Time_Take5ms(void);    /* 每 5ms 返回一次 1，读后自动清零 */

#endif /* BSP_TIME_H */
