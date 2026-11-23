/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bsp_pwm.h"
#include "bsp_pwm_measure.h"
#include "bsp_time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 1 = 标定模式：串口控制台可逐关节微调脉宽，配合捕获读数核对
   0 = 正常运行：控制台整段不编译                                         */
#define CALIB_MODE      1

/* 控制台步进（µs） */
#define CALIB_STEP_FINE   10
#define CALIB_STEP_COARSE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t s_tick5;        /* 5ms 节拍计数（用于验证时基） */
static uint32_t s_last_sec;     /* 上次打印的秒数 */

#if CALIB_MODE
static pwm_joint_t s_calib_joint = PWM_J_BASE;   /* 当前在调哪一路 */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if CALIB_MODE
/* 串口控制台：单字符命令，逐关节微调脉宽，用来标定行程
 *   1~4  选关节（1=底座 2=肩 3=肘 4=爪）
 *   + -  当前关节 ±10µs
 *   ] [  当前关节 ±50µs（快速扫）
 *   c    当前关节回中 1500µs
 *   a    四路全部回中 1500µs
 *   p    打印当前关节与脉宽
 *   h    打印本帮助
 * 注意：必须在 BSP_Pwm_Init() 之后调用（PWM 已启动）
 */
static void Calib_Console(void)
{
    static uint8_t ch;
    uint16_t us;

    if (HAL_UART_Receive(&huart1, &ch, 1U, 0U) != HAL_OK) {
        return;
    }

    us = BSP_Pwm_GetPulseUs(s_calib_joint);

    switch (ch) {
    case '1':
    case '2':
    case '3':
    case '4':
        s_calib_joint = (pwm_joint_t)(ch - '1');
        printf("[J%u] selected, us=%u\r\n",
               (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case '+':
    case '=':
        BSP_Pwm_SetPulseUs(s_calib_joint, (uint16_t)(us + CALIB_STEP_FINE));
        printf("[J%u] %u -> %u\r\n", (unsigned)(s_calib_joint + 1U), us,
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case '-':
    case '_':
        BSP_Pwm_SetPulseUs(s_calib_joint, (uint16_t)(us - CALIB_STEP_FINE));
        printf("[J%u] %u -> %u\r\n", (unsigned)(s_calib_joint + 1U), us,
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case ']':
        BSP_Pwm_SetPulseUs(s_calib_joint, (uint16_t)(us + CALIB_STEP_COARSE));
        printf("[J%u] %u -> %u\r\n", (unsigned)(s_calib_joint + 1U), us,
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case '[':
        BSP_Pwm_SetPulseUs(s_calib_joint, (uint16_t)(us - CALIB_STEP_COARSE));
        printf("[J%u] %u -> %u\r\n", (unsigned)(s_calib_joint + 1U), us,
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case 'c':
        BSP_Pwm_SetPulseUs(s_calib_joint, 1500U);
        printf("[J%u] center 1500\r\n", (unsigned)(s_calib_joint + 1U));
        break;

    case 'a':
        for (uint8_t i = 0; i < PWM_J_NUM; i++) {
            BSP_Pwm_SetPulseUs((pwm_joint_t)i, 1500U);
        }
        printf("all center 1500\r\n");
        break;

    case 'p':
        printf("[J%u] us=%u\r\n", (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    /* 走完整映射链路，用来验证"角度→脉宽"与软限位 */
    case 'z':   /* q = 0    -> 应为 pulse_min */
        BSP_Pwm_SetAngle10(s_calib_joint, 0);
        printf("[J%u] q=0    -> us=%u\r\n", (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case 'm':   /* q = 900  -> 应为中值 */
        BSP_Pwm_SetAngle10(s_calib_joint, 900);
        printf("[J%u] q=900  -> us=%u\r\n", (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case 'x':   /* q = 1800 -> 应为 pulse_max */
        BSP_Pwm_SetAngle10(s_calib_joint, 1800);
        printf("[J%u] q=1800 -> us=%u\r\n", (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case 'o':   /* q = 2500 -> 超限，应被钳到 pulse_max */
        BSP_Pwm_SetAngle10(s_calib_joint, 2500);
        printf("[J%u] q=2500 -> us=%u (clamped)\r\n", (unsigned)(s_calib_joint + 1U),
               BSP_Pwm_GetPulseUs(s_calib_joint));
        break;

    case 'h':
        printf("1-4 joint | +/- 10us | ]/[ 50us | c center | a all\r\n");
        printf("p print  | z q=0 | m q=900 | x q=1800 | o q=2500(clamp)\r\n");
        break;

    default:
        break;   /* 回车/换行等忽略 */
    }
}
#endif /* CALIB_MODE */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
     BSP_Time_Init();
     BSP_Pwm_Init();

#if PWM_MEASURE_ENABLE
    BSP_PwmMeasure_Init();      /* 需要 PA0 与 PA6 短接 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (BSP_Time_Take5ms()) {
        s_tick5++;              /* 每 5ms 加一，1 秒应到 200 */
    }

    if ((BSP_Time_Ms() - s_last_sec) >= 1000U) {   /* 1Hz 自检打印 */
        s_last_sec = BSP_Time_Ms();
        printf("ms=%lu tick5=%lu\r\n",
               (unsigned long)s_last_sec, (unsigned long)s_tick5);
        s_tick5 = 0U;
    }

#if PWM_MEASURE_ENABLE
    BSP_PwmMeasure_Poll();      /* 1Hz 打印一次实测脉宽 */
#endif

#if CALIB_MODE
    Calib_Console();            /* 标定控制台：串口单字符命令 */
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
