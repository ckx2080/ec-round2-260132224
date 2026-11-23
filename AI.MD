# AI 使用记录

> 规范见《任务书》附件三。评审以 `git log --grep="^AI:"` 逐条对照本文件。
> 规则：**没记录 = 不计分；记录与实际不符 = 违规；解释不清 = 该部分不计分。**

## 一、使用的工具

| 工具 | 版本/模型 | 使用时段 |
|---|---|---|
| DeepSeek（对话式，含文件访问） | DeepSeek 系列模型；会话内可直接读写仓库文件 | 2026-11-19 ~ 2026-11-20 |
| STM32CubeMX 内置生成器 | 6.18.0（非 LLM，仅代码生成器） | 2026-11-20 |

## 二、使用记录

| 日期 | 文件 / 模块 | 用途 | 是否采用 | 验证方式 | commit |
|---|---|---|---|---|---|
| 11-19 | `README.md`、`AI.MD`、`COST.md`、`docs/设计说明.md`、`docs/安全检查表.md`、`.gitignore`、`.gitattributes` | 生成仓库骨架与文档模板 | 采用（按实物改写） | 对照任务书 5.1 / 5.3 / 附件三逐条核对栏目是否齐全 | `7b2bf51` |
| 11-20 | `Core/Src/main.c`（`USER CODE` 区） | 生成 TIM3 输入捕获回调、`_write` 重定向、四路 PWM 启动与安全初值 | 采用（改端口与阈值） | 串口打印 CCR 1000/3000/5000 对应 500/1500/2500µs，偏差 ≤2µs；拔掉 PA0–PA6 跳线后读数归零，接回恢复 | `e73307f` |
| 11-20 | `.vscode/tasks.json`、`.vscode/launch.json` | 任务改为绝对路径调用 cmake、任务内重定向 TMP/TEMP、调试器改 CMSIS-DAP | 采用 | `Ctrl+Shift+B` 构建通过；OpenOCD 输出 `Programming Finished` + `Verified OK` | `945589b` |
| 11-20 | `bsp/bsp_pwm_measure.c/.h`、`Core/Src/main.c`、`CMakeLists.txt` | 把捕获回调抽成独立 bsp 模块、main 精简为三处调用、注册源文件与 include | 采用（宏默认 1，交付前可置 0） | 宏=1 构建通过且 `arm-none-eabi-nm` 可见 `T BSP_PwmMeasure_Init/Poll`；宏=0 时符号消失、Flash 由 43276B 降到 37576B | `3e32dde` |
| 11-20 | CubeMX 配置（TIM2 50Hz / TIM3 捕获 / I2C 400kHz / USB CDC） | 仅提供配置步骤与参数建议，未生成代码 | 采用（由本人在 CubeMX 内操作） | 逐项核对 `tim.c`（PSC=35, ARR=39999）、`i2c.c`（400000）、`usbd_desc.c`（VID 0x483 / PID 0x5740） | `700446a`、`6668588` |
| 11-23 | `bsp/bsp_pwm.c`、`Core/Src/main.c`、`CMakeLists.txt`、`application/arm_motion.h` | 新增 `pulse_safe` 每关节安全位与反算逻辑、串口标定控制台（CALIB_MODE）、注册 `arm_motion.c` 并链接 libm | 采用（标定数值为本人实测，AI 只改结构与命令分发） | 编译通过（FLASH 46452B）；上电四轴停在 1500/1980/2000/1495µs，不再顶机械极限；控制台 `o` 键超限时脉宽被钳到 `pulse_max` | `6fdc3c0` |

> 验证方式要求可复现：写清“用什么测、测出什么数、判据是多少”，不要只写“测试通过”。

## 三、未采用的部分

| 方案 | 放弃原因 |
|---|---|
| STM32CubeCLT 官方工具链 | 官网脚本下载被网关拦截（HTTP 567），改用 xPack `arm-none-eabi-gcc 15.2.1` + 官方 CMake/Ninja + OpenOCD；编译链路等价，已记录版本号 |
| 用 `TMPDIR` 解决 gcc 临时文件失败 | Windows 版 gcc 不读 `TMPDIR`，实测无效；改为在 `env.cmd`/`env.ps1`/任务里重定向 `TMP`/`TEMP` |
| PWM Input 模式（硬件同时给周期与高电平） | HAL 无封装，需直接操作寄存器；改用单通道切换极性，代码更易讲解与调试 |
| `fputc` 重定向 printf | 那是 Keil/ARMCC 的路径，GCC + newlib 只认 `_write`，照抄会编译通过但无输出 |

## 四、声明

本人能够解释每一处与 AI 相关的代码，包括原理、边界条件与失效表现。

- 捕获回调：为什么用 volatile、为什么一个通道切换极性、为什么相减要处理 16 位回绕。
- `_write`：newlib 的 printf 最终收敛到该系统调用，nano.specs 下不实现会导致无输出。
- PWM 启动顺序：先写 CCR 再 `HAL_TIM_PWM_Start`，否则上电瞬间输出 0µs 会顶到限位。
- 任务配置：为什么把 TMP/TEMP 指到工具链目录（本机 gcc 无法在 %TEMP% 建临时文件）。
