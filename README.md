# ec-round2-260132224 · 四自由度舵机机械臂

> CVIR 机甲大师 2026 电控组二轮考核 · 提交仓库
> 考生：巢凯翔 · 学号 260132224
> 一句话：STM32F103C8T6 + 四路 50Hz PWM 舵机 + MPU6050（I²C 400kHz）+ USB CDC 上报。

---

## 一、快速开始（干净环境）

```bash
# 1. 配置（Debug 用于调试，Release 用于验收）
cmake --preset Debug
cmake --build --preset Debug

# 2. 烧录（本项目用 OpenOCD + ST-Link）
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "program build/Debug/<工程名>.elf verify reset exit"
```

> 验收会在干净目录用 `cmake --preset Release && cmake --build --preset Release` 重建（任务书 3.3）。

---

## 二、环境与版本

| 项 | 版本 | 备注 |
|---|---|---|
| MCU | STM32F103C8T6（LQFP48，64KB Flash / 20KB RAM） | 野火 F103 核心板（双 USB 款） |
| STM32CubeMX | 6.18.0 | `.ioc` 已提交，可重新生成对照 |
| 固件包 | STM32Cube FW_F1 V1.8.x | HAL 驱动 |
| arm-none-eabi-gcc | 15.2.1 (xPack) | |
| CMake | 4.5.0 | |
| Ninja | 1.13.2 | |
| OpenOCD | 0.12.0 | 烧录 / GDB Server |
| C 标准 | C17 | `CMAKE_C_STANDARD 17` |

系统时钟 72 MHz（HSE 8 MHz × 9），USB 48 MHz（72 / 1.5）。

---

## 三、接线表

| 功能 | 引脚 | 外设 | 说明 |
|---|---|---|---|
| 底座 q1 | PA0 | TIM2_CH1 | SG90，50Hz，500–2500µs |
| 肩 q2 | PA1 | TIM2_CH2 | 同上 |
| 肘 q3 | PA2 | TIM2_CH3 | 同上 |
| 夹爪 q4 | PA3 | TIM2_CH4 | 同上 |
| MPU6050 SCL | PB6 | I2C1 | 400kHz，4.7k 上拉 |
| MPU6050 SDA | PB7 | I2C1 | |
| 调试串口 TX | PA9 | USART1_TX | 115200 8N1 |
| 调试串口 RX | PA10 | USART1_RX | |
| USB D− | PA11 | USB_FS | CDC，VID/PID = 0x0483 / 0x5740 |
| USB D+ | PA12 | USB_FS | |
| 按键 | PB0 | GPIO 输入 | 模式切换，内部上拉 |
| LED | PC13 | GPIO 输出 | 1Hz 心跳 |

电源：舵机独立 5V/3A，与主控**共地**；禁止从板载 3V3/5V 取电驱动舵机。

---

## 四、软件结构

```
Core/            CubeMX 生成（勿改 USER CODE 区外内容）
Drivers/         HAL + CMSIS（CubeMX 生成）
cmake/           CubeMX 生成的工具链文件（不得手改）
application/     任务与行为：状态机、插补调用、回零
bsp/             板级驱动：pwm / i2c / usart / usb_cdc / time
components/      可复用：devices/mpu6050、controller、algorithm
protocol/        帧编解码 + CRC8（全工程唯一一份）
docs/            设计说明、安全检查表、状态机图
```

**协议实现在 `protocol/protocol.c`**，传输层在 `bsp/bsp_usb_cdc.c`；换 Custom HID 时业务代码不改。

---

## 五、协议实现说明

| 项 | 实现 |
|---|---|
| 帧结构 | `A5 | type | len | payload | crc8`，小端，角度 0.1°（int16） |
| 状态上报 0x10 | 19 字节载荷，20–50Hz，发送缓冲满时丢最旧，不阻塞控制拍 |
| 目标角度 0x20 | 11 字节载荷（含 duration / profile） |
| 模式 0x21 / 急停 0x22 / 心跳 0x23 / 版本 0x24 | 见任务书附件一 |
| CRC8 | 多项式 0x07，初值 0x00，不反射，不异或 |
| 超时 | 3s 无 0x20 或心跳 → 保持姿态 + bit5；I²C 失败 1s 内停 + bit4/bit2 |

---

## 六、演示视频

（链接待填 / 现场演示）

---

## 七、AI 使用记录

见仓库根目录 [AI.MD](AI.MD)。所有 AI 辅助提交均带 `AI:` 与 `Verified:` 行，`git log --grep="^AI:"` 可检索。

## 八、已知限制

- （例如）夹爪未做闭环，抓取力靠限位；
- SG90 塑料齿回差约 1–2°，重复定位靠单向逼近保证。
