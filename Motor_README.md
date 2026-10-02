# Motor 电机驱动模块 README

> **版本号：1.0.0**
> 适用于 STM32F103VET6 + TB6612 双路直流电机驱动（基于 HAL 库 / CubeMX）

---

## 1. 模块简介

Motor 模块是直流电机的**底层驱动层**，负责把「目标速度」转换为「PWM 占空比 + 方向脚电平」，驱动 TB6612 控制电机。

设计目标：

- **引脚集中管理**：所有引脚、轮数、方向配置集中在 `MotorConfig.h`，改配置不改逻辑。
- **轮数可扩展**：双轮 / 四轮通过一个宏切换，为后续扩展（麦克纳姆 / 全向轮）预留。
- **方向软件可翻转**：接线方向反了，改一个 `0/1` 宏即可，无需动硬件。
- **接口语义化**：对外只暴露「设速度」「停止」「转弯」等语义清晰接口，不暴露 PWM 细节。

---

## 2. 文件清单

| 文件 | 位置 | 职责 |
|---|---|---|
| `Motor.h` | `include/` | 模块对外接口声明 |
| `MotorConfig.h` | `include/` | 配置集中地（轮数 / 方向 / 引脚映射 / 基础速度） |
| `Motor.c` | `src/` | 驱动实现 |

依赖：`main.h`（CubeMX 生成的引脚宏）、`htim3`（TIM3，PWM 输出）。

---

## 3. 引脚映射

| 轮 | PWM（TIM3 通道） | IN1（方向） | IN2（方向） |
|---|---|---|---|
| A 轮 | PA6 / CH1 | PE13 | PE15 |
| B 轮 | PA7 / CH2 | PC5 | PE9 |
| C 轮（预留） | PB0 / CH3 | 待配 | 待配 |
| D 轮（预留） | PB1 / CH4 | 待配 | 待配 |

PWM 参数（TIM3）：Prescaler = 71，Period(ARR) = 999 → 频率 1kHz，占空比范围 0 ~ 999。

> 引脚宏通过 `MotorConfig.h` **引用** `main.h` 的 CubeMX 生成宏（`AIN_1_GPIO_Port` 等），
> 因此 CubeMX 改引脚后，`main.h` 自动更新，`Motor.c` 无需改动。

---

## 4. 配置说明（MotorConfig.h）

```c
#define MOTOR_WHEEL_COUNT   0    // 0=双轮，1=四轮

#define MOTOR_A_REVERSE    0     // 0=不反转，1=反转
#define MOTOR_B_REVERSE    0
#define MOTOR_C_REVERSE    0
#define MOTOR_D_REVERSE    0

#define BASE_SPEED         220   // 基础速度（PWM 占空比 0~999）
```

| 宏 | 含义 | 取值 |
|---|---|---|
| `MOTOR_WHEEL_COUNT` | 轮数 | `0` 双轮，`1` 四轮 |
| `MOTOR_x_REVERSE` | 单轮方向是否软件翻转 | `0` 不反转，`1` 反转 |
| `BASE_SPEED` | 基础速度 | PWM 占空比（0~999） |

**方向反转说明**：若某轮接线导致「正转命令」实际反转，将该轮的 `MOTOR_x_REVERSE` 置 `1`，软件层自动翻转方向电平，无需改硬件接线。

---

## 5. 对外接口

### 初始化

```c
void Motor_Init(void);
```
启动所有已启用轮子的 PWM 输出。**使用前必须调用一次**（通常在系统初始化阶段）。

### 单轮控制

```c
void Motor_A_Move(int32_t Speed);
void Motor_B_Move(int32_t Speed);
// 四轮时额外提供：
void Motor_C_Move(int32_t Speed);
void Motor_D_Move(int32_t Speed);
```

- `Speed`：带符号速度，**正 = 前进，负 = 后退，0 = 停止**。
- 范围建议 `-999 ~ 999`（对应 PWM 占空比，内部自动取绝对值）。

### 双轮设速（差速合成）

```c
void SetWheelSpeed(int32_t left_speed, int32_t right_speed);
```

- 直接设置左右轮目标速度，**差速逻辑由调用方（控制层）决定**。
- 四轮模式下，C/D 轮会复制左右轮速度（麦克纳姆全向需后续单独扩展）。

### 整车停止

```c
void Stop(void);
```
所有轮停止（惰行，IN1=IN2=LOW）。

### 原地转弯

```c
void lefTurn(void);    // 左转：A 轮后退，B 轮前进
void rightTurn(void);  // 右转：A 轮前进，B 轮后退
```

---

## 6. 使用示例

```c
#include "Motor.h"

// 初始化（在 main 或系统初始化里调用一次）
Motor_Init();

// 前进（双轮同速）
SetWheelSpeed(220, 220);

// 原地左转
lefTurn();

// 差速右转（左轮快，右轮慢）
SetWheelSpeed(300, 100);

// 停止
Stop();
```

---

## 7. 分层职责（重要）

本模块是**驱动层**，只负责「执行速度」，不负责「决定速度」。

| 层 | 职责 | 例子 |
|---|---|---|
| 决策层（main / 任务） | 决定「做什么」 | 前进、转弯、跑一圈 |
| 控制层（Control） | 算「目标速度」 | 速度 + PID + 循迹 + 陀螺仪合成 |
| **驱动层（本模块）** | 执行「目标速度」 | `SetWheelSpeed` / `Motor_x_Move` |

速度合成（PID、循迹修正、陀螺仪转向）**不在本模块内**，应由上层控制模块完成后，调用 `SetWheelSpeed()` 传入。

---

## 8. 已知限制 / 待办

- [ ] 四轮（`MOTOR_WHEEL_COUNT = 1`）的 C/D 轮方向脚（IN1/IN2）尚未在 CubeMX 配置，`CIN_1_PORT` 等宏为空，需先补配引脚。
- [ ] 四轮模式下 `SetWheelSpeed` 暂为「C/D 复制左右轮」，真正的麦克纳姆全向解算未实现。
- [ ] 停止为「惰行」（IN1=IN2=LOW），若需「刹车」（短路制动）需将停止逻辑改为 IN1=IN2=HIGH。

---

## 9. 版本历史

| 版本 | 日期 | 变更 |
|---|---|---|
| 1.0.0 | 2026-08-26 | 初版：合并 PWM 进 Motor；引入 MotorConfig.h 集中配置；支持双轮/四轮、方向软件翻转；提供 SetWheelSpeed 差速接口 |
