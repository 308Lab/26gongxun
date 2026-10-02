#include "Motor.h"
#include "MotorConfig.h"

extern TIM_HandleTypeDef htim3;

/* ==================== 内部工具 ==================== */

/*
 * 单轮驱动：写方向脚 + 设 PWM
 *   in1_port/in1_pin : TB6612 IN1
 *   in2_port/in2_pin : TB6612 IN2
 *   channel          : TIM3 通道
 *   Speed            : 带符号速度，正=前进，负=后退
 *   reverse          : 方向反转标志（0=不反转，1=反转）
 */
static void Motor_Run(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      uint32_t channel, int32_t Speed, uint8_t reverse)
{
    uint8_t dir_forward;    /* 正向时 IN1 应输出的电平 */

    if (Speed == 0)
    {
        /* 停止：IN1=IN2=LOW（惰行） */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, channel, 0);
        return;
    }

    /* 正转还是反转 */
    if (Speed > 0)
    {
        dir_forward = 1;
    }
    else
    {
        dir_forward = 0;
        Speed = -Speed;     /* 取绝对值作 PWM 占空比 */
    }

    /*
     * TB6612 方向逻辑（以 A 轮为基准）：
     *   前进：IN1=HIGH, IN2=LOW
     *   后退：IN1=LOW,  IN2=HIGH
     * 若某轮接线相反，置 reverse=1，则电平整体翻转。
     */
    if (reverse)
    {
        dir_forward = !dir_forward;
    }

    if (dir_forward)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, channel, (uint32_t)Speed);
}

/* ==================== 初始化 ==================== */

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, PWM_A);
    HAL_TIM_PWM_Start(&htim3, PWM_B);
#if (MOTOR_WHEEL_COUNT)
    HAL_TIM_PWM_Start(&htim3, PWM_C);
    HAL_TIM_PWM_Start(&htim3, PWM_D);
#endif
}

/* ==================== 单轮控制 ==================== */

void Motor_A_Move(int32_t Speed)
{
    Motor_Run(AIN_1_PORT, AIN_1_PIN, AIN_2_PORT, AIN_2_PIN,
              PWM_A, Speed, MOTOR_A_REVERSE);
}

void Motor_B_Move(int32_t Speed)
{
    Motor_Run(BIN_1_PORT, BIN_1_PIN, BIN_2_PORT, BIN_2_PIN,
              PWM_B, Speed, MOTOR_B_REVERSE);
}

#if (MOTOR_WHEEL_COUNT)
void Motor_C_Move(int32_t Speed)
{
    Motor_Run(CIN_1_PORT, CIN_1_PIN, CIN_2_PORT, CIN_2_PIN,
              PWM_C, Speed, MOTOR_C_REVERSE);
}

void Motor_D_Move(int32_t Speed)
{
    Motor_Run(DIN_1_PORT, DIN_1_PIN, DIN_2_PORT, DIN_2_PIN,
              PWM_D, Speed, MOTOR_D_REVERSE);
}
#endif

/* ==================== 差速合成 ==================== */

void SetWheelSpeed(int32_t left_speed, int32_t right_speed)
{
    /* 物理映射：Motor_A 接左轮、Motor_B 接右轮 */
    Motor_A_Move(left_speed);
    Motor_B_Move(right_speed);
    #if (MOTOR_WHEEL_COUNT)
    Motor_C_Move(left_speed);
    Motor_D_Move(right_speed);
    #endif
}

void Stop(void)
{
    Motor_A_Move(0);
    Motor_B_Move(0);
#if (MOTOR_WHEEL_COUNT)
    Motor_C_Move(0);
    Motor_D_Move(0);
#endif
}

void lefTurn(void)
{
    int32_t Base_Speed = BASE_SPEED;
    Motor_A_Move(-Base_Speed);  /* 左轮反转 */
    Motor_B_Move(Base_Speed);   /* 右轮正转 */
}

void rightTurn(void)
{
    int32_t Base_Speed = BASE_SPEED;
    Motor_A_Move(Base_Speed);   /* 左轮正转 */
    Motor_B_Move(-Base_Speed);  /* 右轮反转 */
}
