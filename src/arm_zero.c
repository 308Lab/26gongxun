/**
 * @file    arm_zero.c
 * @brief   机械臂零位标定实现（语义见 arm_zero.h）
 */

#include "arm_zero.h"

#include <string.h>

#include "arm_config.h"
#include "param_store.h"
#include "Servo.h"

/* 出厂零位基准，顺序 = 舵机 ID 1/2/3 = P1/P2/P3。
 * 做成数组是为了能按 ID 索引；定义在 arm_config.h，只引用符号、不硬编码数值。 */
static const uint16_t s_zero_ref[3] = {
    (uint16_t)ARM_ZERO_REF_P1,
    (uint16_t)ARM_ZERO_REF_P2,
    (uint16_t)ARM_ZERO_REF_P3,
};

static uint8_t s_calibrated = 0u;

void ArmZero_Init(void)
{
    param_blob_t rec;
    uint8_t      i;

    if (ParamStore_Load(&rec) == 0)
    {
        /* 无有效记录（从没标过 / 数据损坏 / 校验不过）→ 退回出厂，偏移全 0 */
        Servo_ClearZeroOffset();
        s_calibrated = 0u;
        return;
    }

    for (i = 0u; i < 3u; i++)
    {
        Servo_SetZeroOffset((uint8_t)(i + 1u),
                            (int16_t)((int32_t)rec.zero_pwm[i] - (int32_t)s_zero_ref[i]));
    }
    s_calibrated = rec.calibrated;
}

int ArmZero_Record(const uint16_t raw_pwm[3])
{
    param_blob_t rec;
    uint8_t      i;

    if (raw_pwm == NULL)
    {
        return -1;
    }

    /* 三路都必须有有效回读：0 = 该 ID 还没读到过（回读由主循环 50ms 轮询填充）。
     * 只要有 0 就说明数据不可信，绝不能把 0 当成零位记进去。 */
    for (i = 0u; i < 3u; i++)
    {
        if (raw_pwm[i] == 0u)
        {
            return -1;
        }
    }

    /* 合理性检查：偏移超出合理范围，几乎都是"没摆到收起态"或回读无效 */
    for (i = 0u; i < 3u; i++)
    {
        const int32_t d = (int32_t)raw_pwm[i] - (int32_t)s_zero_ref[i];

        if ((d > (int32_t)ARM_ZERO_OFFSET_LIMIT_PWM) ||
            (d < -(int32_t)ARM_ZERO_OFFSET_LIMIT_PWM))
        {
            return -2;
        }
    }

    (void)memset(&rec, 0, sizeof(rec));
    rec.calibrated = 1u;
    for (i = 0u; i < 3u; i++)
    {
        rec.zero_pwm[i] = raw_pwm[i];
    }

    if (ParamStore_Save(&rec) != 0)
    {
        return -3;
    }

    /* 落盘成功后才发布到 RAM —— 避免 Flash 写失败却已开始按新偏移动作 */
    for (i = 0u; i < 3u; i++)
    {
        Servo_SetZeroOffset((uint8_t)(i + 1u),
                            (int16_t)((int32_t)raw_pwm[i] - (int32_t)s_zero_ref[i]));
    }
    s_calibrated = 1u;

    return 0;
}

uint8_t ArmZero_IsCalibrated(void)
{
    return s_calibrated;
}
