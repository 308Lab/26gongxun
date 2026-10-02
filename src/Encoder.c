#include "Encoder.h"

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(TIM_HANDLE_MA, TIM_CHANNEL_1);
    //HAL_TIM_Encoder_Start(TIM_HANDLE_MA, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(TIM_HANDLE_MB, TIM_CHANNEL_1);
    //HAL_TIM_Encoder_Start(TIM_HANDLE_MB, TIM_CHANNEL_2);
    
}

int16_t Encoder_Get_MA_Counter(void)
{
    int16_t temp;
    temp = __HAL_TIM_GET_COUNTER(TIM_HANDLE_MA);
    __HAL_TIM_SET_COUNTER(TIM_HANDLE_MA, 0);
    return temp;
}

int16_t Encoder_Get_MB_Counter(void)
{
    int16_t temp;
    temp = __HAL_TIM_GET_COUNTER(TIM_HANDLE_MB);
    __HAL_TIM_SET_COUNTER(TIM_HANDLE_MB, 0);
    return (-temp);
}

/* 只读不清：返回编码器计数器当前值（int32_t），不清零，供差值法里程计用。
 * 注意：TIM 编码器模式计数方向可能需取负，和读-清版保持一致（MA/MB 都取负）。 */
int32_t Encoder_Get_MA_Raw(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(TIM_HANDLE_MA);
}

int32_t Encoder_Get_MB_Raw(void)
{
    return -(int32_t)__HAL_TIM_GET_COUNTER(TIM_HANDLE_MB);
}
