#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "MotorConfig.h"

#define TIM_HANDLE_MA &htim1
#define TIM_HANDLE_MB &htim8

typedef struct
{
    int32_t enc_l;
    int32_t enc_r;
}Encoder_Sample_t;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void Encoder_Init(void);

int16_t Encoder_Get_MA_Counter(void);
int16_t Encoder_Get_MB_Counter(void);

/* 只读不清的累计值（差值法里程计用）。返回 int32_t 累计脉冲，防高分辨率溢出 */
int32_t Encoder_Get_MA_Raw(void);
int32_t Encoder_Get_MB_Raw(void);

#endif
