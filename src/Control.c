#include "Control.h"

void Control_Init (void)
{
    INA219_Init();        /* INA219 电流监测（PC2=SCL, PC3=SDA） */
    
    Car_Control_Init();
    Host_Init();
    /* 显示不在这里初始化：OLEDTask 会调 Ui_Init()（u8g2+MUI，同一组 PB8/PB9 软件 I2C）。
     * 原先这里还有一行 OLED_Init()，两套驱动会抢同一组引脚，已移除。 */
    Encoder_Init();
    Gyro_Init(&huart2);   /* JY61P 接 USART2（PA3=RX） */
    Servo_UartInit();
}

/* （2026-10-14 删除 EASL_SetServo_Pos：全仓无调用者的死代码，且它的入参语义
 *   ——「相对 1500 中值的偏移」—— 与新的零位偏移容易混淆。舵机下发统一走
 *   Servo_SetAngleTime / Servo_SetAngle，见 Servo.h。） */