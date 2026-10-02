/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Laser_Pin GPIO_PIN_1
#define Laser_GPIO_Port GPIOC
#define INA219_SDA_Pin GPIO_PIN_3
#define INA219_SDA_GPIO_Port GPIOC
#define INA219_SCL_Pin GPIO_PIN_0
#define INA219_SCL_GPIO_Port GPIOA
#define Gyro_Tx_Pin GPIO_PIN_2
#define Gyro_Tx_GPIO_Port GPIOA
#define Gyro_Rx_Pin GPIO_PIN_3
#define Gyro_Rx_GPIO_Port GPIOA
#define PWM_Servo_Pin GPIO_PIN_6
#define PWM_Servo_GPIO_Port GPIOA
#define PWM_MA_Pin GPIO_PIN_0
#define PWM_MA_GPIO_Port GPIOB
#define PWM_MB_Pin GPIO_PIN_1
#define PWM_MB_GPIO_Port GPIOB
#define Key_Row_3_Pin GPIO_PIN_7
#define Key_Row_3_GPIO_Port GPIOE
#define Key_Row_2_Pin GPIO_PIN_8
#define Key_Row_2_GPIO_Port GPIOE
#define Enc_MA_1_Pin GPIO_PIN_9
#define Enc_MA_1_GPIO_Port GPIOE
#define Key_Row_1_Pin GPIO_PIN_10
#define Key_Row_1_GPIO_Port GPIOE
#define Enc_MA_2_Pin GPIO_PIN_11
#define Enc_MA_2_GPIO_Port GPIOE
#define Key_Col_1_Pin GPIO_PIN_13
#define Key_Col_1_GPIO_Port GPIOE
#define Key_Col_2_Pin GPIO_PIN_14
#define Key_Col_2_GPIO_Port GPIOE
#define Key_Col_3_Pin GPIO_PIN_15
#define Key_Col_3_GPIO_Port GPIOE
#define Host_Tx_Pin GPIO_PIN_10
#define Host_Tx_GPIO_Port GPIOB
#define Host_Rx_Pin GPIO_PIN_11
#define Host_Rx_GPIO_Port GPIOB
#define BIN_2_Pin GPIO_PIN_12
#define BIN_2_GPIO_Port GPIOB
#define BIN_1_Pin GPIO_PIN_13
#define BIN_1_GPIO_Port GPIOB
#define AIN_1_Pin GPIO_PIN_8
#define AIN_1_GPIO_Port GPIOD
#define AIN_2_Pin GPIO_PIN_9
#define AIN_2_GPIO_Port GPIOD
#define Enc_MB_1_Pin GPIO_PIN_6
#define Enc_MB_1_GPIO_Port GPIOC
#define Enc_MB_2_Pin GPIO_PIN_7
#define Enc_MB_2_GPIO_Port GPIOC
#define ServoTx_Switch_Pin GPIO_PIN_9
#define ServoTx_Switch_GPIO_Port GPIOC
#define Step_Tx_Pin GPIO_PIN_9
#define Step_Tx_GPIO_Port GPIOA
#define Step_Rx_Pin GPIO_PIN_10
#define Step_Rx_GPIO_Port GPIOA
#define ServoRx_Switch_Pin GPIO_PIN_11
#define ServoRx_Switch_GPIO_Port GPIOA
#define Servo_Tx_Pin GPIO_PIN_10
#define Servo_Tx_GPIO_Port GPIOC
#define Servo_Rx_Pin GPIO_PIN_11
#define Servo_Rx_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
