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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Control.h"
#include "Arm.h"
#include "Servo.h"
#include "arm_config.h"
#include "arm_kinematics.h"
#include "arm_zero.h"
#include "Vision.h"
#include "task_seq.h"
#include "ui_form.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 128 * 12,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for EncCatchTask */
osThreadId_t EncCatchTaskHandle;
const osThreadAttr_t EncCatchTask_attributes = {
  .name = "EncCatchTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RunTask */
osThreadId_t RunTaskHandle;
const osThreadAttr_t RunTask_attributes = {
  .name = "RunTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 128 * 12,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TickTask */
osThreadId_t TickTaskHandle;
const osThreadAttr_t TickTask_attributes = {
  .name = "TickTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AAATask */
osThreadId_t AAATaskHandle;
const osThreadAttr_t AAATask_attributes = {
  .name = "AAATask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BBBTask */
osThreadId_t BBBTaskHandle;
const osThreadAttr_t BBBTask_attributes = {
  .name = "BBBTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CCCTask */
osThreadId_t CCCTaskHandle;
const osThreadAttr_t CCCTask_attributes = {
  .name = "CCCTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DDDTask */
osThreadId_t DDDTaskHandle;
const osThreadAttr_t DDDTask_attributes = {
  .name = "DDDTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncQueue */
osMessageQueueId_t EncQueueHandle;
const osMessageQueueAttr_t EncQueue_attributes = {
  .name = "EncQueue"
};
/* USER CODE BEGIN PV */

/* θ1 步进软累加器：步进无绝对零点，靠软件累加维护当前方位（正 = 左转/左展开）。
 * 由 main.c 的 StepGoTo 与 formal_mode.c 的 FormalStepGoTo 读写。
 * （2026-10-13：此处原有 20 余个「按键测试桩」显示全局，全部无读者，已清掉。） */
volatile int32_t  g_step_accum_pulses = 0;      /* 带符号累计脉冲：左转/左展开为正，右转为负 */

/* ============================================================
 * 总线舵机·位置采样环（主循环每 50ms 读一个，P1→P2→P3 轮替，一轮 ≈150ms）
 *   pwm*   = 读回的【实测】位置（不是命令值）
 *   valid* = 该路是否读到过（0 时 pwm* 无意义）
 * ============================================================ */
volatile uint16_t g_servo_pwm1    = 0;      /* P1 当前位置（ID=1） */
volatile uint16_t g_servo_pwm2    = 0;      /* P2 当前位置（ID=2） */
volatile uint8_t  g_servo_valid1  = 0;      /* P1 位置有效标志 */
volatile uint8_t  g_servo_valid2  = 0;      /* P2 位置有效标志 */
volatile uint16_t g_servo_pwm3    = 0;      /* P3 当前位置（ID=3） */
volatile uint8_t  g_servo_valid3  = 0;      /* P3 位置有效标志 */
volatile uint8_t g_servo_phase = 0;         /* 当前该读哪个舵机：0=P1 1=P2 2=P3 */
volatile uint32_t g_servo_lastsend_tick = 0; /* 上次发送读命令时刻 */


/* ============================================================
 * 总线舵机·独立外环位置环（2026-10-02）
 *   思路：读回实测位置 → err=target-actual → 比例增量修正 cmd → 下发
 *   独立于键 1/2/3 姿态命令，P1/P2 各自闭环、各自启停、各自限幅。
 * ============================================================ */
volatile int16_t  g_servo_target1 = 0;      /* P1 位置环目标 */
volatile int16_t  g_servo_target2 = 0;      /* P2 位置环目标 */
volatile int16_t  g_servo_cmd1 = 0;         /* P1 当前下发命令值（外环修正量） */
volatile int16_t  g_servo_cmd2 = 0;         /* P2 当前下发命令值 */
volatile int16_t  g_pid_err2 = 0;           /* P2 当前误差（显示用） */
volatile int16_t  g_pid_err1 = 0;           /* P1 当前误差（显示用） */
volatile uint32_t g_servo_move_done_tick = 0; /* 键位平滑运动结束时刻（环冷却依据） */
volatile uint16_t g_pid_act1 = 0;           /* P1 位置环累计修正次数（证明环在跑） */
volatile uint16_t g_pid_act2 = 0;           /* P2 位置环累计修正次数 */

/* 姿态类型标志（决定位置环是否纠偏）：
 *   0 = 收起/初始（位置环不动作）
 *   2 = B 展开（位置环 PID 闭环）
 *   其它 = A 展开（开环固定命令，位置环跳过） */
volatile uint8_t  g_servo_pose = 0;

/* 位置环比例增益（x10，实际 Kp = 值/10），现场可调 */
#define SERVO_PID_KP_X10   5    /* Kp=0.5 */
#define SERVO_PID_DEADBAND 5    /* 死区：|err|<=5 视作到位 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
void StartMainTask(void *argument);
void StartEncCatchTask(void *argument);
void StartRunTask(void *argument);
void StartOLEDTask(void *argument);
void StartTickTask(void *argument);
void StartAAATask(void *argument);
void StartBBBTask(void *argument);
void StartCCCTask(void *argument);
void StartDDDTask(void *argument);
void StartServoTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Host(USART3) 逐字节接收缓存，Host.c / stm32f1xx_it.c 均 extern 引用 */
uint8_t rx_byte_cache;

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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* 启动 PWM 舵机（TIM4_CH4，50Hz，500~2500us ↔ 0~270°）。
   * ★ 上电【不再】写归位角度（2026-10-14）：写 135° 会让夹爪舵机一直抱死在中性位，
   *   堵转电流大、发热烧机。现在 CCR 保持 MX_TIM4_Init 里的 Pulse=0 → 无脉冲输出，
   *   舵机进自由态、零电流。夹爪角度只在任务里由 ACT_GRIP 写（task_seq.c），
   *   不跑 B/D 任务就完全不动。
   * ⚠️ 待办：任务里 ACT_GRIP 夹紧后同样会一直顶死，需按熄火策略处理（见讨论）。 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of EncQueue */
  EncQueueHandle = osMessageQueueNew (4, sizeof(Encoder_Sample_t), &EncQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of EncCatchTask */
  EncCatchTaskHandle = osThreadNew(StartEncCatchTask, NULL, &EncCatchTask_attributes);

  /* creation of RunTask */
  RunTaskHandle = osThreadNew(StartRunTask, NULL, &RunTask_attributes);

  /* creation of OLEDTask */
  OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

  /* creation of TickTask */
  TickTaskHandle = osThreadNew(StartTickTask, NULL, &TickTask_attributes);

  /* creation of AAATask */
  AAATaskHandle = osThreadNew(StartAAATask, NULL, &AAATask_attributes);

  /* creation of BBBTask */
  BBBTaskHandle = osThreadNew(StartBBBTask, NULL, &BBBTask_attributes);

  /* creation of CCCTask */
  CCCTaskHandle = osThreadNew(StartCCCTask, NULL, &CCCTask_attributes);

  /* creation of DDDTask */
  DDDTaskHandle = osThreadNew(StartDDDTask, NULL, &DDDTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 20;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Laser_Pin|INA219_SDA_Pin|ServoTx_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INA219_SCL_Pin|ServoRx_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Key_Row_2_Pin|BIN_2_Pin|BIN_1_Pin|OLED_SCL_Pin
                          |OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Key_Row_3_Pin|Key_Row_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AIN_1_Pin|AIN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Laser_Pin ServoTx_Switch_Pin */
  GPIO_InitStruct.Pin = Laser_Pin|ServoTx_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : INA219_SDA_Pin */
  GPIO_InitStruct.Pin = INA219_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INA219_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INA219_SCL_Pin */
  GPIO_InitStruct.Pin = INA219_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INA219_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_Row_2_Pin BIN_2_Pin BIN_1_Pin OLED_SCL_Pin
                           OLED_SDA_Pin */
  GPIO_InitStruct.Pin = Key_Row_2_Pin|BIN_2_Pin|BIN_1_Pin|OLED_SCL_Pin
                          |OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_Row_3_Pin Key_Row_1_Pin */
  GPIO_InitStruct.Pin = Key_Row_3_Pin|Key_Row_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_Col_1_Pin Key_Col_2_Pin Key_Col_3_Pin */
  GPIO_InitStruct.Pin = Key_Col_1_Pin|Key_Col_2_Pin|Key_Col_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN_1_Pin AIN_2_Pin */
  GPIO_InitStruct.Pin = AIN_1_Pin|AIN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ServoRx_Switch_Pin */
  GPIO_InitStruct.Pin = ServoRx_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ServoRx_Switch_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* （2026-10-13：此处原有 Laser_Set()。激光改由 task_seq.c 的 ACT_LASER_ON/OFF 控制，
 *   直接写 Laser_Pin —— 电平约定待上机确认，反了改那两处的 GPIO_PIN_SET/RESET。） */

/* （2026-10-13：此处原有 StepGoTo / StepGoToSlow 两个静态函数，是旧标定序列的
 *   θ1 定位工具。标定序列已删除，同样的逻辑现在在 task_seq.c 的 ACT_STEP1 里，
 *   且带慢速变体 ACT_STEP1_SLOW。main.c 不再需要这两个函数。） */

/* ============================================================
 * 末端位姿上报（0x14，协议 §3.1d）：MCU 每 100ms 主动上报 TCP 位置。
 *
 * FK 的输入用【读回的实测 PWM】+ θ1 软累加器，所以报的是**实际姿态**而不是命令值
 * —— 这正是协议要它的理由（供上位机实时监控/绘制）。
 *
 * ⚠️ 精度声明（重要）：本帧只用于【观察趋势】，不是标定过的真值。arm_fk 目前有两个
 *    已知缺口：① ARM_L23_OFFSET_MM(20mm) 与关节零位角没有参与解算（arm_config.h 有
 *    值，arm_kinematics.c 未用）；② 机械零位基准待实测。所以数值会有几十 mm 量级的
 *    系统偏差。详见 26-GX-upper 的 docs/2026-10-13_接口对齐记录.md。
 * ============================================================ */
#define POSE_REPORT_PERIOD_MS  100   /* 10Hz，与协议 §3.1d「固定周期上报」一致 */
static uint32_t s_pose_report_tick = 0;

/* 暂不启用（原因见 MainTask 里的说明）。加 unused 属性把实现保留在这里，
 * 等 FK 补齐后只取消 MainTask 里那一行的注释即可 —— 同时保持零警告基线。 */
static void __attribute__((unused)) ReportPoseTick(void)
{
  if ((int32_t)(HAL_GetTick() - s_pose_report_tick) < POSE_REPORT_PERIOD_MS)
  {
    return;
  }
  s_pose_report_tick = HAL_GetTick();

  /* 三个舵机的实测位置都读回来过才报：未读回时 g_servo_pwm* 是 0，
   * arm_pwm_to_angle(0) 会换算出很大的负角，报出去就是纯垃圾。 */
  if (!g_servo_valid1 || !g_servo_valid2 || !g_servo_valid3)
  {
    return;
  }

  arm_joint_t j;
  j.th1 = ARM_STEP_PULSE2DEG((float)g_step_accum_pulses);
  j.th2 = arm_pwm_to_angle(ARM_TH2_ID, g_servo_pwm1);
  j.th3 = arm_pwm_to_angle(ARM_TH3_ID, g_servo_pwm2);
  j.th4 = arm_pwm_to_angle(ARM_TH4_ID, g_servo_pwm3);

  arm_pos_t p;
  arm_fk(&j, &p);

  /* z 补上 H0（回转面 → θ2 轴心高度）：协议 §3.1d 要的是基座系坐标。
   * arm_fk 的输出 z 是相对底座回转面的，不含这一段。 */
  Vision_ReportPose((int16_t)p.x, (int16_t)p.y, (int16_t)(p.z + ARM_H0_MM));
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  Control_Init();
  ArmZero_Init();   /* 从 Flash 载入零位偏移 —— 必须在任何任务下发舵机角度之前 */
  Arm_Init();       /* 步进电机 Emm_V5：绑定 USART1 + 启动接收 */

  /* ========================================================================
   * 【步进电机·按键触发读参数 + 解析｜临时桩】
   *   键 1 → 读版本（0x1F）
   *   键 2 → 读实时位置（0x36）
   *   键 3 → 读位置误差（0x37）
   *   键 7 → 读状态标志（0x3A）
   *   键 9 → 读实时转速（0x35）
   * 原「路径测试按键」已注释（#if 0），需要时恢复。
   * ====================================================================== */
  /* （2026-10-13：此处原有一串「步进测试桩」全局的初始化，那些变量全部无读者，已随声明一并清掉。） */

  TickType_t last_wake = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    /* ===== 决策层：路径状态机 tick（原 AAATask） ===== */
    PathRunner_Tick();

    /* ===== 任务序列引擎：到点后跑该点的动作表（取代旧 formal_mode.c） ===== */
    Task_Tick();

    /* ===== Host 层周期驱动：主动播发心跳（协议 §3.1，MCU 为心跳主动方） ===== */
    Host_Tick();

    /* ===== Host 2.0 上下位机通信解析：消费已完整收到的帧 ===== */
    CommunicateToHost();
    if (parsed_flag)
    {
      /* TODO: 根据解析出的指令执行对应动作（业务层接入） */
      parsed_flag = 0;
    }

    /* ===== 视觉通信业务层周期驱动：超时重发 ===== */
    Vision_Tick();

    /* ===== 末端位姿上报（0x14，协议 §3.1d）：10Hz 主动上报 TCP 位置 =====
     * ⚠️ 暂时【不启用】。它依赖 arm_fk，而 FK 有两个已知缺口 —— ARM_L23_OFFSET_MM(20mm)
     *    与关节零位角都没参与解算（arm_config.h 有值、arm_kinematics.c 未用），报出去是
     *    "看着像姿态但不准"的数，比不发更危险（现场会照着它判断机械臂状态）。
     *    等 FK 补齐后再打开。依据见 26-GX-upper/docs/2026-10-13_接口对齐记录.md §4 第 2 条。 */
    /* ReportPoseTick(); */


    /* ==================================================================
     * 【三舵机位置采样】固定周期轮询（防半双工总线串扰乱码）
     *   每 50ms 读一个舵机（P1→P2→P3 轮替），一轮 ≈ 150ms。
     *   只做「定时器到期才发下一条」，不做「收包立刻发下一条」的紧赶，
     *   给前一条回包留足消化时间，彻底消除读/写撞车导致的乱码。
     * ================================================================== */
    if (g_servo_lastsend_tick == 0)
    {
      /* 首次：读 P1（ID=1） */
      g_servo_phase = 0;
      Servo_ReadAngle(1);
      g_servo_lastsend_tick = HAL_GetTick();
    }
    else if ((int32_t)(HAL_GetTick() - g_servo_lastsend_tick) >= 50)
    {
      /* 固定 50ms 到期：推进到下一位，读下一个舵机 */
      g_servo_phase = (g_servo_phase + 1) % 3;
      Servo_ReadAngle(g_servo_phase + 1);
      g_servo_lastsend_tick = HAL_GetTick();
    }

    /* 回读处理：位置 → 存变量 + 就地纠偏。
     * 用 while 一次清空队列（可能积压多条回包）。 */
    while (Servo_HasReply())
    {
      servo_reply_t r = Servo_GetReply();
      if (r.kind == SERVO_REPLY_POSITION)
      {
        if (r.id == 1)
        {
          g_servo_pwm1 = r.pwm;
          g_servo_valid1 = 1;
        }
        else if (r.id == 2)
        {
          g_servo_pwm2 = r.pwm;
          g_servo_valid2 = 1;
        }
        else if (r.id == 3)
        {
          g_servo_pwm3 = r.pwm;
          g_servo_valid3 = 1;
        }

        /* ===== 外环位置环（已停用 2026-10-07：只显示 P123 不闭环） =====
         * 姿态调试阶段仅开环下发固定 PWM，位置环不纠偏。
         * 需要闭环时取消下方 #if 0 包裏并同步更新 g_servo_target1/2。 */
      }
    }


    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEncCatchTask */
/**
* @brief Function implementing the EncCatchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncCatchTask */
void StartEncCatchTask(void *argument)
{
  /* USER CODE BEGIN StartEncCatchTask */
  /* Infinite loop */
  TickType_t last_wake = xTaskGetTickCount();
  Encoder_Init();
  Odom_Reset();
  
  for(;;)
  {
    Encoder_Sample_t motor;
    motor.enc_l = -(Encoder_Get_MA_Counter());
    motor.enc_r = Encoder_Get_MB_Counter();   /* 右轮编码器方向反，去掉外层负号（Encoder_Get_MB 内部已取负） */
    g_enc_l = motor.enc_l;
    g_enc_r = motor.enc_r;
    Encoder_Total_Counter(g_enc_l, g_enc_r);
    osMessageQueuePut (EncQueueHandle, &motor, 0, 0);

    /* 里程计：差值法累计距离（只读不清，独立于上面的读-清速度采样） */
    Odom_Sample();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
  }
  /* USER CODE END StartEncCatchTask */
}

/* USER CODE BEGIN Header_StartRunTask */
/**
* @brief Function implementing the RunTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRunTask */
void StartRunTask(void *argument)
{
  /* USER CODE BEGIN StartRunTask */
  TickType_t last_wake = xTaskGetTickCount();

  /* 执行层：记录已处理过的命令 seq，防同一条命令重复执行 */
  uint16_t last_seq = 0;

  /* Infinite loop */
  for(;;)
  {
    /* 周期读 INA219 四值存全局变量，供 OLED 显示 */
    //INA219_Update();

    /* ===== 执行层：消费 MainTask 下发的命令 g_cmd ===== */
    if (g_cmd.seq != last_seq)
    {
      last_seq = g_cmd.seq;
      switch (g_cmd.cmd)
      {
        case CMD_GO:
          Go_Start(g_cmd.param, g_cmd.heading);
          break;
        case CMD_ARC:
          Arc_Start(g_cmd.dir, g_cmd.param);
          break;
        case CMD_TURN:
          Turn_Start((float)g_cmd.dir * g_cmd.param);
          break;
        case CMD_SCAN:
          Scan_Start(g_cmd.param, g_cmd.heading);
          break;
        case CMD_BCK:
          Back_Start(g_cmd.param);
          break;
        case CMD_NONE:
        default:
          break;
      }
    }

    /* 每周期扫描转弯/弧线/直行/扫描/倒车状态：进行中才动作，到位自动 Stop */
    Turn_Update();
    Arc_Update();
    Go_Update();
    Scan_Update();
    Back_Update();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
  }
  /* USER CODE END StartRunTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* 显示层已换成 u8g2 + MUI 表单（src/ui_form.c / src/u8g2_port.c）——
   * 取代原来的「4 行文本 + 9 键各绑一功能 + 22 个标定序列」。
   * 键盘也归本任务消费（原先 TickTask 采样、MainTask 消费，两个任务读同一份
   * Key_Flag 本来就存在竞态）。 */
  Ui_Init();

  for(;;)
  {
    Ui_Tick();          /* 读键盘 → 喂 MUI → 需要时重画 */
    osDelay(20);        /* 50ms → 20ms：按键响应更跟手，I2C 只在重画时才写 */
  }

  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartTickTask */
/**
* @brief Function implementing the TickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTickTask */
void StartTickTask(void *argument)
{
  /* USER CODE BEGIN StartTickTask */
  /* Infinite loop */
  for(;;)
  {
    Key_Tick();
    osDelay(5);   /* 固定周期 5ms；Key_Tick 内部 Count>=2 采样，约 10ms 消抖一次 */
  }
  /* USER CODE END StartTickTask */
}

/* USER CODE BEGIN Header_StartAAATask */
/**
* @brief Function implementing the AAATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAAATask */
void StartAAATask(void *argument)
{
  /* USER CODE BEGIN StartAAATask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAAATask */
}

/* USER CODE BEGIN Header_StartBBBTask */
/**
* @brief Function implementing the BBBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBBBTask */
void StartBBBTask(void *argument)
{
  /* USER CODE BEGIN StartBBBTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBBBTask */
}

/* USER CODE BEGIN Header_StartCCCTask */
/**
* @brief Function implementing the CCCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCCCTask */
void StartCCCTask(void *argument)
{
  /* USER CODE BEGIN StartCCCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCCCTask */
}

/* USER CODE BEGIN Header_StartDDDTask */
/**
* @brief Function implementing the DDDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDDDTask */
void StartDDDTask(void *argument)
{
  /* USER CODE BEGIN StartDDDTask */
  /* Infinite loop */
  for(;;)
  {
    /* 终点到达：path_runner 置 g_end_reached 后，在这里执行终点动作。
     * 当前占位：检测到标志即停车（终点动作待填实）。 */
    if (g_end_reached)
    {
      /* TODO 终点动作：例如结束蜂鸣/机械臂归位/灯光等，当前占位仅停车 */
      Stop();
      g_end_reached = 0;   /* 消费标志，避免重复执行 */
    }
    osDelay(5);
  }
  /* USER CODE END StartDDDTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
  Servo_UartInit();   /* 启动 UART4 总线舵机半双工接收 */
  /* Infinite loop */
  for(;;)
  {
    Servo_Poll();     /* 轮询解析回读应答 */
    osDelay(5);
  }
  /* USER CODE END StartServoTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/* USER CODE BEGIN 4 */
/**
  * @brief  FreeRTOS 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW = 2 时由内核调用）
  * @note   为什么要它：此前该检测为 0，任务栈踩穿相邻内存时【无任何迹象】——
  *         OLEDTask(原 512B) 在子页面绘制（mui + u8g2 字库 + snprintf ≈ 400+B）
  *         时静默越界，表现为"子页面按键卡死"，且非确定性，极难定位。
  *         现在溢出会走这里：记下出事的任务名，然后停机（比继续跑更安全 ——
  *         内存已损坏，继续执行可能乱发舵机/电机指令）。
  *         现场用调试器看 g_stack_overflow_task 即可知道是哪个任务。
  */
volatile char g_stack_overflow_task[configMAX_TASK_NAME_LEN] = {0};

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  if (pcTaskName != NULL)
  {
    uint8_t i;
    for (i = 0u; (i < (configMAX_TASK_NAME_LEN - 1u)) && (pcTaskName[i] != '\0'); i++)
    {
      g_stack_overflow_task[i] = pcTaskName[i];
    }
    g_stack_overflow_task[i] = '\0';
  }
  /* 关中断停机：栈已损坏，继续跑可能乱发指令。看门狗/复位留给现场处理。 */
  __disable_irq();
  for (;;) { }
}
/* USER CODE END 4 */
