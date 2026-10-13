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
#include "Vision.h"
#include "formal_mode.h"
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for EncCatchTask */
osThreadId_t EncCatchTaskHandle;
const osThreadAttr_t EncCatchTask_attributes = {
  .name = "EncCatchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RunTask */
osThreadId_t RunTaskHandle;
const osThreadAttr_t RunTask_attributes = {
  .name = "RunTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TickTask */
osThreadId_t TickTaskHandle;
const osThreadAttr_t TickTask_attributes = {
  .name = "TickTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AAATask */
osThreadId_t AAATaskHandle;
const osThreadAttr_t AAATask_attributes = {
  .name = "AAATask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BBBTask */
osThreadId_t BBBTaskHandle;
const osThreadAttr_t BBBTask_attributes = {
  .name = "BBBTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CCCTask */
osThreadId_t CCCTaskHandle;
const osThreadAttr_t CCCTask_attributes = {
  .name = "CCCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DDDTask */
osThreadId_t DDDTaskHandle;
const osThreadAttr_t DDDTask_attributes = {
  .name = "DDDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncQueue */
osMessageQueueId_t EncQueueHandle;
const osMessageQueueAttr_t EncQueue_attributes = {
  .name = "EncQueue"
};
/* USER CODE BEGIN PV */

/* 步进电机版本验证测试全局（Arm_ReadVersion 结果，供 OLED 显示） */
int     g_step_ret = -1;   /* 最近一次读命令返回码：0=成功，负=失败 */
uint8_t g_step_fw  = 0;    /* 固件版本（低字节） */
uint8_t g_step_hw  = 0;    /* 硬件版本（低字节） */

/* 【临时桩】上电自动发版本命令：发送次数计数（仅用于验证 MCU→串口发送链路） */
volatile uint32_t g_step_tx_cnt = 0;

/* 【临时桩】驱动器回包原始字节显示：锁存一帧，长期显示直到下次发送清空 */
uint8_t  g_step_rx_buf[16] = {0};   /* 锁存的回包帧字节 */
uint8_t  g_step_rx_len = 0;        /* 锁存帧的字节数 */
volatile uint8_t g_step_rx_valid = 0; /* 1=锁存帧有效，OLED 可长期显示 */

/* ============================================================
 * 按键触发 + 解析显示（升级版临时桩）
 * 每个键发一条读命令 → 解析 → OLED 显示结果
 * ============================================================ */
volatile uint8_t g_step_disp_mode = 0;  /* 0=原始回包 1=版本 2=位置 3=误差 4=标志 5=转速 */
volatile int     g_step_ver_ret   = -1;
volatile uint16_t g_step_fw_ver   = 0;   /* 固件版本 16 位 */
volatile uint8_t  g_step_hw_ser   = 0;
volatile uint8_t  g_step_hw_type  = 0;
volatile int     g_step_val_ret   = -1;  /* Arm_ReadValue 返回码 */
volatile int32_t g_step_val       = 0;   /* 符号+数值 转成的有符号值 */
uint8_t g_step_flags     = 0;   /* 状态标志字节（Arm_ReadFlags 输出，非 volatile，单字节原子读） */

/* 【临时桩】运动测试：按键触发相对运动 + 累计脉冲，便于「收起」回原点 */
volatile uint32_t g_step_move_pulses = 5600;    /* 左展开脉冲增量（转台90°=5600，减速比7:1） */
volatile uint32_t g_step_move_cnt    = 0;       /* 运动命令计数 */
volatile int32_t  g_step_accum_pulses = 0;      /* 带符号累计脉冲：左转/左展开为正，右转为负 */

/* ============================================================
 * 总线舵机·按键测试桩显示变量（2026-10-01）
 * ============================================================ */
volatile uint8_t g_servo_cmd = 0;     /* 当前触发的命令标签（见 OLED 显示 switch） */
volatile uint8_t g_servo_id  = 0;     /* 解析出的舵机 ID */
volatile uint16_t g_servo_pwm = 0;    /* 解析出的位置 PWM */
volatile uint8_t g_servo_state = 0;   /* 0=无应答 1=有应答(OK/ID/POSITION) */
volatile servo_reply_t g_servo_reply; /* 最近一次回读结果快照 */

/* 【裸回环】接收到的原始字节快照（供 OLED 第四行显示，hex） */
volatile uint8_t g_servo_rx_disp[8] = {0};  /* 最近接收的字节（最多显示 8 字节） */
volatile uint8_t g_servo_rx_cnt = 0;         /* 当前已收字节数（可用于显示长度） */

/* ============================================================
 * 总线舵机·双舵机协同测试状态（2026-10-02，P1+P2 协同）
 * ============================================================ */
volatile uint16_t g_servo_time    = 3000;   /* 当前运动时间 ms（初始 3000） */
volatile uint16_t g_servo_pwm1    = 0;      /* P1 当前位置（ID=1） */
volatile uint16_t g_servo_pwm2    = 0;      /* P2 当前位置（ID=2） */
volatile uint8_t  g_servo_valid1  = 0;      /* P1 位置有效标志 */
volatile uint8_t  g_servo_valid2  = 0;      /* P2 位置有效标志 */
volatile uint16_t g_servo_pwm3    = 0;      /* P3 当前位置（ID=3） */
volatile uint8_t  g_servo_valid3  = 0;      /* P3 位置有效标志 */
volatile uint32_t g_servo_query_tick = 0;   /* 上次查询位置的 tick */

/* P1/P2 电压（x10 V，来自 PRTV 解析） */
volatile int16_t g_servo_v1 = 0;            /* P1 电压 x10 V */
volatile int16_t g_servo_v2 = 0;            /* P2 电压 x10 V */
volatile uint8_t g_servo_phase = 0;         /* 当前该读哪个舵机：0=P1 1=P2 2=P3（事件驱动采样环） */
volatile uint32_t g_servo_lastsend_tick = 0; /* 上次发送读命令时刻（超时看门狗） */

volatile uint8_t g_a_expanded = 0;          /* 键2 A 姿态 toggle：0=收起态 1=展开态（展开/收起交替） */

/* ============================================================
 * 展开/回收两步时序状态机（2026-10-07 定稿）
 *   展开：t=0 夹爪开 + 总线舵机先动(3s)；t=+2s 步进 θ1 后动
 *   收起：t=0 步进 θ1 先动；          t=+2s 总线舵机 + 夹爪关 后动
 *   B收起中转(kind=2)：四步先后 舵机→展开态 / 步进→5600 / 步进→0 / 舵机→初始
 *   用 g_seq_kind 区分展开(0)/收起(1)/B中转(2)/A中转(3)/B预存(4)，
 *   g_seq_step_pulses 存步进目标。
 * ============================================================ */
volatile uint8_t  g_seq_kind = 0;           /* 0=展开(总线先) 1=收起(步进先) 2=B收起中转 3=A收起中转 4=B预存 */
volatile uint8_t  g_seq_phase = 0;          /* 0=空闲 1/2/3=等第2/3/4步 */
volatile uint32_t g_seq_due_tick = 0;       /* 下一步触发时刻 */
volatile int32_t  g_seq_step_pulses = 0;    /* 步进 θ1 目标脉冲（下一步执行） */

/* 任务姿态调试：当前激活的任务态（供 OLED 显示） */
volatile uint8_t  g_servo_disp_mode = 0;    /* 0=收起 1=A 2=B 7=B左 8=B中 9=B右 */

/* 三舵机位置标定模式（2026-10-08）：P3 重标
 *   键1=广播释放扭力  键3=广播恢复扭力  键2=P3 设置初始值
 *   值含义（供 OLED 行1 标签）：0=标定中 1=已释放 2=已恢复 3=已设初始 */
volatile uint8_t  g_servo_calib_mode = 0;

/* 手动持续前进开关（键5 按下切换，测 PWM/电机是否起作用） */
volatile uint8_t  g_manual_fwd = 0;   /* 0=停 1=持续前进（直接喂固定 PWM，不闭环） */

/* 激光/夹爪手动开关（键 7/8/9） */
volatile uint8_t  g_laser_on  = 0;   /* 0=关 1=开（键7 切换） */
volatile uint8_t  g_grip_open = 0;   /* 0=回位(135°) 1=张开(45°)（键4 切换） */

/* ============================================================
 * 实测标定序列状态机（2026-10-08 二版）
 *   分阶段实测找位值：一键跑一小段动作序列（见 Cal_Start/Cal_RunStep）。
 * ============================================================ */
volatile uint8_t  g_cal_seq      = 0;    /* 序列号 1~7，0=空闲 */
volatile uint8_t  g_cal_step     = 0;    /* 序列内当前步号 */
volatile uint32_t g_cal_due_tick = 0;    /* 下一步触发时刻 */
volatile uint8_t  g_cal_a_state  = 0;    /* 键1 toggle：0=收起 1=展开 */

/* ============================================================
 * 自动演示链调度（2026-10-08）
 *   全自动跑车+臂：path_runner 到任务点(B/D)后自动串跑对应的臂
 *   序列链，链尾自动 PathRunner_TaskDone()。
 *   g_chain_next：下一个待触发的序列号（0=无待触发，链空闲）。
 * ============================================================ */
volatile uint8_t  g_chain_next = 0;   /* 待触发序列号，0=空闲 */
volatile uint8_t  g_chain_last = 0;   /* 最后启动的序列号（识别链尾完成） */

/* ============================================================
 * 启动前摇（2026-10-08）：按下键 1 后 2s 陀螺仪校准 + 1s 后车启动。
 *   g_start_phase：0=空闲 1=等校准(2s) 2=等启动(1s)
 * ============================================================ */
volatile uint8_t  g_start_phase = 0;      /* 0=空闲 1=等校准 2=等启动 */
volatile uint32_t g_start_due_tick = 0;   /* 下一步触发时刻 */

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

  /* 启动 PWM 舵机（PD12/TIM4_CH1，50Hz，500~2500us ↔ 0~270°） */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  PWM_Servo_SetAngle(135.0f);   /* 上电归位：夹爪置 135° 中性位 */

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

/* ============================================================
 * 激光开关（GPIOC.1 = Laser_Pin）
 *   🔴 LASER_ON_LEVEL 待实测：默认高电平点亮；若反了改 GPIO_PIN_RESET。
 * ============================================================ */
#define LASER_ON_LEVEL   GPIO_PIN_SET
static void Laser_Set(uint8_t on)
{
  GPIO_PinState s = on ? LASER_ON_LEVEL
                       : ((LASER_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, s);
}

/* 步进 θ1 相对原点定位：转到目标累计脉冲（相对定位 + 软累加器）
 *   delta = target - g_step_accum_pulses
 *   delta>0 → 左转 ARM_DIR_CW（正累计）；delta<0 → 右转 ARM_DIR_CCW */
static void StepGoTo(int32_t target_pulses)
{
  int32_t delta = target_pulses - g_step_accum_pulses;
  if (delta > 0)
  {
    Arm_MoveRelative((uint32_t)delta, ARM_DIR_CW);
  }
  else if (delta < 0)
  {
    Arm_MoveRelative((uint32_t)(-delta), ARM_DIR_CCW);
  }
  g_step_accum_pulses = target_pulses;
}

/* 慢速步进（D 救援段用，速度减半 30 RPM） */
static void StepGoToSlow(int32_t target_pulses)
{
  int32_t delta = target_pulses - g_step_accum_pulses;
  if (delta > 0)
  {
    Arm_MoveRelativeSlow((uint32_t)delta, ARM_DIR_CW);
  }
  else if (delta < 0)
  {
    Arm_MoveRelativeSlow((uint32_t)(-delta), ARM_DIR_CCW);
  }
  g_step_accum_pulses = target_pulses;
}

/* ============================================================
 * 实测标定序列状态机（2026-10-08 二版）
 *   目的：分阶段实测找位值。一键跑一小段动作序列，配合 OLED
 *   的 P1/P2/P3 实时读回，人肉读出实际到位 PWM 后回填 arm_config.h。
 *
 *   序列：
 *     SEQ 1  A 点（腕 P3 平伸→回位，其余不动）【演示链使用】
 *     SEQ 2  A 收起（腕 P3→home + 夹爪关）【未使用】
 *     SEQ 3  B 展开接俯身（舵机展开→步进90°→俯身）【演示链使用】
 *     SEQ 4  B 夹取→升起→转身180°→准备放置【演示链使用】
 *     SEQ 5  放置→收夹爪→升起→90°回→收起【演示链使用】
 *     SEQ 6  C 点（步进左转→P3平伸→激光→转回→P3回位）【演示链使用】
 *     SEQ 7  C 收起（步进回→舵机 home + 夹爪关）【未使用】
 *     SEQ 8  D 展开接俯身【演示链使用】
 *     SEQ 9  D 夹取→升起→转身90°【演示链使用】
 *     SEQ 10 D 收起（夹爪保持夹取 95°）【演示链使用】
 *     SEQ 11 键6 起身（B展开）→ 转桶方向【已改由 SEQ13，保留待用】
 *     SEQ 12 键5 D 升起转身（先展开态再转身，慢速）【姿态测试使用】
 *     SEQ 13 键6 D 救援（收起姿态，慢速）【姿态测试使用】
 *     SEQ 14 键1 D 展开（舵机先 2s → 步进后）【姿态测试使用】
 *     SEQ 15 键2 D 左预存（舵机先 → 步进后）【姿态测试使用】
 *     SEQ 16 键3 D 中预存（舵机先 → 步进后）【姿态测试使用】
 *     SEQ 17 键4 D 右预存（舵机先 → 步进后）【姿态测试使用】
 *
 *   规则：展开「先舵机再步进」，收起「先步进再舵机」；
 *         展开即夹爪打开，收起夹爪关回中性 135°（D 收起例外：保持夹取）。
 *   步进 θ1 用软累加器绝对定位（StepGoTo），各步目标脉冲如下：
 *     B/D 展开 +90°=5600；转身 180°=5600→-5600；回中=0。
 * ============================================================ */

static void Cal_Start(uint8_t seq)
{
  g_cal_seq  = seq;
  g_cal_step = 1;
  g_cal_due_tick = 0;   /* 0 = 立即执行第 1 步 */
}

/* 链内启动：启动序列并记录链尾标识（仅供自动演示链使用）。
 * 手动标定键不要走这里，避免误串下一段序列。 */
static void Chain_Launch(uint8_t seq)
{
  Cal_Start(seq);
  g_chain_last = seq;
}

/* ============================================================
 * 自动演示链：启动某个任务点的成套臂序列。
 *   串到尾的序列会把 g_chain_next 置 0（链结束）。
 *   A 点：SEQ1（P3 平伸→回位）
 *   B 点：SEQ3(展开俯身)→SEQ4(夹取升起转身放置)→SEQ5(放置升起回收)
 *   C 点：SEQ6（步进左转→P3平伸→激光→转回→P3回位）
 *   D 点：SEQ8(展开俯身)→SEQ9(夹取升起转身90°)→SEQ10(收起)
 * ============================================================ */
static void __attribute__((unused)) Chain_StartTask(uint8_t target_idx)
{
  if (target_idx == 0)      /* A 点：SEQ1（P3 平伸→回位，其余不动） */
  {
    Chain_Launch(1);
  }
  else if (target_idx == 1) /* B 点：SEQ3→4→5 */
  {
    Chain_Launch(3);
  }
  else if (target_idx == 2) /* C 点：SEQ6（步进左转→P3平伸→激光→转回→P3回位） */
  {
    Chain_Launch(6);
  }
  else if (target_idx == 3) /* D 点：SEQ8→9→10 */
  {
    Chain_Launch(8);
  }
  else
  {
    /* 终点等其它点：直接视为完成任务 */
    PathRunner_TaskDone();
  }
}

/* 串链推进：MainTask 循环在 g_cal_seq==0（一段刚结束）时调用。
 *   若刚结束的是链尾序列，则 task_done 并清链；否则串跑下一个序列。 */
static void __attribute__((unused)) Chain_Tick(void)
{
  if (g_chain_last == 0)
  {
    return;   /* 从未启动过链（或已清链） */
  }

  switch (g_chain_last)
  {
    case 1:   /* A 尾（SEQ1 单序列）：完成 */
    case 5:   /* B 链尾（SEQ5） */
    case 6:   /* C 尾（SEQ6 单序列）：完成 */
    case 10:  /* D 链尾（SEQ10） */
      g_chain_last = 0;
      g_chain_next = 0;
      PathRunner_TaskDone();
      break;

    case 3:   /* B：SEQ3 完 → SEQ4 */
      Chain_Launch(4);
      break;
    case 4:   /* B：SEQ4 完 → SEQ5 */
      Chain_Launch(5);
      break;
    case 8:   /* D：SEQ8 完 → SEQ9 */
      Chain_Launch(9);
      break;
    case 9:   /* D：SEQ9 完 → SEQ10 */
      Chain_Launch(10);
      break;

    default:
      /* 其它序列（如手动标定键触发）结束：不串链，清链 */
      g_chain_last = 0;
      g_chain_next = 0;
      break;
  }
}

/* 执行当前步，并安排下一步时刻；序列结束时清 g_cal_seq=0 */
static void Cal_RunStep(void)
{
  switch (g_cal_seq)
  {
    /* ---------- SEQ 1：A 点（P3 平伸→回位，其余不动） ---------- */
    case 1:
      switch (g_cal_step)
      {
        case 1: /* 腕 P3 → 平伸（P1/P2/夹爪/步进 均不动） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_P3_FLAT_108, 2000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        case 2: /* 腕 P3 → 回位 home（机械臂收好） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 2：A 收起 ---------- */
    case 2:
      switch (g_cal_step)
      {
        case 1: /* 腕 P3→home + 夹爪关（θ1 未动，无步进） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
          PWM_Servo_SetAngle(ARM_GRIP_PARK_DEG);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 3：B 展开接俯身 ---------- */
    case 3:
      switch (g_cal_step)
      {
        case 1: /* 展开（舵机先动） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SendGroup("#001P1710T3000!#002P1700T3000!#003P0500T3000!");
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 左转 90°（后动） */
          StepGoTo(ARM_DEMO_STEP_90);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4500;
          break;
        case 3: /* 俯身 腕部 P3（先） */
          Servo_SetAngleTime(ARM_TH4_ID, 584, 1000);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 4: /* 俯身 小臂 P2（次） */
          Servo_SetAngleTime(ARM_TH3_ID, 1701, 1000);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 5: /* 俯身 大臂 P1（后，3 秒缓动） */
          Servo_SetAngleTime(ARM_TH2_ID, 1281, 3000);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 3000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 4：B 夹取→升起→转身180°→准备放置 ---------- */
    case 4:
      switch (g_cal_step)
      {
        case 1: /* 夹取 */
          PWM_Servo_SetAngle(ARM_GRIP_CLOSE_DEG);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1200;
          break;
        case 2: /* 升起 大臂 P1（先） */
          Servo_SetAngleTime(ARM_TH2_ID, 1521, 1000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 3: /* 升起 小臂 P2（次） */
          Servo_SetAngleTime(ARM_TH3_ID, 1500, 1000);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 4: /* 升起 腕部 P3（后） */
          Servo_SetAngleTime(ARM_TH4_ID, 584, 1000);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 5: /* 转身 180°（θ1 5600 → -5600） */
          StepGoTo(-ARM_DEMO_STEP_90);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 8000;
          break;
        case 6: /* 准备放置（俯身到桶：腕→小臂→大臂） */
          Servo_SetAngleTime(ARM_TH4_ID, 584, 1000);
          g_cal_step = 7; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 7:
          Servo_SetAngleTime(ARM_TH3_ID, 1750, 1000);
          g_cal_step = 8; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 8:
          Servo_SetAngleTime(ARM_TH2_ID, 1553, 1000);
          g_cal_step = 9; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 5：放置→收夹爪→升起→90°回→收起 ---------- */
    case 5:
      switch (g_cal_step)
      {
        case 1: /* 放置：夹爪 45° 张开 */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1200;
          break;
        case 2: /* 收夹爪 135° */
          PWM_Servo_SetAngle(ARM_GRIP_PARK_DEG);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 1200;
          break;
        case 3: /* 升起 大臂 P1（先） */
          Servo_SetAngleTime(ARM_TH2_ID, 1521, 1000);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 4: /* 升起 小臂 P2 */
          Servo_SetAngleTime(ARM_TH3_ID, 1500, 1000);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 5: /* 升起 腕部 P3 */
          Servo_SetAngleTime(ARM_TH4_ID, 584, 1000);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 6: /* 步进 θ1 90°回（-5600 → 0，先步进） */
          StepGoTo(0);
          g_cal_step = 7; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 7: /* 收起（舵机 home，后动） */
          Servo_SendGroup("#001P1976T3000!#002P2425T3000!#003P1762T3000!");
          g_cal_step = 8; g_cal_due_tick = HAL_GetTick() + 3500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 6：C 点（步进左转→P3平伸→激光→转回→P3回位） ---------- */
    case 6:
      switch (g_cal_step)
      {
        case 1: /* 步进 θ1 左转 90°（先动） */
          StepGoTo(ARM_DEMO_STEP_90);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 2: /* 腕 P3 → 平伸 */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_P3_FLAT_108, 2000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        case 3: /* 开激光 */
          Laser_Set(1);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 4: /* 关激光（激光保持 2 秒） */
          Laser_Set(0);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick();
          break;
        case 5: /* 步进 θ1 转回 0 */
          StepGoTo(0);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 6: /* 腕 P3 → 回位（机械臂收好） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
          g_cal_step = 7; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 7：C 收起 ---------- */
    case 7:
      switch (g_cal_step)
      {
        case 1: /* 步进 θ1 回 0（先步进） */
          StepGoTo(0);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 2: /* 舵机 home + 夹爪关（后动） */
          Servo_SendGroup("#001P1976T3000!#002P2425T3000!#003P1762T3000!");
          PWM_Servo_SetAngle(ARM_GRIP_PARK_DEG);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 3500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 8：D 展开接俯身 ---------- */
    case 8:
      switch (g_cal_step)
      {
        case 1: /* 展开（舵机先动，沿用 B 展开值） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SendGroup("#001P1710T3000!#002P1700T3000!#003P0500T3000!");
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 左转 90°（后动） */
          StepGoTo(ARM_DEMO_STEP_90);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4500;
          break;
        case 3: /* 俯身 腕部 P3（先） */
          Servo_SetAngleTime(ARM_TH4_ID, 1119, 1000);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 4: /* 俯身 小臂 P2（次） */
          Servo_SetAngleTime(ARM_TH3_ID, 1968, 1000);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 5: /* 俯身 大臂 P1（后，3 秒缓动） */
          Servo_SetAngleTime(ARM_TH2_ID, 1309, 3000);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 3000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 9：D 夹取→升起→转身180° ---------- */
    case 9:
      switch (g_cal_step)
      {
        case 1: /* 夹取 */
          PWM_Servo_SetAngle(ARM_GRIP_CLOSE_DEG);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1200;
          break;
        case 2: /* 升起 大臂 P1（先） */
          Servo_SetAngleTime(ARM_TH2_ID, 1521, 1000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 3: /* 升起 小臂 P2 */
          Servo_SetAngleTime(ARM_TH3_ID, 1300, 1000);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 4: /* 升起 腕部 P3（上翻到初始） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 1000);
          g_cal_step = 5; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 5: /* 转身 90°（θ1 5600 → 0） */
          StepGoTo(0);
          g_cal_step = 6; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 10：D 收起（夹爪保持夹取 95°） ---------- */
    case 10:
      switch (g_cal_step)
      {
        case 1: /* 步进 θ1 回 0（先步进） */
          StepGoTo(0);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 2: /* 舵机 home（后动），P3 上翻到初始，夹爪保持夹取不关 */
          Servo_SendGroup("#001P1976T3000!#002P2425T3000!#003P1762T3000!");
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 3500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 11：键6 起身（B展开）→ 转桶方向（2026-10-13） ---------- */
    case 11:
      switch (g_cal_step)
      {
        case 1: /* 起身 = B 展开观察态（夹爪开 + 舵机先动） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_P1, 3000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_P2, 3000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_P3, 3000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 → 5600（后动，左转 90°） */
          StepGoTo(ARM_TH1_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4500;
          break;
        case 3: /* 转桶方向：步进 θ1 → -5600（右转，桶在右侧） */
          StepGoTo(-ARM_TH1_MID_PULSE);
          g_cal_step = 4; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 12：键5 D 升起转身（先展开态再转身，慢速 2026-10-13） ---------- */
    case 12:
      switch (g_cal_step)
      {
        case 1: /* 升到展开态：舵机 P1/P2/P3 → D 展开（慢 4s，舵机先动） */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_P1, 4000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_P2, 4000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_P3, 4000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 转身：步进 θ1 → 5600（慢速，后动） */
          StepGoToSlow(ARM_TH1_D_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 13：键6 D 救援（收起姿态，慢速 2026-10-13） ---------- */
    case 13:
      switch (g_cal_step)
      {
        case 1: /* 步进先动：θ1 → 0（慢速） */
          StepGoToSlow(0);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 5000;
          break;
        case 2: /* 舵机后动：P1/P2/P3 → 收起态（慢 4s），夹爪保持夹取 95° 不关 */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_HOME_P1, 4000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_HOME_P2, 4000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 4000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 14：键1 D 展开（舵机先动 2s → 步进后动）---------- */
    case 14:
      switch (g_cal_step)
      {
        case 1: /* 夹爪开 + 舵机展开（先动） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_P1, 3000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_P2, 3000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_P3, 3000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 → 5600（后动） */
          StepGoTo(ARM_TH1_D_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 15：键2 D 左预存（舵机先 2s → 步进后）---------- */
    case 15:
      switch (g_cal_step)
      {
        case 1: /* 舵机 → 左预存（先动） */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_LEFT_P1, 2000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_LEFT_P2, 2000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_LEFT_P3, 2000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1500;
          break;
        case 2: /* 步进 θ1 → 7777（后动） */
          StepGoTo(ARM_TH1_D_LEFT_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 16：键3 D 中预存（舵机先 2s → 步进后）---------- */
    case 16:
      switch (g_cal_step)
      {
        case 1: /* 舵机 → 中预存（先动） */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_MID_P1, 2000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_MID_P2, 2000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_MID_P3, 2000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1500;
          break;
        case 2: /* 步进 θ1 → 5600（后动） */
          StepGoTo(ARM_TH1_D_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 17：键4 D 右预存（舵机先 2s → 步进后）---------- */
    case 17:
      switch (g_cal_step)
      {
        case 1: /* 舵机 → 右预存（先动） */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_RIGHT_P1, 2000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_RIGHT_P2, 2000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_RIGHT_P3, 2000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1500;
          break;
        case 2: /* 步进 θ1 → 3734（后动） */
          StepGoTo(ARM_TH1_D_RIGHT_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 18：键7 B 展开（单独，无转桶）---------- */
    case 18:
      switch (g_cal_step)
      {
        case 1: /* 夹爪开 + 舵机 → B 展开（先动） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_B_P1, 3000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_B_P2, 3000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_B_P3, 3000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 → 5600（后动，左转 90°） */
          StepGoTo(ARM_TH1_B_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 19：键8 C 展开（θ1 左转 + P3 平伸，P1/P2 不动）---------- */
    case 19:
      switch (g_cal_step)
      {
        case 1: /* 步进 θ1 左转 90°（先动） */
          StepGoTo(ARM_TH1_C_MID_PULSE);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        case 2: /* 腕 P3 → 平伸 1858（P1/P2 不动） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_P3_FLAT_108, 2000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 20：键9 D 展开（单独）---------- */
    case 20:
      switch (g_cal_step)
      {
        case 1: /* 夹爪开 + 舵机 → D 展开（先动） */
          PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_D_P1, 3000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_D_P2, 3000);
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_D_P3, 3000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 步进 θ1 → 5600（后动，左转 90°） */
          StepGoTo(ARM_TH1_D_MID_PULSE);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 4000;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 21：键2 A 展开（停在扫码位，不收）---------- */
    case 21:
      switch (g_cal_step)
      {
        case 1: /* 展开：大臂 P1 + 小臂 P2 同时（4s） */
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_A_SCAN_P1, 4000);
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_A_SCAN_P2, 4000);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 2000;
          break;
        case 2: /* 过 2s 腕状 P3 再动（2s） */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_A_SCAN_P3, 2000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    /* ---------- SEQ 22：键2 A 收起（全回 home，严格错峰）---------- */
    case 22:
      switch (g_cal_step)
      {
        case 1: /* 收起：腕 P3 (2s) + 大臂 P1 (3.5s) 同时 */
          Servo_SetAngleTime(ARM_TH4_ID, ARM_POSE_HOME_P3, 2000);
          Servo_SetAngleTime(ARM_TH2_ID, ARM_POSE_HOME_P1, 3500);
          g_cal_step = 2; g_cal_due_tick = HAL_GetTick() + 1000;
          break;
        case 2: /* 过 1s 收小臂 P2（2s） */
          Servo_SetAngleTime(ARM_TH3_ID, ARM_POSE_HOME_P2, 2000);
          g_cal_step = 3; g_cal_due_tick = HAL_GetTick() + 2500;
          break;
        default:
          g_cal_seq = 0;
          break;
      }
      break;

    default:
      g_cal_seq = 0;
      break;
  }
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
  Arm_Init();   /* 步进电机 Emm_V5：绑定 USART1 + 启动接收 */

  /* ========================================================================
   * 【步进电机·按键触发读参数 + 解析｜临时桩】
   *   键 1 → 读版本（0x1F）
   *   键 2 → 读实时位置（0x36）
   *   键 3 → 读位置误差（0x37）
   *   键 7 → 读状态标志（0x3A）
   *   键 9 → 读实时转速（0x35）
   * 原「路径测试按键」已注释（#if 0），需要时恢复。
   * ====================================================================== */
  g_step_ret = -1;
  g_step_fw  = 0;
  g_step_hw  = 0;
  g_step_tx_cnt = 0;
  g_step_disp_mode = 0;
  g_step_ver_ret = -1;
  g_step_val_ret = -1;

  TickType_t last_wake = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    /* ===== 决策层：路径状态机 tick（原 AAATask） ===== */
    PathRunner_Tick();

    /* ===== 正式模式 Supervisor：任务动作状态机 ===== */
    Formal_Tick();

    /* ===== Host 2.0 上下位机通信解析：消费已完整收到的帧 ===== */
    CommunicateToHost();
    if (parsed_flag)
    {
      /* TODO: 根据解析出的指令执行对应动作（业务层接入） */
      parsed_flag = 0;
    }

    /* ===== 视觉通信业务层周期驱动：超时重发 / 位姿上报节拍 ===== */
    Vision_Tick();

    /* ==================================================================
     * 【正式模式 / 标定阶段键位】（2026-10-13 终版）
     *   键 1 = A 姿态 展开⇄收起 toggle
     *   键 2 = 走车到 A → 自动展开；再按 2 或收 0x11 → 收起
     *   键 3 = 清空
     *   键 4 = 夹爪 开启⇄关闭 toggle（45° 开 / 135° 回位）
     *   键 5 = 夹爪 抓取 95°
     *   键 6 = 清空
     *   键 7 = B 展开（SEQ18）  键 8 = C 展开（SEQ19）  键 9 = D 展开（SEQ20）
     * ================================================================== */
    if (Key_Check(KEY_TO_A, KEY_DOWN))   /* 键 1：A 扫码姿态 toggle（展开⇄收起） */
    {
      if (g_a_expanded)
      {
        Cal_Start(22);   /* 收起 */
        g_a_expanded = 0;
      }
      else
      {
        Cal_Start(21);   /* 展开 */
        g_a_expanded = 1;
      }
    }
    if (Key_Check(KEY_TO_B, KEY_DOWN))   /* 键 2：走车到 A → 自动展开；再按2或收 0x11 → 收起 */
    {
      if (!Formal_IsBusy())
      {
        Formal_Start(FORMAL_END_AFTER_A);
      }
      else
      {
        /* 二次按下：若正在 A 扫码阶段等待 0x11，则强制收起 */
        Formal_ForceTaskRetract();
      }
    }
    /* 键 3：清空（暂不绑定） */
    if (Key_Check(KEY_GO_A, KEY_DOWN))   /* 键 4：夹爪 开启⇄关闭 toggle（45° 开 / 135° 回位） */
    {
      if (g_grip_open)
      {
        PWM_Servo_SetAngle(ARM_GRIP_PARK_DEG);   /* 回位 135° */
        g_grip_open = 0;
      }
      else
      {
        PWM_Servo_SetAngle(ARM_GRIP_OPEN_DEG);   /* 张开 45° */
        g_grip_open = 1;
      }
    }
    if (Key_Check(KEY_FWD, KEY_DOWN))    /* 键 5：夹爪 抓取 95° */
    {
      PWM_Servo_SetAngle(ARM_GRIP_CLOSE_DEG);
      g_grip_open = 0;
    }
    if (Key_Check(KEY_TO_D, KEY_DOWN))   /* 键 7：B 展开（单独） */
    {
      Cal_Start(18);
    }
    if (Key_Check(KEY_CALIB, KEY_DOWN))  /* 键 9：D 展开（单独） */
    {
      Cal_Start(20);
    }

    /* 标定序列推进：当前步延时到期后执行下一步 */
    if (g_cal_seq != 0 && ((int32_t)(HAL_GetTick() - g_cal_due_tick) >= 0))
    {
      Cal_RunStep();
    }

    /* 键 6：清空（暂不绑定） */

    /* 键 8：C 展开（θ1 左转 + P3 平伸） */
    if (Key_Check(KEY_TO_END, KEY_DOWN))   /* 键 8：C 展开 */
    {
      Cal_Start(19);
    }

    /* ==================== 启动前摇推进 ====================
     * phase 1：等 2s → 陀螺仪校准；phase 2：等 1s → 启动 path_runner。 */
    if (g_start_phase != 0 && ((int32_t)(HAL_GetTick() - g_start_due_tick) >= 0))
    {
      if (g_start_phase == 1)
      {
        Car_YawCalibrate();               /* 2s 到：陀螺仪校准 */
        g_start_phase = 2;
        g_start_due_tick = HAL_GetTick() + 1000;
      }
      else /* phase 2 */
      {
        g_start_phase = 0;
        PathRunner_Start();               /* 1s 到：车开始动 */
      }
    }

    /* ==================== 自动演示调度（底盘测试阶段：禁用自动串链） ====================
     * 1) 到达任务点（RUN_ST_TASK）且臂链空闲 → 触发该点臂序列链；
     * 2) 某个臂序列刚结束（g_cal_seq==0）→ 串链推进 / 链尾回喂 TaskDone。
     * 底盘路径测试阶段：键 1/2 直接 PathRunner_Goto，定位到点即停（RUN_ST_TASK），
     * 不自动串臂链，故此处整块注释。
     * ======================================================= */
#if 0
    if (g_cal_seq == 0)
    {
      if (g_chain_last != 0)
      {
        Chain_Tick();          /* 刚结束一段：串下一段或完成任务 */
      }
      else if (g_run_state == RUN_ST_TASK)
      {
        Chain_StartTask(g_target_idx);   /* 到任务点：启动对应臂链 */
      }
    }
#endif

#if 0
    /* ===== 步进×舵机 姿态测试（临时注释） ===== */
    if (Key_Check(KEY_TO_A, KEY_DOWN))   /* 键 1：回初始位 */
    {
      g_servo_time = 3000;
      g_servo_cmd = 1;
      g_servo_pose = 0;   /* 收起：位置环不动作 */
      Servo_SendGroup("#001P2137T3000!#002P1776T3000!");
      /* 更新位置环目标到初始位 */
      g_servo_target1 = 2137;
      g_servo_target2 = 1776;
      g_servo_cmd1 = 2137;
      g_servo_cmd2 = 1776;
      g_servo_move_done_tick = HAL_GetTick() + 3000;   /* 3s 平滑运动期结束 */
      /* 步进反向转回累计量（左转累计为正 → 反向 CCW） */
      if (g_step_accum_pulses != 0)
      {
        uint32_t back = (g_step_accum_pulses > 0) ? (uint32_t)g_step_accum_pulses
                                                  : (uint32_t)(-g_step_accum_pulses);
        Arm_MoveRelative(back, ARM_DIR_CCW);
        g_step_accum_pulses = 0;
      }
    }
    if (Key_Check(KEY_TO_B, KEY_DOWN))   /* 键 2：A 展开（仅舵机，开环固定命令，无 PID） */
    {
      g_servo_time = 3000;
      g_servo_cmd = 2;
      g_servo_pose = 1;   /* A 展开：开环，位置环跳过 */
      Servo_SendGroup("#001P0699T3000!#002P0500T3000!");
      g_servo_target1 = 699;
      g_servo_target2 = 500;
      g_servo_cmd1 = 699;
      g_servo_cmd2 = 500;
      g_servo_move_done_tick = HAL_GetTick() + 3000;
    }
    if (Key_Check(KEY_TO_C, KEY_DOWN))   /* 键 3：B 展开（先舵机展开，2s 后步进左转 90°，PID 闭环） */
    {
      g_servo_time = 3000;
      g_servo_cmd = 3;
      g_servo_pose = 2;   /* B 展开：位置环 PID 闭环 */
      Servo_SendGroup("#001P1400T5000!#002P0800T5000!");
      g_servo_target1 = 1400;
      g_servo_target2 = 800;
      g_servo_cmd1 = 1400;
      g_servo_cmd2 = 800;
      g_servo_move_done_tick = HAL_GetTick() + 5000;   /* B 展开 T5000 */
      /* 步进延后触发：记录 3.5s 后的时刻，到点再左转 */
      g_b_step_armed = 1;
      g_b_step_due_tick = HAL_GetTick() + 3500;
    }
#endif

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

    /* 回读处理：位置 → 存变量 + 就地纠偏；ID 应答 → 存 g_servo_id。
     * 用 while 一次清空队列（可能积压多条回包）。 */
    while (Servo_HasReply())
    {
      servo_reply_t r = Servo_GetReply();
      if (r.kind == SERVO_REPLY_ID)
      {
        g_servo_id = r.id;   /* 存回应的 ID，供 OLED 显示「当前 ID」 */
      }
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
#if 0
        uint8_t cooled = ((int32_t)(HAL_GetTick() - g_servo_move_done_tick) >= 0);
        if (cooled && g_servo_pose == 2)
        {
          if (r.id == 1 && g_servo_target1 > 0)
          {
            int32_t e = (int32_t)g_servo_target1 - (int32_t)r.pwm;
            g_pid_err1 = (int16_t)e;
            if (e > SERVO_PID_DEADBAND || e < -SERVO_PID_DEADBAND)
            {
              int32_t nc = (int32_t)g_servo_cmd1 + ((e * SERVO_PID_KP_X10) / 10);
              if (nc > SERVO_PWM_MAX) nc = SERVO_PWM_MAX;
              if (nc < SERVO_PWM_MIN) nc = SERVO_PWM_MIN;
              g_servo_cmd1 = (int16_t)nc;
              Servo_SetAngleTime(1, (uint16_t)nc, 50);
              g_pid_act1++;
            }
          }
          if (r.id == 2 && g_servo_target2 > 0)
          {
            int32_t e = (int32_t)g_servo_target2 - (int32_t)r.pwm;
            g_pid_err2 = (int16_t)e;
            if (e > SERVO_PID_DEADBAND || e < -SERVO_PID_DEADBAND)
            {
              int32_t nc = (int32_t)g_servo_cmd2 + ((e * SERVO_PID_KP_X10) / 10);
              if (nc > SERVO_PWM_MAX) nc = SERVO_PWM_MAX;
              if (nc < SERVO_PWM_MIN) nc = SERVO_PWM_MIN;
              g_servo_cmd2 = (int16_t)nc;
              Servo_SetAngleTime(2, (uint16_t)nc, 50);
              g_pid_act2++;
            }
          }
        }
#endif
      }
    }

#if 0
    /* ==================== 原路径测试：按键触发（分段测试） ==================== */
    /* 键1→去A  键2→去B  键3→去C
     * 键7→去D  键8→去终点
     * 键9→陀螺仪校准
     * 注：用 KEY_DOWN（按下沿），与按键测试同源更可靠。
     * 除正在 MOVING 外，任意状态均可直接跳转（IDLE/TASK/DONE 都能重新导向）。 */
    if (Key_Check(KEY_CALIB, KEY_DOWN))
    {
      Car_YawCalibrate();
    }

    if (g_run_state != RUN_ST_MOVING)
    {
      if (Key_Check(KEY_TO_A, KEY_DOWN))   { PathRunner_Goto(0); }
      if (Key_Check(KEY_TO_B, KEY_DOWN))   { PathRunner_Goto(1); }
      if (Key_Check(KEY_TO_C, KEY_DOWN))   { PathRunner_Goto(2); }
      if (Key_Check(KEY_TO_D, KEY_DOWN))   { PathRunner_Goto(3); }
      if (Key_Check(KEY_TO_END, KEY_DOWN)) { PathRunner_Goto(4); }
    }
#endif

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
  OLED_Init();

  /* ========================================================================
   * 三舵机位置标定 OLED：行1/2/3 = P1/P2/P3 实时读回 PWM。
   *   数据源：g_servo_pwm1/2/3（主循环 50ms 轮询 Servo_ReadAngle 读回）。
   * ====================================================================== */
  for(;;)
  {
    OLED_ShowString(1, 1, "P1 ");
    OLED_ShowNum(1, 4, g_servo_pwm1, 4);

    OLED_ShowString(2, 1, "P2 ");
    OLED_ShowNum(2, 4, g_servo_pwm2, 4);

    OLED_ShowString(3, 1, "P3 ");
    OLED_ShowNum(3, 4, g_servo_pwm3, 4);

    OLED_ShowString(4, 1, "        ");

    osDelay(50);
  }

#if 0
  /* ==================== OLED 调试菜单（已禁用 2026-10-12） ====================
   * 需要时恢复：
   *   Menu_Enter();
   *   for(;;){ if(!Menu_IsActive()) Menu_Enter(); Menu_Tick(); osDelay(10); }
   * ====================================================================== */
#endif
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
