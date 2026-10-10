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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor.h"
#include "Int_Track.h"
#include "Track_Task.h"
#include "Mode_FSM.h"
#include "IMU_Task.h"
#include "Int_OLED.h"
#include "Debug_Log.h"
#include "Delay_us.h"
#include "Int_MPU6050.h"
#include "Attitude.h"
#include "Int_Beep.h"
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

/* USER CODE BEGIN PV */

//电机对象
  extern Motor_Struct motorLeft;
  extern Motor_Struct motorRight;

//循迹数组
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

/* g_imu_data / g_imu_ready 已移至 Int_MPU6050 模块，TIM4 中断里更新 */

/* 运动与日志由 TIM4 更新；DebugLog_Display 在主循环复制完整快照并显示。 */

/* 主循环更新，通信状态可在调试器中直接查看。 */
static volatile OLED_Status g_oled_status = OLED_NOT_INITIALIZED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief 初始化外设、运动控制器与传感器，再进入前台显示和通信恢复循环。
  * @param 无。
  * @return 程序正常运行时不返回。
  * @note TIM4 启动后独占运动控制；主循环每 100 ms 显示状态和日志，并重试通信。
  *       OLED 格式化和 I2C 通信在恢复中断后执行，重连只重绘屏幕，不清除日志或故障现场。
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
  Delay_Init();   /* 使能 DWT CYCCNT，软件 IIC 依赖 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 软件 IIC 的 SDA/SCL 已在 MX_GPIO_Init() 中释放为高，此处不重复 */

  /* OLED 失败只记录状态，后续在主循环重试，不中止控制系统初始化。 */
  g_oled_status = OLED_Init();
  uint8_t oled_ready = (g_oled_status == OLED_OK) ? 1U : 0U;



  //电机对象初始化
  Motor_Init(&motorLeft);
  Motor_Init(&motorRight);
  
  line_following_init(&g_line_controller); //循迹初始化
  
  ModeFSM_Init();                          //运动状态机与 Route 初始化，默认循迹
  
  IMUTask_Init();                          //IMU 角度闭环 PID 初始化

  g_imu_ready = Int_MPU6050_Init();         //获取IMU状态
  if (g_imu_ready)
  {
    HAL_Delay(50);                /* 等传感器输出稳定 */
    if (Int_MPU6050_Calibrate(100)) {   /* 校准成功才响；失败(超时)不响=提醒重试 */
        Int_Beep_On();
        HAL_Delay(200);
        Int_Beep_Off();
    }
    Attitude_Init();              /* 四元数复位，姿态归零 */
  }
  HAL_TIM_Base_Start_IT(&htim4);   /* 启动唯一的 10 ms ModeFSM_Tick 控制入口 */


  if (oled_ready != 0U)
  {
    g_oled_status = DebugLog_Display();
    oled_ready = (g_oled_status == OLED_OK) ? 1U : 0U;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_imu_retry_ms = HAL_GetTick();
  uint32_t last_oled_retry_ms = HAL_GetTick();
  // uint32_t last_print_ms = HAL_GetTick();  /* 开启下面的串口调试块时一并启用。 */
  while (1)
  {
    /* IMU 仅在 IMU 模式由 TIM4 读取；通信失败后前台每 1s 重试，故障不会自动恢复运动。 */
    if (g_imu_ready == 0U &&
        (uint32_t)(HAL_GetTick() - last_imu_retry_ms) >= 1000U)
    {
      last_imu_retry_ms = HAL_GetTick();
      g_imu_ready = Int_MPU6050_Init();
    }

    // /* 每 500ms 串口打印六轴原始值，用于测试 MPU6050 是否好使 */
    // if ((uint32_t)(HAL_GetTick() - last_print_ms) >= 100U)
    // {
    //   last_print_ms = HAL_GetTick();
    //   if (g_imu_ready)
    //   {
    //     printf("E:%6d,%6d,%6d\r\n",
    //             (int)g_euler.yaw, (int)g_euler.pitch, (int)g_euler.roll);
    //   }
    // }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* OLED 初始化失败后每 1s 重试，成功后重绘全部八行并保留日志。 */
    if (oled_ready == 0U &&
        (uint32_t)(HAL_GetTick() - last_oled_retry_ms) >= 1000U) 
    {
      last_oled_retry_ms = HAL_GetTick();
      g_oled_status = OLED_Init();
      if (g_oled_status == OLED_OK)
      {
        oled_ready = 1U;
        g_oled_status = DebugLog_Display();
        oled_ready = (g_oled_status == OLED_OK) ? 1U : 0U;
      }
    }

    /* 每 100 ms 显示五行状态与三条最近日志；通信失败转入一秒重连流程。 */
    static uint32_t last_oled_ms = 0;
    if (oled_ready != 0U &&
        (uint32_t)(HAL_GetTick() - last_oled_ms) >= 100U)
    {
      last_oled_ms = HAL_GetTick();
      /* 临界区只复制现场；八行格式化和 I2C 刷新由日志模块在前台完成。 */
      g_oled_status = DebugLog_Display();
      if (g_oled_status != OLED_OK)
      {
        /* 刷新失败也进入一秒重连流程，OLED 重新上电后需要恢复初始化命令。 */
        oled_ready = 0U;
        last_oled_retry_ms = HAL_GetTick();
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置 HSE 和 PLL，使系统及控制定时器使用既定时钟。
  * @param 无。
  * @return 无。
  * @note 在初始化定时器前调用；配置失败进入 Error_Handler，不能在控制中断中调用。
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

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief 处理 HAL 外设初始化等系统级错误，禁止继续执行失效的初始化流程。
  * @param 无。
  * @return 无，进入等待循环。
  * @note 屏蔽中断后不返回；运行时运动故障由 ModeFSM 单独处理并输出停车。
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
  * @brief 提供 HAL 参数断言失败的调试入口。
  * @param file 发生断言的源文件名称。
  * @param line 发生断言的行号，从 1 开始。
  * @return 无。
  * @note 仅 USE_FULL_ASSERT 开启时编译；可在此设置断点查看 file 和 line。
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
