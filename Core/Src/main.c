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

/* 状态机和 IMU 共享数据由各模块管理，显示通过快照读取 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  //OLED初始化
  OLED_Init();
  //电机对象初始化
  Motor_Init(&motorLeft);
  Motor_Init(&motorRight);
  
  line_following_init(&g_line_controller); //循迹初始化
  
  ModeFSM_Init();                          //状态机初始化(默认 NORMAL_TRACK)
  
  IMUTask_Init();                          //IMU 角度闭环 PID 初始化

  Int_MPU6050_Init();                      //模块内部维护 IMU 通信状态
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
  HAL_TIM_Base_Start_IT(&htim4);   /* 启动 TIM4 10ms 节拍，中断里调度状态机 */


  OLED_ShowStr(0, 0, "Jeason FSM", 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_imu_retry_ms = HAL_GetTick();
  uint32_t last_oled_ms = 0;
  while (1)
  {
    /* IMU 读取在 TIM4 10ms 中断里完成；主循环只负责通信失败后 1s 重试 Init */
    if (g_imu_ready == 0U &&
        (uint32_t)(HAL_GetTick() - last_imu_retry_ms) >= 1000U)
    {
      last_imu_retry_ms = HAL_GetTick();
      Int_MPU6050_Init();
    }

    /* 中断只记录诊断帧；每轮最多输出四条，避免主循环一直排空队列 */
    for (uint8_t i = 0; i < 4U; i++)
    {
      ModeFSM_DebugFrame frame;
      if (!ModeFSM_PopDebugFrame(&frame)) break;
      printf("J,%lu,%02X,%u,%u,%u,%u,%lu,%d,%u,%u\r\n",
             (unsigned long)frame.tick_ms, (unsigned int)frame.raw_mask,
             (unsigned int)frame.state, (unsigned int)frame.left_votes,
             (unsigned int)frame.right_votes, (unsigned int)frame.event,
             (unsigned long)frame.event_count, (int)frame.yaw_ddeg,
             (unsigned int)frame.fault, (unsigned int)frame.imu_ready);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 每 200ms 从同一快照显示状态，短暂的路口事件通过串口日志观察 */
    if ((uint32_t)(HAL_GetTick() - last_oled_ms) >= 200U)
    {
      ModeFSM_t snapshot;
      char text[22];
      last_oled_ms = HAL_GetTick();
      ModeFSM_GetSnapshot(&snapshot);

      OLED_ShowStr(0, 2, "State:", 1);
      switch (snapshot.state)
      {
        case STATE_NORMAL_TRACK:     OLED_ShowStr(48, 2, "NORMAL_TRACK ", 1); break;
        case STATE_CROSS:            OLED_ShowStr(48, 2, "CROSS        ", 1); break;
        case STATE_TURN_LEFT:        OLED_ShowStr(48, 2, "TURN_LEFT    ", 1); break;
        case STATE_TURN_RIGHT:       OLED_ShowStr(48, 2, "TURN_RIGHT   ", 1); break;
        case STATE_FORWARD:          OLED_ShowStr(48, 2, "FORWARD      ", 1); break;
        case STATE_JUNCTION_PENDING: OLED_ShowStr(48, 2, "PENDING      ", 1); break;
        case STATE_APPROACH_TURN:    OLED_ShowStr(48, 2, "APPROACH     ", 1); break;
        case STATE_FAULT_STOP:       OLED_ShowStr(48, 2, "FAULT_STOP   ", 1); break;
        default:                     OLED_ShowStr(48, 2, "UNKNOWN      ", 1); break;
      }

      /* 用带符号字符串显示目标角，避免数字接口的字号和无符号转换问题 */
      snprintf(text, sizeof(text), "Target:%+4d         ", (int)snapshot.target_yaw);
      OLED_ShowStr(0, 4, (unsigned char *)text, 1);
    }
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

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

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
