/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口与外设启动。
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid_controller.h"
#include "supercap_control.h"
#include "adc_reader.h"
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
/*
 * 所有系统状态和 PI 控制器实例已迁移至：
 *   - sc_state   （supercap_control.h / supercap_control.c）
 *   - adc_reader （adc_reader.h / adc_reader.c）
 *
 * main.c 中不再保留任何裸全局变量。
 */
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 初始化超级电容控制系统 ---- */
  // 首次写入supercap_reference
  SC_Init(&sc_state);
  
  // 初始化之后进入self_test阶段
  /* ---- 确保初始化期间输出处于禁用状态 ---- */
  // 使用掩码修改MOE
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  HAL_GPIO_WritePin(CapDownCtrl_GPIO_Port, CapDownCtrl_Pin, GPIO_PIN_RESET);

  /* ---- 启动 FDCAN（CAN 总线通信） ---- */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  FDCAN_Filter_Init();
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* 只使能通道，保持 MOE=0；HAL 的 PWM_Start 会立即打开 MOE。 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE;
  /* 先启动 ADC DMA，再开放 100 kHz 控制中断，避免自检误判采样离线。 */
  ADC_Reader_Init(&hadc1);
  ADC_Reader_Start(&hadc1);
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  // 按照mian依次执行确保上电控制时序正确
  
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* ---- CAN 接收：事件驱动标志（无需延时） ---- */
    /* 仅当前一帧已被 RX ISR 消费后才请求新帧。
     * ISR 在复制数据后将 CAN_ReceiveDataRefresh_Flag 清零。 */
    if (CAN_ReceiveDataRefresh_Flag == 0) {
      CAN_ReceiveDataRefresh_Flag = 1;
    }

    /* ---- CAN 看门狗：500ms 无消息则回退到安全默认值 ---- */
    {
      static uint32_t last_can_rx_ms = 0;

      /* 检测新 CAN 消息（RX ISR 在消息到达时将 CAN_WDG_Count 设为 0） */
      if (CAN_WDG_Count == 0)
      {
        last_can_rx_ms = HAL_GetTick();
        CAN_WDG_Count = 1;  /* 确认收到，等待下一条消息 */
      }

      if (HAL_GetTick() - last_can_rx_ms > 500)
      {
        // can通信超时依旧写入power_reference
        sc_state.power_reference = POWER_REFERENCE_DEFAULT;
      }
    }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
 * 所有业务逻辑函数已迁移至：
 *   - pid_controller.c    （PID_Calculate、PID_Init、PID_Reset 等）
 *   - supercap_control.c  （SC_ADC_ConvertToPhysical 电压电流检测、
 *                          SC_CalculatePower 功率计算、
 *                          SC_RunProtection 保护检测、
 *                          SC_Step 状态机与电流闭环、
 *                          SC_HandleFault 故障处理）
 *   - adc_reader.c        （ADC_Reader_Init、ADC_Reader_Start、ADC_Reader_GetLatest）
 *
 * SC_LIMIT() 工具函数为 supercap_control.h 中的静态内联函数。
 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  if (__HAL_RCC_TIM1_IS_CLK_ENABLED()) {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
  }
  if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
    HAL_GPIO_WritePin(CapDownCtrl_GPIO_Port, CapDownCtrl_Pin, GPIO_PIN_RESET);
  }
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
  /* 可在此添加断言错误记录或调试输出。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
