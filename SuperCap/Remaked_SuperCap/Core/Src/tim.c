/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM1/TIM2 配置及控制中断回调。
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "fdcan.h"
#include "supercap_control.h"
#include "adc_reader.h"
#include "gpio.h"
/* 所有系统状态通过 sc_state 和 adc_reader 访问 — 无裸 extern 变量。 */
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 8;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PB1     ------> TIM1_CH3N
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
 * TIM1：200 kHz 更新中断，每两个中断运行一次 100 kHz 控制循环
 *   获取新 ADC 快照 → 功率与保护 → 状态机/电流 PI → PWM 与 CapDownCtrl 输出
 *
 * TIM2：CAN 反馈发送（2 ms / 500 Hz）
 *   复制板端反馈快照，编码后发送至主控制器。
 */

int control_hz=0;
static uint8_t tim1_pwm_preloaded = 0;
volatile int32_t check = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
      control_hz++;
      if (control_hz%2==0)
      {
        control_hz=0;
        /* ================================================================
 * TIM1：200 kHz 更新中断，每两个中断运行一次 100 kHz 控制循环
 * ================================================================ */
// DEBUG_MODE 不接入CAN通信控制
#if !SC_DEBUG_MODE
        /* ---- 0. 取走 CAN 下行命令并在控制上下文应用 ---- */
        can_command_pending_t can_pending;
        uint32_t can_primask = __get_PRIMASK();
        __disable_irq();
        CAN_Command_TakePending(&can_pending);
        __set_PRIMASK(can_primask);

        uint32_t now_ms = HAL_GetTick();
        if (can_pending.control_pending)
        {
            /* 先应用本周期到达的有效控制帧，再由超时判断兜底。 */
            SC_ApplyControlCommand(&sc_state,
                                   (float)can_pending.budget_w * CAN_COMMAND_BUDGET_SCALE,
                                   can_pending.robot_alive, can_pending.rx_ms, now_ms);
        }
        if (can_pending.enable_request)
        {
            /* 首次使能只置位运行开关；启动与自检仍由状态机负责。 */
            SC_ApplyEnableRequest(&sc_state, now_ms);
        }
        SC_AdvanceControlTimeout(&sc_state, now_ms);
#endif

        /* ---- 1. 获取最新 ADC 快照（双缓冲，无数据撕裂） ---- */
        uint8_t adc_half;
        const uint16_t *adc_raw = ADC_Reader_GetLatest(&adc_half);

        /* 保护仅对新采样运行；没有新样本时累计离线时间，状态机仍每周期执行。 */
        if (adc_raw != NULL)
        {
          sc_state.adc_offline_count = 0;
          SC_ADC_ConvertToPhysical(&sc_state, adc_raw);

          SC_CalculatePower(&sc_state);
// 宏SC_DEBUG_MODE == 0的时候才开启编译
#if !SC_DEBUG_MODE
          SC_RunProtection(&sc_state);
#endif
        }
        // 计算ADC采样掉线时长
        else if (sc_state.adc_offline_count < ADC_OFFLINE_LIMIT)
        {
          ++sc_state.adc_offline_count;
        }
           
#if SC_DEBUG_MODE
           /* 硬件调试：保留 ADC 采样和功率计算，跳过状态机、PI 与软件保护。
            * 首次写 CCR3 后保持输出关闭，下一控制周期再开 MOE 并接入 Q3。
            * 两次控制周期之间已有 TIM1 更新事件，预装载占空比已生效。 */
           __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, SC_DEBUG_PWM_DUTY);

           // 间隔一个TIM中断周期再开启PWM输出，保证CCR更新
           if (tim1_pwm_preloaded)
           {
             TIM1->BDTR |= TIM_BDTR_MOE;  // 开启MOE
             HAL_GPIO_WritePin(CapDownCtrl_GPIO_Port, CapDownCtrl_Pin, GPIO_PIN_SET);
           }
           else
           {
             TIM1->BDTR &= ~TIM_BDTR_MOE;
             HAL_GPIO_WritePin(CapDownCtrl_GPIO_Port, CapDownCtrl_Pin, GPIO_PIN_RESET);
             tim1_pwm_preloaded = 1;
           }

#else
           SC_Step(&sc_state);
           /* SC_CONNECT_Q3 先拉高 PA4、保持 MOE=0；SC_START_PWM 才请求开放。
            * PWM 只由状态机请求开放。关断时先清 MOE，避免 CCR=0
            * 使互补通道在 MOE 仍有效的瞬间变为常高。 */
           if (!SC_PWM_OutputRequested(&sc_state))
           {
             TIM1->BDTR &= ~TIM_BDTR_MOE;
             __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
             tim1_pwm_preloaded = 0;
           }
           else
           {
             __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, sc_state.applied_duty);
             /* CCR3 使用preload-then-enable：等待一次更新事件使新占空比生效，
              * 再开启 MOE，避免首次打开时仍输出旧的 CCR=0。 */
             if (tim1_pwm_preloaded)
             {
               TIM1->BDTR |= TIM_BDTR_MOE;
             }
             else
             {
               tim1_pwm_preloaded = 1;
             }
           }
           HAL_GPIO_WritePin(CapDownCtrl_GPIO_Port, CapDownCtrl_Pin,
                             sc_state.cap_down_ctrl ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
      }
    }
    else if (htim->Instance == TIM2) {
        /* ================================================================
         * TIM2：CAN 反馈发送（标称 2 ms / 500 Hz）
         * ================================================================ */

        sc_feedback_snapshot_t snapshot;
        CAN_TransmitDataTypeDef tx_msg_temp;

        /* TIM1 可抢占本中断：共享字段在短临界区内复制，并恢复进入前的中断状态。 */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        SC_GetFeedbackSnapshot(&snapshot);
        __set_PRIMASK(primask);

        /* 字节 1：状态码与两个语义位；字节组装和 FDCAN 入队在 fdcan.c 完成。 */
        tx_msg_temp.RunEnable = (snapshot.run_enable == 1U) ? 1U : 0U;
        tx_msg_temp.StateFlags = (uint8_t)snapshot.mode & CAN_FEEDBACK_MODE_MASK;
        if (SC_FeedbackNormalRunning(&snapshot))
        {
            tx_msg_temp.StateFlags |= CAN_FEEDBACK_FLAG_NORMAL_RUNNING;
        }
        if (SC_FeedbackSupplyRequestAllowed(&snapshot))
        {
            tx_msg_temp.StateFlags |= CAN_FEEDBACK_FLAG_SUPPLY_REQUEST;
        }
        tx_msg_temp.TotalPower = snapshot.battery_power + snapshot.supercap_power;
        tx_msg_temp.SuperCapVoltage = snapshot.supercap_voltage;

        if (Can_SendMess(&tx_msg_temp) != HAL_OK)
        {
          check++;
            /* 入队失败（如发送队列暂满）：结束本次发送，
             * 下一周期重新生成并发送最新反馈。 */
            return;
        }
    }
}
/* USER CODE END 1 */
