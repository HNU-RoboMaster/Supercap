/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
FDCAN_TxHeaderTypeDef sFDCAN1_TxHeader;

/* 中断与 TIM1 之间的交接缓存：发布先写数据和时间，最后置新数据标志。 */
typedef struct
{
    volatile bool enable_pending;
    volatile bool control_pending;
    volatile uint16_t budget_w;
    volatile uint8_t robot_alive;
    volatile uint32_t rx_ms;
} can_command_cache_t;

static can_command_cache_t s_command_cache;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 24;
  hfdcan1.Init.NominalTimeSeg2 = 7;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_Command_Init(void)
{
    s_command_cache.enable_pending = false;
    s_command_cache.control_pending = false;
    s_command_cache.budget_w = 0U;
    s_command_cache.robot_alive = 0U;
    s_command_cache.rx_ms = 0U;
}

/**
 * 校验 HAL 帧头与命令类型并解码。
 * 非法帧（错误 ID、扩展帧、远程帧、FD 帧、错误长度、未知类型、非法存活值）
 * 返回 false，调用方不得发布任何更新。保留字节不参与解码。
 */
static bool CAN_Command_Decode(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data,
                               can_command_t *out)
{
    if (header == NULL || data == NULL || out == NULL)
    {
        return false;
    }

    /* 本协议只接受 0x221 的标准经典数据帧，长度必须为 8。 */
    if (header->IdType != FDCAN_STANDARD_ID ||
        header->Identifier != CAN_ID_BUS_TO_SUPERCAP ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->DataLength != FDCAN_DLC_BYTES_8)
    {
        return false;
    }

    out->is_control = false;
    out->enable_request = false;
    out->budget_w = 0U;
    out->robot_alive = 0U;

    switch (data[CAN_COMMAND_BYTE_TYPE])
    {
    case CAN_COMMAND_TYPE_ENABLE:
        /* Enable=0 是合法报文但不新增请求，也没有失能语义。 */
        if (data[CAN_COMMAND_BYTE_ENABLE] == CAN_COMMAND_ENABLE_REQUEST)
        {
            out->enable_request = true;
        }
        return true;

    case CAN_COMMAND_TYPE_CONTROL:
        /* 大于 1 的存活值属于非法编码；0 的联合处理受 B1 阻塞，仅保存数值。 */
        if (data[CAN_COMMAND_BYTE_ALIVE] > CAN_COMMAND_ALIVE_MAX)
        {
            return false;
        }
        out->is_control = true;
        out->budget_w = (uint16_t)((uint16_t)data[CAN_COMMAND_BYTE_BUDGET_LSB] |
                                   ((uint16_t)data[CAN_COMMAND_BYTE_BUDGET_MSB] << 8U));
        out->robot_alive = data[CAN_COMMAND_BYTE_ALIVE];
        return true;

    default:
        /* 未知命令类型不更新任何控制意图。 */
        return false;
    }
}

void CAN_Command_Publish(const can_command_t *command, uint32_t rx_ms)
{
    if (command == NULL)
    {
        return;
    }

    if (command->is_control)
    {
        /* 最新有效完整帧覆盖旧完整帧；不跨帧拼接预算与存活状态。 */
        s_command_cache.budget_w = command->budget_w;
        s_command_cache.robot_alive = command->robot_alive;
        s_command_cache.rx_ms = rx_ms;
        s_command_cache.control_pending = true;
    }
    else if (command->enable_request)
    {
        /* 有效使能请求一直保留到消费，不设置过期时间。 */
        s_command_cache.enable_pending = true;
    }
    else
    {
        /* Enable=0：不新增请求，也不取消已保留的请求。 */
    }
}

void CAN_Command_TakePending(can_command_pending_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->enable_request = s_command_cache.enable_pending;
    out->control_pending = s_command_cache.control_pending;
    out->budget_w = s_command_cache.budget_w;
    out->robot_alive = s_command_cache.robot_alive;
    out->rx_ms = s_command_cache.rx_ms;

    /* 复制与消费标志清除属于同一次交接；调用方的临界区保证原子性。 */
    s_command_cache.enable_pending = false;
    s_command_cache.control_pending = false;
}

/**
 * 过滤器初始化函数
 *
 * HAL_FDCAN_ConfigGlobalFilter() 只在 READY 状态有效，因此必须在
 * HAL_FDCAN_Start() 之前调用；接收通知由 main() 在启动后单独启用。
 */
void FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig; // 设为局部变量即可

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 标准CANID
    sFilterConfig.FilterIndex  = 0;                       // 过滤器序列0
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;       // 双ID过滤模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 存入 Rx FIFO0
    sFilterConfig.FilterID1    = CAN_ID_BUS_TO_SUPERCAP;  // 0x221
    sFilterConfig.FilterID2    = CAN_TEST_ID;             // 0x222

    // 配置过滤器
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // 全局过滤器配置：拒绝未匹配的帧
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

/* FDCAN 的 RX FIFO0 固定为 3 个元素（HAL 的 SRAMCAN_RF0_NBR）；
 * 单个接收中断最多消费 3 帧即可取空，保持有界。 */
#define CAN_COMMAND_RX_BATCH_MAX   (3U)
/* FDCAN 最大 DLC 为 64 字节；先按容量读取，再校验长度，避免复制溢出。 */
#define CAN_RX_BUFFER_BYTES        (64U)

/**
 * fdcan接收回调函数
 *
 * 只做帧头校验、命令解码和短临界区发布；不修改状态机、PI、PWM 或 Q3。
 * 帧解析规则见 fdcan.h；测试帧 0x222 与非法帧不会更新控制参数。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t rx_data[CAN_RX_BUFFER_BYTES];
    FDCAN_RxHeaderTypeDef rx_header;
    can_command_t command;

    (void)RxFifo0ITs;

    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    for (uint32_t i = 0U; i < CAN_COMMAND_RX_BATCH_MAX; ++i)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        /* 解码在临界区外完成；帧头与字段校验都在解码函数内。 */
        if (!CAN_Command_Decode(&rx_header, rx_data, &command))
        {
            /* 测试帧、未知类型、错误长度等不更新命令，也不刷新控制有效期。 */
            continue;
        }

        /* 短临界区发布：保存并恢复进入前的中断屏蔽状态。 */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        CAN_Command_Publish(&command, HAL_GetTick());
        __set_PRIMASK(primask);
    }
}

/* 总功率按 0.01 W/LSB 量化；负值或无效值按 0 处理，上界钳位且不回绕。 */
static uint16_t feedback_encode_power(float total_power)
{
    float scaled = total_power * CAN_FEEDBACK_POWER_SCALE;
    if (!(scaled > 0.0f))
    {
        return 0U;
    }
    scaled += 0.5f;
    if (scaled >= (float)CAN_FEEDBACK_POWER_MAX)
    {
        return (uint16_t)CAN_FEEDBACK_POWER_MAX;
    }
    return (uint16_t)scaled;
}

/* 8～25 V 窗口内的储能百分比：100 × (Vcap² − 8²) / (25² − 8²)，就近取整。 */
static uint8_t feedback_encode_energy(float supercap_voltage)
{
    if (supercap_voltage <= CAN_FEEDBACK_ENERGY_MIN_VOLTAGE)
    {
        return 0U;
    }
    if (supercap_voltage >= CAN_FEEDBACK_ENERGY_MAX_VOLTAGE)
    {
        return 100U;
    }
    float ratio = 100.0f *
                  (supercap_voltage * supercap_voltage -
                   CAN_FEEDBACK_ENERGY_MIN_VOLTAGE * CAN_FEEDBACK_ENERGY_MIN_VOLTAGE) /
                  (CAN_FEEDBACK_ENERGY_MAX_VOLTAGE * CAN_FEEDBACK_ENERGY_MAX_VOLTAGE -
                   CAN_FEEDBACK_ENERGY_MIN_VOLTAGE * CAN_FEEDBACK_ENERGY_MIN_VOLTAGE);
    ratio += 0.5f;
    if (ratio >= 100.0f)
    {
        return 100U;
    }
    if (ratio <= 0.0f)
    {
        return 0U;
    }
    return (uint8_t)ratio;
}

/**
 * FDCAN发送函数：组装 0x220 反馈帧，标准帧，DLC 8，逐字节写入。
 * 队列暂满时立即返回 HAL_BUSY，由调用方在下一周期重新生成。
 */
HAL_StatusTypeDef Can_SendMess(const CAN_TransmitDataTypeDef *TX_temp)
{
    static uint8_t FirstSend = 1; // 设为局部静态变量更内聚

    if (TX_temp == NULL)
    {
        return HAL_ERROR;
    }

    if (FirstSend) {
        sFDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        sFDCAN1_TxHeader.FDFormat      = FDCAN_CLASSIC_CAN; // 经典CAN，满足8字节限制
        sFDCAN1_TxHeader.Identifier    = CAN_ID_SUPERCAP_TO_BUS; // 0x220
        sFDCAN1_TxHeader.IdType        = FDCAN_STANDARD_ID;
        sFDCAN1_TxHeader.DataLength    = FDCAN_DLC_BYTES_8;
        sFDCAN1_TxHeader.TxFrameType   = FDCAN_DATA_FRAME;

        FirstSend                      = 0;
    }

    /* 先清零整个缓冲，再填入有效字段；保留字节保持 0。
     * 逐字节编码，避免依赖枚举大小、结构体填充和 CPU 字节序。 */
    uint8_t CAN_TX_BUFF[CAN_FEEDBACK_DLC] = {0};
    uint16_t total_power = feedback_encode_power(TX_temp->TotalPower);

    CAN_TX_BUFF[CAN_FEEDBACK_BYTE_RUN_ENABLE] =
        (TX_temp->RunEnable == 1U) ? 1U : 0U;
    CAN_TX_BUFF[CAN_FEEDBACK_BYTE_STATE_FLAGS] =
        TX_temp->StateFlags & (uint8_t)~CAN_FEEDBACK_FLAGS_RESERVED;
    CAN_TX_BUFF[CAN_FEEDBACK_BYTE_POWER_LSB] = (uint8_t)(total_power & 0xFFU);
    CAN_TX_BUFF[CAN_FEEDBACK_BYTE_POWER_MSB] = (uint8_t)((total_power >> 8U) & 0xFFU);
    CAN_TX_BUFF[CAN_FEEDBACK_BYTE_ENERGY_PCT] =
        feedback_encode_energy(TX_temp->SuperCapVoltage);

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        // 队列暂满：结束本次发送，下一周期重新生成最新反馈
        return HAL_BUSY;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sFDCAN1_TxHeader, CAN_TX_BUFF);
}
/* USER CODE END 1 */
