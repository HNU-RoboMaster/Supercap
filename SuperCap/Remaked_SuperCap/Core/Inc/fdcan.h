/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
// 宏定义
#define CAN_ID_SUPERCAP_TO_BUS  0x220
#define CAN_ID_BUS_TO_SUPERCAP  0x221
#define CAN_TEST_ID             0x222

/* 反馈帧 0x220 的字节布局（标准帧，DLC 8，TIM2 周期发送） */
#define CAN_FEEDBACK_DLC                (8U)   /* 反馈帧字节数 */
#define CAN_FEEDBACK_BYTE_RUN_ENABLE    (0U)   /* 字节 0：sc_run_enable 回读 */
#define CAN_FEEDBACK_BYTE_STATE_FLAGS   (1U)   /* 字节 1：状态码与语义位 */
#define CAN_FEEDBACK_BYTE_POWER_LSB     (2U)   /* 字节 2：总功率低字节 */
#define CAN_FEEDBACK_BYTE_POWER_MSB     (3U)   /* 字节 3：总功率高字节 */
#define CAN_FEEDBACK_BYTE_ENERGY_PCT    (4U)   /* 字节 4：储能百分比 */
#define CAN_FEEDBACK_BYTE_BAT_POWER_LSB (5U)   /* 字节 5：电池侧功率低字节 */
#define CAN_FEEDBACK_BYTE_BAT_POWER_MSB (6U)   /* 字节 6：电池侧功率高字节，0.01W/LSB */
/* 字节 7 保留，发送时填 0 */

#define CAN_FEEDBACK_MODE_MASK            (0x0FU) /* 字节 1 低 4 位：sc_mode_t 状态码 */
#define CAN_FEEDBACK_FLAG_NORMAL_RUNNING  (0x10U) /* 字节 1 bit 4：正常运行 */
#define CAN_FEEDBACK_FLAG_SUPPLY_REQUEST  (0x20U) /* 字节 1 bit 5：允许上位机发起供能请求 */
#define CAN_FEEDBACK_FLAGS_RESERVED       (0xC0U) /* 字节 1 bit 6～7：保留 */

/* 总功率量化：1 LSB = 0.01 W，上界 655.35 W，越界钳位且不回绕 */
#define CAN_FEEDBACK_POWER_SCALE          (100.0f)
#define CAN_FEEDBACK_POWER_MAX            (65535U)

/* 储能百分比窗口：8 V 及以下为 0，25 V 及以上为 100 */
#define CAN_FEEDBACK_ENERGY_MIN_VOLTAGE   (8.0f)
#define CAN_FEEDBACK_ENERGY_MAX_VOLTAGE   (25.0f)

/* 下行命令帧 0x221 的报文契约（标准帧、经典数据帧、DLC 8）：
 * 字节 0 为命令类型，0x01 使能、0x02 控制；
 * 使能帧字节 1 为 Enable（1 请求首次使能，0 无失能语义）；
 * 控制帧字节 1 为 RobotAlive（首版恒为 1，0 的处理待讨论），
 * 字节 2～3 为电池侧预算（小端 uint16_t、1 W/LSB），字节 4～7 保留并忽略。 */
#define CAN_COMMAND_DLC                 (8U)
#define CAN_COMMAND_BYTE_TYPE           (0U)
#define CAN_COMMAND_BYTE_ENABLE         (1U)
#define CAN_COMMAND_BYTE_ALIVE          (1U)
#define CAN_COMMAND_BYTE_BUDGET_LSB     (2U)
#define CAN_COMMAND_BYTE_BUDGET_MSB     (3U)

#define CAN_COMMAND_TYPE_ENABLE         (0x01U)
#define CAN_COMMAND_TYPE_CONTROL        (0x02U)
#define CAN_COMMAND_ENABLE_REQUEST      (1U)
#define CAN_COMMAND_ALIVE_MAX           (1U)
/* 预算换算：budget_w = (float)raw_budget × 1 W/LSB，不额外限制取值。 */
#define CAN_COMMAND_BUDGET_SCALE        (1.0f)

/* 一帧通过校验后的命令内容：使能请求与控制参数各用各的字段。 */
typedef struct
{
  bool is_control;      /* true：控制命令(0x02)；false：使能命令(0x01) */
  bool enable_request;  /* 使能命令且 Enable=1；控制命令恒为 false */
  uint16_t budget_w;    /* 控制命令的电池侧预算，1 W/LSB */
  uint8_t robot_alive;  /* 控制命令的存活标志 */
} can_command_t;

/* 待应用命令的本地交接视图；接收时间属于本地元数据，不占业务字节。 */
typedef struct
{
  bool enable_request;   /* 有待消费的首次使能请求 */
  bool control_pending;  /* 有新的完整控制参数 */
  uint16_t budget_w;     /* 最新完整控制帧的预算 */
  uint8_t robot_alive;   /* 同一控制帧的存活标志 */
  uint32_t rx_ms;        /* 同一控制帧的本地接收时间 */
} can_command_pending_t;

/** 初始化命令缓存；须在接收中断使能前调用。 */
void CAN_Command_Init(void);
/**
 * 发布一帧已解码命令；调用方（FDCAN 接收回调）负责用短临界区包住本函数。
 * 控制帧覆盖旧控制参数；使能请求保留到消费，Enable=0 不产生也不取消请求。
 */
void CAN_Command_Publish(const can_command_t *command, uint32_t rx_ms);
/**
 * 取走待处理命令并清除对应标志；调用方（TIM1 控制周期）负责用短临界区包住本函数。
 * rx_ms 只在 control_pending 为 true 时有效。
 */
void CAN_Command_TakePending(can_command_pending_t *out);

// 发送结构体 (发给电控，0x220，DLC 8)
typedef struct {
  uint8_t RunEnable;            // 字节 0：首次使能确认，回读 sc_run_enable
  uint8_t StateFlags;           // 字节 1：bit0-3 状态码，bit4 正常运行，bit5 允许供能请求
  float TotalPower;             // 字节 2-3：Pbat + Pcap，单位 W，编码为 0.01 W/LSB
  float BatteryPower;           // 字节 5-6：电池侧功率 Pbat，单位 W，编码为 0.01 W/LSB
  float SuperCapVoltage;        // 字节 4：电容组电压，用于 8~25 V 储能百分比
} CAN_TransmitDataTypeDef;

// 暴露给外部的函数
void FDCAN_Filter_Init(void);

/**
 * 组装并发送 0x220 反馈帧：逐字节编码，DLC 固定 8，保留字节填 0。
 * 队列暂满时返回 HAL_BUSY，由调用方放弃本次发送。
 */
HAL_StatusTypeDef Can_SendMess(const CAN_TransmitDataTypeDef *TX_temp);
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

