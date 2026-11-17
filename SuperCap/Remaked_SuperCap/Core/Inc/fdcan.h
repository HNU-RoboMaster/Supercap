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
/* 字节 5～7 保留，发送时填 0 */

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

// 接收结构体 (电控发来，13字节拆分)
typedef struct {
  uint8_t Charge ;              // 超级电容充电。1充电，0放电
  uint8_t Enable ;              // 超级电容给使能。1使能，0失能
  uint8_t PowerLimit;           // 裁判系统功率限制
  uint8_t ChargePower;          // 超级电容充电功率
  uint8_t Dead;                 // 机器人死亡状态
  float PowerLimitAfterOffset;  // 本地计算的补偿限制
  float ChargePowerAfterOffset; // 本地计算的补偿功率
} CAN_ReceiveDataTypeDef;

// 发送结构体 (发给电控，0x220，DLC 8)
typedef struct {
  uint8_t RunEnable;            // 字节 0：首次使能确认，回读 sc_run_enable
  uint8_t StateFlags;           // 字节 1：bit0-3 状态码，bit4 正常运行，bit5 允许供能请求
  float TotalPower;             // 字节 2-3：Pbat + Pcap，单位 W，编码为 0.01 W/LSB
  float SuperCapVoltage;        // 字节 4：电容组电压，用于 8~25 V 储能百分比
} CAN_TransmitDataTypeDef;

// 暴露给外部的全局变量和函数
extern CAN_ReceiveDataTypeDef sCAN_RX_data;
extern uint8_t CAN_ReceiveDataRefresh_Flag;
extern uint8_t CAN_WDG_Count;

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

