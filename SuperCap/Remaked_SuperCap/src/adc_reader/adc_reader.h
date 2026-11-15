/**
 * @file    adc_reader.h
 * @brief   电压电流检测模块 — 基于双缓冲 DMA 的 ADC 读取（适用于 STM32G4）
 *
 * 使用 10 元素的环形 DMA 缓冲区分成两半（A 和 B）。
 * 当 DMA 到达半传输点（5 个元素）时，HT 中断表示 buffer[0..4] 已完成采集；
 * 当 DMA 到达全传输点（10 个元素）时，TC 中断表示 buffer[5..9] 已完成采集。
 *
 * 控制 ISR 调用 ADC_Reader_GetLatest() 获取最近完成的半缓冲区的快照，
 * 该快照保证不会被正在进行的 DMA 传输覆盖——从而消除 DMA 与 ISR 之间的数据撕裂。
 */

#ifndef ADC_READER_H
#define ADC_READER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
   双缓冲常量定义
   ======================================================================== */

/** DMA 缓冲区总大小（2 个半区 × 5 个 ADC 通道） */
#define ADC_BUFFER_SIZE         10

/** 每个半缓冲区的 ADC 通道数 */
#define ADC_HALF_SIZE           5

/* ========================================================================
   半缓冲区选择值
   ======================================================================== */

#define ADC_HALF_NONE           0   /**< 无新数据可用 */
#define ADC_HALF_A              1   /**< buffer[0..4] 最新（HT 已触发） */
#define ADC_HALF_B              2   /**< buffer[5..9] 最新（TC 已触发） */

/* ========================================================================
   ADC 读取器实例
   ======================================================================== */

typedef struct {
    volatile uint16_t buffer[ADC_BUFFER_SIZE];  /**< DMA 环形目标缓冲区 */
    volatile uint8_t  ready_half;               /**< 标识哪个半区已完成 */
    volatile bool     dma_running;               /**< DMA 运行标志 */
    uint16_t          snapshot[ADC_HALF_SIZE];  /**< 给 ISR 使用的本地副本 */
} adc_reader_t;

/** 全局 ADC 读取器实例 */
extern adc_reader_t adc_reader;

/* ========================================================================
   公开 API
   ======================================================================== */

/**
 * @brief  初始化 / 启动指定 ADC 的双缓冲 DMA 采集。
 * @param  hadc  指向 ADC 句柄的指针（例如 &hadc1）
 */
void ADC_Reader_Init(ADC_HandleTypeDef *hadc);
void ADC_Reader_Start(ADC_HandleTypeDef *hadc);

/**
 * @brief  获取最近完成的半缓冲区的指针。
 *
 * 在控制 ISR 中调用。返回的指针指向内部 `snapshot[]` 数组，
 * 该数组是已完成半缓冲的本地副本——DMA 不会覆盖此副本。
 *
 * @param  which_half  [输出] 被设为 ADC_HALF_A、ADC_HALF_B 或 ADC_HALF_NONE
 * @return 指向 5 个 uint16_t ADC 原始值的指针，无新数据时返回 NULL
 */
const uint16_t * ADC_Reader_GetLatest(uint8_t *which_half);

/* ========================================================================
   HAL 回调钩子（在 adc.c 的 USER CODE 中调用）
   ======================================================================== */

void ADC_Reader_HalfCpltCallback(ADC_HandleTypeDef *hadc);
void ADC_Reader_FullCpltCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* ADC_READER_H */
