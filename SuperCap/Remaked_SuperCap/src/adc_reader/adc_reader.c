/**
 * @file    adc_reader.c
 * @brief   电压电流双缓冲 ADC 读取实现
 *
 * DMA 配置为环形模式，使用 10 元素缓冲区：
 * - 半传输（HT）在第 [4] 个元素后触发 → buffer[0..4] 为最新数据
 * - 全传输（TC）在第 [9] 个元素后触发 → buffer[5..9] 为最新数据
 *
 * 控制 ISR 调用 ADC_Reader_GetLatest() 将已完成的半区通过 memcpy
 * 复制到本地快照中。读取快照是安全的，与此同时 DMA 继续写入另一半区。
 */

#include "adc_reader.h"
#include <string.h>

/* ========================================================================
   全局实例
   ======================================================================== */

adc_reader_t adc_reader = {0};

/* ========================================================================
   公开 API
   ======================================================================== */

void ADC_Reader_Init(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    /* 清零缓冲区和状态 */
    memset((void *)adc_reader.buffer, 0, sizeof(adc_reader.buffer));
    memset(adc_reader.snapshot, 0, sizeof(adc_reader.snapshot));
    adc_reader.ready_half  = ADC_HALF_NONE;
    adc_reader.dma_running = false;
}

void ADC_Reader_Start(ADC_HandleTypeDef *hadc)
{
    adc_reader.ready_half  = ADC_HALF_NONE;
    adc_reader.dma_running = true;

    /* 以环形模式启动 DMA，使用 10 元素双缓冲。
     * HAL 将在 5 次传输后触发 ConvHalfCpltCallback（HT），
     * 在 10 次传输后触发 ConvCpltCallback（TC）。 */
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_reader.buffer, ADC_BUFFER_SIZE);

    /* 显式使能半传输中断。
     * （HAL_ADC_Start_DMA 使能了 TC，但 HT 可能需要显式使能。） */
    __HAL_DMA_ENABLE_IT(hadc->DMA_Handle, DMA_IT_HT);
}

const uint16_t * ADC_Reader_GetLatest(uint8_t *which_half)
{
    uint8_t half = adc_reader.ready_half;
    *which_half = half;

    if (half == ADC_HALF_A) {
        /* 立即消费半区 A */
        adc_reader.ready_half = ADC_HALF_NONE;
        /* 复制 buffer[0..4] → snapshot（安全，DMA 正在写入 buffer[5..9]） */
        memcpy(adc_reader.snapshot, (const void *)&adc_reader.buffer[0],
               ADC_HALF_SIZE * sizeof(uint16_t));
        return adc_reader.snapshot;
    }
    else if (half == ADC_HALF_B) {
        /* 立即消费半区 B */
        adc_reader.ready_half = ADC_HALF_NONE;
        /* 复制 buffer[5..9] → snapshot（安全，DMA 正在写入 buffer[0..4]） */
        memcpy(adc_reader.snapshot, (const void *)&adc_reader.buffer[5],
               ADC_HALF_SIZE * sizeof(uint16_t));
        return adc_reader.snapshot;
    }

    /* 无新数据 */
    return NULL;
}

/* ========================================================================
   HAL 回调钩子
   ======================================================================== */

/**
 * @brief  DMA 半传输完成 → buffer[0..4] 为最新数据。
 *         由 adc.c 中的 HAL_ADC_ConvHalfCpltCallback 调用。
 */
void ADC_Reader_HalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    adc_reader.ready_half = ADC_HALF_A;
}

/**
 * @brief  DMA 全传输完成 → buffer[5..9] 为最新数据。
 *         由 adc.c 中的 HAL_ADC_ConvCpltCallback 调用。
 */
void ADC_Reader_FullCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    adc_reader.ready_half = ADC_HALF_B;
}
