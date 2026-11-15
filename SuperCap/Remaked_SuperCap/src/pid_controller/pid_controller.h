/**
 * @file    pid_controller.h
 * @brief   PID 控制器模块 — 独立、可复用的 PID 驱动
 *
 * 功能特性：
 *   - 比例 + 积分 + 微分（KD 预留，暂未使用）
 *   - 独立的积分抗饱和限制（与输出限幅分离）
 *   - 支持多控制器实例，零堆内存开销（静态分配）
 *   - 提供复位和运行时参数整定 API
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
   PID 控制器数据结构
   ======================================================================== */

typedef struct {
    /* 整定参数 */
    float kp;              /* 比例增益 */
    float ki;              /* 积分增益 */
    float kd;              /* 微分增益（预留，设为 0） */
    float ts;              /* 采样时间，单位：秒 */

    /* 内部状态 */
    float integral;        /* 积分累加项 */
    float prev_error;      /* 上一次误差（用于微分计算） */

    /* 输出限幅 */
    float out_min;         /* 输出下限 */
    float out_max;         /* 输出上限 */

    /* 积分抗饱和限幅（可与输出限幅不同） */
    float integral_min;    /* 积分下限钳位 */
    float integral_max;    /* 积分上限钳位 */

    /* 积分分离：|error| > 此阈值时冻结积分累加。0.0 = 禁用（默认） */
    float i_threshold;

    /* 输出饱和抗饱和：输出已达限幅且误差方向继续推着超限时冻结积分。
     * true = 启用，false = 禁用（默认，向后兼容） */
    bool anti_windup_enable;
} PID_Controller;

/* ========================================================================
   公开 API
   ======================================================================== */

/**
 * @brief  使用整定参数和限幅初始化 PID 控制器。
 *         积分限幅默认与输出限幅一致。
 * @param  pid      指向 PID_Controller 实例的指针
 * @param  kp       比例增益
 * @param  ki       积分增益
 * @param  kd       微分增益（不使用则设为 0.0f）
 * @param  ts       采样时间，单位：秒
 * @param  out_min  输出下限钳位值
 * @param  out_max  输出上限钳位值
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float ts, float out_min, float out_max);

/**
 * @brief  执行一次 PID 计算周期。
 * @param  pid       指向 PID_Controller 实例的指针
 * @param  setpoint  期望目标值
 * @param  feedback  实际测量值
 * @return 经钳位后的输出值
 */
float PID_Calculate(PID_Controller *pid, float setpoint, float feedback);

/**
 * @brief  将内部状态（积分项和上一次误差）复位清零。
 * @param  pid  指向 PID_Controller 实例的指针
 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief  运行时更新整定增益。
 * @param  pid  指向 PID_Controller 实例的指针
 * @param  kp   新的比例增益
 * @param  ki   新的积分增益
 * @param  kd   新的微分增益
 */
void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/**
 * @brief  运行时更新输出钳位限幅。
 * @param  pid      指向 PID_Controller 实例的指针
 * @param  out_min  新的输出下限
 * @param  out_max  新的输出上限
 */
void PID_SetOutputLimits(PID_Controller *pid, float out_min, float out_max);

/**
 * @brief  独立于输出限幅，更新积分抗饱和限幅。
 * @param  pid           指向 PID_Controller 实例的指针
 * @param  integral_min  新的积分下限钳位
 * @param  integral_max  新的积分上限钳位
 */
void PID_SetIntegralLimits(PID_Controller *pid, float integral_min, float integral_max);

/**
 * @brief  设置积分分离阈值。|error| > threshold 时冻结积分累加。
 *         设为 0.0 可禁用积分分离（默认值）。
 * @param  pid       指向 PID_Controller 实例的指针
 * @param  threshold 积分分离阈值（误差绝对值），0.0 = 禁用
 */
void PID_SetIntegralThreshold(PID_Controller *pid, float threshold);

/**
 * @brief  启用/禁用输出饱和抗饱和。
 *         启用后：当输出已达限幅且误差方向继续推着输出超限时，
 *         冻结积分累加，防止积分绕组。
 * @param  pid    指向 PID_Controller 实例的指针
 * @param  enable true = 启用，false = 禁用
 */
void PID_SetAntiWindup(PID_Controller *pid, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
