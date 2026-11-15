/**
 * @file    pid_controller.c
 * @brief   PID 控制器实现
 *
 * 算法：
 *   输出 = kp×误差 + 积分累加和 + kd×(误差 - 上次误差)/采样时间
 *
 * 抗饱和机制：
 *   积分项先被钳位在 integral_min ~ integral_max 范围内，
 *   再累加到输出中；输出随后被钳位在 out_min ~ out_max 范围内。
 */

#include "pid_controller.h"
#include <stddef.h>
#include <math.h>

/* ========================================================================
   公开 API 实现
   ======================================================================== */

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float ts, float out_min, float out_max)
{
    if (pid == NULL) return;

    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->ts      = ts;

    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    /* 默认：积分限幅与输出限幅一致（向后兼容） */
    pid->integral_min = out_min;
    pid->integral_max = out_max;

    /* 默认：积分分离禁用，抗饱和禁用（向后兼容） */
    pid->i_threshold        = 0.0f;
    pid->anti_windup_enable = false;
}

float PID_Calculate(PID_Controller *pid, float setpoint, float feedback)
{
    if (pid == NULL) return 0.0f;

    /* ---- 计算误差 ---- */
    float error = setpoint - feedback;

    /* ---- 比例项 ---- */
    float p_term = pid->kp * error;

    /* ---- 微分项（预留，kd == 0 时此项为 0） ---- */
    float d_term = 0.0f;
    if (pid->kd != 0.0f && pid->ts > 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / pid->ts;
    }

    /* ---- 判断是否累加积分 ---- */
    bool integrate = true;

    /* 条件 1：积分分离 — 大误差时冻结积分累加 */
    if (pid->i_threshold > 0.0f && fabsf(error) > pid->i_threshold) {
        integrate = false;
    }

    /* 条件 2：输出饱和抗饱和 — 输出已达限幅且误差方向继续推着超限时冻结 */
    if (pid->anti_windup_enable && integrate) {
        float output_raw = p_term + pid->integral + d_term;
        if (output_raw >= pid->out_max && error > 0.0f) integrate = false;
        if (output_raw <= pid->out_min && error < 0.0f) integrate = false;
    }

    /* ---- 积分累加 ---- */
    if (integrate) {
        pid->integral += pid->ki * error * pid->ts;
    }

    /* ---- 积分限幅 ---- */
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    pid->prev_error = error;

    /* ---- 求和并钳位输出 ---- */
    float output = p_term + pid->integral + d_term;

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) return;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(PID_Controller *pid, float out_min, float out_max)
{
    if (pid == NULL) return;
    pid->out_min = out_min;
    pid->out_max = out_max;

    /* 如有必要，将当前积分项钳位到新限幅内 */
    if (pid->integral > out_max) pid->integral = out_max;
    if (pid->integral < out_min) pid->integral = out_min;
}

void PID_SetIntegralLimits(PID_Controller *pid, float integral_min, float integral_max)
{
    if (pid == NULL) return;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;

    /* 将当前积分项钳位到新限幅内 */
    if (pid->integral > integral_max) pid->integral = integral_max;
    if (pid->integral < integral_min) pid->integral = integral_min;
}

void PID_SetIntegralThreshold(PID_Controller *pid, float threshold)
{
    if (pid == NULL) return;
    pid->i_threshold = threshold;
}

void PID_SetAntiWindup(PID_Controller *pid, bool enable)
{
    if (pid == NULL) return;
    pid->anti_windup_enable = enable;
}
