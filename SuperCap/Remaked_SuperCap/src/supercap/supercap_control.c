/**
 * @file supercap_control.c
 * @brief 超级电容状态机与控制计算；硬件寄存器只在 TIM1 中断中统一写入。
 */
#include "supercap_control.h"
#include <math.h>
#include <stddef.h>

// 全局实例结构体
supercap_state_t sc_state = {0};
// 使能标志位
volatile uint8_t sc_run_enable = 0U;

/* 线上状态码直接取枚举序号：0 = SC_SELF_TEST … 9 = SC_START_PWM。 */
_Static_assert(SC_SELF_TEST == 0 && SC_START_PWM == 9,
               "sc_mode_t order must match the CAN feedback state codes");

// 初始化
static void reset_power_control(supercap_state_t *s)
{
    PID_Reset(&s->pi_battery_power);
    s->power_trim = 0.0f;
    s->cap_power_request = 0.0f;
    s->power_loop_ticks = 0;
}

/* 换向、正常停止或复位时清除 PI 积分与历史电流目标，避免再次接入时突跳。 */
static void reset_control(supercap_state_t *s)
{
    PID_Reset(&s->pi_current);
    s->target_current = 0.0f;
    s->requested_current = 0.0f;
    s->iref_feedforward = 0.0f;
    s->zero_ticks = 0;
}

/* 统一处理状态入口：清计时；接入前保存基线；过零和停机入口将目标置零。 */
static void enter(supercap_state_t *s, sc_mode_t mode)
{
    s->mode = mode;
    s->mode_ticks = 0;
    s->zero_ticks = 0;
    if (mode == SC_SELF_TEST) 
    {
        s->self_test_stable_ticks = 0;
    }

    if (mode == SC_FAULT_LATCHED)
    {
        s->fault_recovery_ticks = 0;
        s->reset_requested = false;
    }
    /* 反馈用停机原因保留到其后的待机；重新进入工作流程时清除。 */
    if (mode == SC_SELF_TEST || mode == SC_CONNECT_Q3)
    {
        s->stop_reason = SC_STOP_REASON_NONE;
    }
    if (mode == SC_CONNECT_Q3) 
    {
        /* PA4 接通前的测量值，用于识别接入瞬间的异常跳变。 */
        s->connect_vbat = s->adc_values.battery_voltage;
        s->connect_vcap = s->adc_values.supercap_voltage;
        // s->connect_icap = s->adc_values.supercap_current;
        PID_Reset(&s->pi_current);
    }
    if (mode == SC_CROSS_ZERO || mode == SC_STOPPING) 
    {
        s->requested_current = 0.0f;
    }
}

/* 每个控制周期限制目标电流变化，防止电流指令阶跃。 */
static float slew(float previous, float desired)
{
    return previous + SC_LIMIT(desired - previous,
                               -SC_CURRENT_SLEW_A, SC_CURRENT_SLEW_A);
}

/* 目标和实测电流必须连续处于零附近，不能凭单次采样换向或断开 Q3。 */
static bool zero_confirmed(supercap_state_t *s)
{
    if (fabsf(s->target_current) < SC_CURRENT_ZERO_A &&
        fabsf(s->adc_values.supercap_current) < SC_CURRENT_ZERO_A) {
        if (s->zero_ticks < SC_ZERO_STABLE_TICKS) ++s->zero_ticks;
    } else {
        s->zero_ticks = 0;
    }
    return s->zero_ticks >= SC_ZERO_STABLE_TICKS;
}

/* 低压仅禁止放电电流；AUTO 保留恢复回差，避免电压回弹后再次放电。 */
static bool discharge_voltage_ready(const supercap_state_t *s)
{
    return s->adc_values.supercap_voltage > SC_VCAP_DISCHARGE_MIN &&
           (s->request != SC_REQUEST_AUTO || s->auto_discharge_ready);
}

/* AUTO 按功率判定，手动放电仍保留原负载电流门限。 */
static bool discharge_allowed(const supercap_state_t *s, float threshold)
{
    return s->active_request == SC_REQUEST_DISCHARGE &&
           discharge_voltage_ready(s) &&
           s->adc_values.battery_voltage >= SC_VBAT_MIN &&
           (s->request == SC_REQUEST_AUTO || s->load_current > threshold);
}

/* 用有符号功率目标选方向；不在放电侧设置容许超出参考上限的死区。 */
static void select_request(supercap_state_t *s)
{
    float vcap = s->adc_values.supercap_voltage;
    if (vcap <= SC_VCAP_DISCHARGE_MIN) s->auto_discharge_ready = false;
    else if (vcap >= SC_AUTO_DISCHARGE_RESUME_VOLTAGE) s->auto_discharge_ready = true;
    if (vcap >= SC_CHARGE_DONE_VOLTAGE) s->auto_charge_full = true;
    else if (vcap <= SC_AUTO_CHARGE_RESTART_VOLTAGE) s->auto_charge_full = false;

    if (s->request != SC_REQUEST_AUTO) {
        s->active_request = s->request;
        return;
    }
    if (!s->adc_valid || s->adc_values.battery_voltage < SC_VBAT_MIN) {
        s->active_request = SC_REQUEST_STOP;
        return;
    }
    if (s->cap_power_request > 0.0f)
        s->active_request = SC_REQUEST_DISCHARGE;
    else if (s->cap_power_request < -SC_AUTO_CHARGE_HYST_W)
        s->active_request = SC_REQUEST_CHARGE;

    /* 低压时保留放电方向，由电流目标限为 0，避免已接入的功率级停机。 */
    if (s->active_request == SC_REQUEST_CHARGE && s->auto_charge_full)
        s->active_request = SC_REQUEST_STOP;
}

/* 7V 以下采用慢充状态，达到 7V 后切换为正常充电。 */
// 根据电压判断现在应采用的充电模式
static sc_mode_t charge_mode(const supercap_state_t *s)
{
    return s->adc_values.supercap_voltage < SUPERCAP_VOLTAGE_CC ?
           SC_SLOW_CHARGE : SC_CHARGE;
}

/* 与功率目标独立的充电能力限制，供电流目标和功率环抗积分饱和使用。 */
static float charge_capacity(const supercap_state_t *s)
{
    float vcap = fmaxf(s->adc_values.supercap_voltage, 1.0f);
    float vbat = s->adc_values.battery_voltage;
    float battery_headroom = fmaxf(0.0f, BATTERY_CURRENT_MAX -
                                  SC_CHARGE_BAT_MARGIN_A - s->load_current);
    float permitted = fminf(battery_headroom * vbat / vcap, -ICAP_CHARGE_MAX);
    if (charge_mode(s) == SC_SLOW_CHARGE) {
        /* 低压时不按 P/VCAP 无限增大电流，只允许恒流慢充。 */
        permitted = fminf(permitted, -ICAP_CONSTANT_CURRENT);
    } else if (s->adc_values.supercap_voltage > SC_VCAP_CHARGE_TAPER) {
        /* 接近充满电压时线性收小充电电流。 */
        float span = SC_CHARGE_DONE_VOLTAGE - SC_VCAP_CHARGE_TAPER;
        permitted *= SC_LIMIT((SC_CHARGE_DONE_VOLTAGE - s->adc_values.supercap_voltage)
                              / span, 0.0f, 1.0f);
    }
    return permitted;
}

/* 快速功率差额前馈 + 1 kHz 电池功率反馈，再由原有电流环执行。
 * 功率前馈换向时清除旧积分，避免旧方向修正妨碍新的负载支援。
 * 换向、接入、停机期间冻结积分；电容能力饱和时禁止向饱和方向积分。 */
static void update_power_control(supercap_state_t *s)
{
    if (s->request != SC_REQUEST_AUTO) reset_power_control(s);
    s->battery_power_target = fmaxf(0.0f, s->power_reference - SC_BATTERY_POWER_RESERVE_W);
    s->battery_power_error = s->battery_power - s->battery_power_target;
    s->power_limit_exceeded = s->adc_valid && s->battery_power > s->power_reference;
    float feedforward = s->load_power - s->battery_power_target;
    if ((feedforward > 0.0f && s->iref_feedforward < 0.0f) ||
        (feedforward < 0.0f && s->iref_feedforward > 0.0f)) {
        PID_Reset(&s->pi_battery_power);
        s->power_trim = 0.0f;
        s->power_loop_ticks = 0;
    }
    bool regulating = s->mode == SC_SLOW_CHARGE || s->mode == SC_CHARGE ||
                      s->mode == SC_DISCHARGE;
    if (s->request == SC_REQUEST_AUTO && regulating &&
        s->power_loop_ticks < SC_POWER_LOOP_DECIMATION) ++s->power_loop_ticks;
    if (s->request == SC_REQUEST_AUTO && regulating && s->adc_offline_count == 0 &&
        s->power_loop_ticks >= SC_POWER_LOOP_DECIMATION) {
        s->power_loop_ticks = 0;
        float integral = s->pi_battery_power.integral;
        float trim = PID_Calculate(&s->pi_battery_power, s->battery_power,
                                   s->battery_power_target);
        float vcap = fmaxf(s->adc_values.supercap_voltage, 1.0f);
        float max_power = s->auto_discharge_ready ? ICAP_DISCHARGE_MAX * vcap : 0.0f;
        float min_power = s->auto_charge_full ? 0.0f : -charge_capacity(s) * vcap;
        if ((feedforward + trim >= max_power && s->battery_power_error > 0.0f) ||
            (feedforward + trim <= min_power && s->battery_power_error < 0.0f)) {
            s->pi_battery_power.integral = integral;
            trim = SC_LIMIT(PI_PBAT_KP * s->battery_power_error + integral,
                            -SC_POWER_TRIM_MAX_W, SC_POWER_TRIM_MAX_W);
        }
        s->power_trim = trim;
    } else if (s->request != SC_REQUEST_AUTO || !regulating) {
        s->power_loop_ticks = 0;
    }
    s->cap_power_request = feedforward + s->power_trim;
    s->iref_feedforward = feedforward / fmaxf(s->adc_values.supercap_voltage, 1.0f);
}

static float charge_target(const supercap_state_t *s)
{
    return -fminf(fmaxf(0.0f, -s->cap_power_request) /
                  fmaxf(s->adc_values.supercap_voltage, 1.0f), charge_capacity(s));
}

static float discharge_target(const supercap_state_t *s)
{
    if (!discharge_voltage_ready(s)) return 0.0f;
    float target = fmaxf(0.0f, s->cap_power_request) /
                   fmaxf(s->adc_values.supercap_voltage, SC_VCAP_DISCHARGE_MIN);
    return fminf(target, ICAP_DISCHARGE_MAX);
}

/* 初始化控制器并保持两路输出关闭；工作请求默认自动充放电。 */
void  SC_Init(supercap_state_t *s)
{
    if (s == NULL) return;

    // PID初始化
    // 电流环
    PID_Init(&s->pi_current, PI_ICAP_KP, PI_ICAP_KI, 0.0f,
             PI_ICAP_TS, CURRENT_PI_TRIM_MIN, CURRENT_PI_TRIM_MAX);                                    
    PID_SetIntegralThreshold(&s->pi_current, ICAP_I_THRESHOLD);
    PID_SetAntiWindup(&s->pi_current, true);
    // 功率环
    PID_Init(&s->pi_battery_power, PI_PBAT_KP, PI_PBAT_KI, 0.0f,
             PI_PBAT_TS, -SC_POWER_TRIM_MAX_W, SC_POWER_TRIM_MAX_W);
    PID_SetAntiWindup(&s->pi_battery_power, true);

    s->adc_values = (sc_adc_values_t){0};
    s->battery_power = s->supercap_power = s->load_current = 0.0f;
    s->power_reference = POWER_REFERENCE_DEFAULT;
    /* 上位机控制状态：本次上电尚无预算、无存活数据、未应用使能。 */
    s->host_control.budget_valid = false;
    s->host_control.budget_rx_ms = 0U;
    s->host_control.timing_from_frame = false;
    s->host_control.robot_alive_valid = false;
    s->host_control.robot_alive = 0U;
    s->host_control.enable_applied = false;
    s->host_control.enable_applied_ms = 0U;
    s->load_power = s->battery_power_target = s->battery_power_error = 0.0f;
    s->power_limit_exceeded = false;
    reset_power_control(s);
    s->pwm_duty = s->applied_duty = 0;
    s->pwm_enabled = s->cap_down_ctrl = s->adc_valid = false;
    s->adc_sample_pending = false;
    s->fault_recovery_ticks = 0;
    s->run_active = false;
    s->run_stop_pending = false;
    s->request = SC_REQUEST_AUTO;
    s->active_request = SC_REQUEST_STOP;
    s->auto_discharge_ready = false;
    s->auto_charge_full = false;
    s->reset_requested = false;
    s->fault_code = SC_FAULT_NONE;
    s->stop_reason = SC_STOP_REASON_NONE;
    s->adc_offline_count = s->fault_counter = s->load_low_ticks = 0;
    s->battery_overvoltage_counter = 0;
    s->pending_overcurrent = SC_FAULT_NONE;
    s->fault_snapshot_valid = false;
    for (uint32_t i = 0; i < ADC_CHANNEL_COUNT; ++i) {
        s->latest_adc_raw[i] = 0;
        s->fault_adc_raw[i] = 0;
    }
    s->fault_adc_values = (sc_adc_values_t){0};
    s->fault_mode = SC_SELF_TEST;
    s->fault_mode_ticks = 0;
    s->fault_applied_duty = 0;
    s->fault_cap_down_ctrl = false;
    reset_control(s);
    enter(s, SC_SELF_TEST);
}


/* 上层请求入口；只修改意图，是否能够运行由状态机及保护条件决定。 */
void SC_SetRequest(supercap_state_t *s, sc_request_t request)
{
    if (s != NULL && request >= SC_REQUEST_STOP && request <= SC_REQUEST_AUTO)
        s->request = request;
}

/* 保留上层手动复位请求；SC_Step 统一执行连续健康确认，不绕过恢复时限。 */
void SC_RequestFaultReset(supercap_state_t *s)
{
    if (s != NULL && s->mode == SC_FAULT_LATCHED) s->reset_requested = true;
}

/* ============================================================================
 * 上位机下行控制接入（仅 TIM1 控制上下文调用）
 * ========================================================================== */

/* 有效控制帧：新鲜时更新预算与存活状态；过期帧不重新应用旧预算。 */
void SC_ApplyControlCommand(supercap_state_t *s, float budget_w,
                            uint8_t robot_alive, uint32_t rx_ms, uint32_t now_ms)
{
    if (s == NULL)
    {
        return;
    }
    if ((uint32_t)(now_ms - rx_ms) >= SC_CONTROL_LOSS_TIMEOUT_MS)
    {
        /* 控制帧自身已达到时限：不更新预算、存活状态或计时。 */
        return;
    }

    s->host_control.budget_valid = true;
    s->host_control.budget_rx_ms = rx_ms;
    s->host_control.robot_alive_valid = true;
    s->host_control.robot_alive = robot_alive;
    /* 使能前也可缓存新鲜预算；但不因此启动板端，第一段计时仍从使能开始。 */
    s->power_reference = budget_w;
    if (sc_run_enable == 1U)
    {
        /* 使能后的有效控制帧把失联计时切换到该帧的接收时间。 */
        s->host_control.timing_from_frame = true;
    }
}

/* 首次使能：只允许置位运行开关；重复请求不重启自检、不清故障、不重置计时。 */
void SC_ApplyEnableRequest(supercap_state_t *s, uint32_t now_ms)
{
    if (s == NULL)
    {
        return;
    }
    if (sc_run_enable == 1U)
    {
        return;
    }

    sc_run_enable = 1U;
    s->host_control.enable_applied = true;
    s->host_control.enable_applied_ms = now_ms;
    s->host_control.timing_from_frame = false;

    /* 使能请求本身不检查年龄；缓存预算已过期或不存在时使用默认预算。 */
    if (!s->host_control.budget_valid ||
        (uint32_t)(now_ms - s->host_control.budget_rx_ms) >= SC_CONTROL_LOSS_TIMEOUT_MS)
    {
        s->power_reference = POWER_REFERENCE_DEFAULT;
    }
}

/* 已使能后按第一段（使能应用时间）或后续（最近有效控制帧）计时；
 * 达到时限只恢复默认预算，不失能、不重置控制器。 */
void SC_AdvanceControlTimeout(supercap_state_t *s, uint32_t now_ms)
{
    uint32_t base_ms;

    if (s == NULL || sc_run_enable != 1U || !s->host_control.enable_applied)
    {
        /* 未使能不启动运行失联计时。 */
        return;
    }

    if (s->host_control.timing_from_frame)
    {
        base_ms = s->host_control.budget_rx_ms;
    }
    else
    {
        base_ms = s->host_control.enable_applied_ms;
    }

    /* 无符号毫秒差值自然处理计数回绕；499 ms 未超时，500 ms 进入超时。 */
    if ((uint32_t)(now_ms - base_ms) >= SC_CONTROL_LOSS_TIMEOUT_MS)
    {
        s->power_reference = POWER_REFERENCE_DEFAULT;
    }
}

/* 按当前板卡 ADC 标定系数换算五路数据，并标记已有可用快照。 */
void SC_ADC_ConvertToPhysical(supercap_state_t *s, const uint16_t *raw)
{
    if (s == NULL || raw == NULL) return;
    for (uint32_t i = 0; i < ADC_CHANNEL_COUNT; ++i)
        s->latest_adc_raw[i] = raw[i];
    s->adc_values.battery_voltage = 0.0074776323f * raw[0] + 0.48288837f;
    s->adc_values.supercap_voltage = 0.0074364104f * raw[1] + 0.48499773f;
    s->adc_values.battery_current = 0.0039736503f * raw[2] + 0.16004153f;
    s->adc_values.supercap_current = 0.0189187727f * raw[3] - 37.25810491f;
    s->adc_values.temperature = (float)raw[4];
    s->adc_valid = true;
    s->adc_sample_pending = true;
}

/* 电池电流含电容充电分量，估计负载功率为 Pbat + Pcap。 */
void SC_CalculatePower(supercap_state_t *s)
{
    if (s == NULL) return;
    s->battery_power = s->adc_values.battery_current * s->adc_values.battery_voltage;
    s->power_limit_exceeded = s->adc_valid && s->battery_power > s->power_reference;
    s->supercap_power = s->adc_values.supercap_current * s->adc_values.supercap_voltage;
    /* Icap 充电为负，因此加上 Pcap 才能从电池侧功率中扣除充电功率。 */
    s->load_power = fmaxf(0.0f, s->battery_power + s->supercap_power);
    s->load_current = s->load_power /
                      fmaxf(s->adc_values.battery_voltage, SC_VBAT_MIN);
}

/* 首次故障锁存原因并禁止 PWM；其后的故障不能覆盖首个故障码。 */
void SC_HandleFault(supercap_state_t *s, sc_fault_code_t code)
{
    if (s == NULL || code == SC_FAULT_NONE)
    {
        return;
    }

    if (s->mode != SC_FAULT_LATCHED)
    {
        // 锁存故障时刻adc采样值
        for (uint32_t i = 0; i < ADC_CHANNEL_COUNT; ++i)
        {
            s->fault_adc_raw[i] = s->latest_adc_raw[i];
        }
        // 快照故障现场
        s->fault_adc_values = s->adc_values;
        s->fault_mode = s->mode;
        s->fault_mode_ticks = s->mode_ticks;
        s->fault_applied_duty = s->applied_duty;
        s->fault_cap_down_ctrl = s->cap_down_ctrl;
        s->fault_snapshot_valid = s->adc_valid;
        s->fault_code = code;
        enter(s, SC_FAULT_LATCHED);
    }

    s->pwm_enabled = false;
    s->pwm_duty = s->applied_duty = 0;
    s->target_current = 0.0f;
    reset_power_control(s);
    /* 故障发生时先关闭 PWM。若电感仍有电流，短时保持 CapDownCtrl=1 供被动续流；
     * 电流接近零或达到最长续流时间后置 0，避免 ADC 离线时一直保持接入。 */
    // 持续多个周期的判断
    if (s->mode_ticks >= SC_FAULT_FREEWHEEL_TICKS ||
        (s->adc_valid && fabsf(s->adc_values.supercap_current) < SC_CURRENT_ZERO_A))
    {
        s->cap_down_ctrl = false;
    }
}

/* 新 ADC 快照保护：电容过压立即动作，电池过压与过流独立连续计数。 */
void SC_RunProtection(supercap_state_t *s)
{
    if (s == NULL)
    {
        return;
    }
    /* 此入口仅对新 ADC 快照调用；故障锁定时仍记录新样本到达。 */
    if (s->adc_valid)
    {
        s->adc_sample_pending = true;
    }

    // 保护执行前提
    if ((sc_run_enable != 1U && !s->run_active) || !s->adc_valid ||
        s->mode == SC_FAULT_LATCHED)
    {
        return;
    }

    // 电容组过压保护（最高优先级，单次过压立刻执行保护）
    if (s->adc_values.supercap_voltage > SUPERCAP_VOLTAGE_MAX)
    {
        SC_HandleFault(s, SC_FAULT_CAP_OVERVOLTAGE);
    }
    else
    {
        // 电池过压保护（连续多次采样过压）
        if (s->adc_values.battery_voltage > BATTERY_VOLTAGE_MAX)
        {
            if (++s->battery_overvoltage_counter > SC_BATTERY_OVERVOLTAGE_COUNT_LIMIT)
            {
                SC_HandleFault(s, SC_FAULT_BAT_OVERVOLTAGE);
                return;
            }
        }
        else
        {
            s->battery_overvoltage_counter = 0;
        }

        // 自检mode的前200ms关闭电流采样，是ADC采样稳定；200ms后恢复过流保护（仍处于self_test mode）；
        if (s->mode == SC_SELF_TEST &&
            s->mode_ticks < SC_SELF_TEST_SETTLE_TICKS)
        {
            s->fault_counter = 0;
            s->pending_overcurrent = SC_FAULT_NONE;
            return;
        }

        // 开始过流保护检测
        sc_fault_code_t overcurrent = SC_FAULT_NONE;
        // 电池电流
        if (s->adc_values.battery_current > BATTERY_CURRENT_MAX)
        {
            overcurrent = SC_FAULT_BAT_OVERCURRENT;
        }
        // 电容放电过流
        else if (s->adc_values.supercap_current > ICAP_MAX)
        {
            overcurrent = SC_FAULT_CAP_DISCHARGE_OC;
        }
        // 电容充电过流
        else if (s->adc_values.supercap_current < ICAP_MIN)
        {
            overcurrent = SC_FAULT_CAP_CHARGE_OC;
        }
        // 检测完之后进状态机更新
        if (overcurrent != SC_FAULT_NONE)
        {
            /* 故障种类变化时重新计数，不能把不同方向的超限次数相加。 */
            if (s->pending_overcurrent != overcurrent)
            {
                s->pending_overcurrent = overcurrent;
                s->fault_counter = 0;
            }
            // 同种过流错误连续触发才进入保护
            if (++s->fault_counter >= PROTECTION_THRESHOLD)
            {
                SC_HandleFault(s, overcurrent);
            }
        }
        else
        {
            s->fault_counter = 0;
            s->pending_overcurrent = SC_FAULT_NONE;
        }
    }
}

/* 由 Q3 接通后的 VCAP/VBAT 得到首个占空比；运行后叠加 PI 并限斜率。 */
static void update_duty(supercap_state_t *s)
{
    if (!s->pwm_enabled || s->mode == SC_FAULT_LATCHED) {
        s->pwm_duty = s->applied_duty = 0;
        return;
    }
    float vbat = fmaxf(s->adc_values.battery_voltage, SC_VBAT_MIN);
    int32_t equilibrium = (int32_t)SC_LIMIT(
        DUTY_FF_SCALE * s->adc_values.supercap_voltage / vbat,
        PWM_DUTY_MIN, PWM_DUTY_MAX);
    if (s->mode == SC_START_PWM) {
        /* 第一个有效 PWM 周期直接使用平衡值，不从 0 缓升，也不叠加 PI。 */
        s->pwm_duty = s->applied_duty = equilibrium;
        return;
    }
    float trim = PID_Calculate(&s->pi_current, s->adc_values.supercap_current,
                               s->target_current);
    /* PI 参数沿用原电流环：输入顺序使电流误差与占空比调节方向一致。 */
    s->pwm_duty = (int32_t)SC_LIMIT((float)equilibrium + trim,
                                   PWM_DUTY_MIN, PWM_DUTY_MAX);
    int32_t difference = s->pwm_duty - s->applied_duty;
    if (difference > PWM_SLEW_RATE_UP) difference = PWM_SLEW_RATE_UP;
    if (difference < -PWM_SLEW_RATE_DOWN) difference = -PWM_SLEW_RATE_DOWN;
    s->applied_duty += difference;
}

bool SC_PWM_OutputRequested(const supercap_state_t *s)
{
    if (s == NULL || !s->run_active || !s->cap_down_ctrl ||
        !s->adc_valid || s->fault_code != SC_FAULT_NONE ||
        !s->pwm_enabled || s->applied_duty < PWM_DUTY_MIN ||
        s->applied_duty > PWM_DUTY_MAX) return false;

    switch (s->mode) {
    case SC_START_PWM:
    case SC_SLOW_CHARGE:
    case SC_CHARGE:
    case SC_DISCHARGE:
    case SC_CROSS_ZERO:
    case SC_STOPPING:
        return true;
    default:
        return false;
    }
}

/* 每两个 TIM1 更新中断调用一次：先处理故障和母线异常，再执行状态转移。 */
void SC_Step(supercap_state_t *s)
{
    if (s == NULL)
    {
        return;
    }

    // 检查新周期中是否有ADC更新
    bool fresh_adc = s->adc_sample_pending;
    s->adc_sample_pending = false;

    // 进入中断周期时enable为0
    if (sc_run_enable != 1U && s->mode != SC_FAULT_LATCHED)
    {
        // 维持在无功率级接入时“未使能”的统一标志位状态
        if (!s->run_active || s->mode == SC_SELF_TEST || s->mode == SC_STANDBY)
        {
            /* 尚未接入 Q3 时直接保持关断，不启动自检计时。 */
            s->pwm_enabled = false;
            s->cap_down_ctrl = false;
            s->pwm_duty = s->applied_duty = 0;
            reset_control(s);
            reset_power_control(s);

            if (s->mode != SC_SELF_TEST)
            {
                enter(s, SC_SELF_TEST);
            }
            s->run_active = false;
            s->run_stop_pending = false;
            s->battery_overvoltage_counter = 0;
            s->active_request = SC_REQUEST_STOP;
            return; // 回到未使能态后直接结束该轮中断
        }

        // 接入功率级后进入“未使能”状态的下电流程
        s->run_stop_pending = true;
        s->request = SC_REQUEST_STOP;
        if (s->mode != SC_STOPPING)
        {
            // 更新下电请求，在同一中断周期中处理
            s->stop_reason = SC_STOP_REASON_REQUEST;
            enter(s, SC_STOPPING);
        }
    }

    // 进入中断周期时enable为1 + 处于“未使能”状态
    if (sc_run_enable == 1U && !s->run_active)
    {
        /* 每次 0→1 都从自检第一步重新计时，默认自动充放电。 */
        s->run_active = true;
            // 防止Fault mode下手动enable使能导致错误状态被掩盖
            if (s->mode != SC_FAULT_LATCHED)
            {
                s->request = SC_REQUEST_AUTO;
                s->active_request = SC_REQUEST_STOP;
                s->auto_discharge_ready = false;
                s->auto_charge_full = false;
                s->adc_offline_count = 0;
                s->fault_counter = 0;
                s->pending_overcurrent = SC_FAULT_NONE;
                reset_control(s);
                reset_power_control(s);
                enter(s, SC_SELF_TEST);
            }
    }

    if (s->mode_ticks < UINT32_MAX)
    {
        ++s->mode_ticks;
    }


    if (s->mode == SC_FAULT_LATCHED)
    {

        SC_HandleFault(s, s->fault_code);
        /* 锁定期间继续采样；任意越界、缺样或关闭运行开关都会重新计时。
         * 输出关闭时要求电容电流近零，不能只以未达到过流门限判断可重启。 */
        // 故障恢复安全检查：以下条件全部满足才累计恢复时间，连续 1 s 后自动解除故障
        if (sc_run_enable == 1U &&  // 用户使能
            !s->cap_down_ctrl &&    // Capdown拉低
            fresh_adc &&            // 周期内有采样
            s->adc_valid &&     // ADC采样
            s->adc_offline_count == 0 &&    // 确认ADC在线
            s->adc_values.battery_voltage >= SC_VBAT_MIN &&     // 母线不欠压
            s->adc_values.battery_voltage <= BATTERY_VOLTAGE_MAX &&     // 母线不过压
            fabsf(s->adc_values.battery_current) <= BATTERY_CURRENT_MAX &&      // 母线不过流
            s->adc_values.supercap_voltage >= SC_VCAP_MIN_VALID &&      // 电容组不欠压
            s->adc_values.supercap_voltage <= SUPERCAP_VOLTAGE_MAX &&       // 电容组不过压
            fabsf(s->adc_values.supercap_current) < SC_CURRENT_ZERO_A)      // 电容组输出电流为0
        {
            if (s->fault_recovery_ticks < SC_FAULT_RECOVERY_STABLE_TICKS)   // 系统持续正常
            {
                ++s->fault_recovery_ticks;
            }
        }
        else
        {
            s->fault_recovery_ticks = 0;
        }

        // 恢复正常状态机，重新进入self_test
        if (s->fault_recovery_ticks >= SC_FAULT_RECOVERY_STABLE_TICKS)
        {
            s->reset_requested = false;
            s->fault_code = SC_FAULT_NONE;
            s->fault_recovery_ticks = 0;
            s->battery_overvoltage_counter = 0;
            s->active_request = SC_REQUEST_STOP;
            s->auto_discharge_ready = false;
            s->auto_charge_full = false;
            s->run_active = true;
            s->run_stop_pending = false;
            s->load_low_ticks = 0;
            s->fault_counter = 0;
            s->pending_overcurrent = SC_FAULT_NONE;
            s->fault_snapshot_valid = false;
            reset_control(s);
            reset_power_control(s);
            enter(s, SC_SELF_TEST);
        }
        return;
    }

    // 进入ADC离线错误
    if (s->adc_valid && s->adc_offline_count >= ADC_OFFLINE_LIMIT)
    {
        /* ADC 持续没有新快照时禁止闭环继续使用旧数据。 */
        SC_HandleFault(s, SC_FAULT_ADC_OFFLINE);
        return;
    }

    /* 母线欠压先正常停机；若电流不能消退，再由停机超时转故障锁定。 */
    if (s->adc_valid && s->mode != SC_SELF_TEST &&
        s->adc_values.battery_voltage < SC_VBAT_MIN)
    {
        // 依旧preload_than_enable
        if (s->cap_down_ctrl && s->mode != SC_STOPPING)
        {
            s->stop_reason = SC_STOP_REASON_BUS_UNDERVOLTAGE;
            enter(s, SC_STOPPING);
        }
        else
        {
            s->pwm_enabled = false;
            s->applied_duty = s->pwm_duty = 0;
        }
    }

    // 确定功率环数值
    update_power_control(s);
    // 确定电流环方向
    select_request(s);


    // 根据状态机确定是否要计算并输出电流（电流环）
    switch (s->mode)
    {
    case SC_SELF_TEST:
        /* 上电自检期间输出始终关闭。先等待采样稳定，再要求测量连续合格；
         * 瞬时异常清零稳定计数，超时仍不合格则锁定故障。 */
        s->pwm_enabled = s->cap_down_ctrl = false;
        reset_control(s);
        // 此处安全检查是self_test自检成功次数
        if (s->mode_ticks >= SC_SELF_TEST_SETTLE_TICKS && s->adc_valid &&
            s->adc_offline_count < ADC_OFFLINE_LIMIT &&
            s->adc_values.battery_voltage >= SC_VBAT_MIN &&
            s->adc_values.supercap_voltage >= SC_VCAP_MIN_VALID &&
            s->adc_values.supercap_voltage <= SUPERCAP_VOLTAGE_MAX &&
            s->adc_values.battery_current <= BATTERY_CURRENT_MAX &&
            fabsf(s->adc_values.supercap_current) <= SC_SELF_TEST_ZERO_A)
        {
            if (++s->self_test_stable_ticks >= SC_SELF_TEST_STABLE_TICKS)
            {
                enter(s, SC_STANDBY);
            }
        }
        else
        {
            s->self_test_stable_ticks = 0;
        }

        // 持续自检失败，超出self_test最大时长
        if (s->mode == SC_SELF_TEST && s->mode_ticks >= SC_SELF_TEST_TIMEOUT)
        {
            SC_HandleFault(s, s->adc_valid ? SC_FAULT_SELF_TEST : SC_FAULT_ADC_OFFLINE);
        }
        break;

    case SC_STANDBY:
        /* 待机保持两路关闭；获得工作许可后先接通 Q3，PWM 仍关闭。 */
        s->cap_down_ctrl = false;
        s->pwm_enabled = false;
        reset_control(s);

        // 多层保护
        if (s->adc_values.battery_voltage < SC_VBAT_MIN ||
            s->active_request == SC_REQUEST_STOP ||
            (s->adc_values.supercap_voltage >= SC_CHARGE_DONE_VOLTAGE &&
             s->active_request == SC_REQUEST_CHARGE))
        {
            break;
        }

        if (s->active_request == SC_REQUEST_DISCHARGE &&
            !discharge_allowed(s, SC_LOAD_ENTER_A))
        {
            break;
        }
        enter(s, SC_CONNECT_Q3);
        s->cap_down_ctrl = true;
        break;

    case SC_CONNECT_Q3:
        /* Q3 先接通并观察，整个接入阶段禁止 PWM。 */
        s->cap_down_ctrl = true;
        s->pwm_enabled = false;
        s->target_current = 0.0f;
        if (s->active_request == SC_REQUEST_STOP ||
            s->adc_values.battery_voltage < SC_VBAT_MIN)
        {
            s->stop_reason = SC_STOP_REASON_REQUEST;
            enter(s, SC_STOPPING);
        }
        else if (
            // fabsf(s->adc_values.supercap_current - s->connect_icap) > SC_CONNECT_JUMP_A ||
            fabsf(s->adc_values.battery_voltage - s->connect_vbat) > SC_CONNECT_JUMP_V ||
            fabsf(s->adc_values.supercap_voltage - s->connect_vcap) > SC_CONNECT_JUMP_V)
        {
            SC_HandleFault(s, SC_FAULT_CONNECT);
        }
        else if (s->mode_ticks > SC_CONNECT_TIMEOUT)
        {
            SC_HandleFault(s, SC_FAULT_CONNECT);
        }
        else if (s->mode_ticks >= SC_CONNECT_SETTLE_TICKS && zero_confirmed(s))
        {
            reset_control(s);
            enter(s, SC_START_PWM);
            s->pwm_enabled = true;
        }
        break;
    case SC_START_PWM:
        /* Q3 已稳定接通；留出 CCR 预装载和 MOE 开放周期，再启动电流环。 */
        s->cap_down_ctrl = true;
        s->target_current = 0.0f;
        if (s->active_request == SC_REQUEST_STOP ||
            s->adc_values.battery_voltage < SC_VBAT_MIN)
        {
            s->stop_reason = SC_STOP_REASON_REQUEST;
            enter(s, SC_STOPPING);
        }
        else if (s->mode_ticks >= SC_PWM_START_TICKS)
        {
            // 由先前功率环更新+方向控制决定充放电状态
            if (s->active_request == SC_REQUEST_CHARGE)
            {
                enter(s, charge_mode(s));
            }
            else if (s->active_request == SC_REQUEST_DISCHARGE &&
                     (!discharge_voltage_ready(s) || discharge_allowed(s, SC_LOAD_ENTER_A)))
            {
                enter(s, SC_DISCHARGE);
            }
            else
            {
                enter(s, SC_STOPPING);
            }
        }
        break;


    case SC_SLOW_CHARGE:    // 神秘C语言fall_through
    case SC_CHARGE:
        /* 充电中请求反向必须先 过零 ；请求取消或充满则正常停止；
         * VCAP 跨过 7V 时在慢充与正常充电之间切换。 */
        if (s->active_request == SC_REQUEST_DISCHARGE)
        {
            s->next_mode = SC_DISCHARGE;
            enter(s, SC_CROSS_ZERO);    // 电流反向先进入置零的状态
        }
        // 电容组满电
        else if (s->active_request != SC_REQUEST_CHARGE ||
                   s->adc_values.supercap_voltage >= SC_CHARGE_DONE_VOLTAGE)
        {
            /* 反馈用：区分满电正常停充与其他正常停机。 */
            if (s->adc_values.supercap_voltage >= SC_CHARGE_DONE_VOLTAGE)
            {
                s->stop_reason = SC_STOP_REASON_CHARGE_FULL;
            }
            else
            {
                s->stop_reason = SC_STOP_REASON_REQUEST;
            }
            enter(s, SC_STOPPING);
        }
        // charge_mode(s)返回当前 应该 处于slow_charge/charge mode
        else if (charge_mode(s) != s->mode)
        {
            enter(s, charge_mode(s));
        }
        else
        {
            s->requested_current = charge_target(s);
        }
        break;

    case SC_DISCHARGE:
        /* 放电请求改为充电时先过零；低压时保持接入，仅将电流目标归零。
         * 明确停止请求仍正常停机。
         * 负载电流低于退出门限须持续指定时间，避免门限附近反复切换。 */
        if (s->active_request == SC_REQUEST_CHARGE)
        {
            s->next_mode = charge_mode(s);
            enter(s, SC_CROSS_ZERO);
        }
        else if (s->active_request != SC_REQUEST_DISCHARGE ||
                   s->adc_values.battery_voltage < SC_VBAT_MIN)
        {
            s->stop_reason = SC_STOP_REASON_REQUEST;
            enter(s, SC_STOPPING);
        }
        else if (!discharge_voltage_ready(s))
        {
            s->requested_current = 0.0f;
            s->load_low_ticks = 0;
        }
        else
        {
            if (s->request != SC_REQUEST_AUTO && s->load_current < SC_LOAD_EXIT_A)
            {
                ++s->load_low_ticks;
            }
            else
            {
                s->load_low_ticks = 0;
            }
            if (s->load_low_ticks >= SC_LOAD_EXIT_TICKS)
            {
                s->stop_reason = SC_STOP_REASON_REQUEST;
                enter(s, SC_STOPPING);
            }
            else
            {
                s->requested_current = discharge_target(s);
            }
        }
        break;

    case SC_CROSS_ZERO:
        /* CapDownCtrl 保持 1，电流目标按斜率回到 0。
         * 实测与目标电流连续接近 0 后清 PI，再按许可进入新方向；超时锁故障。 */
        // 换向电流归零超时
        if (s->mode_ticks > SC_ZERO_TIMEOUT)
        {
            SC_HandleFault(s, SC_FAULT_ZERO_TIMEOUT);
        }

        else if (zero_confirmed(s))
        {
            reset_control(s);
            /* 已接入后即使电压不足，也可保持放电状态并以 0 A 等待回充。 */
            if (s->active_request == SC_REQUEST_DISCHARGE &&
                (!discharge_voltage_ready(s) || discharge_allowed(s, SC_LOAD_ENTER_A)))
            {
                enter(s, SC_DISCHARGE);
            }
            else if (s->active_request == SC_REQUEST_CHARGE)
            {
                enter(s, charge_mode(s));
            }
            else
            {
                s->stop_reason = SC_STOP_REASON_REQUEST;
                enter(s, SC_STOPPING);
            }
        }
        break; 

    case SC_STOPPING:
        /* 先将目标和实测电流降到 0；确认后关 PWM，再断开 Q3。
         * 外部关断完成后停在自检入口，普通停止则回待机。 */
        if (s->mode_ticks > SC_STOP_TIMEOUT)
        {
            SC_HandleFault(s, SC_FAULT_STOP_TIMEOUT);
        }

        else if (zero_confirmed(s))
        {
            s->pwm_enabled = false;
            s->applied_duty = s->pwm_duty = 0;
            s->cap_down_ctrl = false;
            reset_control(s);
            reset_power_control(s);
            if (s->run_stop_pending)
            {
                s->run_stop_pending = false;
                s->run_active = false;
                s->battery_overvoltage_counter = 0;
                s->request = SC_REQUEST_AUTO;
                s->active_request = SC_REQUEST_STOP;
                enter(s, SC_SELF_TEST);
            }
            else
            {
                if (s->request != SC_REQUEST_AUTO)
                {
                    s->request = SC_REQUEST_STOP;
                }
                enter(s, SC_STANDBY);
            }
        }
        break;

    case SC_FAULT_LATCHED:
    default:
        break;
    }
    /* 状态负责给出 requested_current；统一斜坡生成实际送给电流 PI 的目标。 */
    if (s->mode != SC_FAULT_LATCHED && s->mode != SC_SELF_TEST &&
        s->mode != SC_STANDBY)
    {
        s->target_current = slew(s->target_current, s->requested_current);
    }

    // 电流环输出值修改为PWM实际占空比
    update_duty(s);
}

/* ============================================================================
 * CAN 反馈（0x220）
 * ========================================================================== */

/* 复制反馈所需的一致快照；短临界区由调用方负责。 */
void SC_GetFeedbackSnapshot(sc_feedback_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    snapshot->run_enable = sc_run_enable;
    snapshot->run_active = sc_state.run_active;
    snapshot->mode = sc_state.mode;
    snapshot->fault_code = sc_state.fault_code;
    snapshot->stop_reason = sc_state.stop_reason;
    snapshot->adc_valid = sc_state.adc_valid;
    snapshot->adc_offline_count = sc_state.adc_offline_count;
    snapshot->battery_voltage = sc_state.adc_values.battery_voltage;
    snapshot->supercap_voltage = sc_state.adc_values.supercap_voltage;
    snapshot->battery_power = sc_state.battery_power;
    snapshot->supercap_power = sc_state.supercap_power;
}

/* 正常充放电与满电正常停充计为正常运行；首次启用过程与故障为 0。 */
bool SC_FeedbackNormalRunning(const sc_feedback_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }
    switch (snapshot->mode)
    {
    case SC_SLOW_CHARGE:
    case SC_CHARGE:
    case SC_DISCHARGE:
    case SC_CROSS_ZERO:
        return true;
    case SC_STOPPING:
    case SC_STANDBY:
        /* 满电正常停充及其后的待机保持为正常运行。 */
        return snapshot->stop_reason == SC_STOP_REASON_CHARGE_FULL;
    case SC_SELF_TEST:
    case SC_CONNECT_Q3:
    case SC_START_PWM:
    case SC_FAULT_LATCHED:
    default:
        return false;
    }
}

/* 板端判定能否允许上位机发起供能请求：正常工作流程、无故障、
 * ADC 可信且电容组电压严格大于 8 V。ADC 偶发缺样沿用板端认可的电压。 */
bool SC_FeedbackSupplyRequestAllowed(const sc_feedback_snapshot_t *snapshot)
{
    if (snapshot == NULL ||
        snapshot->run_enable != 1U ||
        !snapshot->run_active ||
        !snapshot->adc_valid ||
        snapshot->fault_code != SC_FAULT_NONE ||
        snapshot->adc_offline_count >= ADC_OFFLINE_LIMIT ||
        !(snapshot->supercap_voltage > SC_FEEDBACK_SUPPLY_REQUEST_VOLTAGE) ||
        snapshot->battery_voltage < SC_VBAT_MIN)
    {
        return false;
    }
    switch (snapshot->mode)
    {
    case SC_SLOW_CHARGE:
    case SC_CHARGE:
    case SC_DISCHARGE:
    case SC_CROSS_ZERO:
        return true;
    case SC_STANDBY:
        /* 正常工作待机：已通过自检，且不是母线欠压等保护性停机的待机。 */
        return snapshot->stop_reason != SC_STOP_REASON_BUS_UNDERVOLTAGE;
    default:
        return false;
    }
}

