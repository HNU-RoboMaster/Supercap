/**
 * @file supercap_control.h
 * @brief 超级电容状态机、采样保护与电流控制接口。
 *
 * 电容电流符号：正值为放电，负值为充电。
 * CapDownCtrl 对应 PA4，控制 Q3；PWM 由 TIM1 CH3/CH3N 控制。
 */
#ifndef SUPERCAP_CONTROL_H
#define SUPERCAP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid_controller.h"
#include "supercap_config.h"

// 运行阶段标志位
typedef enum {
    SC_SELF_TEST,     /* 上电自检：输出关闭，等待有效采样 */
    SC_STANDBY,       /* 待机：Q3 和 PWM 均关闭 */
    SC_CONNECT_Q3,    /* 先接入 Q3，PWM 关闭，检查电流和电压跳变 */
    SC_SLOW_CHARGE,   /* VCAP < 7V：限流慢充 */
    SC_CHARGE,        /* 正常充电：负电流目标 */
    SC_DISCHARGE,     /* 放电：低压时电流目标为 0，保持 PWM 和 Q3 接入 */
    SC_CROSS_ZERO,    /* 换向前过零：Q3 保持导通 */
    SC_STOPPING,      /* 正常停止：电流归零后关 PWM，再断 Q3 */
    SC_FAULT_LATCHED, /* 故障锁定 */
    SC_START_PWM      /* Q3 已稳定接通，以 Vcap/Vbat 预置并开启 PWM */
} sc_mode_t;

/* 反馈用停机原因：进入 SC_STOPPING 时记录，并带到其后的 SC_STANDBY。 */
typedef enum {
    SC_STOP_REASON_NONE = 0,          /* 本次使能尚未进入过停机 */
    SC_STOP_REASON_CHARGE_FULL,       /* 电容组达到充满电压，正常停充 */
    SC_STOP_REASON_REQUEST,           /* 请求取消或负载不足，正常停机 */
    SC_STOP_REASON_BUS_UNDERVOLTAGE   /* 母线欠压，保护性停机 */
} sc_stop_reason_t;

/* AUTO 按负载功率自动选择方向；其余请求用于手动控制。 */
typedef enum {
    SC_REQUEST_STOP,       /* 停止：不充电也不放电 */
    SC_REQUEST_CHARGE,     /* 充电：目标电流为负 */
    SC_REQUEST_DISCHARGE,  /* 放电：目标电流为正 */
    SC_REQUEST_AUTO        /* 自动：按负载功率请求选择充电/放电方向 */
} sc_request_t;

/* fault_code 始终保存本次锁定的首个故障。 */
typedef enum {
    SC_FAULT_NONE = 0, /* 无故障 */
    SC_FAULT_BAT_OVERCURRENT, /* 电池过流：Ibat > 10 A，连续 3 次采样 */
    SC_FAULT_CAP_DISCHARGE_OC, /* 电容放电过流：Icap > 10 A，连续 3 次采样 */
    SC_FAULT_CAP_CHARGE_OC, /* 电容充电过流：Icap < -10 A，连续 3 次采样 */
    SC_FAULT_CAP_OVERVOLTAGE, /* 电容过压：Vcap > 26 V，立即触发 */
    SC_FAULT_ADC_OFFLINE, /* ADC 连续无新采样：约 5 ms */
    SC_FAULT_BAT_OVERVOLTAGE, /* 电池过压：Vbat > 28 V，连续 6 个样本 */
    SC_FAULT_SELF_TEST, /* 自检未在 500 ms 内通过 */
    SC_FAULT_CONNECT, /* Q3 接入跳变超限或 200 ms 内未稳定 */
    SC_FAULT_ZERO_TIMEOUT, /* 电流过零超时：500 ms */
    SC_FAULT_STOP_TIMEOUT /* 正常停机电流未及时归零：500 ms */
} sc_fault_code_t;

typedef struct {
    float battery_voltage;     /* ADC-VBat：VBAT，电池输入侧电压 */
    float supercap_voltage;    /* ADC-VCap：IC_P（Q3 前），Q3 接通时近似电容组电压 */
    float battery_current;     /* 电池电流：向负载供电为正 */
    float supercap_current;    /* 电容电流：放电为正，充电为负 */
    float temperature;         /* 预留 ADC 通道 */
} sc_adc_values_t;

/* CAN 反馈用的板端快照：使能、运行状态、保护结果、功率与电容组电压。
 * 由 TIM2 在短临界区内复制，归一化和打包在临界区外完成。 */
typedef struct {
    uint8_t run_enable;             /* 本次上电的使能开关回读 */
    bool run_active;                /* 本次使能已从自检入口启动 */
    sc_mode_t mode;                 /* 当前运行状态 */
    sc_fault_code_t fault_code;     /* 首个锁存故障 */
    sc_stop_reason_t stop_reason;   /* 最近一次进入停机的板端原因 */
    bool adc_valid;                 /* 至少取得过一次有效 ADC 快照 */
    uint32_t adc_offline_count;     /* 连续无新 ADC 快照的控制周期数 */
    float battery_voltage;          /* 电池输入侧电压，V */
    float supercap_voltage;         /* 电容组电压，V */
    float battery_power;            /* 电池功率，W */
    float supercap_power;           /* 电容组功率，W；放电为正，充电为负 */
} sc_feedback_snapshot_t;

/* 上位机下行控制的运行时状态；只由 TIM1 控制上下文更新。
 * 接收信息的有效性与 0/1 存活值分开，避免初始化零值或失联被当作死亡。 */
typedef struct {
    bool budget_valid;          /* 上位机发送的power_reference */
    uint32_t budget_rx_ms;      /* 功率控制帧接收时间，用于判断500ms断连 */
    bool timing_from_frame;     /* 表示使能后已收到过有效控制帧 */
    bool enable_applied;        /* 由 CAN 使能，收到失能命令时清除 */
    uint32_t enable_applied_ms; /* 本次使能应用时刻；第一段控制超时从此刻起算 */

    bool robot_alive_valid;     /* 存活字段有效 */
    uint8_t robot_alive;        /* 最近有效控制帧的存活值；首版不据此改变运行 */
} sc_host_control_t;


typedef struct {

    // ADC采样位点
    sc_adc_values_t adc_values;
    
    float battery_power, supercap_power;    // 电管chassis输出口 电容组实际功率


    float load_power;                      /* Pbat + Pcap，含变换器损耗的负载估计 */
    float load_current;                    // 由load_power反向推算得到

    float power_reference;                 /* 运行时电池侧预算（W）；重启/失联取默认值，预留 CAN 动态更新 */

    sc_host_control_t host_control;        /* 上位机下行控制状态；仅 TIM1 控制上下文更新 */

    float battery_power_target;     // 用于功率环闭环的实际功率值限制，低于power_reference
    float battery_power_error;      // 功率环误差
    // 使用前馈+PID的双环控制器
    float power_trim;   // 功率环 PI 的反馈修正量(输出值)；
    float cap_power_request;               // 前馈 + 修正后的电容功率请求（正=放电，负=充电），单位 W
    bool power_limit_exceeded;             /* 观察变量，用于观察电管cahssis输出口功率是否超出power_reference */

    // 功率环1kHz，电流环100kHz
    uint32_t power_loop_ticks;      // 功率环分频计数器


    float requested_current;    // 功率环计算所得电流值，但不是给电流环的目标值
    float target_current;   // 加入周期变化限幅后实际给到电流环的目标值

    float iref_feedforward;     // 参考电流，用来检测前馈方向是否反转

    int32_t pwm_duty;   // 电流环计算所得CCR值，但不是实际执行的CCR
    int32_t applied_duty;   // 加入周期变化限幅之后实际执行的CCR


    // PI控制器实例
    PID_Controller pi_current;
    PID_Controller pi_battery_power;

    // 运行状态标志位
    volatile sc_mode_t mode;
    sc_mode_t next_mode;    // 目前意义不明
    volatile sc_stop_reason_t stop_reason;  /* 反馈用：最近一次进入 SC_STOPPING 的原因 */

    // 运行模式标志位
    volatile sc_request_t request;  // 板内工作请求；首版 CAN 只传预算，充放电由 AUTO 决定
    sc_request_t active_request;            /* 自动决策后实际执行的停止/充电/放电请求 */

    // 运行阶段标志位
    bool auto_discharge_ready;             /* 低压停放后须回充至恢复电压 */
    bool auto_charge_full;                 /* 充满后须降至回充电压 */

    volatile sc_fault_code_t fault_code; /* 错误标志位，存储首个故障，复位前不覆盖 */
    sc_fault_code_t pending_overcurrent; /* 正在连续计数的过流类型；类型变化时重新计数 */
    // 一些bool标志位
    volatile bool pwm_enabled;            /* TIM1 主输出使能请求 */
    volatile bool cap_down_ctrl;          /* PA4 = CapDownCtrl，1 表示接入 Q3 */
    volatile bool adc_valid;              /* 至少取得过一次有效 ADC 快照 */
    bool adc_sample_pending;              /* 上次 SC_Step 后收到过新 ADC 快照 */
    volatile bool reset_requested;        /* 手动故障复位请求；仍须满足连续健康条件才生效 */
    bool run_active;                       /* 本次使能已从自检入口启动 */
    bool run_stop_pending;                 /* 外部关断正在等待电流归零 */

    // 一堆计数器
    
    uint32_t mode_ticks;                   /* 当前状态已持续的控制周期数 */
    uint32_t zero_ticks;                   /* 目标与实测电流连续接近零的周期数（5 ms 确认） */
    uint32_t load_low_ticks;               /* 手动放电时负载低于退出门限的连续周期数（100 ms 停机） */
    uint32_t adc_offline_count;            /* 连续无新 ADC 快照的控制周期数（500 次判离线） */
    uint32_t self_test_stable_ticks;       /* 自检条件连续满足的周期数（20 ms 后进入待机） */
    uint32_t fault_recovery_ticks;         /* 新 ADC 连续满足故障恢复条件的控制周期数 */
    uint32_t fault_counter;                /* 连续过流样本计数（达到 3 次触发保护） */
    uint32_t battery_overvoltage_counter;  /* 电池电压连续超限的新 ADC 样本数 */


    float connect_vbat;    /* Q3 接入前的电池电压基线（V），用于检测接入瞬间跳变 */
    float connect_vcap;    /* Q3 接入前的电容电压基线（V），用于检测接入瞬间跳变 */
    float connect_icap;    /* Q3 接入前的电容电流基线（A），用于检测接入瞬间跳变 */
    
    /* 首次故障现场，复位前保持，供调试器定位真实过流与采样尖峰。 */
    uint16_t latest_adc_raw[ADC_CHANNEL_COUNT];   /* 最近一次 ADC 原始值（5 通道） */
    uint16_t fault_adc_raw[ADC_CHANNEL_COUNT];    /* 故障瞬间的 ADC 原始值快照 */
    sc_adc_values_t fault_adc_values;             /* 故障瞬间的电压/电流物理量快照 */
    sc_mode_t fault_mode;                         /* 故障发生时所处的状态机模式 */
    uint32_t fault_mode_ticks;                    /* 故障发生时该模式已持续的周期数 */
    int32_t fault_applied_duty;                   /* 故障瞬间实际下发的占空比（CCR 计数） */
    bool fault_cap_down_ctrl;                     /* 故障瞬间 Q3 接入状态（PA4 电平） */
    bool fault_snapshot_valid;                    /* 故障快照是否有效（故障时有可用 ADC 数据） */
} supercap_state_t;

extern supercap_state_t sc_state;   // 储存超电所有业务状态的统一结构体
/** 板端运行开关：上电默认 0；置 1 从自检开始，清零请求正常停机。 */
extern volatile uint8_t sc_run_enable;

/** 初始化 PI 和输出；默认自动充放电，等待运行开关置 1 后开始自检。 */
void SC_Init(supercap_state_t *state);
/** 板内/调试工作请求接口，首版 CAN 不直接调用；故障恢复仍须满足连续健康条件。 */
void SC_SetRequest(supercap_state_t *state, sc_request_t request);
/** 保留手动复位接口；与自动恢复使用相同的连续健康判据，不跳过等待。 */
void SC_RequestFaultReset(supercap_state_t *state);
/** 应用一帧有效上位机控制参数；接收时间超过控制时限的帧被忽略。 */
void SC_ApplyControlCommand(supercap_state_t *state, float budget_w,
                            uint8_t robot_alive, uint32_t rx_ms, uint32_t now_ms);
/** 应用使能请求：置位运行开关并建立计时，不重置状态机或故障。 */
void SC_ApplyEnableRequest(supercap_state_t *state, uint32_t now_ms);
/** 应用失能请求：清除运行开关，正常停机由 SC_Step 执行。 */
void SC_ApplyDisableRequest(supercap_state_t *state);
/** 推进控制失联计时：已使能后连续无有效控制帧达到时限时恢复默认预算。 */
void SC_AdvanceControlTimeout(supercap_state_t *state, uint32_t now_ms);
/** 将五路 ADC 原始值换算为电压、电流和预留温度量。 */
void SC_ADC_ConvertToPhysical(supercap_state_t *state, const uint16_t *adc_raw);
/** 计算电池、电容功率以及供放电许可使用的负载电流估计值。 */
void SC_CalculatePower(supercap_state_t *state);
/** 新 ADC 快照保护：电容过压立即锁定，电池过压和过流独立连续计数。 */
void SC_RunProtection(supercap_state_t *state);
/** 每次 TIM1 控制中断推进状态机、更新电流目标和 PWM 占空比请求。 */
void SC_Step(supercap_state_t *state);
/** 状态机允许打开 PWM 且占空比在有效范围内时返回 true。 */
bool SC_PWM_OutputRequested(const supercap_state_t *state);
/** 锁存首个故障并禁止 PWM；Q3 按电流消退或续流超时断开。 */
void SC_HandleFault(supercap_state_t *state, sc_fault_code_t code);

/** 复制 CAN 反馈所需的一致快照；调用方负责在外层包住短临界区。 */
void SC_GetFeedbackSnapshot(sc_feedback_snapshot_t *snapshot);
/** 按协议表生成反馈字节 1 的「是否正常运行」位。 */
bool SC_FeedbackNormalRunning(const sc_feedback_snapshot_t *snapshot);
/** 按协议表生成反馈字节 1 的「允许上位机发起供能请求」位。 */
bool SC_FeedbackSupplyRequestAllowed(const sc_feedback_snapshot_t *snapshot);

/** 将数值限制在给定闭区间。 */
static inline float SC_LIMIT(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

#endif
