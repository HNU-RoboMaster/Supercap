/**
 * @file    supercap_config.h
 * @brief   超级电容管理系统所有可调常量
 *
 * 将所有 #define 阈值、PI 增益和限幅集中在一处，
 * 整定时无需在多个源文件中查找。
 */

#ifndef SUPERCAP_CONFIG_H
#define SUPERCAP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
   ADC 通道数
   ======================================================================== */
#define ADC_CHANNEL_COUNT       5

/* ========================================================================
   固定 PWM 硬件调试
   ======================================================================== */
/** 1：跳过状态机、PI 和软件保护，固定 PWM 并拉高 CapDownCtrl。
 *  0：正常状态机控制。
 * 调试模式不受 sc_run_enable 控制。 */
#ifndef SC_DEBUG_MODE
#define SC_DEBUG_MODE           0
#endif

/** 调试 CCR3 值：TIM1 ARR=399，200 对应 50% 占空比。 */
#define SC_DEBUG_PWM_DUTY       (200U)

#if (SC_DEBUG_MODE != 0) && (SC_DEBUG_MODE != 1)
#error "SC_DEBUG_MODE must be 0 or 1"
#endif

/* ========================================================================
   电流限幅（单位：安培，由 ADC 测量）
   ======================================================================== */

/** 充电目标下限（负值 = 充电），与 -10 A 保护门限留出 2 A 调节余量。 */
#define ICAP_CHARGE_MAX         (-8.0f)

/** 放电目标上限，与 +10 A 保护门限留出 2 A 调节余量。 */
#define ICAP_DISCHARGE_MAX      (8.0f)

/** 实测充电过流保护门限，不作为正常充电目标。 */
#define ICAP_MIN                (-10.0f)

/** 实测放电过流保护门限，不作为正常放电目标。 */
#define ICAP_MAX                (10.0f)


/** 软启动期间的恒流设定点（充电） */
#define ICAP_CONSTANT_CURRENT   (-3.0f)

/* ========================================================================
   电压限幅
   ======================================================================== */

/** 软启动切换阈值 — 高于此电压进入正常模式 */
#define SUPERCAP_VOLTAGE_CC     (7.0f)

/** 超级电容最高电压，超出则过压故障 */
#define SUPERCAP_VOLTAGE_MAX    (26.0f)

/** 电池过压门限，连续超限次数超过下方限制后锁定故障 */
#define BATTERY_VOLTAGE_MAX     (28.0f)

/** 电池电压连续 >28 V 超过 5 次，即第 6 个新 ADC 样本触发保护。 */
#define SC_BATTERY_OVERVOLTAGE_COUNT_LIMIT (5U)

/* ========================================================================
   电池电流限幅
   ======================================================================== */

/** 电池最大电流，超出则过流故障 */
#define BATTERY_CURRENT_MAX     (10.0f)

/* ========================================================================
   PWM 占空比限幅（TIM1 ARR = 399）
   ======================================================================== */

/** PWM 占空比下限 */
#define PWM_DUTY_MIN            (5)

/** PWM 占空比上限（ARR=399，留 10 计数余量） */
#define PWM_DUTY_MAX            (389)

#if SC_DEBUG_MODE && ((SC_DEBUG_PWM_DUTY < PWM_DUTY_MIN) || \
                      (SC_DEBUG_PWM_DUTY > PWM_DUTY_MAX))
#error "SC_DEBUG_PWM_DUTY is outside the allowed PWM range"
#endif

/* ========================================================================
   前馈控制常量
   ======================================================================== */

/** 占空比前馈比例因子：Duty_ff = (Vcap / Vbat) * DUTY_FF_SCALE。
 *  值 400.0 = ARR + 1，将电压比转换为 CCR 计数值。 */
#define DUTY_FF_SCALE           (400.0f)

/** 电流环 PI trim 输出下限（CCR 计数值，约占空比 -7.5%） */
#define CURRENT_PI_TRIM_MIN     (-30)

/** 电流环 PI trim 输出上限（CCR 计数值，约占空比 +7.5%） */
#define CURRENT_PI_TRIM_MAX     (30)

/** 电流环积分分离阈值（安培）：跟踪误差超过 2 A 时冻结积分。 */
#define ICAP_I_THRESHOLD        (2.0f)

/* ========================================================================
   功率控制默认值
   ======================================================================== */

/** 电池侧功率预算的重启/失联兜底值（W）；CAN 动态预算应写入 power_reference，不修改此宏。 */
#define POWER_REFERENCE_DEFAULT (60.0f)

/** 实际调节目标为 max(0, 参考上限 - 余量)，为误差与动态响应预留空间。 */
#define SC_BATTERY_POWER_RESERVE_W (2.0f)
/** 只在恢复充电侧留回差；任何正的功率缺口都允许请求放电。 */
#define SC_AUTO_CHARGE_HYST_W   (1.0f)
/** 自动模式低压停放/充满停充的恢复门限，避免电压回弹后反复启停。 */
#define SC_AUTO_DISCHARGE_RESUME_VOLTAGE (8.0f)
#define SC_AUTO_CHARGE_RESTART_VOLTAGE  (25.0f)


/* ========================================================================
   CAN 下行控制
   ======================================================================== */

/** 上位机控制帧超时（ms）：使能后连续无有效功率指令达到该值，
 *  恢复 POWER_REFERENCE_DEFAULT；收到新的有效功率指令即切回上位机预算。 */
#define SC_CONTROL_LOSS_TIMEOUT_MS (500U)

/* ========================================================================
   CAN 反馈判据
   ======================================================================== */

/** 反馈 bit 5 的供能请求许可电压：电容组电压须严格大于该值。 */
#define SC_FEEDBACK_SUPPLY_REQUEST_VOLTAGE  (8.0f)

/* ========================================================================
   PI 控制器参数
   ======================================================================== */

/* ----- 内环电流环（以 TIM1 全速 100 kHz 运行） ----- */
/* 注意：前馈承担主体后，PI 仅做微调，增益相应缩小 */
#define PI_ICAP_KP              (1.0f)
#define PI_ICAP_KI              (200.0f)
#define PI_ICAP_TS              (0.00001f)   /* 10 µs = 1/100 kHz，与内环实际周期一致 */

/* 电池功率外环：1 kHz，修正损耗、采样与电流跟踪误差。 */
#define SC_POWER_LOOP_DECIMATION (100U)
#define PI_PBAT_KP              (0.2f)
#define PI_PBAT_KI              (20.0f)
#define PI_PBAT_TS              ((float)SC_POWER_LOOP_DECIMATION / (float)SC_CONTROL_HZ)
#define SC_POWER_TRIM_MAX_W     (20.0f)

/* ========================================================================
   采样与保护常量
   ======================================================================== */

/**
 * 连续 ADC 数据丢失次数阈值，超出则判定 ADC 离线故障。
 */
#define ADC_OFFLINE_LIMIT       (500) /* 100 kHz 下约 5 ms，允许 DMA 与中断存在相位差 */

/**
 * 同一种过流连续超限采样次数，达到阈值则保护跳闸。
 * 电池过压使用独立计数；电容过压立即跳闸。
 */
#define PROTECTION_THRESHOLD    (3)

/* ========================================================================
   PWM 缓升缓降速率限制
   控制周期：100 kHz（10 µs）
   ======================================================================== */

/**
 * 每个控制周期 PWM 占空比最大增量（上升速率限制）。
 *
 *   10 计数值/周期 = 每周期 2.5% 占空比变化（@100 kHz）。
 *   0→389 满幅扫描：389 / 10 × 10 µs ≈ 390 µs。
 *
 * 整定指导：
 *   - 如果设定点跟踪在缓升过程中感觉迟钝，可适当增大此值。
 *   - 如果在较大设定点变化或模式切换时示波器上观测到电流尖峰，
 *     可适当减小此值。
 *   - 稳态时内环电流 PI 每周期仅变化约 1-2 个计数值，
 *     因此 10-20 的值对正常 PI 动态基本透明，同时仍能限制瞬态跳变。
 */
#define PWM_SLEW_RATE_UP       (10)

/**
 * 每个控制周期 PWM 占空比最大减量（下降速率限制）。
 *
 * 通常设得比 PWM_SLEW_RATE_UP 更大，因为减小占空比
 * 会减少通过变换器的能量传输，属于更安全的方向。
 */
#define PWM_SLEW_RATE_DOWN     (10)

/* 状态机计时单位为 100 kHz 控制周期；SC_Step 每两个 TIM1 更新中断执行一次。 */
#define SC_CONTROL_HZ           (100000U) /* TIM1 控制频率：100 kHz */
#define SC_SELF_TEST_SETTLE_TICKS (SC_CONTROL_HZ / 5U)  /* 上电后先等待采样稳定：200 ms */
#define SC_SELF_TEST_STABLE_TICKS (SC_CONTROL_HZ / 50U) /* 合格采样连续保持：20 ms */
#define SC_SELF_TEST_TIMEOUT      (SC_CONTROL_HZ / 2U)  /* 自检最长等待：500 ms */
#define SC_CONNECT_SETTLE_TICKS (SC_CONTROL_HZ / 100U) /* Q3 接入稳定观察：10 ms */
#define SC_CONNECT_TIMEOUT      (SC_CONTROL_HZ / 5U)   /* Q3 接入最长等待：200 ms */
#define SC_PWM_START_TICKS      (2U) /* 平衡占空比预装载及 MOE 开放后，再交给电流环 */
#define SC_ZERO_STABLE_TICKS    (SC_CONTROL_HZ / 200U) /* 电流连续近零：5 ms */
#define SC_ZERO_TIMEOUT        (SC_CONTROL_HZ / 2U)    /* 换向过零超时：500 ms */
#define SC_STOP_TIMEOUT        (SC_CONTROL_HZ / 2U)    /* 正常停机超时：500 ms */
#define SC_LOAD_EXIT_TICKS     (SC_CONTROL_HZ / 10U)   /* 低负载持续：100 ms */
#define SC_FAULT_FREEWHEEL_TICKS (SC_CONTROL_HZ / 100U) /* 故障后 Q3 最长续流：10 ms */
#define SC_FAULT_RECOVERY_STABLE_TICKS (SC_CONTROL_HZ) /* 新 ADC 连续健康 1 s 后自动恢复 */

/* 电压门限，单位 V。以下数值需要结合实测硬件整定。 */
#define SC_VBAT_MIN             (8.0f)  /* 母线最低工作电压 */
#define SC_VCAP_MIN_VALID       (0.4f)  /* 自检时电容电压最低可信值 */
#define SC_VCAP_DISCHARGE_MIN   (7.0f)  /* 不高于此值时放电目标为 0，保持接入 */
#define SC_VCAP_CHARGE_TAPER    (24.0f) /* 充电电流开始收小 */
#define SC_CHARGE_DONE_VOLTAGE  (25.5f) /* 充满并正常停止 */

/* 电流及接入判据，单位 A；接入电压跳变判据单位 V。 */
#define SC_CURRENT_ZERO_A      (0.25f) /* 换向/停机的实测电流近零门限 */
#define SC_SELF_TEST_ZERO_A    (0.6f)  /* 自检时电流零点容差 */
#define SC_CONNECT_JUMP_A      (2.0f)  /* Q3 接入瞬间允许的电流变化 */
#define SC_CONNECT_JUMP_V      (5.0f)  /* Q3 接入瞬间允许的电压变化 */
#define SC_LOAD_ENTER_A        (3.0f)  /* 放电进入负载门限 */
#define SC_LOAD_EXIT_A         (2.0f)  /* 放电退出负载门限，形成回差 */
#define SC_CURRENT_SLEW_A      (0.002f) /* 每 10 us 电流目标最多变化 0.002 A */
#define SC_CHARGE_BAT_MARGIN_A  (3.0f)  /* 电池最大电流预留裕量 */

#ifdef __cplusplus
}
#endif

#endif /* SUPERCAP_CONFIG_H */
