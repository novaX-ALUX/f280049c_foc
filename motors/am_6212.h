// am_6212.h —— NovaX/AM-6212 FOC profile
// 状态: 空模板。电气参数(Rs/Ls/磁链)待 is05_motor_id 实物辨识后回填。
// 见 motors/README.md 与 motors/motor_template.h。
#ifndef MOTOR_AM_6212_H
#define MOTOR_AM_6212_H

#define MOTOR_NAME                        "AM-6212"
#define MOTOR_KV_RPM_PER_V                (0)        // [输入] TODO: 填实际 KV

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (14)       // [输入] 6212 = 14 极对(几何; 上电复核)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)

#define USER_MOTOR_Rr_Ohm                 (NULL)      // PMSM 无转子电阻
// [输出/种子] Rs/Ls/磁链 —— 现为 bench 安全种子, is05_motor_id 辨识后回填覆盖:
#define USER_MOTOR_Rs_Ohm                 (0.02)      // 种子, is05 覆盖
#define USER_MOTOR_Ls_d_H                 (15.0e-6)   // 种子, is05 覆盖
#define USER_MOTOR_Ls_q_H                 (15.0e-6)   // 种子, is05 覆盖
#define USER_MOTOR_RATED_FLUX_VpHz        (0.012)     // 种子, is05 覆盖

// [输入] 辨识种子 / 限流(上电前按电机+电源调; 台架保守):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)

#define USER_MOTOR_NUM_ENC_SLOTS          (1000)      // 无编码器占位(FAST 无感不用此值)
#define USER_MOTOR_INERTIA_Kgm2           (1.0e-5)    // 占位; 影响速度环前馈, is07 调环时调

// [输入] 运行范围(占位/台架值, 按电机+电源调; 影响 FAST 频率范围与限幅):
#define USER_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR_RATED_SPEED_KRPM       (5.0)
#define USER_MOTOR_FREQ_MIN_HZ            (5.0)
#define USER_MOTOR_FREQ_MAX_HZ            (800.0)
#define USER_MOTOR_FREQ_LOW_HZ            (20.0)
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)
#define USER_MOTOR_VOLT_MIN_V             (4.0)
#define USER_MOTOR_VOLT_MAX_V             (24.0)

#endif // MOTOR_AM_6212_H
