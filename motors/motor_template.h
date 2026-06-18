// motor_template.h —— 电机 FOC profile 模板(变化轴 2)
//
// 复制本文件为 motors/<型号>.h, 填入该电机参数。宏名与 SDK 6.0 user.h 的
// USER_MOTOR_* 对齐, 将来可直接被 board user.h #include 做 drop-in。
//
// 字段两类:
//   [输入] 上电/辨识前必须正确给出(几何极对数 + 限流 + 辨识种子)。
//   [输出] 由 is05_motor_id 辨识后回填(Rs / Ls / 磁链)——模板里留空(注释掉)。
#ifndef MOTOR_TEMPLATE_H
#define MOTOR_TEMPLATE_H

#define MOTOR_NAME                        "TEMPLATE"
#define MOTOR_KV_RPM_PER_V                (0)        // [输入] 铭牌 KV(参考/校核用)

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (0)        // [输入] 极对数(几何, 必填; is05 不辨识此项, 错则全错)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)     // PMSM 固定 NULL

#define USER_MOTOR_Rr_Ohm                 (NULL)     // PMSM 无转子电阻
// [输出/种子] Rs/Ls/磁链 —— bench 安全种子, is05_motor_id 辨识后回填覆盖:
#define USER_MOTOR_Rs_Ohm                 (0.02)     // 种子, is05 覆盖
#define USER_MOTOR_Ls_d_H                 (10.0e-6)  // 种子, is05 覆盖
#define USER_MOTOR_Ls_q_H                 (10.0e-6)  // 种子, is05 覆盖
#define USER_MOTOR_RATED_FLUX_VpHz        (0.01)     // 种子, is05 覆盖

// [输入] 辨识种子 / 限流(上电前按电机+电源调; 台架先保守小电流):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)      // 电阻辨识电流
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)     // 电感辨识电流(负)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)      // 最大电流(台架/电源限幅内)
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

#endif // MOTOR_TEMPLATE_H
