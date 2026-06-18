// am_6215.h —— NovaX/AM-6215 FOC profile
// 状态: 空模板。电气参数(Rs/Ls/磁链)待 is05_motor_id 实物辨识后回填。
// (老工程的 6215 参数不可信, 一律以 is05 实测为准。)见 motors/README.md。
#ifndef MOTOR_AM_6215_H
#define MOTOR_AM_6215_H

#define MOTOR_NAME                        "AM-6215"
#define MOTOR_KV_RPM_PER_V                (0)        // [输入] TODO: 填实际 KV

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (14)       // [输入] 6215 = 14 极对(几何; 上电复核)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)

// [输出] 待 is05 辨识回填:
//#define USER_MOTOR_Rs_Ohm               (/* is05 */)
//#define USER_MOTOR_Ls_d_H               (/* is05 */)
//#define USER_MOTOR_Ls_q_H               (/* is05 */)
//#define USER_MOTOR_RATED_FLUX_VpHz      (/* is05 */)

// [输入] 辨识种子 / 限流(上电前按电机+电源调; 台架保守):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)

#endif // MOTOR_AM_6215_H
