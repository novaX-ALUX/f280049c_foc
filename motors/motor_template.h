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

// [输出] 待 is05_motor_id 辨识后回填(取消注释并填值):
//#define USER_MOTOR_Rs_Ohm               (/* is05 */)
//#define USER_MOTOR_Ls_d_H               (/* is05 */)
//#define USER_MOTOR_Ls_q_H               (/* is05 */)
//#define USER_MOTOR_RATED_FLUX_VpHz      (/* is05 */)

// [输入] 辨识种子 / 限流(上电前按电机+电源调; 台架先保守小电流):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)      // 电阻辨识电流
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)     // 电感辨识电流(负)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)      // 最大电流(台架/电源限幅内)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)     // 磁链激励频率

#endif // MOTOR_TEMPLATE_H
