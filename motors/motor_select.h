// motor_select.h —— 按 BUILD_MOTOR_ID 选择电机 profile
//
// board user.h 在 SDK 示例电机链之前 #include 本文件:
//   - BUILD_MOTOR_ID == TEMPLATE: 不引入 profile, 由 board user.h 的 SDK 示例链供参数(默认)。
//   - 其他: 引入 motors/<型号>.h, 它 #define 全部 USER_MOTOR_*; board user.h 跳过示例链。
// build.sh 按 MOTOR= 注入 -DBUILD_MOTOR_ID(见 config/build_config.h)。
#ifndef MOTOR_SELECT_H
#define MOTOR_SELECT_H

#include "build_config.h"

#if   (BUILD_MOTOR_ID == BUILD_MOTOR_ID_TEMPLATE)
  // 不引入 profile —— 用 board user.h 的 SDK 示例电机(向后兼容默认)。
#elif (BUILD_MOTOR_ID == BUILD_MOTOR_ID_AM_4116_KVA)
  #include "am_4116_kva.h"
#elif (BUILD_MOTOR_ID == BUILD_MOTOR_ID_AM_4116_KVB)
  #include "am_4116_kvb.h"
#elif (BUILD_MOTOR_ID == BUILD_MOTOR_ID_AM_6212)
  #include "am_6212.h"
#elif (BUILD_MOTOR_ID == BUILD_MOTOR_ID_AM_6215)
  #include "am_6215.h"
#else
  #error "motor_select.h: unknown BUILD_MOTOR_ID (see config/build_config.h)"
#endif

#endif // MOTOR_SELECT_H
