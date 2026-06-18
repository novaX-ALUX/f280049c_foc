#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

#define BUILD_BOARD_ID_ESC6288_REVA          (1U)
#define BUILD_BOARD_ID_LAUNCHXL_DRV8305EVM   (2U)

// 选板事实来源: build.sh 按 BOARD= 注入 -DBUILD_BOARD_ID; 未注入时默认 esc6288_revA。
#ifndef BUILD_BOARD_ID
#define BUILD_BOARD_ID                   BUILD_BOARD_ID_ESC6288_REVA
#endif

#define BUILD_MOTOR_ID_TEMPLATE          (1U)   // 用 board user.h 的 SDK 示例电机(默认)
#define BUILD_MOTOR_ID_AM_4116_KVA       (2U)
#define BUILD_MOTOR_ID_AM_4116_KVB       (3U)
#define BUILD_MOTOR_ID_AM_6212           (4U)
#define BUILD_MOTOR_ID_AM_6215           (5U)

// 选电机事实来源: build.sh 按 MOTOR= 注入 -DBUILD_MOTOR_ID; 未注入时默认 template。
// 选真实电机时, board user.h 跳过 SDK 示例链, 由 motors/<型号>.h 提供 USER_MOTOR_*。
#ifndef BUILD_MOTOR_ID
#define BUILD_MOTOR_ID                   BUILD_MOTOR_ID_TEMPLATE
#endif

#endif
