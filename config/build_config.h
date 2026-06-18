#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

#define BUILD_BOARD_ID_ESC6288_REVA          (1U)
#define BUILD_BOARD_ID_LAUNCHXL_DRV8305EVM   (2U)

// 选板事实来源: build.sh 按 BOARD= 注入 -DBUILD_BOARD_ID; 未注入时默认 esc6288_revA。
#ifndef BUILD_BOARD_ID
#define BUILD_BOARD_ID                   BUILD_BOARD_ID_ESC6288_REVA
#endif

#define BUILD_MOTOR_ID_TEMPLATE          (1U)
#define BUILD_MOTOR_ID                   BUILD_MOTOR_ID_TEMPLATE

#endif
