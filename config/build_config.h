#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H
// 选择当前构建的 {板 × 电机}。也可由 build.sh 的 BOARD/MOTOR 覆盖。
#define BUILD_BOARD   esc6288_revA      // 见 boards/
#define BUILD_MOTOR   motor_template    // 见 motors/
#endif
