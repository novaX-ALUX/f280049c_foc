// motor_select.h -- selects the motor profile based on BUILD_MOTOR_ID
//
// board user.h #includes this file before the SDK example motor chain:
//   - BUILD_MOTOR_ID == TEMPLATE: no profile is included; parameters are supplied by the SDK example chain in board user.h (default).
//   - Otherwise: includes motors/<model>.h, which #defines all USER_MOTOR_*; board user.h skips the example chain.
// build.sh injects -DBUILD_MOTOR_ID via MOTOR= (see config/build_config.h).
#ifndef MOTOR_SELECT_H
#define MOTOR_SELECT_H

#include "build_config.h"

#if   (BUILD_MOTOR_ID == BUILD_MOTOR_ID_TEMPLATE)
  // No profile included -- use the SDK example motor from board user.h (backward-compatible default).
#elif (BUILD_MOTOR_ID == BUILD_MOTOR_ID_AM_4116_KV450)
  #include "am_4116_kv450.h"
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
