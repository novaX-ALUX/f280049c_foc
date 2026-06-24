#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

#define BUILD_BOARD_ID_ESC6288_REVA          (1U)
#define BUILD_BOARD_ID_LAUNCHXL_DRV8305EVM   (2U)
#define BUILD_BOARD_ID_LAUNCHXL_3PHGANINV    (3U)

// Board selection source of truth: build.sh injects -DBUILD_BOARD_ID via BOARD=; defaults to esc6288_revA if not injected.
#ifndef BUILD_BOARD_ID
#define BUILD_BOARD_ID                   BUILD_BOARD_ID_ESC6288_REVA
#endif

#define BUILD_MOTOR_ID_TEMPLATE          (1U)   // use the SDK example motor from board user.h (default)
#define BUILD_MOTOR_ID_AM_4116_KVA       (2U)
#define BUILD_MOTOR_ID_AM_4116_KVB       (3U)
#define BUILD_MOTOR_ID_AM_6212           (4U)
#define BUILD_MOTOR_ID_AM_6215           (5U)

// Motor selection source of truth: build.sh injects -DBUILD_MOTOR_ID via MOTOR=; defaults to template if not injected.
// When a real motor is selected, board user.h skips the SDK example chain; USER_MOTOR_* parameters are provided by motors/<model>.h.
#ifndef BUILD_MOTOR_ID
#define BUILD_MOTOR_ID                   BUILD_MOTOR_ID_TEMPLATE
#endif

// ESC index source of truth: build.sh injects -DBUILD_ESC_INDEX via ESC_INDEX= (validated 0..19 there);
// defaults to 0 if not injected. This is the product main's index into the DroneCAN RawCommand array.
#ifndef BUILD_ESC_INDEX
#define BUILD_ESC_INDEX                  (0)
#endif
#if (BUILD_ESC_INDEX < 0) || (BUILD_ESC_INDEX > 19)
#error "BUILD_ESC_INDEX out of range (must be 0..19, the DroneCAN esc_index space)"
#endif

// Node-id source of truth: build.sh injects -DBUILD_NODE_ID via NODE_ID= (validated 0..127 there).
// 0 = dynamic node-id allocation (DNA, the default H7E/ArduPilot path); 1..127 = static node id
// (skips DNA so a bare CAN tool can drive RawCommand without an allocator on the bus).
#ifndef BUILD_NODE_ID
#define BUILD_NODE_ID                    (0)
#endif
#if (BUILD_NODE_ID < 0) || (BUILD_NODE_ID > 127)
#error "BUILD_NODE_ID out of range (must be 0..127; 0 = dynamic/DNA)"
#endif

// DroneCAN SoftwareVersion.vcs_commit (reported over GetNodeInfo): build.sh injects
// -DBUILD_SW_VCS_COMMIT as the 32-bit git short hash; defaults to 0 (informational only)
// when not injected (host tests / IDE / non-git builds).
#ifndef BUILD_SW_VCS_COMMIT
#define BUILD_SW_VCS_COMMIT              (0u)
#endif

#endif
