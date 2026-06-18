#ifndef ESC6288_REVA_BOARD_H
#define ESC6288_REVA_BOARD_H

#include "build_config.h"

#define BOARD_NAME                          "esc6288_revA"

#define BOARD_LAUNCHPAD_CONNECTOR_J1_J2     (0U)
#define BOARD_LAUNCHPAD_CONNECTOR_J5_J6     (1U)
#define BOARD_LAUNCHPAD_CONNECTOR           BOARD_LAUNCHPAD_CONNECTOR_J1_J2

#define BOARD_GATE_DRIVER_SIMPLE            (1U)
#define BOARD_GATE_ENABLE_GPIO              (13U)

// TODO: confirm this against the esc6288_revA schematic before hardware tests.
#define BOARD_HAS_GATE_FAULT_INPUT          (1U)
#define BOARD_GATE_FAULT_GPIO               (40U)
#define BOARD_HAS_GATE_WARNING_INPUT        (0U)
#define BOARD_GATE_WARNING_GPIO             BOARD_GATE_FAULT_GPIO

#define BOARD_NUM_CURRENT_SENSORS           (3U)
#define BOARD_NUM_VOLTAGE_SENSORS           (3U)

#if (BUILD_BOARD_ID != BUILD_BOARD_ID_ESC6288_REVA)
#error "config/build_config.h does not select esc6288_revA"
#endif

#endif // ESC6288_REVA_BOARD_H
