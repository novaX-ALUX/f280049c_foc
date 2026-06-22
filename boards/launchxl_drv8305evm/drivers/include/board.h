#ifndef LAUNCHXL_DRV8305EVM_BOARD_H
#define LAUNCHXL_DRV8305EVM_BOARD_H

//#############################################################################
// Board:  LAUNCHXL-F280049C + BOOSTXL-DRV8305EVM   (Site 1 / BoosterPack1, J1-J4)
//
// Validation development kit (decoupled verification platform used before the custom board esc6288_revA is available).
// Gate driver = TI DRV8305 (SPI-programmable, 3-channel integrated CSA, low-side shunt).
//
// Pin mapping sources (from SDK SysConfig, not guesswork):
//   Header pin -> signal: c2000ware/boards/.meta/LAUNCHXL_F280049C.syscfg.json
//   BoosterPack pins:     c2000ware/boards/.meta/boosterpack_json_files/BOOSTXL-DRV8305EVM.syscfg.json
// Scaling / register reference (MotorWare, verified):
//   Current full-scale 47.14A: ../esc_drv8300_foc/.../proj .../user.h (BOOSTXL-DRV8305EVM)
//   DRV8305 register map:      ../esc_drv8300_foc/.../sw/drivers/drvic/drv8305/.../drv8305.h
//#############################################################################

#include "build_config.h"

#define BOARD_NAME                          "launchxl_drv8305evm"

// This board is installed in BoosterPack Site 1. HAL/user.h uses "J1_J2" to denote Site 1 (physically J1-J4 headers).
// The name must match BOOSTX_to_J1_J2/J5_J6 in user.h — otherwise undefined macros in #if default to 0,
// causing both the J1/J2 and J5/J6 branches to compile simultaneously (the latter overrides the former), corrupting the pin mapping.
#define BOARD_LAUNCHPAD_CONNECTOR_J1_J2     (0U)   // Site 1 (J1-J4 headers)
#define BOARD_LAUNCHPAD_CONNECTOR_J5_J6     (1U)   // Site 2 (J5-J8 headers)
#define BOARD_LAUNCHPAD_CONNECTOR           BOARD_LAUNCHPAD_CONNECTOR_J1_J2

// This board's HAL is wired for Site 1 only; prevents accidental selection of Site 2 which would produce a mis-mapped pin configuration.
#if (BOARD_LAUNCHPAD_CONNECTOR != BOARD_LAUNCHPAD_CONNECTOR_J1_J2)
#error "launchxl_drv8305evm HAL is wired for Site 1 (J1_J2) only"
#endif

//----------------------------------------------------------------------------
// Gate driver: DRV8305 (SPI-programmable, not a simple GPIO driver)
//----------------------------------------------------------------------------
#define BOARD_GATE_DRIVER_DRV8305           (1U)

#define BOARD_GATE_ENABLE_GPIO              (39U)   // EN_GATE, header pin13, active-high
#define BOARD_GATE_WAKE_GPIO               (23U)   // WAKE,    header pin12

#define BOARD_HAS_GATE_FAULT_INPUT          (1U)
#define BOARD_GATE_FAULT_GPIO               (13U)   // nFAULT, header pin3, active-low input
// DRV8305 has no dedicated OCTW/warning pin (warnings are read via the SPI status register). Defined to satisfy the
// HAL_PM_nOCTW_GPIO macro in hal.h (currently unreferenced); aliased to the FAULT pin and flagged as not present.
#define BOARD_HAS_GATE_WARNING_INPUT        (0U)
#define BOARD_GATE_WARNING_GPIO             BOARD_GATE_FAULT_GPIO
// PWRGD (header pin16) is tied to XRSn (reset pin) on this LaunchPad and cannot be read as a GPIO -> use the DRV8305 SPI status register instead.
#define BOARD_HAS_GATE_PWRGD_GPIO           (0U)

// DRV8305 SPI = SPIA
#define BOARD_GATE_SPI_BASE                 SPIA_BASE
#define BOARD_GATE_SPI_SIMO_GPIO            (16U)   // header pin15
#define BOARD_GATE_SPI_SOMI_GPIO            (17U)   // header pin14
#define BOARD_GATE_SPI_CLK_GPIO             (56U)   // header pin7
#define BOARD_GATE_SPI_STE_GPIO             (57U)   // header pin19

//----------------------------------------------------------------------------
// Power stage PWM (complementary 6-PWM): U=EPWM6, V=EPWM5, W=EPWM3
//----------------------------------------------------------------------------
#define BOARD_PWM_U_BASE                    EPWM6_BASE   // GPIO10/11, pin40/39
#define BOARD_PWM_V_BASE                    EPWM5_BASE   // GPIO8/9,   pin38/37
#define BOARD_PWM_W_BASE                    EPWM3_BASE   // GPIO4/5,   pin36/35

//----------------------------------------------------------------------------
// Current sensing (low-side shunt -> DRV8305 integrated CSA -> ADC), with CMPSS overcurrent protection
//   Ia=ADCINB2(CMPSS3/HP0), Ib=ADCINC0(CMPSS1/HP1), Ic=ADCINA9(CMPSS6/HP3)
//   (Authoritative values from datasheet Table 6-13; hal.c HAL_setupCMPSSs is wired to CMPSS3/1/6 with
//    these HP/LP mux values, and verified on hardware -- no spurious overcurrent at zero current / PWM on.)
//----------------------------------------------------------------------------
#define BOARD_NUM_CURRENT_SENSORS           (3U)
// (ADC base/channel/SOC and CMPSS instances are configured in hal.c — see PORT_TODO)

//----------------------------------------------------------------------------
// Voltage sensing: Va=ADCINA5, Vb=ADCINB0, Vc=ADCINC2, Vbus=ADCINB1
//----------------------------------------------------------------------------
#define BOARD_NUM_VOLTAGE_SENSORS           (3U)

//----------------------------------------------------------------------------
// DroneCAN transport (CANA, 1 Mbit, 29-bit extended) -- pins per the SDK
// servo_drive_with_can reference (GPIO32 TX / GPIO33 RX on the F280049C LaunchPad).
// Consumed by boards/.../can_bridge.c; values resolve against driverlib (CANA_BASE etc.).
//----------------------------------------------------------------------------
#define BOARD_CAN_BASE                      CANA_BASE
#define BOARD_CAN_TX_GPIO                   (32U)
#define BOARD_CAN_TX_PINCFG                 GPIO_32_CANA_TX
#define BOARD_CAN_RX_GPIO                   (33U)
#define BOARD_CAN_RX_PINCFG                 GPIO_33_CANA_RX
#define BOARD_CAN_BITRATE                   (1000000UL)
#define BOARD_CAN_INT                       INT_CANA0

//----------------------------------------------------------------------------
// Board selection guard: build.sh selects the board via -DBUILD_BOARD_ID, which must match this header
//----------------------------------------------------------------------------
#if (BUILD_BOARD_ID != BUILD_BOARD_ID_LAUNCHXL_DRV8305EVM)
#error "config/build_config.h / -DBUILD_BOARD_ID does not select launchxl_drv8305evm"
#endif

#endif // LAUNCHXL_DRV8305EVM_BOARD_H
