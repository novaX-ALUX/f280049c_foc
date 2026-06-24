#ifndef LAUNCHXL_3PHGANINV_BOARD_H
#define LAUNCHXL_3PHGANINV_BOARD_H

//#############################################################################
// Board:  LAUNCHXL-F280049C + BOOSTXL-3PhGaNInv   (Site 1 / BoosterPack1, J1-J4)
//
// GaN three-phase inverter EVM (TI SENS007, Rev A): three LMG5200 GaN half-bridge
// modules driven by 6 independent PWM signals through an SN74AVC8T245 buffer with
// an active-LOW output-enable (nEn_uC). Phase current is sensed by external INA240A1
// amplifiers (gain 20 V/V) on 5 mOhm in-line shunts, biased at 1.65 V (REF3333). A
// TMP302 over-temp switch provides a digital, active-low PCB over-temperature alert.
// There is NO SPI gate driver and NO gate-driver fault pin on this board.
//
// Pin mapping sources (cross-referenced, not guesswork):
//   GaN header pin -> signal:  docs/BOOSTXL-3PhGaNInv Schematic (Rev. A).pdf, sheets 4-5
//   Header pin -> F280049C:    docs/Launchxl-f280049c.pdf, sheet 2 (BoosterPack Site 1)
//   Electrical specs:          docs/BOOSTXL-3PhGaNInv Evaluation Module User Guide (Rev. A).pdf
//
// Site 1 analog header (J3, pins 23-30): VDC=A5, VA=B0, VB=C2, VC=B1, IA=B2, IB=C0, IC=A9, VREF=A1
// Site 1 digital header: PWM_AH/AL=GPIO10/11(EPWM6), PWM_BH/BL=GPIO8/9(EPWM5),
//                        PWM_CH/CL=GPIO4/5(EPWM3), nEn_uC=GPIO39(pin13), OT=GPIO58(pin34)
//#############################################################################

#include "build_config.h"

#define BOARD_NAME                          "launchxl_3phganinv"

// Sentinel for "pin not present on this board" (defined locally; not inherited from the clone source).
#define BOARD_GPIO_NONE                     (0xFFFFFFFFUL)

// This board is installed in BoosterPack Site 1. HAL/user.h uses "J1_J2" to denote Site 1 (physically J1-J4 headers).
// The name must match BOOSTX_to_J1_J2/J5_J6 in user.h — otherwise undefined macros in #if default to 0,
// causing both the J1/J2 and J5/J6 branches to compile simultaneously (the latter overrides the former), corrupting the pin mapping.
#define BOARD_LAUNCHPAD_CONNECTOR_J1_J2     (0U)   // Site 1 (J1-J4 headers)
#define BOARD_LAUNCHPAD_CONNECTOR_J5_J6     (1U)   // Site 2 (J5-J8 headers)
#define BOARD_LAUNCHPAD_CONNECTOR           BOARD_LAUNCHPAD_CONNECTOR_J1_J2

// This board's HAL is wired for Site 1 only; prevents accidental selection of Site 2 which would produce a mis-mapped pin configuration.
#if (BOARD_LAUNCHPAD_CONNECTOR != BOARD_LAUNCHPAD_CONNECTOR_J1_J2)
#error "launchxl_3phganinv HAL is wired for Site 1 (J1_J2) only"
#endif

//----------------------------------------------------------------------------
// Gate driver: LMG5200 GaN half-bridges behind an SN74AVC8T245 PWM buffer.
// "Enable" is the buffer output-enable (nEn_uC, active-LOW): drive GPIO39 LOW to pass PWM
// to the LMG5200s; HIGH (or released, held by external pull-up R30) = buffer Hi-Z = both
// FETs off via the on-board pull-downs. The safe-off state is therefore GPIO39 HIGH.
//----------------------------------------------------------------------------
#define BOARD_GATE_DRIVER_LMG5200           (1U)

#define BOARD_GATE_ENABLE_GPIO              (39U)   // nEn_uC, header pin13, ACTIVE-LOW (0=enabled)
#define BOARD_GATE_ENABLE_ACTIVE_LOW        (1U)    // gate_driver.c inverts: enable -> write 0

// This board has NO gate-driver fault pin and NO gate-driver warning pin (no DRV-family chip).
// Kept explicit so the board contract is honest and future product code cannot silently
// reuse a non-existent pin (product_main.c reads BOARD_GATE_FAULT_GPIO when BOARD_HAS_GATE_FAULT_INPUT).
#define BOARD_HAS_GATE_FAULT_INPUT          (0U)
#define BOARD_GATE_FAULT_GPIO               BOARD_GPIO_NONE
#define BOARD_HAS_GATE_WARNING_INPUT        (0U)
#define BOARD_GATE_WARNING_GPIO             BOARD_GPIO_NONE

//----------------------------------------------------------------------------
// Over-temperature: TMP302 digital alert (active-low, header pin34 = GPIO58).
// The HAL reuses the existing external-trip path for this (see hal.h: HAL_PM_nFAULT_GPIO),
// so an over-temp assertion trips the PWM in hardware via the ePWM trip zone.
//----------------------------------------------------------------------------
#define BOARD_HAS_OVERTEMP_INPUT            (1U)
#define BOARD_OVERTEMP_GPIO                 (58U)   // OT, header pin34, active-low input

//----------------------------------------------------------------------------
// Power stage PWM (complementary 6-PWM): physical outputs A=EPWM6, B=EPWM5, C=EPWM3.
// Identical pin map to the BOOSTXL-DRV8305EVM (same LaunchPad Site 1 PWM pins).
// build.sh defaults BUILD_PWM_PHASE_ORDER=4 for this board (verify rotation on the bench).
// NOTE: LMG5200 has NO internal dead-time — the MCU dead-band (hal.h) is the only
// shoot-through protection. See hal.h HAL_PWM_DBRED_CNT/HAL_PWM_DBFED_CNT.
//----------------------------------------------------------------------------
#define BOARD_PWM_U_BASE                    EPWM6_BASE   // PWM_AH/AL, GPIO10/11, pin40/39
#define BOARD_PWM_V_BASE                    EPWM5_BASE   // PWM_BH/BL, GPIO8/9,   pin38/37
#define BOARD_PWM_W_BASE                    EPWM3_BASE   // PWM_CH/CL, GPIO4/5,   pin36/35

//----------------------------------------------------------------------------
// Current sensing (in-line shunt -> INA240 external CSA -> ADC), with CMPSS overcurrent.
//   Ia=ADCINB2(CMPSS3/HP0), Ib=ADCINC0(CMPSS1/HP1), Ic=ADCINA9(CMPSS6/HP3)
//   Identical ADC channels/CMPSS instances to the DRV8305EVM board; on-chip PGAs stay
//   DISABLED because the INA240 drives the ADC pins directly. (Configured in hal.c.)
//----------------------------------------------------------------------------
#define BOARD_NUM_CURRENT_SENSORS           (3U)

//----------------------------------------------------------------------------
// Voltage sensing (100k/4.22k divider, ~81.5 V full-scale):
//   Va=ADCINB0, Vb=ADCINC2, Vc=ADCINB1, Vbus(VDC)=ADCINA5
//   (Remapped vs DRV8305EVM, whose Va=A5/Vbus=B1 — the GaN header order is VDC,VA,VB,VC.)
//----------------------------------------------------------------------------
#define BOARD_NUM_VOLTAGE_SENSORS           (3U)

//----------------------------------------------------------------------------
// DroneCAN transport (CANA): pins are free on Site 1 (GPIO32/33 are on the LaunchPad CAN
// header, not the BoosterPack). Macros kept for the deferred can_bridge.c; no can_bridge.c
// is present yet, so CAN_CHECK/PRODUCT_CHECK friendly-skip.
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
#if (BUILD_BOARD_ID != BUILD_BOARD_ID_LAUNCHXL_3PHGANINV)
#error "config/build_config.h / -DBUILD_BOARD_ID does not select launchxl_3phganinv"
#endif

#endif // LAUNCHXL_3PHGANINV_BOARD_H
