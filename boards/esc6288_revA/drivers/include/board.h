#ifndef ESC6288_REVA_BOARD_H
#define ESC6288_REVA_BOARD_H

#include "build_config.h"

#define BOARD_NAME                          "esc6288_revA"

// ---------------------------------------------------------------------------
// esc6288_revA hardware map (F280049CPMSR, 64-pin PM). Derived from the final
// schematic + netlist (boards/esc6288_revA/esc6288_revA_sch.pdf / .NET).
//
//   Gate driver : U12 JSM6288T, 6 independent inputs, NO enable, NO nFAULT.
//   PWM         : phase A = EPWM1 (GPIO0/1), B = EPWM2 (GPIO2/3), C = EPWM3 (GPIO4/5)
//   Current     : 3x INA181A1 (gain 20) over 0.5 mOhm shunts, 1.65 V mid-rail
//                 (bidirectional). IA=ADCINB15, IB=ADCINA1, IC=ADCINC2.
//                 Only phase C (C2) has a CMPSS path (CMPSS3); A/B are software-OC.
//   Voltage     : 30.1k+30.1k+2k divider (31.1x). Udc=ADCINA6 (CMPSS5 -> bus OV),
//                 UA=ADCINB6, UB=ADCINB3, UC=ADCINC6. NTC=ADCINC3.
//   CAN         : CANA, TX=GPIO37, RX=GPIO35 (SIT1042 transceiver, GH2 conn).
//   Encoder SPI : SPIA, SCLK=GPIO9, SIMO=GPIO16, SOMI=GPIO17, STE=GPIO11 (GH1 conn).
//   RGB LED     : GPIO12 = EPWM7A (WS2812 via SN74LVC1T45, RGB1 + GH3 conn).
//   RC-PWM in   : GPIO33 (PPM/PAD2) -> Input XBAR -> eCAP1.
//   Debug       : 2-wire SWD (TMS=GPIO38, TCK=GPIO36); TDI/TDO freed for CAN.
//   Boot straps : GPIO24, GPIO32 pulled high (no firmware action).
//   Clock       : 10 MHz resonator (CSTNE10M0G) -> PLL IMULT(20) -> 100 MHz.
// ---------------------------------------------------------------------------

//! \brief Illegal GPIO sentinel.
//! Compiles wherever a gate enable/fault "pin" macro is referenced, but must
//! never be driven. JSM6288T has no EN/nFAULT pin; gate_driver.c and the product
//! layer hard-check against this before touching the GPIO.
#define BOARD_GPIO_NONE                     (0xFFFFFFFFUL)

// The HAL/user templates branch on a LaunchPad connector "site". esc6288_revA is
// not a LaunchPad, but the J1/J2 path is the one mapped to this board's hardware,
// so keep that selection so the shared template code resolves to the active path.
#define BOARD_LAUNCHPAD_CONNECTOR_J1_J2     (0U)
#define BOARD_LAUNCHPAD_CONNECTOR_J5_J6     (1U)
#define BOARD_LAUNCHPAD_CONNECTOR           BOARD_LAUNCHPAD_CONNECTOR_J1_J2

// ---- Gate driver: JSM6288T (simple 6-input, no EN, no nFAULT) ----
// Safe-off is the EPWM trip-zone (outputs forced low), NOT a GPIO. The enable/
// fault GPIO macros are the illegal sentinel and exist ONLY so the shared HAL +
// product code that still references them compiles; they are never driven.
#define BOARD_GATE_DRIVER_SIMPLE            (1U)
#define BOARD_GATE_ENABLE_GPIO              BOARD_GPIO_NONE
#define BOARD_HAS_GATE_FAULT_INPUT          (0U)
#define BOARD_GATE_FAULT_GPIO               BOARD_GPIO_NONE
#define BOARD_HAS_GATE_WARNING_INPUT        (0U)
#define BOARD_GATE_WARNING_GPIO             BOARD_GPIO_NONE

// ---- Power-stage PWM (EPWM1/2/3 = phase A/B/C; high=ch A, low=ch B) ----
#define BOARD_PWM_U_BASE                    EPWM1_BASE   // phase A: GPIO0(H)/GPIO1(L)
#define BOARD_PWM_V_BASE                    EPWM2_BASE   // phase B: GPIO2(H)/GPIO3(L)
#define BOARD_PWM_W_BASE                    EPWM3_BASE   // phase C: GPIO4(H)/GPIO5(L)

// ---- Current / voltage sensor counts ----
#define BOARD_NUM_CURRENT_SENSORS           (3U)
#define BOARD_NUM_VOLTAGE_SENSORS           (3U)

// ---- DroneCAN / CAN transport (CANA via SIT1042, GH2 connector) ----
#define BOARD_CAN_BASE                      CANA_BASE
#define BOARD_CAN_TX_GPIO                   (37U)
#define BOARD_CAN_TX_PINCFG                 GPIO_37_CANA_TX
#define BOARD_CAN_RX_GPIO                   (35U)
#define BOARD_CAN_RX_PINCFG                 GPIO_35_CANA_RX
#define BOARD_CAN_BITRATE                   (1000000UL)   // 1 Mbit/s
#define BOARD_CAN_INT                       INT_CANA0

// ---- Encoder SPI (SPIA, MT6701 SSI over the SPI clock, GH1 connector) ----
#define BOARD_ENC_SPI_BASE                  SPIA_BASE
#define BOARD_ENC_SPI_SIMO_GPIO             (16U)
#define BOARD_ENC_SPI_SOMI_GPIO             (17U)
#define BOARD_ENC_SPI_CLK_GPIO              (9U)
#define BOARD_ENC_SPI_STE_GPIO              (11U)

// ---- RGB status LED (GPIO12 = EPWM7A, WS2812) ----
#define BOARD_RGB_GPIO                      (12U)
#define BOARD_RGB_EPWM_BASE                 EPWM7_BASE

// ---- RC-PWM throttle input (GPIO33 -> Input XBAR -> eCAP1) ----
#define BOARD_RCPWM_GPIO                    (33U)
#define BOARD_RCPWM_ECAP_BASE               ECAP1_BASE
#define BOARD_RCPWM_XBAR_INPUT              XBAR_INPUT1
#define BOARD_RCPWM_ECAP_INPUT              ECAP_INPUT_INPUTXBAR1

// ---- Board temperature NTC (NCP18XH103, ADCINC3, ADCC SOC2) ----
// Schematic divider: 3V3 -- NTC1 (thermistor) -- [ADCINC3] -- R14 (10k) -- GND.
// => NTC is HIGH-side, R14 low-side: ntc_low_side = false. Both 10k -> half-scale at 25 C.
// [BENCH] confirm R14 value + the 3V3/VREFHI rail and trim r25/beta to the curve on the bench.
#define BOARD_NTC_ADC_RESULT_BASE           ADCCRESULT_BASE
#define BOARD_NTC_ADC_SOC                   ADC_SOC_NUMBER2
#define BOARD_NTC_R_FIXED_OHM               (10000.0f)   // R14
#define BOARD_NTC_R25_OHM                   (10000.0f)   // NCP18XH103 R25
#define BOARD_NTC_BETA_K                    (3380.0f)    // NCP18XH103 B25/85
#define BOARD_NTC_T0_K                      (298.15f)
#define BOARD_NTC_ADC_FULL_COUNTS           (4096.0f)    // 12-bit ADC
#define BOARD_NTC_LOW_SIDE                  (false)      // NTC to 3V3 (high-side)
#define BOARD_NTC_OPEN_TEMP_C               (150.0f)     // fail-safe hot on open/short

#if (BUILD_BOARD_ID != BUILD_BOARD_ID_ESC6288_REVA)
#error "config/build_config.h does not select esc6288_revA"
#endif

#endif // ESC6288_REVA_BOARD_H
