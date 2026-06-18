#ifndef LAUNCHXL_DRV8305EVM_BOARD_H
#define LAUNCHXL_DRV8305EVM_BOARD_H

//#############################################################################
// Board:  LAUNCHXL-F280049C + BOOSTXL-DRV8305EVM   (Site 1 / BoosterPack1, J1-J4)
//
// 验证用开发套件(自制板 esc6288_revA 回来前的解耦验证平台)。
// 栅极驱动 = TI DRV8305(SPI 可编程, 3 路内置 CSA, 低边分流)。
//
// 引脚映射来源(SDK 自带 SysConfig, 不靠猜):
//   排针->信号: c2000ware/boards/.meta/LAUNCHXL_F280049C.syscfg.json
//   booster 脚: c2000ware/boards/.meta/boosterpack_json_files/BOOSTXL-DRV8305EVM.syscfg.json
// 定标/寄存器参考(MotorWare, 已验证):
//   电流满量程 47.14A: ../esc_drv8300_foc/.../proj .../user.h (BOOSTXL-DRV8305EVM)
//   DRV8305 寄存器图:  ../esc_drv8300_foc/.../sw/drivers/drvic/drv8305/.../drv8305.h
//#############################################################################

#include "build_config.h"

#define BOARD_NAME                          "launchxl_drv8305evm"

// 本板插在 BoosterPack Site 1 (J1-J4)。三相 PWM 落在 EPWM6/5/3(=旧 HAL 的 J1/J2 路径)。
#define BOARD_LAUNCHPAD_CONNECTOR_J1_J4     (0U)
#define BOARD_LAUNCHPAD_CONNECTOR_J5_J8     (1U)
#define BOARD_LAUNCHPAD_CONNECTOR           BOARD_LAUNCHPAD_CONNECTOR_J1_J4

//----------------------------------------------------------------------------
// 栅极驱动: DRV8305 (SPI 可编程, 非 simple 驱动)
//----------------------------------------------------------------------------
#define BOARD_GATE_DRIVER_DRV8305           (1U)

#define BOARD_GATE_ENABLE_GPIO              (39U)   // EN_GATE, header pin13, active-high
#define BOARD_GATE_WAKE_GPIO               (23U)   // WAKE,    header pin12

#define BOARD_HAS_GATE_FAULT_INPUT          (1U)
#define BOARD_GATE_FAULT_GPIO               (13U)   // nFAULT, header pin3, active-low input
// PWRGD(header pin16)在本 LaunchPad 接到 XRSn(复位脚), 不能当 GPIO 读 -> 用 DRV8305 SPI 状态寄存器代替。
#define BOARD_HAS_GATE_PWRGD_GPIO           (0U)

// DRV8305 SPI = SPIA
#define BOARD_GATE_SPI_BASE                 SPIA_BASE
#define BOARD_GATE_SPI_SIMO_GPIO            (16U)   // header pin15
#define BOARD_GATE_SPI_SOMI_GPIO            (17U)   // header pin14
#define BOARD_GATE_SPI_CLK_GPIO             (56U)   // header pin7
#define BOARD_GATE_SPI_STE_GPIO             (57U)   // header pin19

//----------------------------------------------------------------------------
// 功率级 PWM (互补 6-PWM): U=EPWM6, V=EPWM5, W=EPWM3
//----------------------------------------------------------------------------
#define BOARD_PWM_U_BASE                    EPWM6_BASE   // GPIO10/11, pin40/39
#define BOARD_PWM_V_BASE                    EPWM5_BASE   // GPIO8/9,   pin38/37
#define BOARD_PWM_W_BASE                    EPWM3_BASE   // GPIO4/5,   pin36/35

//----------------------------------------------------------------------------
// 电流采样 (低边分流 -> DRV8305 内置 CSA -> ADC), 带 CMPSS 过流
//   Ia=ADCINB2(CMPSS1), Ib=ADCINC0(CMPSS3), Ic=ADCINA9(CMPSS5)
//----------------------------------------------------------------------------
#define BOARD_NUM_CURRENT_SENSORS           (3U)
// (ADC base/通道/SOC 与 CMPSS 实例在 hal.c 配置, 见 PORT_TODO 阶段2)

//----------------------------------------------------------------------------
// 电压采样: Va=ADCINA5, Vb=ADCINB0, Vc=ADCINC2, Vbus=ADCINB1
//----------------------------------------------------------------------------
#define BOARD_NUM_VOLTAGE_SENSORS           (3U)

//----------------------------------------------------------------------------
// 选板自检: build.sh 通过 -DBUILD_BOARD_ID 选板, 必须与本头匹配
//----------------------------------------------------------------------------
#if (BUILD_BOARD_ID != BUILD_BOARD_ID_LAUNCHXL_DRV8305EVM)
#error "config/build_config.h / -DBUILD_BOARD_ID does not select launchxl_drv8305evm"
#endif

#endif // LAUNCHXL_DRV8305EVM_BOARD_H
