#ifndef DRV8305_H
#define DRV8305_H

//#############################################################################
// DRV8305 3-phase gate driver - SPI register driver (driverlib style)
//
// Register map ported from MotorWare drv8305.h
//   (../esc_drv8300_foc/.../sw/drivers/drvic/drv8305/.../drv8305.h),
// rewritten on top of C2000Ware driverlib SPI (spi.h) for F28004x.
//
// SPI frame (16-bit, MSB first):
//   bit15      R/W  (1 = read, 0 = write)
//   bits14-11  register address
//   bits10-0   data
// On a read, the addressed register contents come back in bits10-0.
//#############################################################################

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"

#define DRV8305_DATA_MASK       (0x07FFU)
#define DRV8305_RW_READ         (1U << 15)
#define DRV8305_RW_WRITE        (0U << 15)

//! \brief Register addresses, pre-shifted into the frame's bits 14-11.
typedef enum
{
    DRV8305_REG_STATUS_1  = (1U  << 11),  //!< Warning and watchdog reset
    DRV8305_REG_STATUS_2  = (2U  << 11),  //!< OV/VDS faults
    DRV8305_REG_STATUS_3  = (3U  << 11),  //!< IC faults
    DRV8305_REG_STATUS_4  = (4U  << 11),  //!< VGS faults
    DRV8305_REG_CONTROL_5 = (5U  << 11),  //!< HS gate drive control
    DRV8305_REG_CONTROL_6 = (6U  << 11),  //!< LS gate drive control
    DRV8305_REG_CONTROL_7 = (7U  << 11),  //!< Gate drive control (PWM mode)
    DRV8305_REG_CONTROL_9 = (9U  << 11),  //!< IC operation
    DRV8305_REG_CONTROL_A = (10U << 11),  //!< Shunt amplifier control (CSA gain)
    DRV8305_REG_CONTROL_B = (11U << 11),  //!< Voltage regulator control
    DRV8305_REG_CONTROL_C = (12U << 11)   //!< VDS sense control
} DRV8305_Address_e;

//! \brief Control Register A (0x0A) shunt-amplifier gain field shifts.
#define DRV8305_CTRLA_GAIN_CS1_S    (0U)
#define DRV8305_CTRLA_GAIN_CS2_S    (2U)
#define DRV8305_CTRLA_GAIN_CS3_S    (4U)

//! \brief CSA gain codes (2-bit). 10 V/V matches USER_ADC_FULL_SCALE_CURRENT_A=47.14.
#define DRV8305_CSA_GAIN_10VPV      (0U)
#define DRV8305_CSA_GAIN_20VPV      (1U)
#define DRV8305_CSA_GAIN_40VPV      (2U)
#define DRV8305_CSA_GAIN_80VPV      (3U)

//! \brief Write one DRV8305 register over SPI.
extern void DRV8305_writeSpi(uint32_t spiBase, DRV8305_Address_e addr,
                             uint16_t data);

//! \brief Read one DRV8305 register over SPI (returns bits 10-0).
extern uint16_t DRV8305_readSpi(uint32_t spiBase, DRV8305_Address_e addr);

//! \brief Apply the known-good power-on configuration (CSA gain 10 V/V,
//!        clear latched faults). Device must already be awake (EN_GATE high).
extern void DRV8305_configure(uint32_t spiBase);

#endif // DRV8305_H
