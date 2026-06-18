//#############################################################################
// DRV8305 SPI register driver - implementation (driverlib SPI)
//
// Assumes the SPI peripheral is already configured by HAL_setupSPIA():
//   16-bit chars, master, POL0PHA0 (output rising / input falling edge,
//   matching the DRV8305 timing), FIFO enabled.
//#############################################################################

#include "drv8305.h"

//! \brief Build the 16-bit control word. addr already carries its shifted bits.
static inline uint16_t DRV8305_buildCtrlWord(uint16_t rw,
                                             DRV8305_Address_e addr,
                                             uint16_t data)
{
    return (uint16_t)(rw | (uint16_t)addr | (data & DRV8305_DATA_MASK));
}

void DRV8305_writeSpi(uint32_t spiBase, DRV8305_Address_e addr, uint16_t data)
{
    uint16_t ctrlWord = DRV8305_buildCtrlWord(DRV8305_RW_WRITE, addr, data);

    SPI_resetRxFIFO(spiBase);
    SPI_writeDataBlockingFIFO(spiBase, ctrlWord);

    // Every transfer also shifts in a word; drain it to keep the RX FIFO balanced.
    (void)SPI_readDataBlockingFIFO(spiBase);

    return;
}

uint16_t DRV8305_readSpi(uint32_t spiBase, DRV8305_Address_e addr)
{
    uint16_t ctrlWord = DRV8305_buildCtrlWord(DRV8305_RW_READ, addr, 0U);
    uint16_t readWord;

    SPI_resetRxFIFO(spiBase);
    SPI_writeDataBlockingFIFO(spiBase, ctrlWord);
    readWord = SPI_readDataBlockingFIFO(spiBase);

    return (uint16_t)(readWord & DRV8305_DATA_MASK);
}

void DRV8305_configure(uint32_t spiBase)
{
    // Reading the status registers clears latched warnings/faults.
    (void)DRV8305_readSpi(spiBase, DRV8305_REG_STATUS_1);
    (void)DRV8305_readSpi(spiBase, DRV8305_REG_STATUS_2);
    (void)DRV8305_readSpi(spiBase, DRV8305_REG_STATUS_3);
    (void)DRV8305_readSpi(spiBase, DRV8305_REG_STATUS_4);

    // Shunt amplifier: 10 V/V on CS1/CS2/CS3 (matches the 47.14 A full-scale in
    // user.h). Gain code 0 = 10 V/V; DC-cal off, default blanking.
    DRV8305_writeSpi(spiBase, DRV8305_REG_CONTROL_A,
        (uint16_t)((DRV8305_CSA_GAIN_10VPV << DRV8305_CTRLA_GAIN_CS1_S) |
                   (DRV8305_CSA_GAIN_10VPV << DRV8305_CTRLA_GAIN_CS2_S) |
                   (DRV8305_CSA_GAIN_10VPV << DRV8305_CTRLA_GAIN_CS3_S)));

    // Gate drive (Control 5/6/7), IC operation (9), VREG (B) and VDS sense (C)
    // are left at DRV8305 power-on defaults: 6-PWM mode, EVM-tuned gate currents
    // and dead time. Overwriting these blindly is riskier than the defaults.
    // TODO(hardware): confirm 6-PWM mode (Control 7) and tune VDS OCP threshold /
    //   gate-drive current / dead time for the actual test motor before is05+.

    return;
}
