#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "mt6701_ssi.h"
#include <stddef.h>

/*
 * MT6701 SSI read over SPIA (datasheet Rev.1.5, sec 6.8). The SSI frame is 24 bits, MSB
 * first: D[13:0] 14-bit angle, Mg[3:0] 4-bit magnetic-field status, CRC[5:0] (poly X^6+X+1
 * over the 18-bit D+Mg). CLK idles HIGH; DO updates on the CLK rising edge and is captured
 * on the falling edge -> SPI mode 2 = POL1PHA0. TCLK >= 64 ns (<= ~15 MHz).
 *
 * We clock a single 16-bit word (D13..D0 then Mg3,Mg2) and take the top 14 bits as the
 * angle: (rx >> 2) & 0x3FFF. Full Mg-status + CRC validation needs the remaining 8 bits,
 * i.e. CSN held low across a 16+8-bit read -- that requires manual CSN (GPIO) control
 * instead of the SPI auto-STE and is a bench enhancement; for now bus validity is assumed.
 */

#define MT6701_SSI_BITRATE_HZ   (2500000U)   /* well within the MT6701's SSI clock range */

void MT6701_SSI_init(void)
{
    SPI_disableModule(BOARD_ENC_SPI_BASE);

    /* 16-bit word, master, ~2.5 MHz. POL1PHA0 = CLK idle high, capture on the falling edge
     * (MT6701 SSI mode 2, datasheet sec 6.8.1/6.8.2). */
    SPI_setConfig(BOARD_ENC_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, MT6701_SSI_BITRATE_HZ, 16U);

    SPI_disableLoopback(BOARD_ENC_SPI_BASE);
    SPI_setEmulationMode(BOARD_ENC_SPI_BASE, SPI_EMULATION_FREE_RUN);

    SPI_enableModule(BOARD_ENC_SPI_BASE);
}

bool MT6701_SSI_read(uint16_t *raw14)
{
    uint16_t rx;

    if(raw14 == NULL)
    {
        return false;
    }

    /* One 16-bit SSI frame. TX data is don't-care (read-only device); the controller drives
     * the clock and STE. Top 14 bits of the 16-bit response are the angle. */
    rx = (uint16_t)SPI_transmit16Bits(BOARD_ENC_SPI_BASE, 0x0000U);
    *raw14 = (uint16_t)((rx >> 2) & 0x3FFFU);

    /* No CRC check yet (see BENCH TODO) -- the transfer itself always completes. */
    return true;
}
