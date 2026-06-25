#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "mt6701_ssi.h"
#include "mt6701.h"      /* pure SSI frame decode + CRC6 (src/encoder, host-tested) */
#include <stddef.h>

/*
 * MT6701 SSI read over SPIA (datasheet Rev.1.5, sec 6.8). The SSI frame is 24 bits, MSB
 * first: D[13:0] 14-bit angle, Mg[3:0] 4-bit magnetic-field status, CRC[5:0] (poly X^6+X+1
 * over the 18-bit D+Mg). CLK idles HIGH; DO updates on the CLK rising edge and is captured
 * on the falling edge -> SPI mode 2 = POL1PHA0. TCLK >= 64 ns (<= ~15 MHz).
 *
 * A full 24-bit read needs CSN held LOW across the whole frame (the datasheet: transfer
 * starts on CSN low and stops on CSN high). The C28x SPI char length is <= 16 bits and the
 * hardware SPISTE deasserts between chars, so we drive CSN MANUALLY on GPIO11 (re-muxed to
 * GPIO here) and clock two back-to-back 16-bit words inside one CSN-low window. The first
 * 24 of those 32 clocked bits are the frame; the trailing 8 are clocked past the frame end
 * and discarded. The 24-bit frame is then CRC6-checked by mt6701_decode_ssi().
 *
 * BENCH-PENDING: the SSI clock polarity/phase, the manual-CSN timing (TL/TH), and the
 * I2C-vs-SSI default of this MT6701CT-STD part (MODE pin = VDD selects the digital
 * interface; the EEPROM default within it must be confirmed) are all unverified until the
 * board is on the bench -- see boards/esc6288_revA/PORT_TODO.md.
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

    /* Take CSN off the SPI hardware STE and drive it as a plain GPIO so it can be held low
     * across the two 16-bit words of one 24-bit frame. GPIO11 == BOARD_ENC_SPI_STE_GPIO;
     * HAL_setupGPIOs muxed it as SPIA_STE, this overrides that. Idle HIGH (frame inactive). */
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_writePin(BOARD_ENC_SPI_STE_GPIO, 1U);
    GPIO_setDirectionMode(BOARD_ENC_SPI_STE_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(BOARD_ENC_SPI_STE_GPIO, GPIO_PIN_TYPE_PULLUP);
}

bool MT6701_SSI_read(uint16_t *raw14)
{
    uint16_t w0, w1;
    uint32_t frame24;
    mt6701_frame_t f;

    if(raw14 == NULL)
    {
        return false;
    }

    /* CSN low: start the frame. TL (CSN-low -> first CLK) >= 100 ns; SysCtl_delay(5) is
     * ~250 ns @ 100 MHz (each count ~5 cycles), comfortably above the minimum. */
    GPIO_writePin(BOARD_ENC_SPI_STE_GPIO, 0U);
    SysCtl_delay(5U);

    /* 32 contiguous clocks in one CSN-low window. SPI_transmit16Bits is a blocking polling
     * transaction and the received bits are MSB-aligned in the 16-bit word, so:
     *   w0 = frame[23:8]   (D13..D0, Mg3, Mg2)
     *   w1 = frame[7:0] in w1[15:8], then 8 trailing (post-frame) bits in w1[7:0]. */
    w0 = (uint16_t)SPI_transmit16Bits(BOARD_ENC_SPI_BASE, 0x0000U);
    w1 = (uint16_t)SPI_transmit16Bits(BOARD_ENC_SPI_BASE, 0x0000U);

    /* CSN high: end the frame. */
    GPIO_writePin(BOARD_ENC_SPI_STE_GPIO, 1U);

    frame24 = ((uint32_t)w0 << 8) | ((uint32_t)w1 >> 8);

    (void)mt6701_decode_ssi(frame24, &f);
    *raw14 = f.angle;

    /* A usable angle requires a good CRC, a normal field strength, and no loss of track.
     * The push-button bit (Mg[2]) is informational and does not invalidate the angle. */
    return f.crc_ok && f.field_ok && f.track_ok;
}
