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
 * GPIO here) and clock two back-to-back 16-bit words inside one CSN-low window. The MT6701
 * drives one LEADING bit before the frame, so the 24 frame bits sit at [30:7] of the 32
 * clocked bits; mt6701_ssi_frame() does that alignment, then mt6701_decode_ssi() CRC6-checks.
 *
 * BENCH-CONFIRMED on the LaunchXL-F280049C SPIB bench (CLK=GPIO22/DO=GPIO31/CSN=GPIO34):
 * SPI mode POL1PHA0 + the manual-CSN timing below read live MT6701 SSI data, and the 1-bit
 * leading offset is real (a >>8 alignment gave CRC 0/6 on real captures, >>7 gives 6/6 and
 * full-revolution tracking). Re-confirm on the esc6288 SPIA + HT0104 path once the board is
 * on the bench -- see boards/esc6288_revA/PORT_TODO.md.
 */

#define MT6701_SSI_BITRATE_HZ   (2500000U)   /* well within the MT6701's SSI clock range */

void MT6701_SSI_init(void)
{
    SPI_disableModule(BOARD_ENC_SPI_BASE);

    /* 16-bit word, master, ~2.5 MHz. POL1PHA0 = CLK idle high, capture on the falling edge
     * (MT6701 SSI mode 2, datasheet sec 6.8.1/6.8.2). */
    SPI_setConfig(BOARD_ENC_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, MT6701_SSI_BITRATE_HZ, 16U);

    /* The shared HAL_setupSPIA() leaves the SPIA FIFOs enabled, but MT6701_SSI_read() uses
     * non-FIFO polling transactions (SPI_transmit16Bits), which driverlib documents must NOT
     * be used in FIFO mode. The encoder owns SPIA exclusively here, so disable the FIFOs. */
    SPI_disableFIFO(BOARD_ENC_SPI_BASE);

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
     * transaction; the 32 captured bits are [leading bit | 24-bit frame | 7 trailing bits]. */
    w0 = (uint16_t)SPI_transmit16Bits(BOARD_ENC_SPI_BASE, 0x0000U);
    w1 = (uint16_t)SPI_transmit16Bits(BOARD_ENC_SPI_BASE, 0x0000U);

    /* CSN high: end the frame. */
    GPIO_writePin(BOARD_ENC_SPI_STE_GPIO, 1U);

    frame24 = mt6701_ssi_frame(w0, w1);   /* align past the 1-bit lead, keep 24 bits */

    (void)mt6701_decode_ssi(frame24, &f);
    *raw14 = f.angle;

    /* A usable angle requires a good CRC, a normal field strength, and no loss of track.
     * The push-button bit (Mg[2]) is informational and does not invalidate the angle. */
    return f.crc_ok && f.field_ok && f.track_ok;
}
