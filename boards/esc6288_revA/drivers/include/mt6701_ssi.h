/*
 * mt6701_ssi.h - MT6701 SSI bus-read adapter for esc6288_revA (board/driverlib layer).
 *
 * The pure angle math lives in src/encoder/mt6701.{c,h} (host-tested, no driverlib). The
 * host-test purity gate (tools/test/run.sh) forbids driverlib/hal includes under src/, so
 * this SPI/SSI read adapter lives board-side. The product bridges MT6701_SSI_read() ->
 * mt6701_update() -> esc_feedback_t.
 *
 * Pins (board.h): SPIA, CLK=GPIO9, SIMO=GPIO16, SOMI=GPIO17, STE=GPIO11 (GH1 connector,
 * level-shifted by HT0104 U4).
 */
#ifndef ESC6288_MT6701_SSI_H
#define ESC6288_MT6701_SSI_H

#include <stdint.h>
#include <stdbool.h>

//! \brief Configure SPIA as the MT6701 SSI controller. The SPIA pins are already muxed by
//!        HAL_setupGPIOs; this sets the controller mode/word/baud and enables the module.
void MT6701_SSI_init(void);

//! \brief Read one full 24-bit SSI frame (manual CSN on GPIO11, two 16-bit words). Always
//!        writes the decoded 14-bit angle (0..16383) to *raw14. Returns true only when the
//!        frame is USABLE: CRC6 ok AND field strength normal AND no loss of track (the
//!        decode/CRC lives in src/encoder/mt6701.c and is host-tested). The push-button
//!        status bit is informational and does not gate validity.
//! \note  BENCH-PENDING: SSI clock polarity/phase, manual-CSN timing, and the SSI-vs-I2C
//!        EEPROM default of this MT6701CT-STD part are unverified until the board is on the
//!        bench -- see boards/esc6288_revA/PORT_TODO.md.
bool MT6701_SSI_read(uint16_t *raw14);

#endif /* ESC6288_MT6701_SSI_H */
