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

//! \brief Read one SSI frame. On success writes the 14-bit absolute angle code (0..16383)
//!        to *raw14 and returns true.
//! \note  The SSI clock polarity/phase, exact frame width, and the CRC/parity field MUST be
//!        confirmed against the MT6701 datasheet on the bench; this returns the angle field
//!        only and does not yet validate CRC (returns true whenever the transfer completes).
bool MT6701_SSI_read(uint16_t *raw14);

#endif /* ESC6288_MT6701_SSI_H */
