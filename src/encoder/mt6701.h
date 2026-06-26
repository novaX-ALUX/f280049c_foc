/*
 * mt6701.h - Pure angle-processing layer for the MT6701 14-bit absolute encoder.
 *
 * This is ONLY the math: raw 14-bit code -> mechanical/electrical angle, unwrap,
 * velocity estimate, glitch rejection, stale tracking. It touches NO bus and NO
 * driverlib. The bus read layer (MT6701 uses SSI on the esc6288 hardware) is a
 * separate, deferred module (src/encoder/mt6701_ssi.*) that will feed raw codes
 * into mt6701_update(); see src/encoder/README.md.
 *
 * Transform order is fixed and unambiguous:
 *   raw14 -> subtract zero_offset_counts -> multiply by dir -> wrap to [0,1) = position
 *   then: first-frame baseline / wrapped shortest delta / glitch test / unwrap / velocity.
 */
#ifndef MT6701_H
#define MT6701_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- *
 * SSI frame decode (pure): split + CRC6-validate one raw 24-bit MT6701 SSI
 * frame before its 14-bit angle is fed to mt6701_update(). No bus, no driverlib;
 * the board read layer (mt6701_ssi.*) clocks the 24 bits and calls these.
 *
 * Frame (MSB first, datasheet Rev.1.5 sec 6.8.2, Figure 25):
 *   bits [23:10] D[13:0]  14-bit angle    bits [9:6] Mg[3:0] status
 *   bits [5:0]   CRC[5:0] 6-bit CRC over the 18-bit (D[13:0]<<4 | Mg[3:0]), D13 the MSB.
 * Mg status: Mg[1:0] 0=normal/1=field-too-strong/2=field-too-weak; Mg[2]=push button;
 *   Mg[3]=loss of track.
 * CRC: polynomial X^6 + X + 1 (generator 0x03), MSB-first, initial value 0.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t angle;     /* D[13:0], 0..16383 */
    uint16_t mg;        /* Mg[3:0] status nibble */
    uint16_t crc_rx;    /* CRC[5:0] carried in the frame */
    uint16_t crc_calc;  /* CRC[5:0] recomputed over the 18-bit data */
    bool     crc_ok;    /* crc_rx == crc_calc */
    bool     field_ok;  /* Mg[1:0] == 0 (field strength normal) */
    bool     track_ok;  /* Mg[3] == 0 (no loss of track) */
    bool     button;    /* Mg[2] (push button detected; informational) */
} mt6701_frame_t;

/* CRC6 (poly X^6+X+1, MSB-first, init 0) over the low 18 bits of data18. */
uint16_t mt6701_crc6(uint32_t data18);

/*
 * Decode one raw 24-bit SSI frame into *out (NULL allowed). Returns the CRC verdict
 * (crc_rx == crc_calc). A "usable angle" is crc_ok && field_ok && track_ok; the caller
 * (board read layer) is what folds those into the raw_valid passed to mt6701_update().
 */
bool mt6701_decode_ssi(uint32_t frame24, mt6701_frame_t *out);

/*
 * Assemble the 24-bit SSI frame from the two back-to-back 16-bit SPI words of one
 * CSN-low window (w0 clocked first, MSB-first). The MT6701 drives one LEADING bit
 * before the frame once CSN goes low, so the 24 frame bits sit at [30:7] of the 32
 * clocked bits -- NOT [31:8]. The trailing bits are the line idling past frame end
 * and are dropped. Feed the result to mt6701_decode_ssi().
 *
 * Bench-confirmed (LaunchXL-F280049C SPIB): dropping 8 leading bits gave CRC 0/6 on
 * real captures; dropping 7 (this) gives 6/6. See tools/test/test_mt6701.c.
 */
uint32_t mt6701_ssi_frame(uint16_t w0, uint16_t w1);

typedef struct {
    float    counts_per_rev;      /* 16384 for MT6701 */
    int16_t  dir;                 /* +1 or -1 (C28x has no int8_t) */
    uint16_t zero_offset_counts;  /* mechanical zero, in raw counts */
    uint16_t pole_pairs;          /* for electrical angle */
    float    vel_iir_alpha;       /* velocity low-pass, 0..1 */
    float    max_delta_rev;       /* glitch threshold in rev (default 0.25) */
    uint16_t stale_limit_samples; /* consecutive invalid frames -> stale */
    uint16_t glitch_stale_samples;/* consecutive glitches -> stale */
} mt6701_cfg_t;

typedef struct {
    mt6701_cfg_t cfg;
    bool     have_first;     /* first valid frame captured */
    float    position_rev;   /* current wrapped position 0..1 (post offset/dir) */
    int64_t  accum_counts;   /* multi-turn accumulator in counts (int64: no overflow) */
    float    vel_revps;      /* filtered velocity, rev/s */
    bool     valid;          /* last update produced a fresh position */
    bool     stale;          /* sensor considered dead */
    uint16_t invalid_run;    /* consecutive invalid frames */
    uint16_t glitch_run;     /* consecutive glitches */
    uint32_t glitch_count;   /* diagnostic */
} mt6701_state_t;

void mt6701_init(mt6701_state_t *st, const mt6701_cfg_t *cfg);

/*
 * Feed one raw sample. raw_valid is the caller's bus-level validity (NACK/CRC/etc.).
 * A raw14 > 0x3FFF is treated as invalid (bus/framing error surfaced, not masked).
 * dt_s <= 0 holds the previous velocity (no divide-by-zero).
 */
void mt6701_update(mt6701_state_t *st, uint16_t raw14, bool raw_valid, float dt_s);

float mt6701_mech_rev(const mt6701_state_t *st);   /* 0..1 */
float mt6701_mech_rad(const mt6701_state_t *st);
float mt6701_elec_rad(const mt6701_state_t *st);   /* [-pi, pi) */
float mt6701_elec_pu(const mt6701_state_t *st);    /* [0,1) */
float mt6701_vel_revps(const mt6701_state_t *st);
float mt6701_multiturn_rev(const mt6701_state_t *st);
bool  mt6701_valid(const mt6701_state_t *st);
bool  mt6701_stale(const mt6701_state_t *st);

#endif /* MT6701_H */
