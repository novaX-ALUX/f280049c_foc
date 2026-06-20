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
