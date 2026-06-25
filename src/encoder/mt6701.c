#include "mt6701.h"
#include <math.h>

#define MT6701_TWO_PI 6.28318530717958647692f
#define MT6701_PI     3.14159265358979323846f
#define MT6701_RAW_MAX 0x3FFFu   /* 14-bit */

/* CRC6, polynomial X^6 + X + 1 (generator 0x03 below x^6), MSB-first, init 0,
 * over the low 18 bits of data18 (D[13:0]<<4 | Mg[3:0], D13 the MSB). */
uint16_t mt6701_crc6(uint32_t data18)
{
    uint16_t crc = 0u;
    int i;
    for (i = 17; i >= 0; --i) {
        uint16_t fb = (uint16_t)(((crc >> 5) ^ (uint16_t)(data18 >> i)) & 1u);
        crc = (uint16_t)((crc << 1) & 0x3Fu);
        if (fb) {
            crc ^= 0x03u;
        }
    }
    return (uint16_t)(crc & 0x3Fu);
}

bool mt6701_decode_ssi(uint32_t frame24, mt6701_frame_t *out)
{
    uint16_t angle  = (uint16_t)((frame24 >> 10) & 0x3FFFu);
    uint16_t mg     = (uint16_t)((frame24 >> 6) & 0x0Fu);
    uint16_t crc_rx = (uint16_t)(frame24 & 0x3Fu);
    uint32_t data18 = ((uint32_t)angle << 4) | (uint32_t)mg;
    uint16_t crc_calc = mt6701_crc6(data18);
    bool ok = (crc_calc == crc_rx);

    if (out != 0) {
        out->angle    = angle;
        out->mg       = mg;
        out->crc_rx   = crc_rx;
        out->crc_calc = crc_calc;
        out->crc_ok   = ok;
        out->field_ok = ((mg & 0x3u) == 0u);   /* Mg[1:0] == normal */
        out->track_ok = ((mg & 0x8u) == 0u);   /* Mg[3]   == no loss of track */
        out->button   = ((mg & 0x4u) != 0u);   /* Mg[2]   == push button */
    }
    return ok;
}

void mt6701_init(mt6701_state_t *st, const mt6701_cfg_t *cfg)
{
    st->cfg = *cfg;
    st->have_first   = false;
    st->position_rev = 0.0f;
    st->accum_counts = 0;
    st->vel_revps    = 0.0f;
    st->valid        = false;
    st->stale        = false;
    st->invalid_run  = 0;
    st->glitch_run   = 0;
    st->glitch_count = 0;
}

/* Wrap x into [0,1). */
static float wrap01(float x)
{
    return x - floorf(x);
}

void mt6701_update(mt6701_state_t *st, uint16_t raw14, bool raw_valid, float dt_s)
{
    const mt6701_cfg_t *c = &st->cfg;

    /* (1) validity: caller-level valid AND a real 14-bit code. */
    if (!raw_valid || raw14 > MT6701_RAW_MAX) {
        st->valid = false;
        if (st->invalid_run < 0xFFFFu) {
            st->invalid_run++;
        }
        if (st->invalid_run >= c->stale_limit_samples) {
            st->stale = true;
        }
        return; /* do not touch position/velocity */
    }
    st->invalid_run = 0;

    /* (2) transform: raw -> offset -> dir -> wrap to [0,1). */
    float dirf = (c->dir < 0) ? -1.0f : 1.0f;
    float pos_counts = ((float)raw14 - (float)c->zero_offset_counts) * dirf;
    float pos_rev = wrap01(pos_counts / c->counts_per_rev);

    /* (3) first valid frame: baseline only, vel=0, no glitch, no big jump. */
    if (!st->have_first) {
        st->have_first   = true;
        st->position_rev = pos_rev;
        st->accum_counts = (int64_t)llroundf(pos_rev * c->counts_per_rev);
        st->vel_revps    = 0.0f;
        st->valid        = true;
        st->stale        = false;
        st->glitch_run   = 0;
        return;
    }

    /* (4) wrapped shortest signed delta FIRST, then glitch test (so 0-crossing
     *     is not mistaken for a glitch). */
    float d = pos_rev - st->position_rev;
    if (d > 0.5f) {
        d -= 1.0f;
    } else if (d < -0.5f) {
        d += 1.0f;
    }

    if (fabsf(d) > c->max_delta_rev) {
        /* glitch: drop sample, do not update position/velocity. */
        st->glitch_count++;
        st->valid = false;
        if (st->glitch_run < 0xFFFFu) {
            st->glitch_run++;
        }
        if (st->glitch_run >= c->glitch_stale_samples) {
            st->stale = true;
        }
        return;
    }
    st->glitch_run = 0;

    /* (5) accept: position, multi-turn accumulator, velocity. */
    st->position_rev = pos_rev;
    st->accum_counts += (int64_t)llroundf(d * c->counts_per_rev);

    if (dt_s > 0.0f) {
        float vel_raw = d / dt_s;
        st->vel_revps += c->vel_iir_alpha * (vel_raw - st->vel_revps);
    } /* dt_s <= 0: keep previous velocity */

    st->valid = true;
    st->stale = false;
}

float mt6701_mech_rev(const mt6701_state_t *st)
{
    return st->position_rev;
}

float mt6701_mech_rad(const mt6701_state_t *st)
{
    return st->position_rev * MT6701_TWO_PI;
}

float mt6701_elec_pu(const mt6701_state_t *st)
{
    float e = st->position_rev * (float)st->cfg.pole_pairs;
    return wrap01(e);
}

float mt6701_elec_rad(const mt6701_state_t *st)
{
    float r = mt6701_elec_pu(st) * MT6701_TWO_PI;
    if (r >= MT6701_PI) {
        r -= MT6701_TWO_PI;
    }
    return r; /* [-pi, pi) */
}

float mt6701_vel_revps(const mt6701_state_t *st)
{
    return st->vel_revps;
}

float mt6701_multiturn_rev(const mt6701_state_t *st)
{
    return (float)st->accum_counts / st->cfg.counts_per_rev;
}

bool mt6701_valid(const mt6701_state_t *st)
{
    return st->valid;
}

bool mt6701_stale(const mt6701_state_t *st)
{
    return st->stale;
}
