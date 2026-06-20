#include "prop_park.h"
#include <math.h>

void prop_park_reset(prop_park_state_t *st)
{
    st->dir_latch      = 0.0f;
    st->settle_timer_s = 0.0f;
    st->trip_timer_s   = 0.0f;
    st->trip           = false;
}

/* Wrap an error in rev to the shortest path [-0.5, 0.5). */
static float wrap_half(float e)
{
    return e - floorf(e + 0.5f);
}

void prop_park_step(const prop_park_cfg_t *cfg, prop_park_state_t *st,
                    float target_rev, float actual_rev, float vel_revps,
                    bool enc_valid, float dt_s, prop_park_out_t *out)
{
    float dt = (dt_s > 0.0f) ? dt_s : 0.0f;

    /* Encoder invalid -> safe zero, no closed loop, freeze/clear trip timer. */
    if (!enc_valid) {
        st->settle_timer_s = 0.0f;
        st->trip_timer_s   = 0.0f;
        out->speed_ref_krpm = 0.0f;
        out->iq_limit_A     = 0.0f;
        out->settled        = false;
        out->active         = false;
        out->err_rev        = 0.0f;
        out->trip           = st->trip;
        out->trip_timer_s   = st->trip_timer_s;
        return;
    }

    /* Shortest-path error, then mod-180 for a 2-blade prop (worst-case travel <= 90 deg). */
    float err = wrap_half(target_rev - actual_rev);
    if (cfg->two_blade) {
        if (err > 0.25f) {
            err -= 0.5f;
        } else if (err < -0.25f) {
            err += 0.5f;
        }
    }
    float aerr = fabsf(err);
    out->err_rev = err;

    /* Settled: within tolerance AND slow, sustained for settle_hold_s. */
    if (aerr < cfg->settle_tol_rev && fabsf(vel_revps) < cfg->settle_vel_revps) {
        st->settle_timer_s += dt;
    } else {
        st->settle_timer_s = 0.0f;
    }
    out->settled = (st->settle_timer_s >= cfg->settle_hold_s);

    /* PARK_TRIP: |err| beyond park_trip_rev sustained park_trip_s. */
    if (aerr > cfg->park_trip_rev) {
        st->trip_timer_s += dt;
        if (st->trip_timer_s >= cfg->park_trip_s) {
            st->trip = true;
        }
    } else {
        st->trip_timer_s = 0.0f;
    }
    out->trip         = st->trip;
    out->trip_timer_s = st->trip_timer_s;

    out->active     = true;
    out->iq_limit_A = (cfg->iq_max_A >= 0.0f) ? cfg->iq_max_A : 0.0f;

    /* Deadband FIRST: inside -> force 0, release direction latch (no kick, no chatter). */
    if (aerr < cfg->deadband_rev) {
        st->dir_latch = 0.0f;
        out->speed_ref_krpm = 0.0f;
        return;
    }

    /* Direction-latch hysteresis: commit a direction; a positional reversal (sign(err) flips)
     * is allowed only when the error is small. reversal_blocked marks "error wants the other
     * way but we keep the committed direction". */
    float desired = (err > 0.0f) ? 1.0f : -1.0f;
    bool reversal_blocked = false;
    if (st->dir_latch == 0.0f) {
        st->dir_latch = desired;
    } else if (desired != st->dir_latch) {
        if (aerr < cfg->hyst_rev) {
            st->dir_latch = desired;      /* small error -> allow the flip */
        } else {
            reversal_blocked = true;      /* keep the committed direction */
        }
    }

    /* PD -> speed command. */
    float cmd = cfg->kp_krpm_per_rev * err - cfg->kd_krpm_per_revps * vel_revps;

    /* Force the "long way" ONLY when a positional reversal is blocked. When the error still
     * agrees with the committed direction, leave cmd alone so the D term can still brake
     * (a negative cmd that decelerates a fast approach must not be flipped back). */
    if (reversal_blocked) {
        if (st->dir_latch > 0.0f && cmd < 0.0f) {
            cmd = -cmd;
        } else if (st->dir_latch < 0.0f && cmd > 0.0f) {
            cmd = -cmd;
        }
    }

    /* Clamp magnitude. */
    if (cmd > cfg->speed_max_krpm) {
        cmd = cfg->speed_max_krpm;
    } else if (cmd < -cfg->speed_max_krpm) {
        cmd = -cfg->speed_max_krpm;
    }

    /* Min-kick only OUTSIDE the deadband and only when the command is too small. */
    if (fabsf(cmd) < cfg->min_kick_krpm) {
        cmd = st->dir_latch * cfg->min_kick_krpm;
    }

    out->speed_ref_krpm = cmd;
}
