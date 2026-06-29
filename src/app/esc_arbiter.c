#include "esc_arbiter.h"
#include <math.h>

static float clamp01(float v)
{
    if (v < 0.0f) { return 0.0f; }
    if (v > 1.0f) { return 1.0f; }
    return v;
}

esc_src_sample_t esc_pwm_decode(const esc_pwm_decode_cfg_t *cfg,
                                bool fresh, bool overflow, float width_us)
{
    esc_src_sample_t s;
    s.fresh = fresh;
    s.valid = false;
    s.throttle = 0.0f;
    s.arm_req = false;   /* PWM arming is the arbiter's dwell gate, never the decode */

    if (!fresh || overflow) {
        return s;        /* stale or stalled line */
    }
    if (width_us < cfg->us_valid_min || width_us > cfg->us_valid_max) {
        return s;        /* outside the RC servo window -> invalid */
    }
    s.valid = true;
    {
        float span = cfg->us_max - cfg->us_min;
        float t = (span > 0.0f) ? (width_us - cfg->us_min) / span : 0.0f;
        s.throttle = clamp01(t);
    }
    return s;
}

static void src_update(esc_src_state_t *s, const esc_src_sample_t *in,
                       float stale_s, float dt)
{
    if (in->fresh && in->valid) {
        s->seen    = true;            /* cold-boot guard: only a real frame makes us healthy */
        s->age_s   = 0.0f;
        s->throttle = clamp01(in->throttle);
        s->arm_req = in->arm_req;
    } else {
        s->age_s += dt;
    }
    /* NEVER healthy before the first real sample, else a zeroed age_s would make a
     * never-connected source look fresh for one stale window at boot (phantom command). */
    s->healthy = (s->seen && (s->age_s <= stale_s));
    if (!s->healthy) {
        s->throttle = 0.0f;
        s->arm_req  = false;
    }
}

static void pwm_update_arm(esc_src_state_t *p, const esc_arbiter_cfg_t *c, float dt)
{
    if (!p->healthy) {
        p->low_dwell_s = 0.0f;
        p->armed = false;            /* signal loss disarms */
        return;
    }
    if (p->throttle <= c->pwm_arm_throttle_eps) {
        p->low_dwell_s += dt;
        if (p->low_dwell_s >= c->pwm_arm_low_s) { p->armed = true; }
    } else if (!p->armed) {
        p->low_dwell_s = 0.0f;       /* must see sustained low before the FIRST arm */
    }
}

void esc_arbiter_init(esc_arbiter_state_t *st, const esc_arbiter_cfg_t *cfg)
{
    /* Word-wise zero to match the repo's pure-C idiom (product_build_esc_cfg); C28x has
     * no 8-bit storage, so do NOT iterate uint8_t bytes. sizeof of these structs is a
     * whole number of 16-bit words (all fields are >= 16-bit aligned). */
    uint16_t i;
    uint16_t *p = (uint16_t *)st;
    for (i = 0; i < (sizeof(*st) / sizeof(uint16_t)); i++) { p[i] = 0u; }
    st->cfg    = *cfg;
    st->active = ESC_SRC_NONE;
}

/* Emit the selected candidate with handoff slew. Shared by all policies.
 * fresh == the selected source delivered a fresh+valid sample THIS tick; out_seq is
 * bumped only then (see the SEQ CONTRACT on esc_arbiter_state_t.out_seq). */
static void emit(esc_arbiter_state_t *st, esc_src_id_t cand,
                 float target, bool arm, bool fresh, float dt, esc_arbiter_result_t *res)
{
    uint32_t bits = ESC_ARB_ST_NONE;

    if (cand != st->active && cand != ESC_SRC_NONE && st->active != ESC_SRC_NONE) {
        st->blending = true;         /* rate-limit only across a real source change */
    }
    if (cand == ESC_SRC_NONE) {
        st->out_throttle = 0.0f;
        st->blending = false;
    } else if (st->blending) {
        float step = st->cfg.handoff_slew_per_s * dt;
        if (target > st->out_throttle + step)      { st->out_throttle += step; }
        else if (target < st->out_throttle - step) { st->out_throttle -= step; }
        else { st->out_throttle = target; st->blending = false; }
        bits |= ESC_ARB_ST_HANDOFF;
    } else {
        st->out_throttle = target;
    }
    st->active = cand;

    /* Always fully initialize res->cmd (the product main copies it unconditionally), even
     * on NONE where it will not be consumed -- no indeterminate fields in safety code. */
    if (cand != ESC_SRC_NONE && fresh) { st->out_seq++; } /* held -> same seq -> esc_control ages */
    res->cmd.seq      = st->out_seq;
    res->cmd.throttle = (cand == ESC_SRC_NONE) ? 0.0f : st->out_throttle;
    res->cmd.arm      = (cand == ESC_SRC_NONE) ? false : arm;
    res->active       = cand;
    if (cand == ESC_SRC_NONE) {
        res->have_cmd = false;
        bits |= ESC_ARB_ST_NO_SOURCE;
    } else {
        res->have_cmd = true;
        if (cand == ESC_SRC_PWM) { bits |= ESC_ARB_ST_PWM_ACTIVE; }
    }
    res->status_bits = bits;
}

static void pwm_update_track(esc_src_state_t *p, const esc_src_state_t *can,
                             const esc_arbiter_cfg_t *c, float dt)
{
    if (!p->healthy) {
        p->track_dwell_s = 0.0f;
        p->track_ok = false;          /* lost PWM -> ineligible */
        return;
    }
    if (can->healthy) {
        float d = p->throttle - can->throttle;
        if (d < 0.0f) { d = -d; }
        if (d <= c->track_tol) {
            p->track_dwell_s += dt;
            if (p->track_dwell_s >= c->track_required_s) { p->track_ok = true; }
        } else {
            p->track_dwell_s = 0.0f;
            p->track_ok = false;       /* diverged while CAN healthy -> lock out */
        }
    }
    /* CAN unhealthy but PWM healthy: HOLD track_ok latched so a previously-tracking
     * PWM stays eligible to take over. */
}

void esc_arbiter_step(esc_arbiter_state_t *st,
                      const esc_src_sample_t *can, const esc_src_sample_t *pwm,
                      float dt_s, esc_arbiter_result_t *res)
{
    const esc_arbiter_cfg_t *c = &st->cfg;
    esc_src_id_t cand = ESC_SRC_NONE;
    float target = 0.0f;
    bool  arm = false;

    src_update(&st->can, can, c->can_stale_s, dt_s);
    src_update(&st->pwm, pwm, c->pwm_stale_s, dt_s);
    pwm_update_arm(&st->pwm, c, dt_s);
    pwm_update_track(&st->pwm, &st->can, c, dt_s);

    switch (c->policy) {
    case ESC_ARB_EXPLICIT_CAN:
        if (st->can.healthy) { cand = ESC_SRC_CAN; }
        break;
    case ESC_ARB_EXPLICIT_PWM:
        if (st->pwm.healthy && st->pwm.armed) { cand = ESC_SRC_PWM; }
        break;
    case ESC_ARB_CAN_PRIMARY:
    default:
        if (st->can.healthy) {
            cand = ESC_SRC_CAN;
        } else if (st->pwm.healthy && st->pwm.armed && st->pwm.track_ok) {
            cand = ESC_SRC_PWM;
        }
        break;
    }

    if (cand == ESC_SRC_CAN)      { target = st->can.throttle; arm = st->can.arm_req; }
    else if (cand == ESC_SRC_PWM) { target = st->pwm.throttle; arm = st->pwm.armed; }

    {
        /* Fresh ONLY if the winning source delivered a new valid sample this tick. The
         * held case (healthy source, no new frame) re-emits the same seq so esc_control's
         * watchdog ages from the last real frame -- see the SEQ CONTRACT. */
        bool cand_fresh =
            (cand == ESC_SRC_CAN && can->fresh && can->valid) ||
            (cand == ESC_SRC_PWM && pwm->fresh && pwm->valid);
        emit(st, cand, target, arm, cand_fresh, dt_s, res);

        /* Report a PWM that is healthy+armed but not eligible to take over (would-be
         * fallback denied because it never tracked CAN). Only meaningful under CAN_PRIMARY:
         * under EXPLICIT_PWM tracking is not a gate, so this bit would mislead at bench. */
        if (c->policy == ESC_ARB_CAN_PRIMARY &&
            st->pwm.healthy && st->pwm.armed && !st->pwm.track_ok) {
            res->status_bits |= (uint32_t)ESC_ARB_ST_PWM_LOCKOUT;
        }
    }
}
