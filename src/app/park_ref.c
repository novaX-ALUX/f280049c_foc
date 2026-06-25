#include "park_ref.h"
#include <stddef.h>
#include <math.h>

void park_ref_init(park_ref_state_t *st, const park_ref_cfg_t *cfg,
                   park_ref_load_fn load, void *ctx)
{
    st->cfg           = *cfg;
    st->still_timer_s = 0.0f;
    st->needs_store   = false;
    st->new_target    = 0.0f;

    if (load != NULL) {
        bool v = false;
        float t = load(ctx, &v);
        st->valid      = v;
        st->target_rev = v ? t : 0.0f;
    } else {
        st->valid      = false;
        st->target_rev = 0.0f;
    }
}

void park_ref_update(park_ref_state_t *st, bool disarmed, bool armed, float throttle,
                     float speed_est_krpm, float enc_vel_revps, bool enc_valid,
                     float enc_mech_rev, float dt_s)
{
    const park_ref_cfg_t *c = &st->cfg;

    if (st->valid) {
        st->still_timer_s = 0.0f;
        return; /* already learned, nothing to do */
    }

    bool gate = disarmed
             && !armed
             && (throttle <= c->throttle_eps)
             && (fabsf(speed_est_krpm) < c->speed_thresh_krpm)
             && (fabsf(enc_vel_revps) < c->enc_vel_thresh_revps)
             && enc_valid;

    if (!gate) {
        st->still_timer_s = 0.0f; /* any disturbance resets the dwell */
        return;
    }

    st->still_timer_s += (dt_s > 0.0f) ? dt_s : 0.0f;
    if (st->still_timer_s >= c->learn_hold_s) {
        st->target_rev    = enc_mech_rev;
        st->valid         = true;
        st->new_target    = enc_mech_rev;
        st->needs_store   = true;   /* request persist; module itself never writes Flash */
        st->still_timer_s = 0.0f;
    }
}

void park_ref_invalidate(park_ref_state_t *st)
{
    st->valid         = false;
    st->still_timer_s = 0.0f;
    /* Drop any store request still pending from a prior capture: otherwise the app would persist
     * the now-invalidated angle and skip the re-learn on the next boot. */
    st->needs_store   = false;
    st->new_target    = 0.0f;
}

void park_ref_clear_store_request(park_ref_state_t *st)
{
    st->needs_store = false;
}

bool park_ref_valid(const park_ref_state_t *st)
{
    return st->valid;
}

float park_ref_target(const park_ref_state_t *st)
{
    return st->target_rev;
}
