#include "check.h"
#include "prop_park.h"
#include <math.h>

static prop_park_cfg_t base_cfg(void)
{
    prop_park_cfg_t c = {0};
    c.kp_krpm_per_rev   = 0.15f;
    c.kd_krpm_per_revps = 0.3f;
    c.deadband_rev      = 6.0f / 360.0f;   /* 6 deg */
    c.settle_tol_rev    = 2.0f / 360.0f;   /* 2 deg */
    c.settle_vel_revps  = 0.05f;
    c.settle_hold_s     = 0.05f;
    c.speed_max_krpm    = 0.03f;
    c.min_kick_krpm     = 0.01f;
    c.iq_max_A          = 8.0f;
    c.two_blade         = true;
    c.hyst_rev          = 0.125f;
    c.park_trip_rev     = 30.0f / 360.0f;  /* 30 deg */
    c.park_trip_s       = 0.2f;
    return c;
}

int main(void)
{
    /* Shortest path: target ahead by 0.05 rev -> positive command. */
    {
        prop_park_cfg_t c = base_cfg();
        c.two_blade = false;
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.05f, 0.0f, 0.0f, true, 0.001f, &o);
        CHECK(o.active);
        CHECK_NEAR(o.err_rev, 0.05f, 1e-4f);
        CHECK(o.speed_ref_krpm > 0.0f);
    }

    /* Shortest path across 0: target 0.0, actual 0.9 -> +0.1 err, not -0.9. */
    {
        prop_park_cfg_t c = base_cfg();
        c.two_blade = false;
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.0f, 0.9f, 0.0f, true, 0.001f, &o);
        CHECK_NEAR(o.err_rev, 0.1f, 1e-4f);
    }

    /* mod-180: 2-blade, error 0.4 rev folds to -0.1 (travel <= 0.25). */
    {
        prop_park_cfg_t c = base_cfg(); /* two_blade = true */
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.4f, 0.0f, 0.0f, true, 0.001f, &o);
        CHECK(fabsf(o.err_rev) <= 0.25f + 1e-4f);
        CHECK_NEAR(o.err_rev, -0.1f, 1e-4f);
    }

    /* Deadband: |err| < deadband -> speed_ref forced 0, no kick. */
    {
        prop_park_cfg_t c = base_cfg();
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.005f, 0.0f, 0.0f, true, 0.001f, &o); /* ~1.8 deg < 6 deg */
        CHECK_NEAR(o.speed_ref_krpm, 0.0f, 1e-9f);
    }

    /* Outside deadband with tiny PD output -> min-kick applied. */
    {
        prop_park_cfg_t c = base_cfg();
        c.kp_krpm_per_rev = 0.0001f; /* PD output near zero */
        c.kd_krpm_per_revps = 0.0f;
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.03f, 0.0f, 0.0f, true, 0.001f, &o); /* ~10.8 deg > deadband */
        CHECK_NEAR(fabsf(o.speed_ref_krpm), c.min_kick_krpm, 1e-6f);
        CHECK(o.speed_ref_krpm > 0.0f);
    }

    /* iq_limit always >= 0, even with a negative configured value. */
    {
        prop_park_cfg_t c = base_cfg();
        c.iq_max_A = -5.0f;
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.05f, 0.0f, 0.0f, true, 0.001f, &o);
        CHECK(o.iq_limit_A >= 0.0f);
    }

    /* enc_valid=false -> all zero, inactive. */
    {
        prop_park_cfg_t c = base_cfg();
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.05f, 0.0f, 0.0f, false, 0.001f, &o);
        CHECK_NEAR(o.speed_ref_krpm, 0.0f, 1e-9f);
        CHECK_NEAR(o.iq_limit_A, 0.0f, 1e-9f);
        CHECK(!o.active);
        CHECK(!o.settled);
    }

    /* Settled only after sustained within tolerance + low speed. */
    {
        prop_park_cfg_t c = base_cfg();
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        bool settled_first = false;
        prop_park_step(&c, &st, 0.001f, 0.0f, 0.0f, true, 0.02f, &o);
        settled_first = o.settled;                     /* not yet (0.02 < 0.05 hold) */
        prop_park_step(&c, &st, 0.001f, 0.0f, 0.0f, true, 0.02f, &o);
        prop_park_step(&c, &st, 0.001f, 0.0f, 0.0f, true, 0.02f, &o); /* 0.06 >= 0.05 */
        CHECK(!settled_first);
        CHECK(o.settled);
    }

    /* PARK_TRIP after sustained large error; cleared by reset. */
    {
        prop_park_cfg_t c = base_cfg();
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        for (int i = 0; i < 5; ++i) {
            prop_park_step(&c, &st, 0.3f, 0.0f, 0.0f, true, 0.05f, &o); /* 108 deg > 30 deg */
        }
        CHECK(o.trip);
        prop_park_reset(&st);
        CHECK(!st.trip);
        CHECK_NEAR(st.trip_timer_s, 0.0f, 1e-9f);
    }

    /* Direction hysteresis: once committed, a small opposite error does not flip the latch. */
    {
        prop_park_cfg_t c = base_cfg();
        c.two_blade = false;
        prop_park_state_t st;
        prop_park_reset(&st);
        prop_park_out_t o;
        prop_park_step(&c, &st, 0.30f, 0.0f, 0.0f, true, 0.001f, &o); /* err +0.30 -> latch +1 */
        CHECK_NEAR(st.dir_latch, 1.0f, 1e-9f);
        /* Now a small negative error (within hyst not satisfied since |err|>hyst keeps latch). */
        prop_park_step(&c, &st, -0.20f, 0.0f, 0.0f, true, 0.001f, &o); /* |err|=0.20 > hyst 0.125 */
        CHECK_NEAR(st.dir_latch, 1.0f, 1e-9f);
    }

    CHECK_DONE();
}
