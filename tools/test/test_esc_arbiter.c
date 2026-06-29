#include "check.h"
#include "esc_arbiter.h"

static esc_pwm_decode_cfg_t pwm_cfg(void)
{
    esc_pwm_decode_cfg_t c;
    c.us_min = 1000.0f; c.us_max = 2000.0f;
    c.us_valid_min = 900.0f; c.us_valid_max = 2100.0f;
    return c;
}

int main(void)
{
    esc_pwm_decode_cfg_t c = pwm_cfg();

    /* mid pulse -> 0.5, valid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 1500.0f);
        CHECK(s.fresh); CHECK(s.valid);
        CHECK_NEAR(s.throttle, 0.5f, 1e-4f);
        CHECK(!s.arm_req);            /* PWM never self-arms in decode */
    }
    /* min/max clamp to [0,1] */
    {
        esc_src_sample_t lo = esc_pwm_decode(&c, true, false, 1000.0f);
        esc_src_sample_t hi = esc_pwm_decode(&c, true, false, 2000.0f);
        CHECK_NEAR(lo.throttle, 0.0f, 1e-4f);
        CHECK_NEAR(hi.throttle, 1.0f, 1e-4f);
    }
    /* in-window but below us_min still clamps to 0 and stays valid (e.g. 950 us) */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 950.0f);
        CHECK(s.valid);
        CHECK_NEAR(s.throttle, 0.0f, 1e-4f);
    }
    /* outside valid window -> invalid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 800.0f);
        CHECK(!s.valid);
    }
    /* no fresh edge -> not fresh, not valid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, false, false, 1500.0f);
        CHECK(!s.fresh); CHECK(!s.valid);
    }
    /* overflow (line stalled) -> not valid even if width looks fine */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, true, 1500.0f);
        CHECK(!s.valid);
    }

    /* ---- Task 2: health + explicit policies ---- */
    {
        esc_arbiter_cfg_t ac = {0};
        ac.can_stale_s = 0.10f;  ac.pwm_stale_s = 0.06f;
        ac.pwm_arm_low_s = 0.50f; ac.pwm_arm_throttle_eps = 0.05f;
        ac.track_tol = 0.10f;    ac.track_required_s = 0.50f;
        ac.handoff_slew_per_s = 2.0f;
        ac.pwm = pwm_cfg();

        const float dt = 0.001f;
        esc_src_sample_t can = {0}, pwm = {0};
        esc_arbiter_result_t r;

        /* EXPLICIT_CAN: a fresh CAN command is selected immediately, PWM ignored. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.4f; can.arm_req = true;
            pwm.fresh = true; pwm.valid = true; pwm.throttle = 0.9f; /* must be ignored */
            esc_arbiter_step(&st, &can, &pwm, dt, &r);
            CHECK(r.have_cmd);
            CHECK(r.active == ESC_SRC_CAN);
            CHECK_NEAR(r.cmd.throttle, 0.4f, 1e-4f);
            CHECK(r.cmd.arm);
            CHECK(r.cmd.seq != 0u);
        }

        /* CAN goes stale -> no source -> have_cmd false after can_stale_s. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.3f; can.arm_req = true;
            esc_arbiter_step(&st, &can, &pwm, dt, &r);  /* prime */
            esc_src_sample_t none = {0};                /* fresh=false */
            int i; bool coasted = false;
            for (i = 0; i < 200; i++) {                 /* 200 ms > 100 ms stale */
                esc_arbiter_step(&st, &none, &none, dt, &r);
                if (!r.have_cmd) { coasted = true; break; }
            }
            CHECK(coasted);
            CHECK(r.active == ESC_SRC_NONE);
            CHECK(r.status_bits & ESC_ARB_ST_NO_SOURCE);
        }

        /* EXPLICIT_PWM: PWM does NOT arm until it has been valid-low for pwm_arm_low_s. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t plo = {0};
            plo.fresh = true; plo.valid = true; plo.throttle = 0.0f;  /* low */
            int i;
            for (i = 0; i < 100; i++) {                 /* 100 ms < 500 ms dwell */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            }
            CHECK(!r.have_cmd);                         /* not armed yet -> no command */
            for (; i < 600; i++) {                      /* cross 500 ms */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            }
            CHECK(r.have_cmd);                          /* armed now */
            CHECK(r.active == ESC_SRC_PWM);
            CHECK(r.status_bits & ESC_ARB_ST_PWM_ACTIVE);
        }

        /* Stuck-high PWM at boot never sees low -> never arms -> never commands. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t phi = {0};
            phi.fresh = true; phi.valid = true; phi.throttle = 0.8f;  /* stuck high */
            int i;
            for (i = 0; i < 2000; i++) {                /* 2 s of stuck-high */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &phi, dt, &r);
            }
            CHECK(!r.have_cmd);
            CHECK(r.active == ESC_SRC_NONE);
        }

        /* COLD BOOT (seen-gate): with NO CAN ever received, a zeroed arbiter must NOT emit
         * a phantom command. have_cmd stays false so the product passes NULL and
         * esc_control's "never received first frame is not a timeout" invariant holds. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t none = {0};
            int i;
            for (i = 0; i < 50; i++) {                  /* well within can_stale_s (100 ms) */
                esc_arbiter_step(&st, &none, &none, dt, &r);
                CHECK(!r.have_cmd);
                CHECK(r.active == ESC_SRC_NONE);
            }
        }

        /* SEQ CONTRACT: while CAN is healthy but no fresh frame, the emitted seq is HELD
         * (so esc_control's seq-change watchdog ages); a fresh frame bumps it. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.3f; can.arm_req = true;
            esc_arbiter_step(&st, &can, &(esc_src_sample_t){0}, dt, &r);
            uint32_t s0 = r.cmd.seq;
            /* hold: no fresh CAN for several ticks (still within can_stale_s) */
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r);
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r);
            CHECK(r.have_cmd);
            CHECK(r.cmd.seq == s0);                     /* seq HELD while holding */
            CHECK_NEAR(r.cmd.throttle, 0.3f, 1e-4f);    /* last value held */
            /* a fresh frame bumps seq */
            esc_arbiter_step(&st, &can, &(esc_src_sample_t){0}, dt, &r);
            CHECK(r.cmd.seq != s0);
        }

        /* ARM-GATE RESET: PWM arms, loses signal (disarms), and must dwell low AGAIN
         * before it can re-arm -- a brief recovered pulse must not instantly re-arm. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t plo = {0};
            plo.fresh = true; plo.valid = true; plo.throttle = 0.0f;
            int i;
            for (i = 0; i < 600; i++) { esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r); }
            CHECK(r.have_cmd);                          /* armed */
            /* signal lost long enough to go unhealthy (> pwm_stale_s) */
            for (i = 0; i < 100; i++) { esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r); }
            CHECK(!r.have_cmd);                         /* disarmed by loss */
            /* one fresh low pulse: must NOT re-arm yet (dwell restarts) */
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            CHECK(!r.have_cmd);
        }
    }

    CHECK_DONE();
}
