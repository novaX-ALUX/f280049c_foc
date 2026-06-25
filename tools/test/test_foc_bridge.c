#include "check.h"
#include "foc_bridge.h"
#include <math.h>

static foc_bridge_cfg_t base_cfg(void)
{
    foc_bridge_cfg_t c = {0};
    c.pole_pairs    = 7.0f;     /* AM-4116: 14 poles -> 7 pole pairs */
    c.iq_cmd_limit_A = 20.0f;
    c.speed_max_hz   = 1000.0f; /* high enough not to clamp the nominal-speed cases */
    return c;
}

int main(void)
{
    /* torque mode: iq passes through, speed_ref 0, speed_mode false, flags pass through. */
    {
        foc_bridge_cfg_t c = base_cfg();
        esc_output_t out = {0};
        out.mode = ESC_CTRL_TORQUE;
        out.iq_ref_A = 5.5f;
        out.speed_ref_krpm = 0.0f;
        out.iq_limit_A = 0.0f;
        out.enable = true;
        out.brake = false;
        foc_setpoint_t sp;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK(!sp.speed_mode);
        CHECK(sp.enable);
        CHECK(!sp.brake);
        CHECK_NEAR(sp.iq_ref_A, 5.5f, 1e-6f);
        CHECK_NEAR(sp.speed_ref_hz, 0.0f, 1e-6f);
    }

    /* iq command clamped to +/- iq_cmd_limit_A. */
    {
        foc_bridge_cfg_t c = base_cfg();   /* limit 20 A */
        esc_output_t out = {0};
        out.mode = ESC_CTRL_TORQUE;
        out.iq_ref_A = 100.0f;
        foc_setpoint_t sp;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.iq_ref_A, 20.0f, 1e-6f);

        out.iq_ref_A = -100.0f;            /* negative also clamped (symmetric) */
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.iq_ref_A, -20.0f, 1e-6f);
    }

    /* speed mode: krpm -> electrical Hz with pole pairs; speed_mode true; iq_limit passed. */
    {
        foc_bridge_cfg_t c = base_cfg();   /* 7 pole pairs */
        esc_output_t out = {0};
        out.mode = ESC_CTRL_SPEED;
        out.speed_ref_krpm = 0.6f;         /* 600 rpm mech */
        out.iq_limit_A = 8.0f;
        out.enable = true;
        foc_setpoint_t sp;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK(sp.speed_mode);
        /* 0.6 krpm * 1000/60 * 7 = 70 Hz electrical */
        CHECK_NEAR(sp.speed_ref_hz, 70.0f, 1e-4f);
        CHECK_NEAR(sp.iq_limit_A, 8.0f, 1e-6f);
    }

    /* negative iq_cmd_limit config is treated as 0 (no command leaks through). */
    {
        foc_bridge_cfg_t c = base_cfg();
        c.iq_cmd_limit_A = -1.0f;
        esc_output_t out = {0};
        out.mode = ESC_CTRL_TORQUE;
        out.iq_ref_A = 5.0f;
        foc_setpoint_t sp;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.iq_ref_A, 0.0f, 1e-6f);
    }

    /* feedback: scalars copied verbatim; encoder facts pass straight through. */
    {
        foc_raw_feedback_t raw = {0};
        raw.vbus_V = 24.3f;
        raw.iq_meas_A = 4.1f;
        raw.i_motor_A = 6.7f;
        raw.speed_est_krpm = 1.2f;
        raw.temp_C = 31.0f;
        raw.gate_fault = true;
        raw.enc_mech_rev = 0.42f;
        raw.enc_vel_revps = 9.0f;
        raw.enc_valid = true;
        raw.enc_stale = false;
        esc_feedback_t fb;
        /* pre-poison the destination to prove the mapper writes every field. */
        fb.enc_mech_rev = -1.0f;
        fb.enc_vel_revps = -1.0f;
        fb.enc_valid = false;
        fb.enc_stale = true;
        foc_bridge_map_feedback(&raw, &fb);
        CHECK_NEAR(fb.vbus_V, 24.3f, 1e-6f);
        CHECK_NEAR(fb.iq_meas_A, 4.1f, 1e-6f);
        CHECK_NEAR(fb.i_motor_A, 6.7f, 1e-6f);
        CHECK_NEAR(fb.speed_est_krpm, 1.2f, 1e-6f);
        CHECK_NEAR(fb.temp_C, 31.0f, 1e-6f);
        CHECK(fb.gate_fault);
        CHECK(fb.enc_valid);
        CHECK(!fb.enc_stale);
        CHECK_NEAR(fb.enc_mech_rev, 0.42f, 1e-6f);
        CHECK_NEAR(fb.enc_vel_revps, 9.0f, 1e-6f);
    }

    /* encoder-less board (launchxl): enc_valid=false passes through unchanged. */
    {
        foc_raw_feedback_t raw = {0};   /* enc_valid defaults false */
        raw.vbus_V = 12.0f;
        esc_feedback_t fb;
        fb.enc_valid = true;            /* must be overwritten to false */
        foc_bridge_map_feedback(&raw, &fb);
        CHECK(!fb.enc_valid);
        CHECK(!fb.enc_stale);
        CHECK_NEAR(fb.enc_mech_rev, 0.0f, 1e-6f);
    }

    /* speed-mode clamp: |speedRef_Hz| limited to cfg.speed_max_hz, sign preserved. */
    {
        foc_bridge_cfg_t c = base_cfg();
        c.speed_max_hz = 50.0f;            /* 7 pp */
        esc_output_t out = {0};
        out.mode = ESC_CTRL_SPEED;
        out.enable = true;

        out.speed_ref_krpm = 0.6f;         /* 70 Hz elec -> clamp to 50 */
        foc_setpoint_t sp;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.speed_ref_hz, 50.0f, 1e-4f);

        out.speed_ref_krpm = -0.6f;        /* -70 -> -50 (sign preserved) */
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.speed_ref_hz, -50.0f, 1e-4f);

        out.speed_ref_krpm = 0.3f;         /* 35 Hz -> under the cap, unchanged */
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.speed_ref_hz, 35.0f, 1e-4f);

        c.speed_max_hz = 0.0f;             /* cap 0 -> speedRef forced to 0 */
        out.speed_ref_krpm = 0.6f;
        foc_bridge_map_output(&c, &out, &sp);
        CHECK_NEAR(sp.speed_ref_hz, 0.0f, 1e-6f);
    }

    /* speed gate: when speed is NOT allowed, a speed-mode setpoint is forced to coast-disable. */
    {
        foc_setpoint_t sp = {0};
        sp.speed_mode = true;
        sp.enable = true;
        sp.speed_ref_hz = 40.0f;
        sp.iq_ref_A = 3.0f;
        foc_bridge_gate_speed(&sp, false);  /* not allowed */
        CHECK(!sp.enable);                  /* -> glue coasts */
        CHECK_NEAR(sp.speed_ref_hz, 0.0f, 1e-6f);
        CHECK_NEAR(sp.iq_ref_A, 0.0f, 1e-6f);
    }
    /* speed gate: when allowed, a speed-mode setpoint passes through untouched. */
    {
        foc_setpoint_t sp = {0};
        sp.speed_mode = true;
        sp.enable = true;
        sp.speed_ref_hz = 40.0f;
        foc_bridge_gate_speed(&sp, true);   /* allowed */
        CHECK(sp.enable);
        CHECK_NEAR(sp.speed_ref_hz, 40.0f, 1e-6f);
    }
    /* speed gate: torque-mode setpoints are never affected by the gate. */
    {
        foc_setpoint_t sp = {0};
        sp.speed_mode = false;
        sp.enable = true;
        sp.iq_ref_A = 5.0f;
        foc_bridge_gate_speed(&sp, false);  /* not allowed, but torque is fine */
        CHECK(sp.enable);
        CHECK_NEAR(sp.iq_ref_A, 5.0f, 1e-6f);
    }

    CHECK_DONE();
}
