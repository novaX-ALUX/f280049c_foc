#include "check.h"
#include "foc_bridge.h"
#include <math.h>

static foc_bridge_cfg_t base_cfg(void)
{
    foc_bridge_cfg_t c = {0};
    c.pole_pairs    = 7.0f;     /* AM-4116: 14 poles -> 7 pole pairs */
    c.iq_cmd_limit_A = 20.0f;
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

    /* feedback: scalars copied verbatim, encoder forced invalid. */
    {
        foc_raw_feedback_t raw = {0};
        raw.vbus_V = 24.3f;
        raw.iq_meas_A = 4.1f;
        raw.i_motor_A = 6.7f;
        raw.speed_est_krpm = 1.2f;
        raw.temp_C = 31.0f;
        raw.gate_fault = true;
        esc_feedback_t fb;
        /* pre-poison the encoder fields to prove the mapper overwrites them. */
        fb.enc_mech_rev = 0.42f;
        fb.enc_vel_revps = 9.0f;
        fb.enc_valid = true;
        fb.enc_stale = true;
        foc_bridge_map_feedback(&raw, &fb);
        CHECK_NEAR(fb.vbus_V, 24.3f, 1e-6f);
        CHECK_NEAR(fb.iq_meas_A, 4.1f, 1e-6f);
        CHECK_NEAR(fb.i_motor_A, 6.7f, 1e-6f);
        CHECK_NEAR(fb.speed_est_krpm, 1.2f, 1e-6f);
        CHECK_NEAR(fb.temp_C, 31.0f, 1e-6f);
        CHECK(fb.gate_fault);
        CHECK(!fb.enc_valid);
        CHECK(!fb.enc_stale);
        CHECK_NEAR(fb.enc_mech_rev, 0.0f, 1e-6f);
        CHECK_NEAR(fb.enc_vel_revps, 0.0f, 1e-6f);
    }

    CHECK_DONE();
}
