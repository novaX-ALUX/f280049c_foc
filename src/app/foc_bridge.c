/*
 * foc_bridge.c - see foc_bridge.h. Pure mapping, no SDK/HAL.
 */
#include "foc_bridge.h"

/* krpm -> electrical Hz: rev/min * (1 min / 60 s) * pole_pairs. */
#define KRPM_TO_HZ_PER_POLEPAIR  (1000.0f / 60.0f)

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

void foc_bridge_map_output(const foc_bridge_cfg_t *cfg,
                           const esc_output_t *out, foc_setpoint_t *sp)
{
    /* iq command hard cap: redundant safety on top of esc_control's own slew/limit. */
    float lim = (cfg->iq_cmd_limit_A >= 0.0f) ? cfg->iq_cmd_limit_A : 0.0f;

    sp->enable     = out->enable;
    sp->brake      = out->brake;
    sp->speed_mode = (out->mode == ESC_CTRL_SPEED);
    sp->iq_ref_A   = clampf(out->iq_ref_A, -lim, lim);
    sp->speed_ref_hz =
        out->speed_ref_krpm * KRPM_TO_HZ_PER_POLEPAIR * cfg->pole_pairs;
    /* keep the park current limit non-negative (esc_control already guarantees this). */
    sp->iq_limit_A = (out->iq_limit_A >= 0.0f) ? out->iq_limit_A : 0.0f;
}

void foc_bridge_map_feedback(const foc_raw_feedback_t *raw, esc_feedback_t *fb)
{
    fb->vbus_V         = raw->vbus_V;
    fb->iq_meas_A      = raw->iq_meas_A;
    fb->i_motor_A      = raw->i_motor_A;
    fb->speed_est_krpm = raw->speed_est_krpm;
    fb->temp_C         = raw->temp_C;
    fb->gate_fault     = raw->gate_fault;

    /* launchxl has no encoder -> force invalid so esc_control stays out of parking. */
    fb->enc_mech_rev  = 0.0f;
    fb->enc_vel_revps = 0.0f;
    fb->enc_valid     = false;
    fb->enc_stale     = false;
}
