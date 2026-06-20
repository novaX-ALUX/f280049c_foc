#include "check.h"
#include "esc_control.h"

/* ---- fixtures ------------------------------------------------------------ */

static bool  g_ref_valid;
static float g_ref_value;
static float ref_load(void *ctx, bool *out_valid)
{
    (void)ctx;
    *out_valid = g_ref_valid;
    return g_ref_value;
}

static esc_control_cfg_t make_cfg(bool auto_park, float slew)
{
    esc_control_cfg_t c = {0};
    c.iq_max_A               = 10.0f;
    c.iq_slew_A_s            = slew;
    c.cmd_timeout_s          = 0.1f;
    c.throttle_run_thresh    = 0.05f;
    c.throttle_idle_eps      = 0.02f;
    c.park_engage_speed_krpm = 0.3f;
    c.auto_park_enable       = auto_park;
    c.failsafe_brake         = false;

    c.oc_set_A = 50.0f;  c.oc_clr_A = 45.0f;
    c.vbus_ov_set = 30.0f; c.vbus_ov_clr = 29.0f;
    c.vbus_uv_set = 10.0f; c.vbus_uv_clr = 11.0f;
    c.temp_ot_set = 100.0f; c.temp_ot_clr = 90.0f;

    c.park.kp_krpm_per_rev   = 0.15f;
    c.park.kd_krpm_per_revps = 0.3f;
    c.park.deadband_rev      = 6.0f / 360.0f;
    c.park.settle_tol_rev    = 2.0f / 360.0f;
    c.park.settle_vel_revps  = 0.05f;
    c.park.settle_hold_s     = 0.05f;
    c.park.speed_max_krpm    = 0.03f;
    c.park.min_kick_krpm     = 0.01f;
    c.park.iq_max_A          = 8.0f;
    c.park.two_blade         = true;
    c.park.hyst_rev          = 0.125f;
    c.park.park_trip_rev     = 30.0f / 360.0f;
    c.park.park_trip_s       = 0.2f;

    c.park_ref.speed_thresh_krpm    = 0.01f;
    c.park_ref.enc_vel_thresh_revps = 0.02f;
    c.park_ref.throttle_eps         = 0.01f;
    c.park_ref.learn_hold_s         = 0.10f;
    return c;
}

static esc_feedback_t nominal_fb(void)
{
    esc_feedback_t fb = {0};
    fb.vbus_V         = 24.0f;
    fb.speed_est_krpm = 0.0f;
    fb.enc_valid      = true;
    fb.enc_mech_rev   = 0.0f;
    fb.temp_C         = 25.0f;
    fb.gate_fault     = false;
    return fb;
}

static uint32_t g_seq;
static esc_result_t drive(esc_control_state_t *st, float throttle, bool arm,
                          esc_feedback_t *fb, float dt, esc_output_t *o, esc_telemetry_t *t)
{
    esc_command_t cmd;
    cmd.seq      = ++g_seq;
    cmd.throttle = throttle;
    cmd.arm      = arm;
    return esc_control_step(st, &cmd, fb, dt, o, t);
}

static void reach_run(esc_control_state_t *st, esc_feedback_t *fb,
                      esc_output_t *o, esc_telemetry_t *t)
{
    drive(st, 0.0f, true, fb, 0.001f, o, t); /* DISARMED -> ARMED */
    drive(st, 0.5f, true, fb, 0.001f, o, t); /* ARMED -> RUN_TORQUE */
    drive(st, 0.5f, true, fb, 0.001f, o, t); /* RUN_TORQUE compute */
}

/* ---- tests --------------------------------------------------------------- */

int main(void)
{
    esc_output_t o;
    esc_telemetry_t t;

    /* Before the first command: NULL cmd is not a timeout. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        esc_result_t r = esc_control_step(&st, NULL, &fb, 0.5f, &o, &t);
        CHECK(r == ESC_OK);
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) == 0u);
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(!o.enable);
    }

    /* fb == NULL -> ESC_ERR_BAD_ARG, COAST, valid telemetry. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_command_t cmd = {1u, 0.5f, true};
        esc_result_t r = esc_control_step(&st, &cmd, NULL, 0.001f, &o, &t);
        CHECK(r == ESC_ERR_BAD_ARG);
        CHECK(!o.enable);
        CHECK_NEAR(o.iq_ref_A, 0.0f, 1e-9f);
        CHECK((t.status_bits & ESC_ST_FAILSAFE_COAST) != 0u);
    }

    /* Throttle -> Iq mapping (fast slew reaches target). */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        CHECK(t.state == ESC_STATE_RUN_TORQUE);
        CHECK(o.mode == ESC_CTRL_TORQUE);
        CHECK(o.enable);
        CHECK_NEAR(o.iq_ref_A, 5.0f, 0.01f); /* 0.5 * 10 A */
    }

    /* Iq slew limits the ramp. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1000.0f); /* 1 A per 1ms tick */
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t); /* ARMED */
        drive(&st, 1.0f, true, &fb, 0.001f, &o, &t); /* RUN entry */
        drive(&st, 1.0f, true, &fb, 0.001f, &o, &t); /* first ramp step */
        CHECK_NEAR(o.iq_ref_A, 1.0f, 0.01f);          /* not 10 A */
    }

    /* Negative throttle clamps to 0. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, -0.5f, true, &fb, 0.001f, &o, &t);
        CHECK_NEAR(st.last_throttle, 0.0f, 1e-9f);
        CHECK_NEAR(o.iq_ref_A, 0.0f, 0.01f);
    }

    /* seq wrap-around (UINT32_MAX -> 0) is a fresh command, resets the watchdog. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        esc_command_t a = {0xFFFFFFFFu, 0.0f, true};
        esc_control_step(&st, &a, &fb, 0.0f, &o, &t);   /* establish */
        esc_command_t b = {0xFFFFFFFFu, 0.0f, true};
        esc_control_step(&st, &b, &fb, 0.09f, &o, &t);  /* same seq, age 0.09 */
        esc_command_t cc = {0x00000000u, 0.0f, true};
        esc_control_step(&st, &cc, &fb, 0.05f, &o, &t); /* wrapped seq -> fresh */
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) == 0u);
    }

    /* Command timeout -> COAST; recovery to ARMED (not auto-RUN) on a fresh command. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        esc_control_step(&st, NULL, &fb, 0.2f, &o, &t); /* link lost */
        CHECK(!o.enable);
        CHECK_NEAR(o.iq_ref_A, 0.0f, 1e-9f);
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) != 0u);
        CHECK(t.state == ESC_STATE_ARMED);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);    /* fresh cmd, low throttle */
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) == 0u);
        CHECK(t.state == ESC_STATE_ARMED);              /* not RUN */
    }

    /* Hard fault (overvolt) latches; clears only after disarm + condition gone. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        fb.vbus_V = 35.0f;
        drive(&st, 0.5f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_FAULT);
        CHECK((t.hard_fault_bits & ESC_HF_OVERVOLT) != 0u);
        CHECK(!o.enable);
        /* still high + armed -> stays FAULT */
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_FAULT);
        /* condition gone + disarm -> clears */
        fb.vbus_V = 24.0f;
        drive(&st, 0.0f, false, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(t.hard_fault_bits == 0u);
    }

    /* Encoder stale is NOT a hard fault in RUN_TORQUE (sensorless FAST). */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        fb.enc_stale = true;
        drive(&st, 0.5f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_RUN_TORQUE);
        CHECK((t.hard_fault_bits & ESC_HF_ENCODER_STALE) == 0u);
    }

    /* Autonomous parking: throttle->0 at low speed with a valid ref -> PARKING/PARKED. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t); /* RUN sees idle -> PARKING */
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_PARKING || t.state == ESC_STATE_PARKED);
        CHECK(o.mode == ESC_CTRL_SPEED);
    }

    /* auto_park_enable=false -> never parks. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_RUN_TORQUE);
    }

    /* Reference unlearned -> no PARKING, but RUN_TORQUE works; status flagged. */
    {
        g_ref_valid = false; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        CHECK(t.state == ESC_STATE_RUN_TORQUE);
        CHECK((t.status_bits & ESC_ST_PARK_REF_UNLEARNED) != 0u);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_RUN_TORQUE); /* still no parking */
    }

    /* PARKED holds and re-corrects after a disturbance. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t); /* -> PARKING */
        for (int i = 0; i < 10; ++i) {
            drive(&st, 0.0f, true, &fb, 0.02f, &o, &t); /* settle into PARKED */
        }
        CHECK(t.state == ESC_STATE_PARKED);
        fb.enc_mech_rev = 0.1f; /* shoved 36 deg off */
        drive(&st, 0.0f, true, &fb, 0.02f, &o, &t);
        CHECK(t.state == ESC_STATE_PARKED);        /* still parked */
        CHECK(fabsf(o.speed_ref_krpm) > 0.0f);     /* actively correcting */
    }

    /* Encoder lost during park -> safe zero speed (no command), no crash. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t); /* -> PARKING */
        drive(&st, 0.0f, true, &fb, 0.02f, &o, &t);
        fb.enc_valid = false;                         /* sensor dropped */
        drive(&st, 0.0f, true, &fb, 0.02f, &o, &t);
        CHECK_NEAR(o.speed_ref_krpm, 0.0f, 1e-9f);
    }

    /* Disarm is coast on the SAME tick from ARMED. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        drive(&st, 0.0f, true,  &fb, 0.001f, &o, &t); /* -> ARMED */
        drive(&st, 0.0f, true,  &fb, 0.001f, &o, &t); /* ARMED, enabled */
        CHECK(o.enable);
        drive(&st, 0.0f, false, &fb, 0.001f, &o, &t); /* disarm */
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(!o.enable);
    }

    /* Disarm is coast on the SAME tick from RUN_TORQUE. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.5f, false, &fb, 0.001f, &o, &t); /* throttle up but disarmed */
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(!o.enable);
        CHECK_NEAR(o.iq_ref_A, 0.0f, 1e-9f);
    }

    /* Disarm is coast on the SAME tick from PARKED. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t);
        for (int i = 0; i < 10; ++i) {
            drive(&st, 0.0f, true, &fb, 0.02f, &o, &t);
        }
        CHECK(t.state == ESC_STATE_PARKED);
        drive(&st, 0.0f, false, &fb, 0.02f, &o, &t); /* disarm */
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(!o.enable);
        CHECK_NEAR(o.speed_ref_krpm, 0.0f, 1e-9f);
    }

    /* An unrelated hard fault clears even with a stale encoder (stale was never latched). */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        fb.enc_stale = true;          /* encoder absent on the bench, but RUN is sensorless */
        fb.vbus_V = 35.0f;            /* overvolt -> FAULT (not an encoder fault) */
        drive(&st, 0.5f, true, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_FAULT);
        CHECK((t.hard_fault_bits & ESC_HF_ENCODER_STALE) == 0u);
        fb.vbus_V = 24.0f;            /* overvolt gone; encoder still stale */
        drive(&st, 0.0f, false, &fb, 0.001f, &o, &t);
        CHECK(t.state == ESC_STATE_DISARMED);
        CHECK(t.hard_fault_bits == 0u);
    }

    /* failsafe_brake=true: command timeout actively brakes (gate on, brake asserted, zero refs). */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        c.failsafe_brake = true;
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        esc_control_step(&st, NULL, &fb, 0.2f, &o, &t);
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) != 0u);
        CHECK((t.status_bits & ESC_ST_FAILSAFE_BRAKE) != 0u);   /* reported as brake ... */
        CHECK((t.status_bits & ESC_ST_FAILSAFE_COAST) == 0u);   /* ... not coast */
        CHECK(o.brake);
        CHECK(o.enable);
        CHECK_NEAR(o.iq_ref_A, 0.0f, 1e-9f);
        CHECK_NEAR(o.speed_ref_krpm, 0.0f, 1e-9f);
        CHECK(t.state == ESC_STATE_ARMED);
    }

    /* failsafe_brake=true but timeout after a DISARM frame -> must NOT energize: gate-off coast. */
    {
        g_ref_valid = true; g_seq = 0;
        esc_control_cfg_t c = make_cfg(false, 1.0e6f);
        c.failsafe_brake = true;
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, false, &fb, 0.001f, &o, &t); /* disarm frame -> DISARMED */
        CHECK(t.state == ESC_STATE_DISARMED);
        esc_control_step(&st, NULL, &fb, 0.2f, &o, &t); /* now the link ages out */
        CHECK((t.status_bits & ESC_ST_CMD_TIMEOUT) != 0u);
        CHECK(!o.enable);                                /* gate stays OFF */
        CHECK(!o.brake);                                 /* no active brake while disarmed */
        CHECK((t.status_bits & ESC_ST_FAILSAFE_BRAKE) == 0u);
        CHECK((t.status_bits & ESC_ST_FAILSAFE_COAST) != 0u);
        CHECK(t.state == ESC_STATE_DISARMED);
    }

    /* Sustained large park error -> PARK_TRIP latches to FAULT. */
    {
        g_ref_valid = true; g_ref_value = 0.0f; g_seq = 0;
        esc_control_cfg_t c = make_cfg(true, 1.0e6f);
        esc_control_state_t st;
        esc_control_init(&st, &c, ref_load, NULL);
        esc_feedback_t fb = nominal_fb();
        reach_run(&st, &fb, &o, &t);
        drive(&st, 0.0f, true, &fb, 0.001f, &o, &t); /* -> PARKING */
        fb.enc_mech_rev = 0.2f;                        /* held 72 deg off */
        for (int i = 0; i < 6; ++i) {
            drive(&st, 0.0f, true, &fb, 0.05f, &o, &t);
        }
        CHECK((t.hard_fault_bits & ESC_HF_PARK_TRIP) != 0u);
        CHECK(t.state == ESC_STATE_FAULT);
        CHECK(!o.enable);
    }

    CHECK_DONE();
}
