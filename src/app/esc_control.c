#include "esc_control.h"
#include <stddef.h>
#include <math.h>

static void coast_output(esc_output_t *o)
{
    o->mode           = ESC_CTRL_TORQUE;
    o->iq_ref_A       = 0.0f;
    o->speed_ref_krpm = 0.0f;
    o->iq_limit_A     = 0.0f;
    o->enable         = false;
    o->brake          = false;
}

static void slew_to(float *cur, float target, float rate, float dt)
{
    if (rate <= 0.0f || dt <= 0.0f) {
        *cur = target;
        return;
    }
    float step = rate * dt;
    float d = target - *cur;
    if (d > step) {
        *cur += step;
    } else if (d < -step) {
        *cur -= step;
    } else {
        *cur = target;
    }
}

void esc_control_init(esc_control_state_t *st, const esc_control_cfg_t *cfg,
                      park_ref_load_fn load, void *load_ctx)
{
    st->cfg             = *cfg;
    st->state           = ESC_STATE_DISARMED;
    st->hard_fault_bits = 0;
    st->status_bits     = 0;
    st->have_command    = false;
    st->last_seq        = 0;
    st->cmd_age_s       = 0.0f;
    st->last_throttle   = 0.0f;
    st->last_arm        = false;
    st->iq_ref_A        = 0.0f;
    prop_park_reset(&st->park);
    park_ref_init(&st->ref, &cfg->park_ref, load, load_ctx);
}

/* True when every latched hard-fault condition has cleared and the operator disarmed.
 * Encoder-stale only blocks clearing when ENCODER_STALE was actually latched (it is a fault
 * only while parking) -- otherwise an absent encoder on the bench would wedge an unrelated
 * over-volt / gate fault permanently. */
static bool fault_clearable(const esc_control_state_t *st, const esc_feedback_t *fb, bool arm)
{
    const esc_control_cfg_t *c = &st->cfg;
    bool enc_ok = ((st->hard_fault_bits & ESC_HF_ENCODER_STALE) == 0u) || !fb->enc_stale;
    return (!arm)
        && !fb->gate_fault
        && (fb->vbus_V < c->vbus_ov_clr)
        && (fb->vbus_V > c->vbus_uv_clr)
        && (fb->temp_C < c->temp_ot_clr)
        && (fabsf(fb->i_motor_A) < c->oc_clr_A)
        && enc_ok;
}

esc_result_t esc_control_step(esc_control_state_t *st,
                              const esc_command_t *cmd, const esc_feedback_t *fb,
                              float dt_s, esc_output_t *out, esc_telemetry_t *tel)
{
    /* Defensive: only API misuse returns non-OK. Still produce COAST + valid telemetry. */
    if (st == NULL || fb == NULL || out == NULL || tel == NULL) {
        if (out != NULL) {
            coast_output(out);
        }
        if (tel != NULL) {
            tel->state           = (st != NULL) ? st->state : ESC_STATE_DISARMED;
            tel->hard_fault_bits = (st != NULL) ? st->hard_fault_bits : 0u;
            tel->status_bits     = ((st != NULL) ? st->status_bits : 0u) | ESC_ST_FAILSAFE_COAST;
            tel->rpm             = 0.0f;
            tel->vbus_V          = 0.0f;
            tel->current_A       = 0.0f;
            tel->temp_C          = 0.0f;
        }
        return ESC_ERR_BAD_ARG;
    }

    const esc_control_cfg_t *c = &st->cfg;
    if (dt_s < 0.0f) {
        dt_s = 0.0f;
    }

    /* --- command watchdog (soft, non-latching). seq change-detect only, wrap-safe. --- */
    if (cmd != NULL) {
        float t = cmd->throttle;
        if (t < 0.0f) {
            t = 0.0f;
        } else if (t > 1.0f) {
            t = 1.0f;
        }
        st->last_throttle = t;
        st->last_arm      = cmd->arm;
        if (!st->have_command) {
            st->have_command = true;
            st->last_seq     = cmd->seq;
            st->cmd_age_s    = 0.0f;
        } else if (cmd->seq != st->last_seq) {
            st->last_seq  = cmd->seq;
            st->cmd_age_s = 0.0f;
        } else {
            st->cmd_age_s += dt_s; /* same seq this tick: link aging */
        }
    } else if (st->have_command) {
        st->cmd_age_s += dt_s; /* no frame this tick: keep aging toward timeout */
    }
    /* Never-received-first-frame is NOT a timeout. */
    bool timed_out = st->have_command && (st->cmd_age_s > c->cmd_timeout_s);

    float throttle = st->have_command ? st->last_throttle : 0.0f;
    bool  arm      = st->have_command ? st->last_arm : false;

    /* --- hard-fault evaluation (latching set; hysteresis clear handled in FAULT). Only run once the
     * feedback is valid (offset cal done): the boot/cal transient reads garbage current + unsettled
     * Vbus, which would false-latch OC/UV before the ESC ever sees a command. --- */
    if (fb->valid) {
        if (fb->vbus_V >= c->vbus_ov_set) {
            st->hard_fault_bits |= ESC_HF_OVERVOLT;
        }
        if (fb->vbus_V <= c->vbus_uv_set) {
            st->hard_fault_bits |= ESC_HF_UNDERVOLT;
        }
        if (fb->temp_C >= c->temp_ot_set) {
            st->hard_fault_bits |= ESC_HF_OVERTEMP;
        }
        if (fabsf(fb->i_motor_A) >= c->oc_set_A) {
            st->hard_fault_bits |= ESC_HF_OVERCURRENT;
        }
        if (fb->gate_fault) {
            st->hard_fault_bits |= ESC_HF_GATE_FAULT;
        }
        /* Encoder stale only matters as a hard fault while we depend on it (closed-loop park). */
        if ((st->state == ESC_STATE_PARKING || st->state == ESC_STATE_PARKED) && fb->enc_stale) {
            st->hard_fault_bits |= ESC_HF_ENCODER_STALE;
        }
    }

    /* Park-reference learning (only progresses while disarmed + still). */
    park_ref_update(&st->ref, st->state == ESC_STATE_DISARMED, arm, throttle,
                    fb->speed_est_krpm, fb->enc_vel_revps, fb->enc_valid,
                    fb->enc_mech_rev, dt_s);

    /* Default outputs for this tick. */
    esc_ctrl_mode_t mode = ESC_CTRL_TORQUE;
    float iq_ref = 0.0f, speed_ref = 0.0f, iq_limit = 0.0f;
    bool  enable = false, brake = false;

    /* A latched hard fault forces FAULT before the switch runs. */
    if (st->hard_fault_bits != 0u) {
        st->state = ESC_STATE_FAULT;
    }

    switch (st->state) {
    case ESC_STATE_FAULT:
        st->iq_ref_A = 0.0f;
        if (fault_clearable(st, fb, arm)) {
            st->hard_fault_bits = 0u;
            prop_park_reset(&st->park);
            st->state = ESC_STATE_DISARMED;
        }
        break;

    case ESC_STATE_DISARMED:
        st->iq_ref_A = 0.0f;
        if (arm && !timed_out) {
            st->state = ESC_STATE_ARMED;
        }
        break;

    case ESC_STATE_ARMED:
        /* Disarm is handled BEFORE enabling output so the emitted tick is coast. */
        if (!arm) {
            st->state    = ESC_STATE_DISARMED;
            st->iq_ref_A = 0.0f;
            break;
        }
        /* ARMED = logical standby / coast: NO gate enable at zero/idle throttle. Only RUN_TORQUE
         * enables output. This keeps arming quiet (the open-loop I/f self-start triggers on the RUN
         * transition, not on arm) -- critical before a prop is mounted. */
        st->iq_ref_A = 0.0f;
        if (throttle > c->throttle_run_thresh) {
            st->state = ESC_STATE_RUN_TORQUE;
        }
        break;

    case ESC_STATE_RUN_TORQUE:
        if (!arm) {
            st->state    = ESC_STATE_DISARMED;
            st->iq_ref_A = 0.0f;
            break;
        }
        if (c->speed_run_enable) {
            /* Closed speed loop (is07): throttle maps to a speed reference; the FOC-side speed
             * PI owns Iq. iq_limit advertises the torque authority bound (same cap as torque
             * mode). No slew here: the downstream trajectory generator rate-limits the ref. */
            mode      = ESC_CTRL_SPEED;
            enable    = true;
            speed_ref = throttle * c->speed_max_krpm;
            iq_limit  = c->iq_max_A;
            st->iq_ref_A = 0.0f;
        } else {
            mode   = ESC_CTRL_TORQUE;
            enable = true;
            slew_to(&st->iq_ref_A, throttle * c->iq_max_A, c->iq_slew_A_s, dt_s);
            iq_ref = st->iq_ref_A;
        }
        if (c->auto_park_enable
            && throttle <= c->throttle_idle_eps
            && fabsf(fb->speed_est_krpm) < c->park_engage_speed_krpm
            && st->ref.valid) {
            prop_park_reset(&st->park);
            st->state = ESC_STATE_PARKING;
        }
        break;

    case ESC_STATE_PARKING:
    case ESC_STATE_PARKED: {
        if (!arm) {
            prop_park_reset(&st->park);
            st->state = ESC_STATE_DISARMED;
            break;
        }
        mode   = ESC_CTRL_SPEED;
        enable = true;
        prop_park_out_t po;
        prop_park_step(&c->park, &st->park, st->ref.target_rev, fb->enc_mech_rev,
                       fb->enc_vel_revps, fb->enc_valid, dt_s, &po);
        speed_ref = po.speed_ref_krpm;
        iq_limit  = po.iq_limit_A;
        if (po.trip) {
            st->hard_fault_bits |= ESC_HF_PARK_TRIP;
        }
        if (st->state == ESC_STATE_PARKING && po.settled) {
            st->state = ESC_STATE_PARKED;
        }
        if (throttle > c->throttle_run_thresh) {
            prop_park_reset(&st->park);
            st->state = ESC_STATE_RUN_TORQUE;
        }
        break;
    }

    default:
        st->state = ESC_STATE_DISARMED;
        break;
    }

    /* Post-switch fault enforcement (covers PARK_TRIP / encoder-stale latched this tick). */
    if (st->hard_fault_bits != 0u) {
        st->state = ESC_STATE_FAULT;
        mode = ESC_CTRL_TORQUE;
        iq_ref = 0.0f; speed_ref = 0.0f; iq_limit = 0.0f;
        enable = false; brake = false;
        st->iq_ref_A = 0.0f;
    } else if (timed_out) {
        /* Link-loss failsafe: immediate, no slew. Default COAST (gate off, freewheel);
         * failsafe_brake=true actively brakes (gate on, zero refs, brake asserted) -- but ONLY
         * while armed. A timeout after a disarm frame (arm=false) must stay gate-off coast and
         * never energize the bridge. Drop running states to ARMED for a clean re-entry. */
        mode = ESC_CTRL_TORQUE;
        iq_ref = 0.0f; speed_ref = 0.0f; iq_limit = 0.0f;
        st->iq_ref_A = 0.0f;
        if (c->failsafe_brake && arm) {
            enable = true;  brake = true;
        } else {
            enable = false; brake = false;
        }
        if (st->state == ESC_STATE_RUN_TORQUE
            || st->state == ESC_STATE_PARKING
            || st->state == ESC_STATE_PARKED) {
            prop_park_reset(&st->park);
            st->state = ESC_STATE_ARMED;
        }
    }

    /* --- status bits --- */
    st->status_bits = 0u;
    if (timed_out) {
        st->status_bits |= ESC_ST_CMD_TIMEOUT;
        /* Report the mode actually emitted: brake only happens while armed AND no hard fault is
         * active (a fault force-coasts the bridge above, overriding the failsafe brake). */
        st->status_bits |= (st->hard_fault_bits == 0u && c->failsafe_brake && arm)
                               ? ESC_ST_FAILSAFE_BRAKE
                               : ESC_ST_FAILSAFE_COAST;
    }
    if (!st->ref.valid) {
        st->status_bits |= ESC_ST_PARK_REF_UNLEARNED;
    }
    if (st->state == ESC_STATE_PARKING || st->state == ESC_STATE_PARKED) {
        st->status_bits |= ESC_ST_PARK_ACTIVE;
    }

    /* --- emit --- */
    out->mode           = mode;
    out->iq_ref_A       = iq_ref;
    out->speed_ref_krpm = speed_ref;
    out->iq_limit_A     = (iq_limit >= 0.0f) ? iq_limit : 0.0f;
    out->enable         = enable;
    out->brake          = brake;

    tel->state           = st->state;
    tel->hard_fault_bits = st->hard_fault_bits;
    tel->status_bits     = st->status_bits;
    tel->rpm             = fb->speed_est_krpm * 1000.0f;
    tel->vbus_V          = fb->vbus_V;
    tel->current_A       = fb->i_motor_A;
    tel->temp_C          = fb->temp_C;

    return ESC_OK;
}
