/*
 * esc_control.h - ESC control state machine (pure logic, no SDK).
 *
 * Owns: arming, throttle->Iq torque mode, autonomous prop-park (speed-overlay), command
 * watchdog/failsafe, and fault handling. It consumes already-processed feedback
 * (esc_feedback_t) and emits a FOC reference (esc_output_t) + telemetry; the product main
 * bridges those to FAST/driverlib. It does NOT touch SDK handles or MT6701 raw codes.
 *
 * Fault model:
 *   - hard faults (over/under-volt, over-temp, over-current, gate, park-trip, and encoder
 *     stale WHILE parking) latch -> FAULT; cleared only by disarm + all conditions gone.
 *   - command timeout is a SOFT, non-latching status -> COAST; auto-recovers to ARMED on a
 *     fresh command. esc_control_step() still returns ESC_OK for any protection state; the
 *     only non-OK return is ESC_ERR_BAD_ARG (NULL st/fb/out/tel).
 */
#ifndef ESC_CONTROL_H
#define ESC_CONTROL_H

#include "esc_types.h"
#include "prop_park.h"
#include "park_ref.h"

typedef struct {
    float iq_max_A;             /* throttle = 1.0 -> this Iq */
    float iq_slew_A_s;          /* torque ramp rate */
    float cmd_timeout_s;        /* link-loss failsafe */
    float throttle_run_thresh;  /* above -> RUN_TORQUE */

    /* Closed speed loop in RUN (is07 port). When enabled, the RUN state emits ESC_CTRL_SPEED
     * with speed_ref = throttle * speed_max_krpm (downstream TRAJ rate-limits the reference;
     * the ISR speed PI is current-limited by the SDK controller setup). Default false: RUN
     * stays the validated throttle->Iq torque path. */
    bool  speed_run_enable;
    float speed_max_krpm;       /* throttle = 1.0 -> this speed reference */
    float throttle_idle_eps;    /* at/below -> idle/park-eligible */
    float park_engage_speed_krpm; /* |speed| below -> may auto-park */
    bool  auto_park_enable;     /* master enable for autonomous parking */
    bool  failsafe_brake;       /* false = coast (default) */

    /* over-current limit on |i_motor_A| (set/clear hysteresis) */
    float oc_set_A, oc_clr_A;
    /* dc-bus and temperature limits (separate set/clear to avoid boundary chatter) */
    float vbus_ov_set, vbus_ov_clr;
    float vbus_uv_set, vbus_uv_clr;
    float temp_ot_set, temp_ot_clr;

    prop_park_cfg_t park;
    park_ref_cfg_t  park_ref;
} esc_control_cfg_t;

typedef struct {
    esc_control_cfg_t cfg;
    esc_state_t state;
    uint32_t    hard_fault_bits;
    uint32_t    status_bits;

    /* command tracking */
    bool     have_command;
    uint32_t last_seq;
    float    cmd_age_s;
    float    last_throttle;   /* clamped [0,1], held across no-command ticks */
    bool     last_arm;

    /* torque ramp state */
    float    iq_ref_A;

    prop_park_state_t park;
    park_ref_state_t  ref;
} esc_control_state_t;

void esc_control_init(esc_control_state_t *st, const esc_control_cfg_t *cfg,
                      park_ref_load_fn load, void *load_ctx);

/*
 * One control tick. cmd may be NULL (no new command this tick). fb/out/tel must be non-NULL;
 * if any of st/fb/out/tel is NULL the call forces COAST output (when out/tel exist) and
 * returns ESC_ERR_BAD_ARG. All normal protection (timeout, faults, failsafe) returns ESC_OK.
 */
esc_result_t esc_control_step(esc_control_state_t *st,
                              const esc_command_t *cmd, const esc_feedback_t *fb,
                              float dt_s, esc_output_t *out, esc_telemetry_t *tel);

#endif /* ESC_CONTROL_H */
