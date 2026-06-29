/*
 * esc_arbiter.h - Command-source arbitration (pure logic, no SDK).
 *
 * Fuses the DroneCAN throttle (esc_command_t, post arm-handshake) with an RC-PWM
 * throttle into a single esc_command_t + active source. Owns per-source staleness,
 * a PWM low-dwell arm gate, a "PWM must track CAN before it may take over" lockout
 * (defeats a stuck PWM line), policy selection, and handoff slew. It does NOT read
 * eCAP or CAN hardware: the product main hands it already-decoded samples each tick.
 *
 * Layering: product main -> esc_arbiter_step() -> esc_command_t -> esc_control_step().
 * When no source is healthy the arbiter emits have_cmd=false; the product main then
 * passes NULL to esc_control, whose existing watchdog coasts the stage.
 */
#ifndef ESC_ARBITER_H
#define ESC_ARBITER_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_types.h"

/* Which source currently owns the command. */
typedef enum {
    ESC_SRC_NONE = 0,
    ESC_SRC_CAN  = 1,
    ESC_SRC_PWM  = 2
} esc_src_id_t;

/* Selection policy. Default ships EXPLICIT_CAN (PWM ignored); the flight policy
 * CAN_PRIMARY is enabled only after the bench gate. */
typedef enum {
    ESC_ARB_EXPLICIT_CAN = 0,   /* CAN only; PWM ignored entirely (safe default) */
    ESC_ARB_EXPLICIT_PWM = 1,   /* PWM only (bench bring-up of the RC path) */
    ESC_ARB_CAN_PRIMARY  = 2     /* CAN primary, PWM tracked hot-standby fallback */
} esc_arb_policy_t;

/* Per-source sample, built by the product main each tick. */
typedef struct {
    bool  fresh;     /* a new frame/pulse arrived since last tick */
    bool  valid;     /* signal physically valid (in range / good frame) */
    float throttle;  /* [0,1], meaningful only if valid */
    bool  arm_req;   /* source arm intent (CAN: post-handshake arm; PWM: ignored) */
} esc_src_sample_t;

/* Pure RC-PWM pulse decode (us -> normalized sample). */
typedef struct {
    float us_min, us_max;            /* maps to [0,1]; e.g. 1000, 2000 */
    float us_valid_min, us_valid_max; /* reject window; e.g. 900, 2100 */
} esc_pwm_decode_cfg_t;

esc_src_sample_t esc_pwm_decode(const esc_pwm_decode_cfg_t *cfg,
                                bool fresh, bool overflow, float width_us);

/* Arbiter status bits (reported via esc_arbiter_result_t.status_bits). */
typedef enum {
    ESC_ARB_ST_NONE        = 0u,
    ESC_ARB_ST_PWM_ACTIVE  = 1u << 0,  /* PWM is the selected source */
    ESC_ARB_ST_NO_SOURCE   = 1u << 1,  /* no healthy source this tick */
    ESC_ARB_ST_PWM_LOCKOUT = 1u << 2,  /* PWM healthy+armed but not tracking -> ineligible */
    ESC_ARB_ST_HANDOFF     = 1u << 3   /* blending across a source change */
} esc_arb_status_t;

typedef struct {
    esc_arb_policy_t policy;
    float can_stale_s;            /* CAN unhealthy after this with no fresh+valid */
    float pwm_stale_s;            /* PWM unhealthy after this (~3 missed 50 Hz pulses) */
    float pwm_arm_low_s;          /* PWM must be valid-low this long before it may arm */
    float pwm_arm_throttle_eps;   /* throttle <= this counts as "low" */
    float track_tol;              /* |pwm-can| <= this counts as tracking */
    float track_required_s;       /* tracking must persist this long to enable fallback */
    float handoff_slew_per_s;     /* throttle slew limit across a source change */
    esc_pwm_decode_cfg_t pwm;     /* decode window (used by the product main) */
} esc_arbiter_cfg_t;

typedef struct {
    bool  seen;           /* a fresh+valid sample has EVER arrived (cold-boot guard) */
    float age_s;          /* time since last fresh+valid sample */
    bool  healthy;        /* seen AND age_s <= stale_s */
    float throttle;       /* last valid throttle, held while healthy */
    bool  arm_req;        /* last arm intent */
    float low_dwell_s;    /* PWM only: accumulated valid-low time (arming) */
    bool  armed;          /* PWM only: passed the low-dwell arm gate */
    float track_dwell_s;  /* PWM only: accumulated time tracking CAN */
    bool  track_ok;       /* PWM only: eligible as fallback */
} esc_src_state_t;

typedef struct {
    esc_arbiter_cfg_t cfg;
    esc_src_state_t   can;
    esc_src_state_t   pwm;
    esc_src_id_t      active;       /* current selection */
    uint32_t          out_seq;      /* SEQ CONTRACT: bumped ONLY when the selected source is
                                     * fresh+valid THIS tick; held (unchanged) while a healthy
                                     * source is being held with no new frame. esc_control's
                                     * watchdog is seq-change based, so this makes the failsafe
                                     * time from the last REAL frame -- identical to passing CAN
                                     * straight through as today. Never free-runs per tick. */
    float             out_throttle; /* slew-limited output throttle */
    bool              blending;     /* mid handoff slew */
} esc_arbiter_state_t;

typedef struct {
    bool          have_cmd;   /* false -> product main passes NULL to esc_control */
    esc_command_t cmd;        /* seq/throttle/arm, valid only if have_cmd */
    esc_src_id_t  active;     /* winning source (NONE if !have_cmd) */
    uint32_t      status_bits; /* esc_arb_status_t bitmask */
} esc_arbiter_result_t;

void esc_arbiter_init(esc_arbiter_state_t *st, const esc_arbiter_cfg_t *cfg);

/* One arbitration tick. can/pwm may carry fresh=false (no new sample). dt_s > 0. */
void esc_arbiter_step(esc_arbiter_state_t *st,
                      const esc_src_sample_t *can, const esc_src_sample_t *pwm,
                      float dt_s, esc_arbiter_result_t *res);

#endif /* ESC_ARBITER_H */
