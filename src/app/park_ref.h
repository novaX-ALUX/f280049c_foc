/*
 * park_ref.h - Park reference-position learning + NV load hook (pure logic).
 *
 * The parked propeller orientation is LEARNED, not commanded by the flight controller:
 * the operator hand-positions the prop, and on first power-up (when no valid reference
 * is in NV) the ESC captures the current absolute angle as the reference and persists it.
 * Replacing the motor / encoder / prop requires a re-learn (park_ref_invalidate()).
 *
 * This module NEVER writes Flash. It holds only a LOAD callback for boot-time read, and
 * when a fresh reference is captured it raises a STORE REQUEST (needs_store + new_target)
 * for the app / product main to persist via driverlib. That keeps the pure logic free of
 * side effects and host-testable.
 *
 * The learn gate is intentionally strict so wind-milling, handling, or a power-on transient
 * angle is never written as the reference.
 */
#ifndef PARK_REF_H
#define PARK_REF_H

#include <stdbool.h>

typedef struct {
    float speed_thresh_krpm;    /* |speed_est| must be below this */
    float enc_vel_thresh_revps; /* |enc_vel| must be below this */
    float throttle_eps;         /* throttle must be at/below this */
    float learn_hold_s;         /* gate must hold this long before capture */
} park_ref_cfg_t;

/* Boot-time NV read. Returns the stored target_rev; sets *out_valid. ctx is opaque. */
typedef float (*park_ref_load_fn)(void *ctx, bool *out_valid);

typedef struct {
    park_ref_cfg_t cfg;
    float  target_rev;
    bool   valid;
    float  still_timer_s;
    bool   needs_store;   /* store request to the app; app clears after persisting */
    float  new_target;    /* candidate value to persist */
} park_ref_state_t;

/* Reads NV via the load callback (may be NULL -> starts unlearned). */
void park_ref_init(park_ref_state_t *st, const park_ref_cfg_t *cfg,
                   park_ref_load_fn load, void *ctx);

/*
 * Run the learn gate. Captures a reference only when DISARMED, not armed, throttle low,
 * both speed estimates low, encoder valid, sustained for learn_hold_s. On capture it sets
 * valid + raises needs_store/new_target. No-op once already valid.
 */
void park_ref_update(park_ref_state_t *st, bool disarmed, bool armed, float throttle,
                     float speed_est_krpm, float enc_vel_revps, bool enc_valid,
                     float enc_mech_rev, float dt_s);

void park_ref_invalidate(park_ref_state_t *st);     /* force a re-learn */
void park_ref_clear_store_request(park_ref_state_t *st); /* app calls after it persists */

bool  park_ref_valid(const park_ref_state_t *st);
float park_ref_target(const park_ref_state_t *st);

#endif /* PARK_REF_H */
