/*
 * prop_park.h - Prop-park PD controller (pure logic, no SDK).
 *
 * Holds the propeller at a fixed mechanical angle for low-drag cruise. Output is a
 * SPEED reference (krpm) that overlays the FAST sensorless speed loop (the product
 * main switches FAST to speed mode while parking) plus an iq current limit. It never
 * injects an electrical angle.
 *
 * Units follow the validated bench convention (so tuned values transfer to hardware):
 *   position/error : rev (0..1)        Kp : krpm / rev
 *   velocity       : rev/s             Kd : krpm / (rev/s)
 *
 * Safety: closed loop only when enc_valid; output current limit always >= 0; PARK_TRIP
 * is produced here (esc_control only latches it).
 */
#ifndef PROP_PARK_H
#define PROP_PARK_H

#include <stdbool.h>

typedef struct {
    float kp_krpm_per_rev;
    float kd_krpm_per_revps;
    float deadband_rev;       /* inside -> force speed_ref 0 */
    float settle_tol_rev;     /* ~2 deg (~0.0056) */
    float settle_vel_revps;
    float settle_hold_s;      /* settled must be sustained this long */
    float speed_max_krpm;
    float min_kick_krpm;      /* break cogging at very low command */
    float iq_max_A;           /* park current limit (>=0) */
    bool  two_blade;          /* mod-180 wrap for 2-blade props */
    float hyst_rev;           /* direction-latch reversal allowed only when |err| < this */
    float park_trip_rev;      /* trip if |err| exceeds this ... */
    float park_trip_s;        /* ... for this long */
} prop_park_cfg_t;

typedef struct {
    float dir_latch;     /* 0 / +1 / -1 committed direction */
    float settle_timer_s;
    float trip_timer_s;
    bool  trip;
} prop_park_state_t;

typedef struct {
    float speed_ref_krpm;
    float iq_limit_A;
    bool  settled;       /* sustained within tolerance */
    bool  active;        /* loop is closing (enc valid) */
    float err_rev;       /* wrapped (and mod-180 if two_blade) error */
    bool  trip;          /* PARK_TRIP condition met */
    float trip_timer_s;
} prop_park_out_t;

void prop_park_reset(prop_park_state_t *st);

/*
 * One control step. enc_valid=false -> all-zero safe output, active/settled false,
 * trip timer paused/cleared.
 */
void prop_park_step(const prop_park_cfg_t *cfg, prop_park_state_t *st,
                    float target_rev, float actual_rev, float vel_revps,
                    bool enc_valid, float dt_s, prop_park_out_t *out);

#endif /* PROP_PARK_H */
