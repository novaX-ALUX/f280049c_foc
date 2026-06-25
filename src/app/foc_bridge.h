/*
 * foc_bridge.h - Pure mapping between the esc_control DTOs and FAST setpoints/scalars.
 *
 * This is the seam that keeps esc_control (and the rest of src/) free of any SDK/HAL
 * dependency: the product main collects raw FAST scalars, calls foc_bridge_map_feedback()
 * to get an esc_feedback_t, runs esc_control_step(), then calls foc_bridge_map_output()
 * to turn the esc_output_t into a plain foc_setpoint_t that the (SDK-coupled) glue writes
 * onto motorVars / IdqSet_A. Nothing here touches driverlib/HAL, so it is host-tested.
 *
 * Units: esc_control speaks krpm; FAST speed is electrical Hz. The krpm->Hz conversion
 * needs the motor pole pairs (cfg), so it lives here (pure, testable) rather than in glue.
 */
#ifndef FOC_BRIDGE_H
#define FOC_BRIDGE_H

#include "esc_types.h"

typedef struct {
    float pole_pairs;      /* krpm -> electrical Hz; from the motor profile */
    float iq_cmd_limit_A;  /* redundant hard cap applied to the iq command (>= 0) */
} foc_bridge_cfg_t;

/* Plain FAST setpoint; the SDK glue lands these on motorVars / IdqSet_A. */
typedef struct {
    bool  enable;        /* -> flagRunIdentAndOnLine target (still gated by offset cal) */
    bool  speed_mode;    /* false = torque (launchxl only); true = speed (esc6288 park) */
    float iq_ref_A;      /* torque mode: write IdqSet_A.value[1] */
    float speed_ref_hz;  /* speed mode: write motorVars.speedRef_Hz */
    float iq_limit_A;    /* speed-mode current limit (park), always >= 0 */
    bool  brake;         /* active-brake request */
} foc_setpoint_t;

/* Raw FAST scalars the product main samples each tick. The encoder fields carry PROCESSED
 * facts (mt6701_*() outputs): the board that owns an MT6701 fills them; encoder-less boards
 * (launchxl) leave enc_valid=false. map_feedback passes them straight through. */
typedef struct {
    float vbus_V;
    float iq_meas_A;
    float i_motor_A;        /* peak |phase current| -> esc_control over-current latch */
    float speed_est_krpm;
    float temp_C;
    bool  gate_fault;
    float enc_mech_rev;     /* 0..1 processed mechanical position */
    float enc_vel_revps;
    bool  enc_valid;        /* false on encoder-less boards -> esc_control never parks */
    bool  enc_stale;
} foc_raw_feedback_t;

/* esc_output_t -> FAST setpoint (mode select, iq clamp, krpm->Hz). */
void foc_bridge_map_output(const foc_bridge_cfg_t *cfg,
                           const esc_output_t *out, foc_setpoint_t *sp);

/*
 * Raw FAST scalars -> esc_feedback_t. The encoder fields pass straight through from raw:
 * the board that owns the MT6701 (esc6288) fills raw.enc_* from the processed encoder layer,
 * and encoder-less boards (launchxl) leave raw.enc_valid=false so esc_control never parks.
 */
void foc_bridge_map_feedback(const foc_raw_feedback_t *raw, esc_feedback_t *fb);

#endif /* FOC_BRIDGE_H */
