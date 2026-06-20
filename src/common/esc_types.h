/*
 * esc_types.h - Shared DTOs and enums for the esc6288 product layer.
 *
 * Pure, hardware-/SDK-agnostic. This header MUST NOT include any TI driverlib,
 * device, board HAL, or motor (user.h) header: the whole src/ product layer is
 * built and unit-tested on the host with plain C. The bridge to FAST/driverlib
 * lives in the (later) product main, not here.
 *
 * Three axes meet here only as plain data:
 *   comms  -> app : esc_command_t   (throttle / arm, from DroneCAN)
 *   sensors-> app : esc_feedback_t  (already-processed facts; never raw codes)
 *   app    -> FOC : esc_output_t    (torque/speed reference for the bridge)
 *   app    -> comms: esc_telemetry_t (status back to the flight controller)
 */
#ifndef ESC_TYPES_H
#define ESC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Which inner loop the FOC bridge should run this tick. */
typedef enum {
    ESC_CTRL_TORQUE = 0,   /* RUN_TORQUE: drive iq_ref_A  */
    ESC_CTRL_SPEED  = 1     /* PARKING/PARKED: drive speed_ref_krpm */
} esc_ctrl_mode_t;

/* High-level control state. */
typedef enum {
    ESC_STATE_DISARMED = 0,
    ESC_STATE_ARMED,
    ESC_STATE_RUN_TORQUE,
    ESC_STATE_PARKING,
    ESC_STATE_PARKED,
    ESC_STATE_FAULT
} esc_state_t;

/* Latching hard faults: require an explicit disarm + condition-clear to leave FAULT. */
typedef enum {
    ESC_HF_NONE          = 0u,
    ESC_HF_OVERCURRENT   = 1u << 0,
    ESC_HF_OVERVOLT      = 1u << 1,
    ESC_HF_UNDERVOLT     = 1u << 2,
    ESC_HF_OVERTEMP      = 1u << 3,
    ESC_HF_GATE_FAULT    = 1u << 4,
    ESC_HF_ENCODER_STALE = 1u << 5,   /* only raised while closed-loop parking */
    ESC_HF_PARK_TRIP     = 1u << 6
} esc_hard_fault_t;

/* Non-latching soft status bits: auto-recover, reported separately from hard faults. */
typedef enum {
    ESC_ST_NONE               = 0u,
    ESC_ST_CMD_TIMEOUT        = 1u << 0,
    ESC_ST_FAILSAFE_COAST     = 1u << 1,
    ESC_ST_PARK_REF_UNLEARNED = 1u << 2,
    ESC_ST_PARK_ACTIVE        = 1u << 3
} esc_status_t;

/* Return code for esc_control_step(): API misuse only, never a protection state. */
typedef enum {
    ESC_OK = 0,
    ESC_ERR_BAD_ARG = 1
} esc_result_t;

/*
 * Command from comms -> app.
 * seq: comms increments it on EVERY received frame (even if throttle is unchanged),
 *      so a constant throttle is not mistaken for a stale link. The app only tests
 *      whether seq CHANGED (no numeric diff); wrap-around (UINT32_MAX -> 0) is fine.
 * throttle: clamped to [0,1] by the app. AM-4116 is a single-direction lift motor,
 *           so negative is treated as 0 (no reverse).
 */
typedef struct {
    uint32_t seq;
    float    throttle;
    bool     arm;
} esc_command_t;

/*
 * Measured feedback -> app. Encoder is delivered as PROCESSED facts (the app never
 * sees MT6701 raw counts); the encoder layer + product main fill these in.
 */
typedef struct {
    float vbus_V;
    float iq_meas_A;
    float i_motor_A;
    float speed_est_krpm;   /* from FAST sensorless estimator */
    float enc_mech_rev;     /* 0..1 mechanical, processed */
    float enc_vel_revps;
    bool  enc_valid;
    bool  enc_stale;
    float temp_C;
    bool  gate_fault;
} esc_feedback_t;

/* App -> FOC bridge. Bridge selects the loop by .mode. */
typedef struct {
    esc_ctrl_mode_t mode;
    float iq_ref_A;
    float speed_ref_krpm;
    float iq_limit_A;       /* always >= 0 */
    bool  enable;           /* gate/run enable */
    bool  brake;
} esc_output_t;

/* App -> comms (DroneCAN esc.Status later). */
typedef struct {
    esc_state_t state;
    uint32_t    hard_fault_bits;   /* esc_hard_fault_t bitmask */
    uint32_t    status_bits;       /* esc_status_t bitmask */
    float       rpm;
    float       vbus_V;
    float       current_A;
    float       temp_C;
} esc_telemetry_t;

#endif /* ESC_TYPES_H */
