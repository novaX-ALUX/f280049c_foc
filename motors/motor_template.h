// motor_template.h -- FOC motor profile template (axis of change 2)
//
// Copy this file to motors/<model>.h and fill in the motor parameters. Macro names
// align with USER_MOTOR_* in SDK 6.0 user.h, allowing board user.h to #include it
// as a direct drop-in replacement.
//
// Two field categories:
//   [Input]  Must be set correctly before power-on / identification (pole-pair count + current limits + identification seeds).
//   [Output] Back-filled by is05_motor_id after identification (Rs / Ls / flux linkage) -- leave blank (commented out) in the template.
#ifndef MOTOR_TEMPLATE_H
#define MOTOR_TEMPLATE_H

#define MOTOR_NAME                        "TEMPLATE"
#define MOTOR_KV_RPM_PER_V                (0)        // [Input] Nameplate KV (for reference / cross-check)

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (0)        // [Input] Pole-pair count (geometric; required -- is05 does not identify this; if wrong, everything is wrong)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)     // Fixed NULL for PMSM

#define USER_MOTOR_Rr_Ohm                 (NULL)     // No rotor resistance for PMSM
// [Output/seed] Rs/Ls/flux linkage -- safe bench seeds; overwritten after is05_motor_id identification:
#define USER_MOTOR_Rs_Ohm                 (0.02)     // seed; overwritten by is05
#define USER_MOTOR_Ls_d_H                 (10.0e-6)  // seed; overwritten by is05
#define USER_MOTOR_Ls_q_H                 (10.0e-6)  // seed; overwritten by is05
#define USER_MOTOR_RATED_FLUX_VpHz        (0.01)     // seed; overwritten by is05

// [Input] Identification seeds / current limits (tune to motor + supply before power-on; start conservatively low on the bench):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)      // Resistance identification current
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)     // Inductance identification current (negative)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)      // Maximum current (within bench / supply limit)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)

#define USER_MOTOR_NUM_ENC_SLOTS          (1000)      // Placeholder for encoderless operation (FAST sensorless does not use this value)
#define USER_MOTOR_INERTIA_Kgm2           (1.0e-5)    // Placeholder; affects speed loop feed-forward -- tune during is07 loop tuning

// [Input] Operating range (placeholder / bench values; tune to motor + supply; affects FAST frequency range and limiting):
#define USER_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR_RATED_SPEED_KRPM       (5.0)
#define USER_MOTOR_FREQ_MIN_HZ            (5.0)
#define USER_MOTOR_FREQ_MAX_HZ            (800.0)
#define USER_MOTOR_FREQ_LOW_HZ            (20.0)
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)
#define USER_MOTOR_VOLT_MIN_V             (4.0)
#define USER_MOTOR_VOLT_MAX_V             (24.0)

#endif // MOTOR_TEMPLATE_H
