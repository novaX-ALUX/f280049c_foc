// am_4116_kvb.h -- NovaX/AM-4116 (KV variant B, different KV from A) FOC profile
// Status: blank template. Electrical parameters (Rs/Ls/flux linkage) to be back-filled after is05_motor_id bench identification.
// See motors/README.md and motors/motor_template.h.
#ifndef MOTOR_AM_4116_KVB_H
#define MOTOR_AM_4116_KVB_H

#define MOTOR_NAME                        "AM-4116-KVB"
#define MOTOR_KV_RPM_PER_V                (0)        // [Input] TODO: fill in the actual KV for variant B (different from A)

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (7)        // [Input] 4116 = 12N14P -> 7 pole pairs (geometric; verify at power-on)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)

#define USER_MOTOR_Rr_Ohm                 (NULL)      // No rotor resistance for PMSM
// [Output/seed] Rs/Ls/flux linkage -- currently safe bench seeds; overwritten after is05_motor_id identification:
#define USER_MOTOR_Rs_Ohm                 (0.02)      // seed; overwritten by is05
#define USER_MOTOR_Ls_d_H                 (10.0e-6)   // seed; overwritten by is05
#define USER_MOTOR_Ls_q_H                 (10.0e-6)   // seed; overwritten by is05
#define USER_MOTOR_RATED_FLUX_VpHz        (0.01)      // seed; overwritten by is05

// [Input] Identification seeds / current limits (tune to motor + supply before power-on; keep conservative on the bench):
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
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

#endif // MOTOR_AM_4116_KVB_H
