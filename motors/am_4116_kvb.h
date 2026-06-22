// am_4116_kvb.h -- NovaX/AM-4116 (KV variant B = KV470 wind) FOC profile
// Status: Rs/Ls/flux BACK-FILLED from verified legacy FAST identification (../esc_drv8300_foc,
// esc_drv8300 board, MotorWare lab02c). The KV470 wind was re-identified 2026-06-11 (EST 16 kHz,
// motor swapped from the KV450 unit); two runs repeatable to 0.2%. vs the KV450 wind: Rs unchanged,
// Ls -10.7%, flux -7.7% (turns-squared scaling for the higher-KV wind). Physical control-path
// values; transfer across the MotorWare<->driverlib IQ scaling. See motors/README.md.
#ifndef MOTOR_AM_4116_KVB_H
#define MOTOR_AM_4116_KVB_H

#define MOTOR_NAME                        "AM-4116-KVB"
#define MOTOR_KV_RPM_PER_V                (470)      // KV470 wind (flux 0.0111 implies KV~487; nameplate 470)

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (7)        // 4116 = 12N14P -> 14 poles -> 7 pole pairs
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)

#define USER_MOTOR_Rr_Ohm                 (NULL)      // No rotor resistance for PMSM
// [Identified] legacy FAST re-ID (esc_drv8300, KV470 wind, 2026-06-11). Rs includes leads/contacts.
#define USER_MOTOR_Rs_Ohm                 (0.040344)  // ~40 mOhm (Rs unchanged vs KV450 wind)
#define USER_MOTOR_Ls_d_H                 (30.0e-6)   // ~30 uH (-10.7% vs KV450 wind)
#define USER_MOTOR_Ls_q_H                 (30.0e-6)   // = Ls_d (FAST identifies a single average Ls)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.011072)  // -7.7% vs KV450 wind; confirm KV by free spin

// [Input] FAST identification currents -- LEGACY-VALIDATED ID conditions (24 V / 10 A bench on
// esc_drv8300). MOTOR-intrinsic, NOT a launchxl run budget: the BOOSTXL-DRV8305 current sense
// saturates at +-23.57 A, so on the launchxl bench keep Iq small (low-load sanity only); this
// motor's loaded current is out of range there. Full power belongs to esc6288.
#define USER_MOTOR_RES_EST_CURRENT_A      (4.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-4.0)
#define USER_MOTOR_MAX_CURRENT_A          (8.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)

#define USER_MOTOR_NUM_ENC_SLOTS          (1000)      // Placeholder for encoderless operation (FAST sensorless does not use this value)
#define USER_MOTOR_INERTIA_Kgm2           (1.0e-5)    // Placeholder; affects speed loop feed-forward -- tune during is07 loop tuning

// [Input] Operating range (bench values; affects FAST frequency range and limiting):
#define USER_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR_RATED_SPEED_KRPM       (5.0)
#define USER_MOTOR_FREQ_MIN_HZ            (5.0)
#define USER_MOTOR_FREQ_MAX_HZ            (800.0)
// NOTE: the V/f (scalar) defines below are an is03 template. is03 open-loop V/f is UNUSABLE for this
// ~40 mOhm motor: VOLT_MIN_V across Rs is an instant overcurrent. DO NOT run is03 V/f for the 4116
// -- use FAST (is05) / current control only. Kept only so is03 still links; see product/BENCH.md.
#define USER_MOTOR_FREQ_LOW_HZ            (20.0)
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)
#define USER_MOTOR_VOLT_MIN_V             (4.0)       // V/f template -- NOT for 4116 (see note above)
#define USER_MOTOR_VOLT_MAX_V             (24.0)      // V/f template -- NOT for 4116 (see note above)

#endif // MOTOR_AM_4116_KVB_H
