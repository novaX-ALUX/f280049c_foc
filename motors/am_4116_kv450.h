// am_4116_kv450.h -- NovaX/AM-4116 (KV450 wind; formerly "am_4116_kva" / KV variant A) FOC profile
// Status: Rs/Ls/flux BACK-FILLED from verified legacy FAST identification (../esc_drv8300_foc,
// esc_drv8300 board, MotorWare lab02c). The KV450 wind was identified 2026-06-05 (EST 20 kHz);
// flux = 0.012 V/Hz is the authoritative FAST-identified value. A free-shaft spin reached 8.16 krpm
// no-load @24 V, which RULED OUT the earlier 0.024 V/Hz readout (a 2x scale error -- that larger
// flux would have capped no-load at ~4.9 krpm). NOTE: that no-load speed is set by the
// USER_MAX_VS_MAG_PU=0.5 modulation cap (BEMF ~11.4 V phase at 8.16 krpm), so it is NOT a direct
// rpm/V reading -- do NOT back-derive KV as 8160/24 (=340, wrong). KV450 is the nameplate. These are
// physical control-path values; they transfer across the IQ scaling differences between MotorWare
// and this driverlib SDK6 project. See motors/README.md.
#ifndef MOTOR_AM_4116_KV450_H
#define MOTOR_AM_4116_KV450_H

#define MOTOR_NAME                        "AM-4116-KV450"
#define MOTOR_KV_RPM_PER_V                (450)      // nameplate KV450 wind (no-load spin is modulation-capped, not a direct KV -- see header)

#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (7)        // 4116 = 12N14P -> 14 poles -> 7 pole pairs
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)

#define USER_MOTOR_Rr_Ohm                 (NULL)      // No rotor resistance for PMSM
// [Corrected 2026-07-01] Y phase-to-neutral stator Rs = bench line-line 42-43 mOhm / 2. The legacy
// 0.0403 was the LINE-LINE value (the MotorWare pu->Ohm conversion encoded phase-phase, not phase-
// neutral); using it tunes the current regulator ~2x too stiff. esc6288 is05 FAST ID reads 0.0167
// (low-signal, ~21% under); the bench-derived 0.0213 is the more reliable phase-neutral seed.
#define USER_MOTOR_Rs_Ohm                 (0.0213)    // ~21 mOhm Y phase-to-neutral (line-line 42-43 mOhm / 2)
// [Re-ID 2026-07-01] esc6288 SDK6 FAST, median of 4 completed is05 runs (23.1/23.5/23.6/24.5 uH,
// tight +-3%). Was 33.6 from the same legacy recipe that mis-scaled Rs; the direct SDK6 getter reads
// 23.5. NOTE: like Rs, this sits ~30% below the old value -- a Kelvin current-scale check (pending)
// would confirm whether a ~1.3x current over-read means true Ls is ~30 uH. Lower Ls = safer (lower
// current-loop Kp), so 23.5 is the conservative committed value.
#define USER_MOTOR_Ls_d_H                 (23.5e-6)   // ~23.5 uH (esc6288 is05 FAST median)
#define USER_MOTOR_Ls_q_H                 (23.5e-6)   // = Ls_d (FAST identifies a single average Ls)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.012)     // FAST-identified; spin ruled out the 2x-flux error (see header). NOT a seed

// [Input] FAST identification currents -- these are the LEGACY-VALIDATED ID conditions (24 V / 10 A
// bench on esc_drv8300). They are MOTOR-intrinsic ID settings, NOT a launchxl run budget: the
// BOOSTXL-DRV8305 current sense saturates at +-23.57 A (7 mOhm shunt x CSA gain 10), so on the
// launchxl bench keep Iq small (low-load sanity only) -- this motor's loaded current (40-58 A) is
// out of range there; full power belongs to esc6288.
#define USER_MOTOR_RES_EST_CURRENT_A      (4.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-4.0)   // legacy esc_drv8300 4116 value; -2.0 (2026-06-30) made Ls ID worse (Codex: weaker injection = worse), reverted
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
// ~40 mOhm motor: VOLT_MIN_V across Rs is an instant overcurrent (4.0 V / 0.04 Ohm ~ 100 A; the
// launchxl CMPSS correctly trips). DO NOT run is03 V/f for the 4116 -- use FAST (is05) / current
// control only. Kept only so is03 still links; see product/BENCH.md.
#define USER_MOTOR_FREQ_LOW_HZ            (20.0)
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)
#define USER_MOTOR_VOLT_MIN_V             (4.0)       // V/f template -- NOT for 4116 (see note above)
#define USER_MOTOR_VOLT_MAX_V             (24.0)      // V/f template -- NOT for 4116 (see note above)

#endif // MOTOR_AM_4116_KV450_H
