//#############################################################################
// product_main.c - esc6288 product main: (DroneCAN RawCommand | RC-PWM) -> esc_arbiter -> esc_control -> FAST.
//
// This REPLACES the SDK lab .c (it owns main() + mainISR()). The FOC pipeline is the
// SDK torque lab (is06_torque_control) verbatim; the only product injections are:
//   * self-set motorVars.flagEnableSys = true (no watch-window hand-off),
//   * an explicit HAL_enableDRV() (launchxl DRV8305 SPI enable lives inside it),
//   * register/enable the CANA0 bridge ISR alongside the ADC ISR,
//   * a 1 ms background tick that runs: CAN RX drain + RC-PWM read -> esc_arbiter_step
//     -> esc_control_step -> foc_bridge -> motorVars/IdqSet_A, and dronecan_tick -> can_bridge_write.
//
// Board-agnostic (drives motorVars / board.h / HAL handle); esc6288 reuses it once its
// CAN pins / encoder / CMPSS are defined. The SDK-coupling is confined to this file; all
// product logic (esc_control / dronecan / foc_bridge) stays pure in src/.
//
// launchxl scope: NO encoder -> foc_bridge forces enc invalid, auto_park disabled -> only
// the RUN_TORQUE path is reachable. Speed/park, Flash persistence, GetSet, and a real
// active brake are deferred (see product TODOs / plan ③-c).
//#############################################################################

#include "labs.h"
#include "board.h"
#include "build_config.h"

#include "esc_control.h"
#include "esc_arbiter.h"
#include "foc_bridge.h"
#include "nvparam.h"
#include "dronecan.h"
#include "dronecan_frame.h"
#include "can_bridge.h"

#include <math.h>

#ifdef ESC6288_VSF
#ifndef _VSF_EN_
#error "ESC6288_VSF requires EXTRA_DEFINES=\"--define=_VSF_EN_\" so user.h derives VSF estimator/controller timing"
#endif
#ifndef ESC6288_OVERMOD
#error "ESC6288_VSF requires ESC6288_OVERMOD because the IS12 path depends on SVGENCURRENT reconstruction"
#endif
#endif

#pragma CODE_SECTION(mainISR, ".TI.ramfunc");
#ifdef ESC6288_VSF
#pragma CODE_SECTION(estISR, ".TI.ramfunc");
#endif

//
// the globals (mirrors the SDK torque lab; FAST/HAL handles + FOC objects)
//
HAL_ADCData_t adcData = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0};

HAL_PWMData_t pwmData = {{0.0, 0.0, 0.0}};

uint16_t counterLED = 0;

uint16_t counterSpeed = 0;
uint16_t counterTrajSpeed = 0;
uint16_t counterTrajId = 0;

uint32_t offsetCalcCount = 0;
uint32_t offsetCalcWaitTime = 50000;

EST_InputData_t estInputData = {0, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0};

EST_OutputData_t estOutputData = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  {0.0, 0.0}, {0.0, 0.0}, 0, 0.0};
float32_t angleDelta_rad;
float32_t angleFoc_rad;

MATH_Vec2 Idq_ref_A;
MATH_Vec2 Idq_offset_A;
MATH_Vec2 Iab_in_A;
MATH_Vec2 Idq_in_A;
MATH_Vec2 Vab_out_V;
MATH_Vec2 Vdq_out_V;

USER_Params userParams;
#pragma DATA_SECTION(userParams, "ctrl_data");

volatile MOTOR_Vars_t motorVars = MOTOR_VARS_INIT;
#pragma DATA_SECTION(motorVars, "ctrl_data");

CTRL_Handle   ctrlHandle;
CTRL_Obj      ctrl;

CLARKE_Handle clarkeHandle_I;
CLARKE_Obj    clarke_I;

CLARKE_Handle clarkeHandle_V;
CLARKE_Obj    clarke_V;

EST_Handle    estHandle;

HAL_Handle    halHandle;
HAL_Obj       hal;

IPARK_Handle  iparkHandle;
IPARK_Obj     ipark;

PARK_Handle   parkHandle;
PARK_Obj      park;

PI_Handle     piHandle_Id;
PI_Obj        pi_Id;

PI_Handle     piHandle_Iq;
PI_Obj        pi_Iq;

PI_Handle     piHandle_fwc;
PI_Obj        pi_fwc;

PI_Handle     piHandle_spd;
PI_Obj        pi_spd;

SVGEN_Handle  svgenHandle;
SVGEN_Obj     svgen;

#ifdef ESC6288_FWC_MTPA
FWC_Handle    fwcHandle;
FWC_Obj       fwc;

MTPA_Handle   mtpaHandle;
MTPA_Obj      mtpa;

static uint16_t counterFWCandMTPA = 0u;
#define PRODUCT_FWC_MTPA_TICKS    (10u)
#ifndef ESC6288_FWC_MTPA_DEFAULT
#define ESC6288_FWC_MTPA_DEFAULT  (false)
#endif
static volatile bool g_fwc_mtpa_enable = ESC6288_FWC_MTPA_DEFAULT;
#endif

#ifdef ESC6288_OVERMOD
// is08 overmodulation port (opt-in, not flight-ready): SVGENCURRENT reconstructs phase currents
// whose low-side shunt window collapses at high duty, compensates PWM min-width, and steers the
// ADC trigger into the measurable window. Bench 2026-07-02: 0.5+overmod still collapses at WOT,
// so keep disabled until loaded test-stand validation proves the reconstruction path.
SVGENCURRENT_Obj    svgencurrent;
SVGENCURRENT_Handle svgencurrentHandle;
MATH_Vec3 adcDataPrev = {0.0f, 0.0f, 0.0f};      //!< previous-cycle currents for reconstruction
MATH_Vec3 pwmDataPrev = {0.0f, 0.0f, 0.0f};      //!< previous-cycle PWM for min-width comp
SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_USE_ALL;
SVGENCURRENT_VmidShunt_e   midVolShunt = SVGENCURRENT_VMID_A;
#endif

#ifdef ESC6288_VSF
VSF_Obj       vsf;
VSF_Handle    vsfHandle;

#define PRODUCT_PWM_FREQ_DEFAULT_HZ ((uint16_t)(USER_PWM_FREQ_kHz * 1000.0f))
#ifndef ESC6288_VSF_DEFAULT
#define ESC6288_VSF_DEFAULT  (false)
#endif
static volatile bool     g_vsf_enable = ESC6288_VSF_DEFAULT;
static volatile uint16_t g_vsf_pwm_freq_Hz = PRODUCT_PWM_FREQ_DEFAULT_HZ;
#endif

TRAJ_Handle   trajHandle_spd;
TRAJ_Obj      traj_spd;

TRAJ_Handle   trajHandle_Id;
TRAJ_Obj      traj_Id;

TRAJ_Handle   trajHandle_fwc;
TRAJ_Obj      traj_fwc;

FILTER_FO_Handle  filterHandle_I[USER_NUM_CURRENT_SENSORS];
FILTER_FO_Obj     filter_I[USER_NUM_CURRENT_SENSORS];

FILTER_FO_Handle  filterHandle_V[USER_NUM_VOLTAGE_SENSORS];
FILTER_FO_Obj     filter_V[USER_NUM_VOLTAGE_SENSORS];

MATH_Vec2 IdqSet_A = {0.0, 0.0};   //!< product-driven Iq reference (Amps)

//! \brief Open-loop I/f sensorless self-start (esc6288). A free/low-flux shaft cannot start on the
//! FAST angle at standstill (it oscillates). This drives the rotor open-loop -- align to a known
//! angle, then ramp an open-loop electrical angle while holding a d-axis current -- until FAST's angle
//! coheres with the open-loop angle, then HANDS OFF to the normal FAST-angle + throttle-Iq path.
//! Bench-proven on is04 (align 3 A, ramp to 30-50 Hz, FAST locks with <7 deg angle error). Params are
//! live-tunable over DSS. enable=0 -> exact legacy behavior (FAST angle from standstill, no override).
typedef enum { SU_IDLE=0, SU_ALIGN=1, SU_RAMP=2, SU_BLEND=3, SU_RUN=4, SU_FAULT=5 } su_state_t;
typedef struct {
    uint16_t   enable;                  //!< 1 = run the open-loop I/f startup; 0 = legacy FAST-angle path
    su_state_t state;                   //!< current startup state
    float32_t  angle_rad;               //!< open-loop electrical angle output (ALIGN/RAMP)
    float32_t  freq_Hz;                 //!< current open-loop electrical frequency
    float32_t  blend;                   //!< 0..1 handoff current-blend fraction (SU_BLEND)
    uint32_t   tick;                    //!< ISR ticks elapsed in the current state
    uint32_t   cohTick;                 //!< consecutive coherent ticks (handoff dwell)
    uint32_t   slipTick;                //!< consecutive slip ticks (ramp slip guard)
    uint16_t   fault;                   //!< 1 = startup stalled -> SU_FAULT (safe-off at ISR site)
    // --- live-tunable parameters (DSS) ---
    float32_t  id_align_A;              //!< d-axis align current (rotor lock)
    float32_t  id_ramp_A;               //!< d-axis current held during the open-loop ramp
    float32_t  align_s;                 //!< align dwell time, s
    float32_t  accel_Hzps;              //!< open-loop ramp rate, electrical Hz/s (PROP: use ~2.0)
    float32_t  handoff_Hz;              //!< min open-loop freq before handoff is allowed
    float32_t  handoff_err_rad;         //!< max |FAST angle - open-loop angle| for handoff
    float32_t  dwell_s;                 //!< coherence must hold this long before handoff
    float32_t  blend_s;                 //!< SU_BLEND: ramp Id->0 / Iq->throttle over this time
    float32_t  timeout_s;               //!< ramp watchdog: no handoff by here -> SU_FAULT
    float32_t  slip_check_Hz;           //!< begin slip guard once open-loop freq exceeds this
    float32_t  slip_frac;               //!< require FAST speed >= slip_frac * open-loop freq
    float32_t  slip_win_s;              //!< slip must persist this long before SU_FAULT
    float32_t  max_err_rad;             //!< sustained angle error over this -> SU_FAULT
} su_t;
su_t g_su = {
    1u, SU_IDLE, 0.0f, 0.0f, 0.0f, 0u, 0u, 0u, 0u,
    3.0f,   /* id_align_A      */
    3.0f,   /* id_ramp_A       */
    1.5f,   /* align_s         */
    25.0f,  /* accel_Hzps (no-load; PROP -> ~2.0) */
    35.0f,  /* handoff_Hz      */
    0.35f,  /* handoff_err_rad (~20 deg) */
    0.05f,  /* dwell_s (50 ms) */
    0.10f,  /* blend_s (100 ms)*/
    4.0f,   /* timeout_s       */
    10.0f,  /* slip_check_Hz   */
    0.5f,   /* slip_frac       */
    0.20f,  /* slip_win_s      */
    1.0f    /* max_err_rad     */
};

#ifdef ESC6288_BENCH_THROTTLE
//! \brief Bench-only throttle injection (bypasses the CAN arbiter) to exercise the I/f self-start over
//! DSS. Set g_bench_iq_A, then g_bench_arm=1 to arm + command a torque; g_bench_arm=0 coasts. Built
//! only with --define=ESC6288_BENCH_THROTTLE; never in a production image.
float32_t g_bench_iq_A = 0.0f;
uint16_t  g_bench_arm  = 0u;
#endif

#ifdef PWMDAC_ENABLE
HAL_PWMDACData_t pwmDACData;
#pragma DATA_SECTION(pwmDACData, "ctrl_data");
#endif  // PWMDAC_ENABLE

//
// the product objects (pure src/ state machines + protocol core + CAN glue config)
//
static esc_control_state_t g_esc;
static dronecan_t          g_dn;
static foc_bridge_cfg_t    g_fbcfg;

//! In-RAM mirror of the persisted parameter record (node-id + learned park ref). The
//! storage FORMAT is validated host-side (src/app/nvparam); the actual Flash erase/program
//! is deferred (target work). Until then this stays at defaults, so behaviour is unchanged:
//! node_id falls back to BUILD_NODE_ID and the park ref starts unlearned.
static nvparam_t           g_nvparam;

static esc_command_t       g_cmd;        //!< last command emitted by the arbiter (valid if g_have_cmd)
static bool                g_have_cmd;
static esc_arbiter_state_t g_arb;
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
static volatile esc_src_id_t g_arb_active;      /* debugger-visible: who owns throttle */
static volatile uint32_t     g_arb_status_bits; /* debugger-visible: esc_arb_status_t bits
                                                 * (PWM_ACTIVE / NO_SOURCE / PWM_LOCKOUT /
                                                 * HANDOFF) -- bench step 5/6 watch these,
                                                 * since esc_telemetry_t only carries SRC_PWM. */
#endif
static uint32_t            g_now_ms;      //!< 1 ms tick counter (dronecan scheduling)

//! GetNodeInfo identity so the node enumerates on ArduPilot / yakut / DroneCAN GUI.
#define PRODUCT_NODE_NAME         "com.alux.novax.esc6288"   // <= DRONECAN_NODE_NAME_MAX (32)
#define PRODUCT_SW_VERSION_MAJOR  0u
#define PRODUCT_SW_VERSION_MINOR  1u
#define PRODUCT_HW_VERSION_MAJOR  1u   // esc6288 rev A (launchxl bench shares the product fw id)
#define PRODUCT_HW_VERSION_MINOR  0u

//! One tick may emit the worst-case GetNodeInfo response (up to 11 frames for a 32-char name)
//! ahead of NodeStatus(1) + esc.Status(3); size the buffer + tick cap to hold them all.
#define PRODUCT_TXBUF_FRAMES      16
static dronecan_frame_t    g_txbuf[PRODUCT_TXBUF_FRAMES];

//
// the functions
//

//! \brief Park-reference NV loader: returns the park ref held in the in-RAM nvparam mirror.
//!        Until the Flash read is implemented g_nvparam stays at defaults (invalid), so this
//!        still reports "unlearned" on every boot -- and with enc_valid=false on launchxl the
//!        learn path never runs anyway. esc6288 with a real encoder learns + raises a store.
static float product_park_ref_load(void *ctx, bool *out_valid)
{
    (void)ctx;
    *out_valid = g_nvparam.park_ref_valid;
    return g_nvparam.park_ref_target_rev;
}

//! \brief Derive the 16-byte DroneCAN unique-id from the device OTP UID (PSRAND0..3, the
//!        real per-chip 128-bit region), one wire byte (low 8 bits) per array element.
static void product_read_unique_id(uint16_t uid[16])
{
    const uint16_t off[4] = { OTP_O_UID_PSRAND0, OTP_O_UID_PSRAND1,
                              OTP_O_UID_PSRAND2, OTP_O_UID_PSRAND3 };
    uint16_t k;

    for(k = 0; k < 4u; k++)
    {
        uint32_t w = HWREG(UID_BASE + off[k]);
        uid[(k * 4u) + 0u] = (uint16_t)(w & 0xFFu);
        uid[(k * 4u) + 1u] = (uint16_t)((w >> 8) & 0xFFu);
        uid[(k * 4u) + 2u] = (uint16_t)((w >> 16) & 0xFFu);
        uid[(k * 4u) + 3u] = (uint16_t)((w >> 24) & 0xFFu);
    }
}

//! \brief Build the launchxl bench control configuration. Limits are conservative bench
//!        defaults (TODO: tune per AM-4116 + board on the bench, plan ③-c commit 4).
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
// esc6288_revA board-side aux drivers (driverlib): RC-PWM throttle capture (eCAP1),
// MT6701 SSI encoder read (SPIA), and the WS2812 RGB status LED (GPIO12).
#include "rc_pwm.h"
#include "mt6701_ssi.h"
#include "mt6701.h"        // pure angle processing (raw SSI code -> angle/velocity)
#include "ntc.h"           // pure NTC thermistor ADC-count -> degrees C
#include "rgb_led.h"

// MT6701 processed-angle state: raw 14-bit SSI code -> mechanical/electrical angle, unwrap,
// velocity, glitch/stale. Fed one frame per 1 ms tick from MT6701_SSI_read().
static mt6701_state_t g_enc;

// Board NTC divider + curve (values from board.h; trim on the bench). Drives esc_control's
// over-temp latch via raw.temp_C.
static const ntc_cfg_t g_ntc_cfg = {
    BOARD_NTC_R_FIXED_OHM, BOARD_NTC_R25_OHM, BOARD_NTC_BETA_K, BOARD_NTC_T0_K,
    BOARD_NTC_ADC_FULL_COUNTS, BOARD_NTC_LOW_SIDE, BOARD_NTC_OPEN_TEMP_C
};
#endif

// esc6288_revA fast (per-ISR, 20 kHz) software overcurrent backstop, amps. Phases A/B have
// no CMPSS comparator, so this is their fast trip (phase C also has CMPSS3). Hard backstop
// above the 1 ms product OC limit (oc_set_A = 30 A) and below the +/-127 A sense FS (current
// scale corrected 330->254 on 2026-07-01; all A-thresholds now trip at TRUE amps). Tune on the
// bench against the FET/shunt rating; for first 15" prop starts drop oc_set_A to ~10-15 A.
#define ESC6288_ISR_OC_A   (60.0f)

static void product_build_esc_cfg(esc_control_cfg_t *c)
{
    uint16_t i;
    uint16_t *p = (uint16_t *)c;
    for(i = 0; i < (sizeof(*c) / sizeof(uint16_t)); i++) { p[i] = 0u; }  // zero (incl. dormant park*)

    c->iq_max_A             = 5.0f;     // throttle=1.0 -> 5 A (bench-safe)
    c->iq_slew_A_s          = 50.0f;
    c->cmd_timeout_s        = 0.5f;     // link-loss failsafe
    c->throttle_run_thresh  = 0.02f;
    c->throttle_idle_eps    = 0.01f;
    c->park_engage_speed_krpm = 0.2f;
    c->auto_park_enable     = false;    // launchxl: no encoder -> never park
    c->failsafe_brake       = false;    // coast on link loss

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // Closed speed loop in RUN (is07 port): OFF by default (torque RUN stays the validated
    // path). The same ESC6288_SPEED_MODE_DEFAULT build define that opens the product-side gate
    // (SPEED_PATH_ALLOWED) also selects the esc_control speed-RUN mode, so one define flips the
    // whole chain consistently. Full throttle maps to the motor's rated speed.
#ifndef ESC6288_SPEED_MODE_DEFAULT
#define ESC6288_SPEED_MODE_DEFAULT  (false)
#endif
    c->speed_run_enable = (bool)(ESC6288_SPEED_MODE_DEFAULT);
    c->speed_max_krpm   = (float)USER_MOTOR_RATED_SPEED_KRPM;   // 4116: 5.0 krpm at throttle=1.0

    // esc6288_revA / 12S target (conservative bench bring-up defaults; raise after
    // validation). Hardware backstops: CMPSS3 phase-C OC + CMPSS5 bus OV (~56 V). These
    // software limits are the PRIMARY protection for phases A/B (which have no CMPSS).
    c->oc_set_A   = 30.0f;  c->oc_clr_A   = 25.0f;   // TRUE amps (FS corrected to +/-127 A 2026-07-01); drop to ~10-15 A for first prop
    c->vbus_ov_set = 54.0f; c->vbus_ov_clr = 50.0f;  // 12S max charge 50.4 V; HW OV ~56 V
    c->vbus_uv_set = 18.0f; c->vbus_uv_clr = 22.0f;  // low for bench; raise for flight
    c->temp_ot_set = 100.0f; c->temp_ot_clr = 85.0f; // live: NTC->degC via ntc.c (trim curve on bench)
#ifdef ESC6288_BENCH_LOWVOLT
    // Bench-only (12 V PSU, NTC not yet populated): the flight undervolt trip (18 V) false-fires at 12 V,
    // and the open/unpopulated NTC reads ~150 C (open_temp_C) -> false over-temp. Both put esc_control in
    // FAULT and block CAN commanding. Relax them for bench bring-up. Built with --define=ESC6288_BENCH_LOWVOLT;
    // NEVER in a flight image (revert once a real battery >18 V and a populated NTC are on the board).
    c->vbus_uv_set = 8.0f;  c->vbus_uv_clr = 6.0f;
    c->temp_ot_set = 200.0f; c->temp_ot_clr = 190.0f;
#endif
#else
    c->oc_set_A   = 8.0f;  c->oc_clr_A   = 6.0f;
    c->vbus_ov_set = 30.0f; c->vbus_ov_clr = 28.0f;
    c->vbus_uv_set = 9.0f;  c->vbus_uv_clr = 11.0f;
    c->temp_ot_set = 100.0f; c->temp_ot_clr = 85.0f;
#endif
    // c->park / c->park_ref left zeroed (dormant on launchxl)
}

static void product_build_arb_cfg(esc_arbiter_cfg_t *a)
{
    uint16_t i;
    uint16_t *p = (uint16_t *)a;
    for (i = 0; i < (sizeof(*a) / sizeof(uint16_t)); i++) { p[i] = 0u; }

    /* SAFE DEFAULT: CAN only. RC-PWM is ignored until the bench gate flips this to
     * ESC_ARB_CAN_PRIMARY (see docs plan Appendix). Shipping behavior == CAN-only. */
    a->policy              = ESC_ARB_EXPLICIT_CAN;
    a->can_stale_s         = 0.10f;   /* ~10 missed 100 Hz RawCommands */
    a->pwm_stale_s         = 0.06f;   /* ~3 missed 50 Hz servo pulses  */
    a->pwm_arm_low_s       = 0.50f;   /* sustained idle before PWM may arm */
    a->pwm_arm_throttle_eps = 0.05f;
    a->track_tol           = 0.10f;   /* PWM within 10% of CAN counts as tracking */
    a->track_required_s    = 0.50f;
    a->handoff_slew_per_s  = 4.0f;    /* full-scale handoff over ~0.25 s */
    a->pwm.us_min = 1000.0f; a->pwm.us_max = 2000.0f;
    a->pwm.us_valid_min = 900.0f; a->pwm.us_valid_max = 2100.0f;
}

//! \brief Initialize the product layer (control state machine, protocol core, CAN bridge).
static void product_init(void)
{
    esc_control_cfg_t cfg;
    dronecan_cfg_t    dncfg;
    uint16_t i;

    g_have_cmd = false;
    g_now_ms   = 0u;
    g_cmd.seq = 0u; g_cmd.throttle = 0.0f; g_cmd.arm = false;

    g_fbcfg.pole_pairs     = (float)USER_MOTOR_NUM_POLE_PAIRS;
    g_fbcfg.iq_cmd_limit_A = 6.0f;   // redundant hard cap, just above iq_max_A
    // speed-mode |speedRef| ceiling (elec Hz), from the motor profile. Full-throttle speed-RUN
    // commands rated speed (speed_max_krpm), which must clear this clamp: e.g. 4116 rated
    // 5 krpm * 7 pp = 583 Hz < 800 Hz profile max. (Was 100 Hz in the park-only era -- that
    // silently capped the is07 speed loop at ~857 rpm on the bench, 2026-07-02.)
    g_fbcfg.speed_max_hz   = (float)USER_MOTOR_FREQ_MAX_HZ;

    // Seed the persisted-parameter mirror. A real build would decode it from Flash here
    // (nvparam_decode of the read-back words); that read is deferred, so we start at
    // defaults -> park ref unlearned, node_id 0 (DNA) -> dncfg.node_id keeps BUILD_NODE_ID.
    nvparam_set_defaults(&g_nvparam);

    product_build_esc_cfg(&cfg);
    esc_control_init(&g_esc, &cfg, product_park_ref_load, NULL);

    {
        esc_arbiter_cfg_t acfg;
        product_build_arb_cfg(&acfg);
        esc_arbiter_init(&g_arb, &acfg);
    }

    for(i = 0; i < 16u; i++) { dncfg.unique_id[i] = 0u; }
    product_read_unique_id(dncfg.unique_id);
    dncfg.esc_index             = (uint16_t)BUILD_ESC_INDEX;
    dncfg.arm_zero_frames       = 0u;   // -> default 10
    // A persisted static id (nvparam) wins over the build default; 0 -> fall back to
    // BUILD_NODE_ID (itself 0 = DNA on a normal build). Defaults today -> BUILD_NODE_ID.
    dncfg.node_id               = (g_nvparam.node_id != 0u) ? g_nvparam.node_id
                                                            : (uint16_t)BUILD_NODE_ID;
    dncfg.node_status_period_ms = 0u;   // -> default 1000
    dncfg.esc_status_period_ms  = 0u;   // -> default 100
    dncfg.dna_request_period_ms = 0u;   // -> default 1000
    dncfg.dna_start_delay_ms    = 0u;
    dncfg.hw_version_major      = PRODUCT_HW_VERSION_MAJOR;
    dncfg.hw_version_minor      = PRODUCT_HW_VERSION_MINOR;
    dncfg.sw_version_major      = PRODUCT_SW_VERSION_MAJOR;
    dncfg.sw_version_minor      = PRODUCT_SW_VERSION_MINOR;
    dncfg.sw_vcs_commit         = (uint32_t)BUILD_SW_VCS_COMMIT;  // git short hash (build.sh)
    dncfg.node_name             = PRODUCT_NODE_NAME;
    dncfg.nvparam               = &g_nvparam;  // GetSet param face reads/writes this mirror
    dronecan_init(&g_dn, &dncfg);

    can_bridge_init();

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // Bring up the esc6288 board-side aux interfaces (after HAL setup; before the main
    // loop). RC_PWM_init re-enables the eCAP1 clock the shared HAL leaves off. These read
    // APIs (RC_PWM_read / MT6701_SSI_read / RGB_setColor) are wired into the control
    // path during bench bring-up.
    RC_PWM_init();
    MT6701_SSI_init();
    RGB_init();

#ifdef RGB_SELFTEST
    /* [BENCH] one-shot WS2812 color sweep so the GPIO12 -> SN74LVC1T45 -> RGB1 path can be eyeballed.
     * NOT in the default image: build with EXTRA_DEFINES="--define=RGB_SELFTEST". Runs once during
     * init while the gates are still held off (OST, disarmed); ~5 s total. */
    {
        const uint16_t sw[5][3] = { {255u,0u,0u}, {0u,255u,0u}, {0u,0u,255u}, {255u,255u,255u}, {0u,0u,0u} };
        uint16_t ci, di;
        for (ci = 0u; ci < 5u; ci++) {
            /* Drive 2 chained LEDs (RGB1 + GH3 external connector) so the external one is exercised too. */
            RGB_setColorN(2u, (uint8_t)sw[ci][0], (uint8_t)sw[ci][1], (uint8_t)sw[ci][2]);
            for (di = 0u; di < 120u; di++) { SysCtl_delay(500000U); }   /* ~3 s hold per color */
        }
    }
#endif

    // MT6701 angle-processing config. Bench-pending tuning: zero_offset (mechanical zero),
    // dir (rotation sign vs the motor), and the glitch/stale thresholds. auto_park stays
    // DISABLED (product_build_esc_cfg) until the encoder is validated on the bench with a prop.
    {
        mt6701_cfg_t ecfg;
        ecfg.counts_per_rev       = 16384.0f;
        ecfg.dir                  = 1;
        ecfg.zero_offset_counts   = 0u;
        ecfg.pole_pairs           = (uint16_t)USER_MOTOR_NUM_POLE_PAIRS;
        ecfg.vel_iir_alpha        = 0.2f;    // light low-pass at the 1 kHz tick
        ecfg.max_delta_rev        = 0.25f;   // glitch threshold (rev/tick)
        ecfg.stale_limit_samples  = 5u;
        ecfg.glitch_stale_samples = 5u;
        mt6701_init(&g_enc, &ecfg);
    }
#endif
}

// ---- Speed-mode gate ----
// The product speed path is OFF by default and is a fail-safe disable unless BOTH (a) this is the
// esc6288 board AND (b) the runtime flag g_speed_mode_enable is set (flip via debugger, or a
// future DroneCAN param). The build default of that flag is ESC6288_SPEED_MODE_DEFAULT; flip it
// from the standard build entry with EXTRA_DEFINES="--define=ESC6288_SPEED_MODE_DEFAULT=1" (see
// build.sh). When allowed, apply_setpoint lands motorVars.speedRef_Hz AND arms the ISR speed PI
// (is07 port): once the I/f self-start hands off (SU_RUN), PI_run_series(piHandle_spd) closes
// speed_ref -> Iq_ref every numCtrlTicksPerSpeedTick. Bench-validate before enabling by default.
#ifndef ESC6288_SPEED_MODE_DEFAULT
#define ESC6288_SPEED_MODE_DEFAULT  (false)
#endif
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
static volatile bool g_speed_mode_enable = ESC6288_SPEED_MODE_DEFAULT;
#define SPEED_PATH_ALLOWED()   (g_speed_mode_enable)
#else
#define SPEED_PATH_ALLOWED()   (false)
#endif

// True while apply_setpoint is landing a closed-speed-loop command; read by the ISR to decide
// whether the speed PI owns Idq_ref_A.value[1] (vs the torque path copying IdqSet_A each tick).
// Single-word bool: 1 ms writer / 20 kHz reader, no atomicity issue; a one-tick stale value at a
// mode transition is bounded by the current-PI limits.
static volatile bool g_speed_pi_run = false;

//! \brief Land the FOC setpoint on the SDK control variables. Torque is the only closed path
//!        today; the speed path is a gated skeleton (see SPEED_PATH_ALLOWED above).
//!        Caller is the 1 ms background tick; the ISR only READS these single-word fields.
static void apply_setpoint(const foc_setpoint_t *sp_in)
{
    foc_setpoint_t sp = *sp_in;

    // Gate the speed path: when not allowed, a speed-mode request becomes a coast-disable
    // (pure helper), so the disable branch below handles it -- exactly the previous behavior.
    foc_bridge_gate_speed(&sp, SPEED_PATH_ALLOWED());

    if((sp.enable == false) || (sp.brake == true))
    {
        // Coast: drop the run flag and zero Iq. launchxl has no active short-brake yet
        // (brake degrades to disable); a real brake is deferred to esc6288.
        g_speed_pi_run = false;
        IdqSet_A.value[1] = 0.0f;
        motorVars.flagRunIdentAndOnLine = 0;
        return;
    }

    if(sp.speed_mode == true)
    {
        // Closed speed loop (is07 port; reached only when the gate allows it): land the clamped
        // speed reference on the SDK control variable and hand Iq ownership to the ISR speed PI
        // (it closes speed_ref -> Iq_ref once the self-start reaches SU_RUN).
        IdqSet_A.value[0] = 0.0f;
        IdqSet_A.value[1] = 0.0f;              // ISR speed PI owns Iq from SU_RUN onward
        motorVars.speedRef_Hz = sp.speed_ref_hz;
        g_speed_pi_run = true;
        // Arm once offset cal is done AND no fault is latched. The fault gate makes a startup
        // slip/OC latch STICKY: without it, the main loop clears flagRun on faultUse but this
        // re-arms every 1 ms while the throttle is held -> the startup SM retries and repeatedly
        // pulses the motor. Latched until power cycle / image reload (coast does not clear
        // faultNow); auto-recovery on a genuine throttle-drop is a follow-up.
        if((motorVars.flagEnableOffsetCalc == false) && (motorVars.faultUse.all == 0))
        {
            motorVars.flagRunIdentAndOnLine = 1;
        }
        return;
    }

    // Torque: the ISR copies IdqSet_A -> Idq_ref_A directly (no speed loop).
    g_speed_pi_run = false;
    IdqSet_A.value[0] = 0.0f;
    IdqSet_A.value[1] = sp.iq_ref_A;
    // Arm once offset cal is done AND no fault is latched (fault gate makes a startup slip/OC
    // latch sticky instead of auto-re-arming while the throttle is held -- see the speed branch).
    if((motorVars.flagEnableOffsetCalc == false) && (motorVars.faultUse.all == 0))
    {
        motorVars.flagRunIdentAndOnLine = 1;
    }
}

//! \brief One 1 ms product tick: RX drain -> control step -> apply, then periodic TX.
static void product_tick_1ms(void)
{
    dronecan_frame_t    rxf;
    dronecan_rx_result_t rr;
    foc_raw_feedback_t  raw;
    esc_feedback_t      fb;
    esc_output_t        out;
    esc_telemetry_t     tel;
    foc_setpoint_t      sp;
    float imot;
    int n, i;

    g_now_ms++;

    // 1) drain CAN frames -> build the CAN source sample (last command addressed to us)
    {
        esc_src_sample_t can_s = {0};
        esc_src_sample_t pwm_s = {0};
        esc_arbiter_result_t ar;

        while (can_bridge_read(&rxf))
        {
            dronecan_on_rx(&g_dn, &rxf, &rr);
            if (rr.command_updated)
            {
                can_s.fresh    = true;
                can_s.valid    = true;
                can_s.throttle = rr.command.throttle;
                can_s.arm_req  = rr.command.arm;
            }
        }

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
        // RC-PWM source: poll eCAP1, decode the raw high-time to a normalized sample.
        {
            rc_pwm_sample_t pwm_raw;
            RC_PWM_read(&pwm_raw);
            pwm_s = esc_pwm_decode(&g_arb.cfg.pwm, pwm_raw.fresh, pwm_raw.overflow, pwm_raw.width_us);
        }
#endif

        esc_arbiter_step(&g_arb, &can_s, &pwm_s, 0.001f, &ar);
        g_cmd      = ar.cmd;          // valid only if ar.have_cmd
        g_have_cmd = ar.have_cmd;
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
        g_arb_active      = ar.active;       // debugger / future telemetry
        g_arb_status_bits = ar.status_bits;  // PWM_ACTIVE / NO_SOURCE / PWM_LOCKOUT / HANDOFF
#endif
    }

    // 2) assemble feedback (gate_fault: only boards with a gate-fault input set it -- read as
    //    active-low when BOARD_HAS_GATE_FAULT_INPUT; esc6288_revA (JSM6288T) and launchxl_3phganinv
    //    have no such pin, so it stays false. i_motor = peak |phase current|; temperature is the
    //    esc6288 NTC (below) or a 25 C placeholder on boards with no sensor)
    imot = fmaxf(fmaxf(fabsf(adcData.I_A.value[0]), fabsf(adcData.I_A.value[1])),
                 fabsf(adcData.I_A.value[2]));
    raw.vbus_V         = motorVars.VdcBus_V;
    raw.iq_meas_A      = Idq_in_A.value[1];
    raw.i_motor_A      = imot;
    raw.speed_est_krpm = motorVars.speed_krpm;
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // board NTC (ADCINC3 -> ADCC SOC2): raw count -> degrees C; feeds esc_control's over-temp
    // latch (temp_ot_set). A dead sensor (open/short) reads back open_temp_C (150 C) -> trips.
    raw.temp_C         = ntc_counts_to_celsius(&g_ntc_cfg,
                             (uint16_t)ADC_readResult(BOARD_NTC_ADC_RESULT_BASE, BOARD_NTC_ADC_SOC));
#else
    raw.temp_C         = 25.0f;   // launchxl has no temperature sensor wired
#endif
    raw.gate_fault     = (BOARD_HAS_GATE_FAULT_INPUT != 0) &&
                         (GPIO_readPin(BOARD_GATE_FAULT_GPIO) == 0U);

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // MT6701 absolute encoder: read one CRC/field-validated SSI frame, process into a
    // mechanical angle + velocity. enc_valid gates esc_control's parking path.
    {
        uint16_t enc_raw14;
        bool     enc_ok = MT6701_SSI_read(&enc_raw14);
        mt6701_update(&g_enc, enc_raw14, enc_ok, 0.001f);
        raw.enc_mech_rev  = mt6701_mech_rev(&g_enc);
        raw.enc_vel_revps = mt6701_vel_revps(&g_enc);
        raw.enc_valid     = mt6701_valid(&g_enc);
        raw.enc_stale     = mt6701_stale(&g_enc);
    }
#else
    raw.enc_mech_rev  = 0.0f;   // no encoder on this board ->
    raw.enc_vel_revps = 0.0f;
    raw.enc_valid     = false;  // esc_control never enters parking
    raw.enc_stale     = false;
#endif
    foc_bridge_map_feedback(&raw, &fb);
    // Feedback is only trustworthy once the ADC offset calibration is done; before that the current/
    // Vbus readings are garbage and would false-latch OC/UV in esc_control. Gate the hard-fault eval.
    fb.valid = (motorVars.flagEnableOffsetCalc == false);

    // 3) run the control state machine (NULL only when no source is healthy; a held command re-emits the same seq so esc_control's watchdog ages from the last real frame)
    (void)esc_control_step(&g_esc, g_have_cmd ? &g_cmd : NULL, &fb, 0.001f, &out, &tel);
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    if (g_arb_active == ESC_SRC_PWM) { tel.status_bits |= (uint32_t)ESC_ST_SRC_PWM; } // forward hook: ESC_ST_SRC_PWM not yet serialized in esc.Status; observe via g_arb_active/g_arb_status_bits in the debugger
#endif
#ifdef ESC6288_RS_TELEM_DEBUG
    // BENCH ONLY (is10 validation): stream Rs tracking over the esc.Status debug channel,
    // debugger-free. low16 = model Rs in 10*milliohm, high16 = live RsOnLine estimate.
    tel.dbg_u32 = ((uint32_t)(motorVars.RsOnLine_Ohm * 1.0e5f) << 16)
                |  (uint32_t)(motorVars.Rs_Ohm       * 1.0e5f);
#endif

    // 4) map the control output to a FAST setpoint and apply it
    foc_bridge_map_output(&g_fbcfg, &out, &sp);
#ifdef ESC6288_BENCH_THROTTLE
    if(g_bench_arm != 0u)
    {   // bench override: arm + command a raw torque, bypassing the arbiter (I/f self-start test)
        sp.enable = true;  sp.brake = false;  sp.speed_mode = false;  sp.iq_ref_A = g_bench_iq_A;
    }
#endif
    apply_setpoint(&sp);

    // 5) park-ref store request: fold the freshly learned reference into the nvparam mirror,
    //    then clear the request. The Flash write (nvparam_encode -> erase/program) is deferred;
    //    on launchxl the request never fires (enc_valid=false), so this is a no-op there.
    if(g_esc.ref.needs_store)
    {
        (void)nvparam_update_park_ref(&g_nvparam, true, g_esc.ref.new_target);
        park_ref_clear_store_request(&g_esc.ref);
        // TODO(target): nvparam_encode(&g_nvparam, words) -> Flash erase/program.
    }

    // 6) periodic TX: DNA while unallocated; GetNodeInfo response (if requested) + NodeStatus
    //    + esc.Status once allocated
    n = dronecan_tick(&g_dn, g_now_ms, &tel, g_txbuf, PRODUCT_TXBUF_FRAMES);
    for(i = 0; i < n; i++)
    {
        (void)can_bridge_write(&g_txbuf[i]);
    }
    if(dronecan_node_id_dirty(&g_dn))
    {
        // DNA allocated an id: record it in the nvparam mirror, then clear the dirty flag.
        (void)nvparam_update_node_id(&g_nvparam, dronecan_node_id(&g_dn));
        dronecan_clear_node_id_dirty(&g_dn);
        // TODO(target): nvparam_encode(&g_nvparam, words) -> Flash erase/program.
    }
    if(dronecan_param_dirty(&g_dn))
    {
        // A GetSet write already updated g_nvparam (via nvparam_update_*); just acknowledge.
        dronecan_clear_param_dirty(&g_dn);
        // TODO(target): nvparam_encode(&g_nvparam, words) -> Flash erase/program.
    }
}

//! \brief Rs online recalibration service (is10 port, main-loop cadence). While the motor RUNS
//! (product run flag set, I/f startup handed off, FAST online) and the feature is enabled, FAST
//! injects an Id ripple of RsOnLineCurrent_A to estimate the hot stator resistance; the estimate
//! is committed to the model (EST_setFlag_updateRs) once within 5% of the current model Rs.
//!
//! Edge-tracked (deviation from is10): the lab re-writes the "disabled" EST state every main-loop
//! pass; in the product's tight 1 ms loop that continuous hammering runs all through the I/f
//! startup and corrupts FAST's own startup Rs recalc (bench 2026-07-02: model Rs walked
//! 21.3 -> 44.3 -> 59.5 mOhm across start attempts and the startup slipped). The disable/reset
//! actions therefore fire ONCE on the active->inactive edge and the EST is left alone otherwise.
static void product_run_rs_online(EST_Handle handle)
{
    static bool s_was_active = false;

    bool active = (motorVars.flagRunIdentAndOnLine != 0) &&
                  (g_su.state == SU_RUN) &&
                  (EST_getState(handle) == EST_STATE_ONLINE) &&
                  (motorVars.flagEnableRsOnLine == true);

    if(active)
    {
        EST_setFlag_enableRsOnLine(handle, true);
        EST_setRsOnLineId_mag_A(handle, motorVars.RsOnLineCurrent_A);

        float32_t RsError_Ohm = motorVars.RsOnLine_Ohm - motorVars.Rs_Ohm;

        if(fabsf(RsError_Ohm) < (motorVars.Rs_Ohm * 0.05f))    // within 5% -> commit
        {
            EST_setFlag_updateRs(handle, true);
        }
    }
    else if(s_was_active)
    {
        // one-shot disable/reset on the falling edge only
        EST_setRsOnLineId_mag_A(handle, 0.0f);
        EST_setRsOnLineId_A(handle, 0.0f);
        EST_setRsOnLine_Ohm(handle, EST_getRs_Ohm(handle));
        EST_setFlag_enableRsOnLine(handle, false);
        EST_setFlag_updateRs(handle, false);
    }

    s_was_active = active;
}

#ifdef ESC6288_FWC_MTPA
static inline void product_apply_fwc_mtpa_current_angle(void)
{
    if(g_fwc_mtpa_enable && (g_su.state == SU_RUN))
    {
        counterFWCandMTPA++;
        if(counterFWCandMTPA >= PRODUCT_FWC_MTPA_TICKS)
        {
            MATH_Vec2 currentPhasor;

            counterFWCandMTPA = 0u;
            motorVars.IsRef_A = Idq_ref_A.value[1];
            motorVars.Vs_V = sqrtf((Vdq_out_V.value[0] * Vdq_out_V.value[0]) +
                                   (Vdq_out_V.value[1] * Vdq_out_V.value[1]));
            motorVars.VsRef_V = motorVars.VsRef_pu * adcData.dcBus_V;

            FWC_computeCurrentAngle(fwcHandle, motorVars.Vs_V, motorVars.VsRef_V);
            MTPA_computeCurrentAngle(mtpaHandle, motorVars.IsRef_A);
            motorVars.angleCurrent_rad =
                MATH_max(FWC_getCurrentAngle_rad(fwcHandle),
                         MTPA_getCurrentAngle_rad(mtpaHandle));

            currentPhasor.value[0] = cosf(motorVars.angleCurrent_rad);
            currentPhasor.value[1] = sinf(motorVars.angleCurrent_rad);

            if(motorVars.IsRef_A >= 0.0f)
            {
                Idq_ref_A.value[0] = motorVars.IsRef_A * currentPhasor.value[0];
            }
            else
            {
                Idq_ref_A.value[0] = -(motorVars.IsRef_A * currentPhasor.value[0]);
            }
            Idq_ref_A.value[1] = motorVars.IsRef_A * currentPhasor.value[1];
            Idq_ref_A.value[0] += EST_getIdRated_A(estHandle);
        }
    }
    else
    {
        counterFWCandMTPA = 0u;
        motorVars.IsRef_A = Idq_ref_A.value[1];
        motorVars.angleCurrent_rad = 0.0f;
    }
}
#endif

void main(void)
{
    uint16_t estNumber = 0;
    bool flagEstStateChanged = false;

    //
    // initialize the user parameters
    //
    USER_setParams(&userParams);

    userParams.flag_bypassMotorId = true;

    USER_setParams_priv(&userParams);

    //
    // initialize the driver
    //
    halHandle = HAL_init(&hal, sizeof(hal));

    //
    // set the driver parameters
    //
    HAL_setParams(halHandle);

    //
    // initialize and configure the Clarke modules
    //
    clarkeHandle_I = CLARKE_init(&clarke_I, sizeof(clarke_I));
    clarkeHandle_V = CLARKE_init(&clarke_V, sizeof(clarke_V));
    setupClarke_I(clarkeHandle_I, userParams.numCurrentSensors);
    setupClarke_V(clarkeHandle_V, userParams.numVoltageSensors);

    //
    // initialize and configure the estimator
    //
    estHandle = EST_initEst(estNumber);
    EST_setParams(estHandle, &userParams);
    EST_setFlag_enableForceAngle(estHandle, motorVars.flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(estHandle, motorVars.flagEnableRsRecalc);

    // Rs online recalibration (is10 port): tracks stator-resistance thermal drift while the
    // motor RUNS by injecting an Id ripple (RsOnLineCurrent_A) and committing the estimate when
    // it is within 5% of the model Rs (see product_run_rs_online, called from the main loop). OFF by
    // default: the injection (USER_MOTOR_RES_EST_CURRENT_A) adds copper loss / torque ripple --
    // bench-validate heating before enabling for flight
    // (EXTRA_DEFINES="--define=ESC6288_RS_ONLINE_DEFAULT=1").
#ifndef ESC6288_RS_ONLINE_DEFAULT
#define ESC6288_RS_ONLINE_DEFAULT  (false)
#endif
    motorVars.flagEnableRsOnLine = (bool)(ESC6288_RS_ONLINE_DEFAULT);
    // Injection magnitude: is10 uses USER_MOTOR_RES_EST_CURRENT_A (4 A here), but that is an
    // offline-identification current -- as a CONTINUOUS online ripple it dwarfs this motor's
    // 1-2 A running current and destabilizes FAST (bench 2026-07-02: Rs misread 2x at handoff,
    // speed estimate collapsed, startup slipped). Use a small fraction of running current.
    motorVars.RsOnLineCurrent_A  = 1.0f;

    EST_setOneOverFluxGain_sf(estHandle, &userParams, USER_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(estHandle, &userParams, USER_EST_FREQ_HF_SF);
    EST_setBemf_sf(estHandle, &userParams, USER_EST_BEMF_HF_SF);

    if(userParams.motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_enablePowerWarp(estHandle, motorVars.flagEnablePowerWarp);
        EST_setFlag_bypassLockRotor(estHandle, motorVars.flagBypassLockRotor);
    }

    //
    // initialize the inverse Park / Park modules
    //
    iparkHandle = IPARK_init(&ipark, sizeof(ipark));
    parkHandle = PARK_init(&park, sizeof(park));

    //
    // initialize the PI controllers and set the gains
    //
    piHandle_Id  = PI_init(&pi_Id, sizeof(pi_Id));
    piHandle_Iq  = PI_init(&pi_Iq, sizeof(pi_Iq));
    piHandle_fwc = PI_init(&pi_fwc, sizeof(pi_fwc));
    piHandle_spd = PI_init(&pi_spd, sizeof(pi_spd));
    setupControllers();

#ifdef ESC6288_FWC_MTPA
    // is13: initialize FWC/MTPA, but leave the runtime gate off by default. Enable only
    // after the speed loop and overmod path are stable on the bench.
    fwcHandle = FWC_init(&fwc, sizeof(fwc));
    FWC_setParams(fwcHandle, USER_FWC_KP, USER_FWC_KI,
                  USER_FWC_MIN_ANGLE_RAD, USER_FWC_MAX_ANGLE_RAD);

    mtpaHandle = MTPA_init(&mtpa, sizeof(mtpa));
    MTPA_computeParameters(mtpaHandle,
                           userParams.motor_Ls_d_H,
                           userParams.motor_Ls_q_H,
                           userParams.motor_ratedFlux_Wb);

    motorVars.flagEnableFWC = (bool)(ESC6288_FWC_MTPA_DEFAULT);
    motorVars.flagEnableMTPA = (bool)(ESC6288_FWC_MTPA_DEFAULT);
    motorVars.flagUpdateMTPAParams = false;
    motorVars.VsRef_pu = USER_VS_REF_MAG_PU;
#endif

    //
    // initialize the space vector generator module
    //
    svgenHandle = SVGEN_init(&svgen, sizeof(svgen));

#ifdef ESC6288_OVERMOD
    // is08: 100% SVM generator setup (min PWM width + shunt-measurability limits)
    svgencurrentHandle = SVGENCURRENT_init(&svgencurrent, sizeof(svgencurrent));
    {
        float32_t minWidth_usec = 1.0f;
        uint16_t minWidth_counts = (uint16_t)(minWidth_usec * USER_SYSTEM_FREQ_MHz);
        float32_t dutyLimit = 0.5f - (2.0f * minWidth_usec * USER_PWM_FREQ_kHz * 0.001f);
        SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
        SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, SVGENCURRENT_USE_ALL);
        SVGENCURRENT_setMode(svgencurrentHandle, SVGENCURRENT_ALL_PHASE_MEASURABLE);
        SVGENCURRENT_setVlimit(svgencurrentHandle, dutyLimit);
    }
#endif

#ifdef ESC6288_VSF
    // is12: variable PWM frequency. Runtime target is g_vsf_pwm_freq_Hz; keep disabled
    // until the fixed-frequency overmod path is bench-proven.
    vsfHandle = VSF_init(&vsf, sizeof(vsf));
    VSF_initParams(vsfHandle, &userParams);
    HAL_setupVSFPWMMode(halHandle);
#endif

    //
    // initialize and configure the speed reference trajectory (Hz)
    //
    trajHandle_spd = TRAJ_init(&traj_spd, sizeof(traj_spd));
    TRAJ_setTargetValue(trajHandle_spd, 0.0);
    TRAJ_setIntValue(trajHandle_spd, 0.0);
    TRAJ_setMinValue(trajHandle_spd, -USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxValue(trajHandle_spd, USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxDelta(trajHandle_spd, (USER_MAX_ACCEL_Hzps / USER_ISR_FREQ_Hz));

    //
    // initialize and configure the Id reference trajectory
    //
    trajHandle_Id = TRAJ_init(&traj_Id, sizeof(traj_Id));
    TRAJ_setTargetValue(trajHandle_Id, 0.0);
    TRAJ_setIntValue(trajHandle_Id, 0.0);
    TRAJ_setMinValue(trajHandle_Id, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_Id,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_Id, (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize and configure the fwc reference trajectory
    //
    trajHandle_fwc = TRAJ_init(&traj_fwc, sizeof(traj_fwc));
    TRAJ_setTargetValue(trajHandle_fwc, 0.0);
    TRAJ_setIntValue(trajHandle_fwc, 0.0);
    TRAJ_setMinValue(trajHandle_fwc, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_fwc,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_fwc, (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize and configure offsets using first-order filter
    //
    {
        uint16_t cnt = 0;
        float32_t b0 = userParams.offsetPole_rps / userParams.ctrlFreq_Hz;
        float32_t a1 = (b0 - 1.0);
        float32_t b1 = 0.0;

        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            filterHandle_I[cnt] = FILTER_FO_init(&filter_I[cnt], sizeof(filter_I[cnt]));
            FILTER_FO_setDenCoeffs(filterHandle_I[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_I[cnt], b0, b1);
            FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                        motorVars.offsets_I_A.value[cnt],
                                        motorVars.offsets_I_A.value[cnt]);
        }

        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            filterHandle_V[cnt] = FILTER_FO_init(&filter_V[cnt], sizeof(filter_V[cnt]));
            FILTER_FO_setDenCoeffs(filterHandle_V[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_V[cnt], b0, b1);
            FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                        motorVars.offsets_V_V.value[cnt],
                                        motorVars.offsets_V_V.value[cnt]);
        }

        motorVars.flagEnableOffsetCalc = true;
        offsetCalcCount = 0;
    }

    motorVars.faultMask.all = FAULT_MASK_OC_OV;

#ifdef DATALOG_ENABLE
    // Initialize Datalog
    datalogHandle = DATALOG_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_resetDlogWithDMA();
    HAL_setupDlogWithDMA(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDlogWithDMA(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);
    #if (DATA_LOG_BUFF_NUM == 4)
    HAL_setupDlogWithDMA(halHandle, 2, &datalogBuff3[0], &datalogBuff3[1]);
    HAL_setupDlogWithDMA(halHandle, 3, &datalogBuff4[0], &datalogBuff4[1]);
    #endif  // DATA_LOG_BUFF_NUM == 4

    datalogObj->iptr[0] = &adcData.I_A.value[0];
    datalogObj->iptr[1] = &adcData.I_A.value[1];
    #if (DATA_LOG_BUFF_NUM == 4)
    datalogObj->iptr[2] = &adcData.V_V.value[0];
    datalogObj->iptr[3] = &adcData.V_V.value[1];
    #endif  // DATA_LOG_BUFF_NUM == 4
#endif  // DATALOG_ENABLE

    //
    // setup faults
    //
    HAL_setupFaults(halHandle);

#ifdef ESC6288_OVERMOD
    HAL_setOvmParams(halHandle, &pwmData);
#endif

    //
    // enable the gate driver. On launchxl this asserts EN_GATE and configures the DRV8305
    // over SPI inside HAL_enableDRV() (the lab's #ifdef DRV8320_SPI path does not apply here).
    //
    HAL_enableDRV(halHandle);

    //
    // initialize the product layer (control SM, DroneCAN core, CAN bridge) before interrupts
    //
    product_init();

    //
    // set up global variables
    //
    motorVars.pwmISRCount = 0;
    motorVars.speedRef_Hz = 0.0;

    IdqSet_A.value[0] = 0.0;
    IdqSet_A.value[1] = 0.0;

    //
    // disable the PWM (kept off until offset cal completes)
    //
    HAL_disablePWM(halHandle);

    //
    // initialize the interrupt vector table, then register/enable the CAN bridge ISR
    //
    HAL_initIntVectorTable(halHandle);
    can_bridge_enable_ints();   // INT_CANA0 (PIE group 9), coexists with the ADC ISR

    HAL_enableADCInts(halHandle);
    HAL_enableGlobalInts(halHandle);
    HAL_enableDebugInt(halHandle);

    //
    // Run immediately: unlike the SDK lab there is no watch-window hand-off. Offset cal
    // still runs first (it gates on flagEnableSys), then the product tick arms via
    // flagRunIdentAndOnLine once cal completes.
    //
    motorVars.flagEnableSys = true;

    //
    // loop while the enable system flag is true
    //
    while(motorVars.flagEnableSys == true)
    {
        //
        // 1ms time base: run the product tick (CAN RX/TX + control step)
        //
        if(HAL_getTimerStatus(halHandle, HAL_CPU_TIMER1))
        {
            motorVars.timerCnt_1ms++;
            HAL_clearTimerFlag(halHandle, HAL_CPU_TIMER1);

            product_tick_1ms();

#ifdef ESC6288_FWC_MTPA
            // is13: live debugger tuning for FWC/MTPA. The master gate is
            // g_fwc_mtpa_enable; motorVars flags mirror it only while SU_RUN below.
            FWC_setKp(fwcHandle, motorVars.Kp_fwc);
            FWC_setKi(fwcHandle, motorVars.Ki_fwc);
            FWC_setAngleMax(fwcHandle, motorVars.angleMax_fwc_rad);

            if(motorVars.flagUpdateMTPAParams == true)
            {
                motorVars.LsOnline_d_H =
                    MTPA_updateLs_d_withLUT(mtpaHandle, motorVars.Is_A);
                motorVars.LsOnline_q_H =
                    MTPA_updateLs_q_withLUT(mtpaHandle, motorVars.Is_A);
                motorVars.fluxOnline_Wb = motorVars.flux_Wb;
                MTPA_computeParameters(mtpaHandle,
                                       motorVars.LsOnline_d_H,
                                       motorVars.LsOnline_q_H,
                                       motorVars.fluxOnline_Wb);
            }
            motorVars.mtpaKconst = MTPA_getKconst(mtpaHandle);
#endif
        }

        motorVars.mainLoopCount++;

#ifdef ESC6288_VSF
        {
            uint16_t targetFreq_Hz = g_vsf_enable ? g_vsf_pwm_freq_Hz
                                                   : PRODUCT_PWM_FREQ_DEFAULT_HZ;
            VSF_setFreq(vsfHandle, targetFreq_Hz);
            VSF_computeFreqParams(vsfHandle);
            if((motorVars.flagMotorIdentified == true) &&
               (VSF_getState(vsfHandle) != VSF_STATE_IDLE))
            {
                computeCurrentControllers();
            }
        }
#endif

        //
        // set the reference value for internal DACA and DACB
        //
        HAL_setDACValue(halHandle, 0, motorVars.dacaVal);
        HAL_setDACValue(halHandle, 1, motorVars.dacbVal);

        //
        // set internal DAC value for on-chip comparator for current protection
        //
        {
            uint16_t  cmpssCnt;

            for(cmpssCnt = 0; cmpssCnt < HAL_NUM_CMPSS_CURRENT; cmpssCnt++)
            {
                HAL_setCMPSSDACValueHigh(halHandle, cmpssCnt, motorVars.dacValH);
                HAL_setCMPSSDACValueLow(halHandle, cmpssCnt, motorVars.dacValL);
            }
        }

        //
        // enable or disable force angle
        //
        EST_setFlag_enableForceAngle(estHandle, motorVars.flagEnableForceAngle);

        if(userParams.motor_type == MOTOR_TYPE_INDUCTION)
        {
            EST_setFlag_bypassLockRotor(estHandle, motorVars.flagBypassLockRotor);
        }

        if(HAL_getPwmEnableStatus(halHandle) == true)
        {
            if(HAL_getTripFaults(halHandle) != 0)
            {
                motorVars.faultNow.bit.moduleOverCurrent = 1;
            }
        }

        motorVars.faultUse.all = motorVars.faultNow.all & motorVars.faultMask.all;

        //
        // a hardware fault stops the motor
        //
        if(motorVars.faultUse.all != 0)
        {
            motorVars.flagRunIdentAndOnLine = 0;
        }

        if((motorVars.flagRunIdentAndOnLine == true) &&
           (motorVars.flagEnableOffsetCalc == false))
        {
            EST_enable(estHandle);
            EST_enableTraj(estHandle);

            if(HAL_getPwmEnableStatus(halHandle) == false)
            {
                HAL_enablePWM(halHandle);
            }

            TRAJ_setTargetValue(trajHandle_spd, motorVars.speedRef_Hz);
            TRAJ_setMaxDelta(trajHandle_spd,
                           (motorVars.accelerationMax_Hzps / USER_ISR_FREQ_Hz));

#ifdef ESC6288_FWC_MTPA
            {
                bool fwc_mtpa_live = g_fwc_mtpa_enable && (g_su.state == SU_RUN);
                motorVars.flagEnableFWC = fwc_mtpa_live;
                motorVars.flagEnableMTPA = fwc_mtpa_live;
                FWC_setFlagEnable(fwcHandle, motorVars.flagEnableFWC);
                MTPA_setFlagEnable(mtpaHandle, motorVars.flagEnableMTPA);
            }
#endif
        }
        else if(motorVars.flagEnableOffsetCalc == false)
        {
            EST_disable(estHandle);
            EST_disableTraj(estHandle);
            HAL_disablePWM(halHandle);

            PI_setUi(piHandle_Id, 0.0);
            PI_setUi(piHandle_Iq, 0.0);
            PI_setUi(piHandle_fwc, 0.0);
            PI_setUi(piHandle_spd, 0.0);

            Idq_ref_A.value[0] = 0.0;
            Idq_ref_A.value[1] = 0.0;
            Idq_offset_A.value[0] = 0.0;
            Idq_offset_A.value[1] = 0.0;

            motorVars.IdRated_A = EST_getIdRated_A(estHandle);

            motorVars.VsRef_pu = USER_MAX_VS_MAG_PU;
            motorVars.IsRef_A = 0.0;
            motorVars.angleCurrent_rad = 0.0;
#ifdef ESC6288_FWC_MTPA
            motorVars.flagEnableFWC = false;
            motorVars.flagEnableMTPA = false;
            FWC_setFlagEnable(fwcHandle, false);
            MTPA_setFlagEnable(mtpaHandle, false);
            counterFWCandMTPA = 0u;
            motorVars.VsRef_pu = USER_VS_REF_MAG_PU;
#endif

            TRAJ_setTargetValue(trajHandle_spd, 0.0);
            TRAJ_setIntValue(trajHandle_spd, 0.0);
        }

        if(EST_isTrajError(estHandle) == true)
        {
            HAL_disablePWM(halHandle);
            motorVars.flagEnableSys = false;
        }
        else
        {
            EST_updateTrajState(estHandle);
        }

        if(EST_isError(estHandle) == true)
        {
            HAL_disablePWM(halHandle);
            motorVars.flagEnableSys = false;
        }
        else
        {
            motorVars.Id_target_A = EST_getIntValue_Id_A(estHandle);

            flagEstStateChanged = EST_updateState(estHandle, 0.0);

            if(flagEstStateChanged == true)
            {
                EST_configureTraj(estHandle);
            }
        }

        if(EST_isMotorIdentified(estHandle) == true)
        {
            if(motorVars.flagSetupController == true)
            {
                updateControllers();
            }
            else
            {
                motorVars.flagMotorIdentified = true;
                motorVars.flagSetupController = true;
                setupControllers();
            }
        }

        product_run_rs_online(estHandle);

        updateGlobalVariables(estHandle);

    } // end of while() loop

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

} // end of main() function

//! \brief Advance the open-loop I/f startup one ISR tick (updates g_su.state). The ISR then applies the
//! per-state FOC override: ALIGN/RAMP -> open-loop angle + open-loop-frame Id; BLEND -> FAST angle +
//! blended current; RUN -> normal FAST path; FAULT -> safe-off. est_angle_rad / est_speed_Hz are the
//! live FAST electrical angle/speed (Hz) used for the handoff and slip guards.
static inline void startup_step(bool armed, float32_t est_angle_rad, float32_t est_speed_Hz)
{
    const float32_t dt = 1.0f / USER_ISR_FREQ_Hz;
    float32_t err;

    if(!armed)
    {   // disarmed -> reset all; legacy/idle
        g_su.state = SU_IDLE;  g_su.freq_Hz = 0.0f;  g_su.angle_rad = 0.0f;  g_su.blend = 0.0f;
        g_su.tick = 0u;  g_su.cohTick = 0u;  g_su.slipTick = 0u;
        return;
    }
    if(g_su.enable == 0u) { g_su.state = SU_RUN; return; }   // disabled -> exact legacy behavior

    switch(g_su.state)
    {
        case SU_IDLE:                              // just armed -> begin alignment
            g_su.state = SU_ALIGN;  g_su.tick = 0u;  g_su.freq_Hz = 0.0f;  g_su.angle_rad = 0.0f;
            g_su.blend = 0.0f;  g_su.cohTick = 0u;  g_su.slipTick = 0u;  g_su.fault = 0u;
            return;

        case SU_ALIGN:                             // hold rotor at a known electrical angle
            g_su.angle_rad = 0.0f;
            if(++g_su.tick >= (uint32_t)(g_su.align_s * USER_ISR_FREQ_Hz))
            {
                g_su.state = SU_RAMP;  g_su.tick = 0u;
            }
            return;

        case SU_RAMP:                              // open-loop electrical angle accelerates
            g_su.freq_Hz += g_su.accel_Hzps * dt;
            g_su.angle_rad = MATH_incrAngle(g_su.angle_rad, g_su.freq_Hz * MATH_TWO_PI * dt);
            err = est_angle_rad - g_su.angle_rad;             // wrapped |FAST angle - open-loop angle|
            while(err >  MATH_PI) { err -= MATH_TWO_PI; }
            while(err < -MATH_PI) { err += MATH_TWO_PI; }
            if(err < 0.0f) { err = -err; }

            // slip guard: past slip_check_Hz, the rotor (FAST speed) must track the open-loop freq and the
            // angle error must stay bounded, else the rotor is slipping -> abort to safe-off (don't wait
            // for the timeout; a slipping prop pulls high load-angle current and would OC first).
            if(g_su.freq_Hz >= g_su.slip_check_Hz)
            {
                if((est_speed_Hz < (g_su.slip_frac * g_su.freq_Hz)) || (err > g_su.max_err_rad))
                {
                    if(++g_su.slipTick >= (uint32_t)(g_su.slip_win_s * USER_ISR_FREQ_Hz))
                    {
                        g_su.fault = 1u;  g_su.state = SU_FAULT;  return;
                    }
                }
                else { g_su.slipTick = 0u; }
            }

            // handoff: coherent (freq high, small angle error, same rotation sign) held for dwell_s
            if((g_su.freq_Hz >= g_su.handoff_Hz) && (err < g_su.handoff_err_rad) && (est_speed_Hz > 0.0f))
            {
                if(++g_su.cohTick >= (uint32_t)(g_su.dwell_s * USER_ISR_FREQ_Hz))
                {
                    g_su.state = SU_BLEND;  g_su.blend = 0.0f;  g_su.tick = 0u;
                }
            }
            else { g_su.cohTick = 0u; }

            if(++g_su.tick >= (uint32_t)(g_su.timeout_s * USER_ISR_FREQ_Hz))
            {
                g_su.fault = 1u;  g_su.state = SU_FAULT;       // stalled: safe-off
            }
            return;

        case SU_BLEND:                             // FAST angle owns; blend Id->0 / Iq->throttle
            g_su.blend += dt / g_su.blend_s;
            if(g_su.blend >= 1.0f) { g_su.blend = 1.0f;  g_su.state = SU_RUN; }
            return;

        case SU_FAULT:                             // ISR forces OST safe-off + latches the fault
        case SU_RUN:
        default:
            return;
    }
}

#ifdef ESC6288_VSF
__interrupt void estISR(void)
{
    motorVars.estISRCount++;
    HAL_ackEstInt(halHandle);

    if(motorVars.flagEnableOffsetCalc == false)
    {
        counterTrajSpeed++;
        if(counterTrajSpeed >= userParams.numIsrTicksPerTrajTick)
        {
            counterTrajSpeed = 0;
            TRAJ_run(trajHandle_spd);
            motorVars.speedTraj_Hz = TRAJ_getIntValue(trajHandle_spd);
        }

        estInputData.dcBus_V = adcData.dcBus_V;
        estInputData.speed_ref_Hz = motorVars.speedTraj_Hz;
        estInputData.speed_int_Hz = motorVars.speedTraj_Hz;

        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            estInputData.speed_ref_Hz = g_su.freq_Hz;
            estInputData.speed_int_Hz = g_su.freq_Hz;
        }

        EST_run(estHandle, &estInputData, &estOutputData);
        EST_getIdq_A(estHandle, (MATH_Vec2 *)(&(Idq_in_A)));

        Idq_ref_A.value[0] = IdqSet_A.value[0];
        if(g_speed_pi_run && (g_su.state == SU_RUN))
        {
            counterSpeed++;
            if(counterSpeed >= userParams.numCtrlTicksPerSpeedTick)
            {
                counterSpeed = 0;
                PI_run_series(piHandle_spd,
                              estInputData.speed_ref_Hz,
                              estOutputData.fm_lp_rps * MATH_ONE_OVER_TWO_PI,
                              0.0,
                              (float32_t *)(&(Idq_ref_A.value[1])));
            }
        }
        else
        {
            Idq_ref_A.value[1] = IdqSet_A.value[1];
        }

        startup_step((motorVars.flagRunIdentAndOnLine != 0), estOutputData.angle_rad,
                     estOutputData.fm_lp_rps / MATH_TWO_PI);
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            Idq_ref_A.value[0] = (g_su.state == SU_ALIGN) ? g_su.id_align_A : g_su.id_ramp_A;
            Idq_ref_A.value[1] = 0.0f;
        }
        else if(g_su.state == SU_BLEND)
        {
            Idq_ref_A.value[0] = (1.0f - g_su.blend) * g_su.id_ramp_A;
            Idq_ref_A.value[1] = g_su.blend * IdqSet_A.value[1];
        }
        else if(g_su.state == SU_FAULT)
        {
            Idq_ref_A.value[0] = 0.0f;
            Idq_ref_A.value[1] = 0.0f;
            HAL_disablePWM(halHandle);
            motorVars.faultNow.bit.moduleOverCurrent = 1;
        }

#ifdef ESC6288_FWC_MTPA
        product_apply_fwc_mtpa_current_angle();
#endif

        EST_updateId_ref_A(estHandle, (float32_t *)&(Idq_ref_A.value[0]));
        EST_setIdq_ref_A(estHandle, &Idq_ref_A);

        angleDelta_rad = userParams.angleDelayed_sf_sec * estOutputData.fm_lp_rps;
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            angleFoc_rad = g_su.angle_rad;
        }
        else
        {
            angleFoc_rad = MATH_incrAngle(estOutputData.angle_rad, angleDelta_rad);
        }
    }

    return;
}
#endif

__interrupt void mainISR(void)
{
    motorVars.pwmISRCount++;

    //
    // toggle status LED
    //
    counterLED++;

    if(counterLED > (uint32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLED(halHandle, HAL_GPIO_LED2);
        counterLED = 0;
    }

    //
    // acknowledge the ADC interrupt
    //
    HAL_ackADCInt(halHandle, ADC_INT_NUMBER1);

    //
    // read the ADC data with offsets
    //
    HAL_readADCDataWithOffsets(halHandle, &adcData);

    //
    // remove offsets
    //
    adcData.I_A.value[0] -= motorVars.offsets_I_A.value[0];
    adcData.I_A.value[1] -= motorVars.offsets_I_A.value[1];
    adcData.I_A.value[2] -= motorVars.offsets_I_A.value[2];
    adcData.V_V.value[0] -= motorVars.offsets_V_V.value[0] * adcData.dcBus_V;
    adcData.V_V.value[1] -= motorVars.offsets_V_V.value[1] * adcData.dcBus_V;
    adcData.V_V.value[2] -= motorVars.offsets_V_V.value[2] * adcData.dcBus_V;

#ifdef ESC6288_OVERMOD
    // is08: reconstruct phases whose shunt window was unmeasurable this cycle from the
    // previous cycle. Runs BEFORE the fast-OC check so a collapsed shunt reading cannot
    // false-trip at high duty.
    SVGENCURRENT_RunRegenCurrent(svgencurrentHandle, &adcData.I_A, &adcDataPrev);
#endif

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // Fast per-phase software overcurrent (20 kHz). Only meaningful once armed (PWM live);
    // during offset cal the gates are OST-forced so getPwmEnableStatus() is false. On trip:
    // force the OST safe-off immediately AND latch the product fault (same bit the CMPSS/TZ
    // trip detection sets), so the main-loop arm path drops flagRunIdentAndOnLine and the
    // motor stays stopped (no re-enable fight).
    if((HAL_getPwmEnableStatus(halHandle) == true) &&
       ((fabsf(adcData.I_A.value[0]) > ESC6288_ISR_OC_A) ||
        (fabsf(adcData.I_A.value[1]) > ESC6288_ISR_OC_A) ||
        (fabsf(adcData.I_A.value[2]) > ESC6288_ISR_OC_A)))
    {
        HAL_disablePWM(halHandle);
        motorVars.faultNow.bit.moduleOverCurrent = 1;
    }
#endif

    if(motorVars.flagEnableOffsetCalc == false)
    {
        float32_t outMax_V;
        MATH_Vec2 phasor;

        CLARKE_run(clarkeHandle_I, &adcData.I_A, &(estInputData.Iab_A));
        CLARKE_run(clarkeHandle_V, &adcData.V_V, &(estInputData.Vab_V));

#ifndef ESC6288_VSF
        counterTrajSpeed++;

        if(counterTrajSpeed >= userParams.numIsrTicksPerTrajTick)
        {
            counterTrajSpeed = 0;
            TRAJ_run(trajHandle_spd);
            motorVars.speedTraj_Hz = TRAJ_getIntValue(trajHandle_spd);
        }

        estInputData.dcBus_V = adcData.dcBus_V;
        estInputData.speed_ref_Hz = motorVars.speedTraj_Hz;
        estInputData.speed_int_Hz = motorVars.speedTraj_Hz;

        // During the open-loop startup, feed FAST the open-loop electrical frequency (previous tick)
        // instead of the 0 Hz torque-mode trajectory, so its flux/angle observer converges to the
        // rotor's actual motion and the handoff angle test is trustworthy.
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            estInputData.speed_ref_Hz = g_su.freq_Hz;
            estInputData.speed_int_Hz = g_su.freq_Hz;
        }

        EST_run(estHandle, &estInputData, &estOutputData);

        EST_getIdq_A(estHandle, (MATH_Vec2 *)(&(Idq_in_A)));

        // set reference current. Torque mode copies Iq from the product layer every tick; in
        // closed speed mode (is07 port) the speed PI below OWNS Idq_ref_A.value[1] -- it runs
        // every numCtrlTicksPerSpeedTick and its output must persist between speed ticks, so it
        // must not be clobbered from IdqSet_A. The speed loop engages only after the I/f
        // self-start hands off (SU_RUN); during ALIGN/RAMP/BLEND the startup overrides below own
        // the reference anyway.
        Idq_ref_A.value[0] = IdqSet_A.value[0];
        if(g_speed_pi_run && (g_su.state == SU_RUN))
        {
            counterSpeed++;
            if(counterSpeed >= userParams.numCtrlTicksPerSpeedTick)
            {
                counterSpeed = 0;
                // is07 speed loop: series PI on (speed_ref - FAST speed estimate) -> Iq_ref.
                // Output limits are +/- userParams.maxCurrent_A (setupControllers); Ui is reset
                // by the main-loop stop path (PI_setUi) whenever the motor is stopped.
                PI_run_series(piHandle_spd,
                              estInputData.speed_ref_Hz,
                              estOutputData.fm_lp_rps * MATH_ONE_OVER_TWO_PI,
                              0.0,
                              (float32_t *)(&(Idq_ref_A.value[1])));
            }
        }
        else
        {
            Idq_ref_A.value[1] = IdqSet_A.value[1];
        }

        // open-loop I/f self-start (esc6288): ALIGN/RAMP drive Id at an open-loop angle until FAST
        // coheres, then BLEND to the normal FAST-angle + throttle path. See g_su / startup_step().
        startup_step((motorVars.flagRunIdentAndOnLine != 0), estOutputData.angle_rad,
                     estOutputData.fm_lp_rps / MATH_TWO_PI);
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            Idq_ref_A.value[0] = (g_su.state == SU_ALIGN) ? g_su.id_align_A : g_su.id_ramp_A;
            Idq_ref_A.value[1] = 0.0f;

            // Measure Idq in the OPEN-LOOP frame. EST_getIdq_A (above) uses the FAST angle, which is
            // force-held near 0 while our open-loop angle ramps -> feedback/drive frame mismatch ->
            // the current loop diverges -> OC. Re-Park the measured Iab onto our open-loop angle so the
            // current-PI feedback matches the applied vector.
            phasor.value[0] = cosf(g_su.angle_rad);
            phasor.value[1] = sinf(g_su.angle_rad);
            PARK_setPhasor(parkHandle, &phasor);
            PARK_run(parkHandle, &(estInputData.Iab_A), (MATH_Vec2 *)(&(Idq_in_A)));
        }
        else if(g_su.state == SU_BLEND)
        {   // FAST angle owns now (Idq_in_A already FAST-frame from EST_getIdq_A); blend the reference
            // Id_ramp -> 0 and Iq 0 -> throttle over blend_s so the handoff is not a one-tick vector step.
            Idq_ref_A.value[0] = (1.0f - g_su.blend) * g_su.id_ramp_A;
            Idq_ref_A.value[1] = g_su.blend * IdqSet_A.value[1];
        }
        else if(g_su.state == SU_FAULT)
        {   // startup stalled/slipped -> REAL safe-off: force OST now and latch the product fault so the
            // main loop drops flagRunIdentAndOnLine (not just active zero-current control).
            Idq_ref_A.value[0] = 0.0f;
            Idq_ref_A.value[1] = 0.0f;
            HAL_disablePWM(halHandle);
            motorVars.faultNow.bit.moduleOverCurrent = 1;
        }

#ifdef ESC6288_FWC_MTPA
        product_apply_fwc_mtpa_current_angle();
#endif

        EST_updateId_ref_A(estHandle, (float32_t *)&(Idq_ref_A.value[0]));
#else
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            // VSF mode runs FAST in estISR; the ADC/current ISR still has to measure
            // current in the open-loop frame before the current PI executes.
            phasor.value[0] = cosf(g_su.angle_rad);
            phasor.value[1] = sinf(g_su.angle_rad);
            PARK_setPhasor(parkHandle, &phasor);
            PARK_run(parkHandle, &(estInputData.Iab_A), (MATH_Vec2 *)(&(Idq_in_A)));
            angleFoc_rad = g_su.angle_rad;
        }
        else if(g_su.state == SU_FAULT)
        {
            Idq_ref_A.value[0] = 0.0f;
            Idq_ref_A.value[1] = 0.0f;
            HAL_disablePWM(halHandle);
            motorVars.faultNow.bit.moduleOverCurrent = 1;
        }
#endif

        userParams.maxVsMag_V = userParams.maxVsMag_pu * adcData.dcBus_V;
        PI_setMinMax(piHandle_Id, (-userParams.maxVsMag_V), userParams.maxVsMag_V);

        PI_run_series(piHandle_Id,
                      Idq_ref_A.value[0] + Idq_offset_A.value[0],
                      Idq_in_A.value[0],
                      0.0,
                      &(Vdq_out_V.value[0]));

        outMax_V = sqrt((userParams.maxVsMag_V * userParams.maxVsMag_V) -
                        (Vdq_out_V.value[0] * Vdq_out_V.value[0]));

        PI_setMinMax(piHandle_Iq, -outMax_V, outMax_V);
        PI_run_series(piHandle_Iq,
                      Idq_ref_A.value[1] + Idq_offset_A.value[1],
                      Idq_in_A.value[1],
                      0.0,
                      &(Vdq_out_V.value[1]));

#ifndef ESC6288_VSF
        EST_setIdq_ref_A(estHandle, &Idq_ref_A);

        angleDelta_rad = userParams.angleDelayed_sf_sec * estOutputData.fm_lp_rps;
        if((g_su.state == SU_ALIGN) || (g_su.state == SU_RAMP))
        {
            angleFoc_rad = g_su.angle_rad;          // open-loop I/f angle during self-start
        }
        else
        {
            angleFoc_rad = MATH_incrAngle(estOutputData.angle_rad, angleDelta_rad);
        }
#endif

        phasor.value[0] = cosf(angleFoc_rad);
        phasor.value[1] = sinf(angleFoc_rad);

        IPARK_setPhasor(iparkHandle, &phasor);
        IPARK_run(iparkHandle, &Vdq_out_V, &Vab_out_V);

        SVGEN_setup(svgenHandle, estOutputData.oneOverDcBus_invV);
        SVGEN_run(svgenHandle, &Vab_out_V, &(pwmData.Vabc_pu));
    }
    else if(motorVars.flagEnableOffsetCalc == true)
    {
        runOffsetsCalculation();
    }

    if(HAL_getPwmEnableStatus(halHandle) == false)
    {
        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;
    }

#ifdef ESC6288_OVERMOD
    // is08: min-width compensation of the PWM commands (only while the bridge is live), then
    // steer the next-cycle ADC trigger into the measurable shunt window.
    if(HAL_getPwmEnableStatus(halHandle) == true)
    {
        SVGENCURRENT_compPWMData(svgencurrentHandle, &(pwmData.Vabc_pu), &pwmDataPrev);
    }
#endif

    //
    // write the PWM compare values
    //
#ifdef ESC6288_VSF
    if(g_vsf_enable ||
       (VSF_getFreq(vsfHandle) != PRODUCT_PWM_FREQ_DEFAULT_HZ) ||
       (VSF_getState(vsfHandle) != VSF_STATE_IDLE))
    {
        VSF_setPeriod(vsfHandle);
        VSF_getPeriod(vsfHandle, &(pwmData.period));
        HAL_writePWMAllData(halHandle, &pwmData);
    }
    else
    {
        HAL_writePWMData(halHandle, &pwmData);
    }
#else
    HAL_writePWMData(halHandle, &pwmData);
#endif

#ifdef ESC6288_OVERMOD
    ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
    midVolShunt = SVGENCURRENT_getVmid(svgencurrentHandle);
    HAL_setTrigger(halHandle, &pwmData, ignoreShuntNextCycle, midVolShunt);
#endif

#ifdef DATALOG_ENABLE
    DATALOG_updateWithDMA(datalogHandle);
    HAL_trigDlogWithDMA(halHandle, 0);
    HAL_trigDlogWithDMA(halHandle, 1);
    HAL_trigDlogWithDMA(halHandle, 2);
    HAL_trigDlogWithDMA(halHandle, 3);
#endif  //  DATALOG_ENABLE

    return;
} // end of mainISR() function

void runOffsetsCalculation(void)
{
    float32_t invVdcbus;
    uint16_t cnt;

    if(motorVars.flagEnableSys == true)
    {
        // esc6288_revA has no gate-enable pin: keep the outputs OST-forced through offset
        // calibration. The EPWM timebase/SOCA keep running under the trip, so the ADC still
        // samples the true zero-current value; bringing the gates live here (as other boards
        // do, idling the PWM at 0%) would defeat the safe-off. The board-agnostic arm path
        // (flagRunIdentAndOnLine && cal done) is the only place that un-trips for esc6288.
#if (BUILD_BOARD_ID != BUILD_BOARD_ID_ESC6288_REVA)
        HAL_enablePWM(halHandle);
#endif

        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;

        invVdcbus = 1.0f / adcData.dcBus_V;

        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            motorVars.offsets_I_A.value[cnt] = 0.0;
            FILTER_FO_run(filterHandle_I[cnt], adcData.I_A.value[cnt]);
        }

        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            motorVars.offsets_V_V.value[cnt] = 0.0;
            FILTER_FO_run(filterHandle_V[cnt], adcData.V_V.value[cnt] * invVdcbus);
        }

        offsetCalcCount++;

        if(offsetCalcCount >= offsetCalcWaitTime)
        {
            for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
            {
                motorVars.offsets_I_A.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_I[cnt]);
                FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                              motorVars.offsets_I_A.value[cnt],
                                              motorVars.offsets_I_A.value[cnt]);
            }

            for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
            {
                motorVars.offsets_V_V.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_V[cnt]);
                FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                              motorVars.offsets_V_V.value[cnt],
                                              motorVars.offsets_V_V.value[cnt]);
            }

            offsetCalcCount = 0;
            motorVars.flagEnableOffsetCalc = false;

            HAL_disablePWM(halHandle);
        }
    }

    return;
} // end of runOffsetsCalculation() function

//
// end of file
//
