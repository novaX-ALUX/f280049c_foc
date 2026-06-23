//#############################################################################
// product_main.c - esc6288 product main: DroneCAN RawCommand -> esc_control -> FAST.
//
// This REPLACES the SDK lab .c (it owns main() + mainISR()). The FOC pipeline is the
// SDK torque lab (is06_torque_control) verbatim; the only product injections are:
//   * self-set motorVars.flagEnableSys = true (no watch-window hand-off),
//   * an explicit HAL_enableDRV() (launchxl DRV8305 SPI enable lives inside it),
//   * register/enable the CANA0 bridge ISR alongside the ADC ISR,
//   * a 1 ms background tick that runs: CAN RX drain -> dronecan_on_rx -> esc_control_step
//     -> foc_bridge -> motorVars/IdqSet_A, and dronecan_tick -> can_bridge_write.
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
#include "foc_bridge.h"
#include "dronecan.h"
#include "dronecan_frame.h"
#include "can_bridge.h"

#include <math.h>

#pragma CODE_SECTION(mainISR, ".TI.ramfunc");

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

static esc_command_t       g_cmd;        //!< last command from comms (valid if g_have_cmd)
static bool                g_have_cmd;
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

//! \brief Park-reference NV loader stub. launchxl has no Flash-persisted park ref yet
//!        (and with enc_valid=false the learn path never runs), so report "unlearned".
static float product_park_ref_load(void *ctx, bool *out_valid)
{
    (void)ctx;
    *out_valid = false;
    return 0.0f;
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
#include "rgb_led.h"
#endif

// esc6288_revA fast (per-ISR, 20 kHz) software overcurrent backstop, amps. Phases A/B have
// no CMPSS comparator, so this is their fast trip (phase C also has CMPSS3). Hard backstop
// above the 1 ms product OC limit (oc_set_A = 30 A) and below the +/-165 A sense FS. Tune
// on the bench against the FET/shunt rating.
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
    // esc6288_revA / 12S target (conservative bench bring-up defaults; raise after
    // validation). Hardware backstops: CMPSS3 phase-C OC + CMPSS5 bus OV (~56 V). These
    // software limits are the PRIMARY protection for phases A/B (which have no CMPSS).
    c->oc_set_A   = 30.0f;  c->oc_clr_A   = 25.0f;   // current FS is +/-165 A; start low
    c->vbus_ov_set = 54.0f; c->vbus_ov_clr = 50.0f;  // 12S max charge 50.4 V; HW OV ~56 V
    c->vbus_uv_set = 18.0f; c->vbus_uv_clr = 22.0f;  // low for bench; raise for flight
    c->temp_ot_set = 100.0f; c->temp_ot_clr = 85.0f; // NTC->degC mapping is a TODO (raw 25 C now)
#else
    c->oc_set_A   = 8.0f;  c->oc_clr_A   = 6.0f;
    c->vbus_ov_set = 30.0f; c->vbus_ov_clr = 28.0f;
    c->vbus_uv_set = 9.0f;  c->vbus_uv_clr = 11.0f;
    c->temp_ot_set = 100.0f; c->temp_ot_clr = 85.0f;
#endif
    // c->park / c->park_ref left zeroed (dormant on launchxl)
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

    product_build_esc_cfg(&cfg);
    esc_control_init(&g_esc, &cfg, product_park_ref_load, NULL);

    for(i = 0; i < 16u; i++) { dncfg.unique_id[i] = 0u; }
    product_read_unique_id(dncfg.unique_id);
    dncfg.esc_index             = (uint16_t)BUILD_ESC_INDEX;
    dncfg.arm_zero_frames       = 0u;   // -> default 10
    dncfg.node_id               = (uint16_t)BUILD_NODE_ID;  // 0 = DNA; 1..127 = static (bench)
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
    dronecan_init(&g_dn, &dncfg);

    can_bridge_init();

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    // Bring up the esc6288 board-side aux interfaces (after HAL setup; before the main
    // loop). RC_PWM_init re-enables the eCAP1 clock the shared HAL leaves off. These read
    // APIs (RC_PWM_getThrottle / MT6701_SSI_read / RGB_setColor) are wired into the control
    // path during bench bring-up.
    RC_PWM_init();
    MT6701_SSI_init();
    RGB_init();
#endif
}

//! \brief Land the FOC setpoint on the SDK control variables (torque path only on launchxl).
//!        Caller is the 1 ms background tick; the ISR only READS these single-word fields.
static void apply_setpoint(const foc_setpoint_t *sp)
{
    if((sp->enable == false) || (sp->brake == true))
    {
        // Coast: drop the run flag and zero Iq. launchxl has no active short-brake yet
        // (brake degrades to disable); a real brake is deferred to esc6288.
        IdqSet_A.value[1] = 0.0f;
        motorVars.flagRunIdentAndOnLine = 0;
        return;
    }

    if(sp->speed_mode == true)
    {
        // Should not occur on launchxl (auto_park disabled). Fail safe to disable.
        // TODO(esc6288): drive motorVars.speedRef_Hz + the is07 speed-PI ISR branch here.
        IdqSet_A.value[1] = 0.0f;
        motorVars.flagRunIdentAndOnLine = 0;
        return;
    }

    // Torque: the ISR copies IdqSet_A -> Idq_ref_A directly (no speed loop).
    IdqSet_A.value[0] = 0.0f;
    IdqSet_A.value[1] = sp->iq_ref_A;
    if(motorVars.flagEnableOffsetCalc == false)
    {
        motorVars.flagRunIdentAndOnLine = 1;   // arm only once ADC offset cal is done
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

    // 1) drain received CAN frames into the protocol core; keep the latest command for us
    while(can_bridge_read(&rxf))
    {
        dronecan_on_rx(&g_dn, &rxf, &rr);
        if(rr.command_updated)
        {
            g_cmd = rr.command;
            g_have_cmd = true;
        }
    }

    // 2) assemble feedback (gate fault = nFAULT active-low; i_motor = peak |phase current|;
    //    temperature is a placeholder until a sensor is wired on the board)
    imot = fmaxf(fmaxf(fabsf(adcData.I_A.value[0]), fabsf(adcData.I_A.value[1])),
                 fabsf(adcData.I_A.value[2]));
    raw.vbus_V         = motorVars.VdcBus_V;
    raw.iq_meas_A      = Idq_in_A.value[1];
    raw.i_motor_A      = imot;
    raw.speed_est_krpm = motorVars.speed_krpm;
    raw.temp_C         = 25.0f;   // TODO: launchxl has no temperature sensor wired
    raw.gate_fault     = (BOARD_HAS_GATE_FAULT_INPUT != 0) &&
                         (GPIO_readPin(BOARD_GATE_FAULT_GPIO) == 0U);
    foc_bridge_map_feedback(&raw, &fb);

    // 3) run the control state machine (NULL command this tick if nothing fresh arrived)
    (void)esc_control_step(&g_esc, g_have_cmd ? &g_cmd : NULL, &fb, 0.001f, &out, &tel);
    g_have_cmd = false;

    // 4) map the control output to a FAST setpoint and apply it
    foc_bridge_map_output(&g_fbcfg, &out, &sp);
    apply_setpoint(&sp);

    // 5) park-ref store request (launchxl never fires it; placeholder, Flash deferred)
    if(g_esc.ref.needs_store)
    {
        park_ref_clear_store_request(&g_esc.ref);
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
        dronecan_clear_node_id_dirty(&g_dn);   // TODO: persist node-id to Flash (deferred)
    }
}

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

    //
    // initialize the space vector generator module
    //
    svgenHandle = SVGEN_init(&svgen, sizeof(svgen));

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
        }

        motorVars.mainLoopCount++;

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

        updateGlobalVariables(estHandle);

    } // end of while() loop

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

} // end of main() function

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

        EST_run(estHandle, &estInputData, &estOutputData);

        EST_getIdq_A(estHandle, (MATH_Vec2 *)(&(Idq_in_A)));

        // set reference current (torque control: direct Iq from the product layer)
        Idq_ref_A.value[0] = IdqSet_A.value[0];
        Idq_ref_A.value[1] = IdqSet_A.value[1];

        EST_updateId_ref_A(estHandle, (float32_t *)&(Idq_ref_A.value[0]));

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

        EST_setIdq_ref_A(estHandle, &Idq_ref_A);

        angleDelta_rad = userParams.angleDelayed_sf_sec * estOutputData.fm_lp_rps;
        angleFoc_rad = MATH_incrAngle(estOutputData.angle_rad, angleDelta_rad);

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

    //
    // write the PWM compare values
    //
    HAL_writePWMData(halHandle, &pwmData);

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
