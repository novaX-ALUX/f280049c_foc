#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "rc_pwm.h"

/*
 * RC servo-PWM throttle capture: GPIO33 -> Input XBAR1 -> eCAP1, continuous mode.
 * Event 1 = rising edge (resets the counter), Event 2 = falling edge. CAP2 therefore
 * holds the high-time in SYSCLK counts. 100 MHz SYSCLK -> 100 counts/us.
 */
#define RCPWM_COUNTS_PER_US   ((float)(DEVICE_SYSCLK_FREQ / 1000000UL))  /* 100 */
#define RCPWM_MIN_US          (1000.0f)   /* zero throttle */
#define RCPWM_MAX_US          (2000.0f)   /* full throttle */
#define RCPWM_VALID_MIN_US    (900.0f)    /* reject glitch / no-signal */
#define RCPWM_VALID_MAX_US    (2100.0f)

/* driverlib has no ECAP_ISR_SOURCE_ALL; OR every source for a full clear/disable. */
#define RCPWM_ECAP_ALL_FLAGS  (ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |  \
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |  \
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |  \
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |  \
                               ECAP_ISR_SOURCE_COUNTER_OVERFLOW | \
                               ECAP_ISR_SOURCE_COUNTER_PERIOD |   \
                               ECAP_ISR_SOURCE_COUNTER_COMPARE)

void RC_PWM_init(void)
{
    /* The shared HAL (HAL_setupPeripheralClks) disables all eCAP clocks; turn eCAP1 on. */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);

    /* GPIO33 (already a digital input from HAL_setupGPIOs) -> Input XBAR -> eCAP1. */
    XBAR_setInputPin(BOARD_RCPWM_XBAR_INPUT, BOARD_RCPWM_GPIO);
    ECAP_selectECAPInput(BOARD_RCPWM_ECAP_BASE, BOARD_RCPWM_ECAP_INPUT);

    ECAP_disableInterrupt(BOARD_RCPWM_ECAP_BASE, RCPWM_ECAP_ALL_FLAGS);
    ECAP_clearInterrupt(BOARD_RCPWM_ECAP_BASE, RCPWM_ECAP_ALL_FLAGS);
    ECAP_stopCounter(BOARD_RCPWM_ECAP_BASE);

    /* continuous capture, wrap after event 2 (CAP1=rising ts, CAP2=falling ts) */
    ECAP_setCaptureMode(BOARD_RCPWM_ECAP_BASE, ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_2);
    ECAP_setEventPolarity(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_2, ECAP_EVNT_FALLING_EDGE);
    ECAP_enableCounterResetOnEvent(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_1);   /* reset on rising */
    ECAP_disableCounterResetOnEvent(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_2);  /* CAP2 = high-time */

    ECAP_enableTimeStampCapture(BOARD_RCPWM_ECAP_BASE);
    ECAP_enableCaptureMode(BOARD_RCPWM_ECAP_BASE);
    ECAP_startCounter(BOARD_RCPWM_ECAP_BASE);
    ECAP_reArm(BOARD_RCPWM_ECAP_BASE);
}

uint32_t RC_PWM_getPulseWidth_counts(void)
{
    return ECAP_getEventTimeStamp(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_2);
}

float RC_PWM_getThrottle(void)
{
    uint32_t flags = ECAP_getInterruptSource(BOARD_RCPWM_ECAP_BASE);
    uint32_t cap2;
    float us;

    /* Counter overflow => no edges for ~43 s of counts, i.e. the line stalled high/low
     * (lost signal). Clear everything and report invalid. */
    if((flags & ECAP_ISR_SOURCE_COUNTER_OVERFLOW) != 0U)
    {
        ECAP_clearInterrupt(BOARD_RCPWM_ECAP_BASE, RCPWM_ECAP_ALL_FLAGS);
        return -1.0f;
    }

    /* Require a fresh falling edge (CEVT2) since the last successful read. */
    if((flags & ECAP_ISR_SOURCE_CAPTURE_EVENT_2) == 0U)
    {
        return -1.0f;
    }

    cap2 = ECAP_getEventTimeStamp(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_2);
    ECAP_clearInterrupt(BOARD_RCPWM_ECAP_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2);

    us = (float)cap2 / RCPWM_COUNTS_PER_US;
    if((us < RCPWM_VALID_MIN_US) || (us > RCPWM_VALID_MAX_US))
    {
        return -1.0f;   /* outside the RC servo window */
    }

    if(us < RCPWM_MIN_US) { us = RCPWM_MIN_US; }
    if(us > RCPWM_MAX_US) { us = RCPWM_MAX_US; }
    return (us - RCPWM_MIN_US) / (RCPWM_MAX_US - RCPWM_MIN_US);
}
