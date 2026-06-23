/*
 * rc_pwm.h - RC servo-PWM throttle capture for esc6288_revA.
 *
 * Captures a 1-2 ms RC/PPM pulse on GPIO33 (PAD2/PPM net) via Input XBAR -> eCAP1.
 * Board-side driverlib glue; not host-tested. Pins/handles come from board.h.
 */
#ifndef ESC6288_RC_PWM_H
#define ESC6288_RC_PWM_H

#include <stdint.h>

//! \brief Configure eCAP1 to capture the RC-PWM high time on GPIO33. Enables the eCAP1
//!        clock (the shared HAL leaves it disabled). GPIO33 is already an input.
void RC_PWM_init(void);

//! \brief Last captured pulse high-time in SYSCLK counts (100 counts/us at 100 MHz).
uint32_t RC_PWM_getPulseWidth_counts(void);

//! \brief Throttle in [0,1] mapped from a fresh, in-range [1.0,2.0] ms pulse.
//!        Returns -1.0f if no fresh pulse, on counter overflow (lost signal), or if the
//!        width is outside the valid [0.9,2.1] ms RC window.
float RC_PWM_getThrottle(void);

#endif /* ESC6288_RC_PWM_H */
