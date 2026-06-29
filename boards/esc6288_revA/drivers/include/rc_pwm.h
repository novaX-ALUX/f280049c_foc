/*
 * rc_pwm.h - RC servo-PWM throttle capture for esc6288_revA.
 *
 * Captures a 1-2 ms RC/PPM pulse on GPIO33 (PAD2/PPM net) via Input XBAR -> eCAP1.
 * Board-side driverlib glue; not host-tested. Pins/handles come from board.h.
 */
#ifndef ESC6288_RC_PWM_H
#define ESC6288_RC_PWM_H

#include <stdbool.h>
#include <stdint.h>

//! \brief Configure eCAP1 to capture the RC-PWM high time on GPIO33. Enables the eCAP1
//!        clock (the shared HAL leaves it disabled). GPIO33 is already an input.
void RC_PWM_init(void);

//! \brief Last captured pulse high-time in SYSCLK counts (100 counts/us at 100 MHz).
uint32_t RC_PWM_getPulseWidth_counts(void);

//! Raw RC-PWM capture for the arbiter's pure decode (esc_pwm_decode()).
typedef struct {
    bool  fresh;     //!< a new falling edge was captured since the last read
    bool  overflow;  //!< eCAP counter overflow (line stalled high/low -> lost signal)
    float width_us;  //!< last captured high-time in microseconds (valid only if fresh)
} rc_pwm_sample_t;

//! \brief Non-blocking eCAP1 poll. Reports fresh/overflow and the raw high-time in us.
//!        No range logic here -- esc_pwm_decode() owns the valid window + normalization.
void RC_PWM_read(rc_pwm_sample_t *out);

#endif /* ESC6288_RC_PWM_H */
