#include "esc_arbiter.h"
#include <math.h>

static float clamp01(float v)
{
    if (v < 0.0f) { return 0.0f; }
    if (v > 1.0f) { return 1.0f; }
    return v;
}

esc_src_sample_t esc_pwm_decode(const esc_pwm_decode_cfg_t *cfg,
                                bool fresh, bool overflow, float width_us)
{
    esc_src_sample_t s;
    s.fresh = fresh;
    s.valid = false;
    s.throttle = 0.0f;
    s.arm_req = false;   /* PWM arming is the arbiter's dwell gate, never the decode */

    if (!fresh || overflow) {
        return s;        /* stale or stalled line */
    }
    if (width_us < cfg->us_valid_min || width_us > cfg->us_valid_max) {
        return s;        /* outside the RC servo window -> invalid */
    }
    s.valid = true;
    {
        float span = cfg->us_max - cfg->us_min;
        float t = (span > 0.0f) ? (width_us - cfg->us_min) / span : 0.0f;
        s.throttle = clamp01(t);
    }
    return s;
}
