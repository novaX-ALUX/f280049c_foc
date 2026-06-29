#include "check.h"
#include "esc_arbiter.h"

static esc_pwm_decode_cfg_t pwm_cfg(void)
{
    esc_pwm_decode_cfg_t c;
    c.us_min = 1000.0f; c.us_max = 2000.0f;
    c.us_valid_min = 900.0f; c.us_valid_max = 2100.0f;
    return c;
}

int main(void)
{
    esc_pwm_decode_cfg_t c = pwm_cfg();

    /* mid pulse -> 0.5, valid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 1500.0f);
        CHECK(s.fresh); CHECK(s.valid);
        CHECK_NEAR(s.throttle, 0.5f, 1e-4f);
        CHECK(!s.arm_req);            /* PWM never self-arms in decode */
    }
    /* min/max clamp to [0,1] */
    {
        esc_src_sample_t lo = esc_pwm_decode(&c, true, false, 1000.0f);
        esc_src_sample_t hi = esc_pwm_decode(&c, true, false, 2000.0f);
        CHECK_NEAR(lo.throttle, 0.0f, 1e-4f);
        CHECK_NEAR(hi.throttle, 1.0f, 1e-4f);
    }
    /* in-window but below us_min still clamps to 0 and stays valid (e.g. 950 us) */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 950.0f);
        CHECK(s.valid);
        CHECK_NEAR(s.throttle, 0.0f, 1e-4f);
    }
    /* outside valid window -> invalid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, false, 800.0f);
        CHECK(!s.valid);
    }
    /* no fresh edge -> not fresh, not valid */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, false, false, 1500.0f);
        CHECK(!s.fresh); CHECK(!s.valid);
    }
    /* overflow (line stalled) -> not valid even if width looks fine */
    {
        esc_src_sample_t s = esc_pwm_decode(&c, true, true, 1500.0f);
        CHECK(!s.valid);
    }

    CHECK_DONE();
}
