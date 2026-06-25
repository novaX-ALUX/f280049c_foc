#include "check.h"
#include "ntc.h"
#include <math.h>

/* esc6288: NCP18XH103, NTC high-side to 3V3, R14 (10k) low-side, 12-bit ADC. */
static ntc_cfg_t base_cfg(void)
{
    ntc_cfg_t c = {0};
    c.r_fixed_ohm     = 10000.0f;
    c.r25_ohm         = 10000.0f;
    c.beta_K          = 3380.0f;
    c.t0_K            = 298.15f;
    c.adc_full_counts = 4096.0f;
    c.ntc_low_side    = false;     /* NTC high-side */
    c.open_temp_C     = 150.0f;
    return c;
}

/* Forward model: temperature -> expected ADC count (inverse of the function under test). */
static uint16_t counts_for_temp(const ntc_cfg_t *c, float t_c)
{
    float T = t_c + 273.15f;
    float r = c->r25_ohm * expf(c->beta_K * (1.0f / T - 1.0f / c->t0_K));
    float ratio = c->ntc_low_side ? (r / (c->r_fixed_ohm + r))
                                  : (c->r_fixed_ohm / (c->r_fixed_ohm + r));
    float cnt = ratio * c->adc_full_counts;
    if (cnt < 0.0f) { cnt = 0.0f; }
    if (cnt > c->adc_full_counts) { cnt = c->adc_full_counts; }
    return (uint16_t)(cnt + 0.5f);
}

int main(void)
{
    /* Beta-INDEPENDENT anchor: Rntc == R25 => ratio 0.5 => count 2048 => exactly t0 (25 C).
     * This pins the absolute calibration regardless of beta. */
    {
        ntc_cfg_t c = base_cfg();
        CHECK_NEAR(ntc_counts_to_celsius(&c, 2048), 25.0f, 0.3f);
    }

    /* Round-trip over the rated span: temp -> counts (forward model) -> temp. Validates the
     * divider algebra + topology + beta inversion are mutually consistent. */
    {
        ntc_cfg_t c = base_cfg();
        float temps[] = {-30.0f, -10.0f, 0.0f, 25.0f, 50.0f, 85.0f, 120.0f};
        int i;
        for (i = 0; i < (int)(sizeof(temps) / sizeof(temps[0])); ++i) {
            uint16_t cnt = counts_for_temp(&c, temps[i]);
            CHECK_NEAR(ntc_counts_to_celsius(&c, cnt), temps[i], 2.0f);
        }
    }

    /* High-side monotonicity: hotter NTC -> lower Rntc -> higher count. */
    {
        ntc_cfg_t c = base_cfg();
        float t_lo = ntc_counts_to_celsius(&c, 1000);
        float t_mid = ntc_counts_to_celsius(&c, 2000);
        float t_hi = ntc_counts_to_celsius(&c, 3000);
        CHECK(t_lo < t_mid);
        CHECK(t_mid < t_hi);
    }

    /* Fail-safe: open/shorted thermistor (near either rail) returns open_temp_C, which sits
     * above the over-temp trip so a dead sensor latches the fault. */
    {
        ntc_cfg_t c = base_cfg();
        CHECK_NEAR(ntc_counts_to_celsius(&c, 0), 150.0f, 1e-3f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, 1), 150.0f, 1e-3f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, 4095), 150.0f, 1e-3f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, 4096), 150.0f, 1e-3f);
        /* Boundary: exactly NTC_RAIL_GUARD_COUNTS (2) from either rail must ALSO fail-safe
         * (inclusive guard). The low rail is the safety-critical one: count 2 would otherwise
         * read extreme-cold and suppress the over-temp trip on a dead high-side sensor. */
        CHECK_NEAR(ntc_counts_to_celsius(&c, 2), 150.0f, 1e-3f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, 4094), 150.0f, 1e-3f);
    }

    /* Just inside the guard (count 3) is in-band: pins the boundary at exactly 2, proving the
     * guard is inclusive-at-2 and not over-guarding. Impossibly-cold -> clamps to -40 (not open). */
    {
        ntc_cfg_t c = base_cfg();
        CHECK_NEAR(ntc_counts_to_celsius(&c, 3), -40.0f, 1e-3f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, 50), -40.0f, 1e-3f);
    }

    /* Low-side variant: count 2048 still reads 25 C (symmetric divider), monotonicity reverses. */
    {
        ntc_cfg_t c = base_cfg();
        c.ntc_low_side = true;
        CHECK_NEAR(ntc_counts_to_celsius(&c, 2048), 25.0f, 0.3f);
        CHECK(ntc_counts_to_celsius(&c, 1000) > ntc_counts_to_celsius(&c, 3000));

        /* round-trip also holds for the low-side topology */
        uint16_t cnt = counts_for_temp(&c, 60.0f);
        CHECK_NEAR(ntc_counts_to_celsius(&c, cnt), 60.0f, 2.0f);
    }

    CHECK_DONE();
}
