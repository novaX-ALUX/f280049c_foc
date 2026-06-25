#include "ntc.h"
#include <math.h>

#define NTC_MIN_C  (-40.0f)   /* NCP18XH103 rated span */
#define NTC_MAX_C  (150.0f)
#define NTC_RAIL_GUARD_COUNTS  (2.0f)  /* within this of either rail => open/short */

float ntc_counts_to_celsius(const ntc_cfg_t *cfg, uint16_t counts)
{
    float full = cfg->adc_full_counts;
    float c = (float)counts;
    float ratio, r_ntc, inv_t, t_c;

    /* Within NTC_RAIL_GUARD_COUNTS of either rail = open or shorted thermistor (either topology)
     * -> fail-safe hot. Inclusive (<=/>=) so the boundary LSB also trips: on the high-side esc6288
     * NTC a dead sensor reading exactly the guard count must fault, not pass as an extreme-cold
     * value that would suppress the over-temp trip. */
    if (c <= NTC_RAIL_GUARD_COUNTS || c >= (full - NTC_RAIL_GUARD_COUNTS)) {
        return cfg->open_temp_C;
    }

    ratio = c / full;   /* Vadc/Vref */

    if (cfg->ntc_low_side) {
        /* Rntc = Rfixed * ratio/(1-ratio) */
        r_ntc = cfg->r_fixed_ohm * ratio / (1.0f - ratio);
    } else {
        /* Rntc = Rfixed * (1-ratio)/ratio */
        r_ntc = cfg->r_fixed_ohm * (1.0f - ratio) / ratio;
    }

    if (r_ntc <= 0.0f) {
        return cfg->open_temp_C;
    }

    /* Beta model: 1/T = 1/t0 + (1/beta) ln(Rntc/R25). */
    inv_t = 1.0f / cfg->t0_K + (1.0f / cfg->beta_K) * logf(r_ntc / cfg->r25_ohm);
    t_c = 1.0f / inv_t - 273.15f;

    if (t_c < NTC_MIN_C) { t_c = NTC_MIN_C; }
    if (t_c > NTC_MAX_C) { t_c = NTC_MAX_C; }
    return t_c;
}
