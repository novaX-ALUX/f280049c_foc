/*
 * ntc.h - Pure NTC thermistor ADC-count -> temperature conversion.
 *
 * ONLY the math: a raw ADC count from a resistor/thermistor divider -> degrees C, via the
 * beta (B-constant) model. No bus, no driverlib (math.h only), so it is host-tested. The board
 * supplies the divider topology + thermistor curve as an ntc_cfg_t and reads the raw ADC count;
 * the product feeds the result into esc_feedback_t.temp_C so esc_control's over-temp latch works.
 *
 * Divider (ratiometric: VREFHI must equal the divider's top rail, so the count ratio is rail-
 * independent):
 *   low_side  (ntc_low_side=true) : Vtop -- Rfixed -- [ADC] -- Rntc -- GND
 *                                   ratio = Rntc/(Rfixed+Rntc) -> Rntc = Rfixed*ratio/(1-ratio)
 *   high_side (ntc_low_side=false): Vtop -- Rntc  -- [ADC] -- Rfixed -- GND   (esc6288: NTC to 3V3)
 *                                   ratio = Rfixed/(Rfixed+Rntc) -> Rntc = Rfixed*(1-ratio)/ratio
 * Beta model: 1/T = 1/t0_K + (1/beta_K)*ln(Rntc/r25_ohm),  T in K, result in C.
 *
 * Fail-safe: a near-rail count (open or shorted thermistor, either topology) returns
 * open_temp_C -- set it ABOVE the over-temp trip so a dead sensor latches the fault rather than
 * silently reading "cold".
 */
#ifndef NTC_H
#define NTC_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float r_fixed_ohm;     /* the fixed divider resistor */
    float r25_ohm;         /* thermistor resistance at t0_K (NCP18XH103: 10000) */
    float beta_K;          /* B-constant (NCP18XH103 B25/85: 3380) */
    float t0_K;            /* reference temperature for r25, in K (298.15) */
    float adc_full_counts; /* ADC counts at full scale (12-bit: 4096) */
    bool  ntc_low_side;    /* true: NTC to GND; false: NTC to the top rail (esc6288) */
    float open_temp_C;     /* returned on a near-rail (open/short) read; set above the OT trip */
} ntc_cfg_t;

/*
 * Raw ADC count -> degrees C. A count within ~2 LSB of either rail (open/short thermistor) returns
 * cfg->open_temp_C. Valid results are clamped to the NCP18XH103 rated span [-40, 150] C.
 */
float ntc_counts_to_celsius(const ntc_cfg_t *cfg, uint16_t counts);

#endif /* NTC_H */
