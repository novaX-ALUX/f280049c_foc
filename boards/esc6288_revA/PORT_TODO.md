# esc6288_revA — bring-up checklist

Status: **ported from the final schematic + netlist; all build gates green** (SRC_CHECK,
CAN_CHECK, PRODUCT_CHECK, PRODUCT, `LAB=all` 12/12, host tests 8/8). The board is at fab;
the items below are what to verify/tune once the prototype returns. Anything marked
**[BENCH]** needs the hardware (scope/meter) to confirm.

Hardware: F280049CPMSR (64-pin PM), gate driver **U12 = JSM6288T** (6 independent inputs,
**no EN, no nFAULT**), 3× INA181A1 (gain 20) over 0.5 mΩ shunts (1.65 V mid-rail,
bidirectional), 30.1k+30.1k+2k voltage dividers, **10 MHz** resonator. Full pin map is the
header comment in `drivers/include/board.h`.

## Schematic-verified (implemented)
- **Clock**: 10 MHz resonator → `SYSCTL_IMULT(20)` → 100 MHz; both clock asserts use a
  board-local 10 MHz osc const (`hal.c`). *If SYSCLK were wrong everything downstream is 2× off.*
- **PWM**: phase A=EPWM1 (GPIO0/1), B=EPWM2 (GPIO2/3), C=EPWM3 (GPIO4/5); `PWM_PHASE_ORDER`
  default 0=ABC. Dead-band 50 cnt (~500 ns) — extra margin ON TOP of the JSM6288T's own
  built-in ~200 ns anti-shoot-through dead time + interlock (datasheet); see the bench item.
- **Current**: `USER_ADC_FULL_SCALE_CURRENT_A = 330` (±165 A), offsets −165 A; PGAs disabled
  (external INA path). IA=ADCINB15, IB=ADCINA1, IC=ADCINC2 (distinct cores).
- **Voltage**: `USER_ADC_FULL_SCALE_VOLTAGE_V = 102.63`; Udc=ADCINA6, UA/UB/UC=B6/B3/C6.
- **DAC outputs disabled** in `HAL_setupDACs` — DACA_OUT/DACB_OUT share the IA/IB sense pads
  and would corrupt current sensing if enabled.
- **Protection**: software OC on all 3 phases (1 ms product layer + a 20 kHz ISR backstop,
  `ESC6288_ISR_OC_A = 60 A`) + **CMPSS3** hardware OC on phase C + **CMPSS5** DC-bus OV
  (~56 V). No digital nFAULT trip (there is no fault pin).
- **Safe-off without an enable pin**: EPWM trip-zone (OST). `HAL_enableDRV` is a no-op;
  `HAL_setupFaults` leaves outputs OST-forced; **offset-cal does NOT clear the trip** (gates
  stay off during cal, ADC still samples); only the arm path (`HAL_enablePWM`) un-trips;
  `HAL_setPWMBrake` is stubbed (would defeat safe-off). Gate macros kept as `BOARD_GPIO_NONE`
  sentinels; `gate_driver.c` hard-skips the sentinel.
- **CAN**: CANA, GPIO37 TX / GPIO35 RX, 1 Mbit (`can_bridge.c`, mirrors launchxl).
- **Aux drivers** (board-side, compiled + init-hooked from product_init): RC-PWM throttle on
  eCAP1 (`rc_pwm.c`), MT6701 SSI on SPIA (`mt6701_ssi.c`), WS2812 RGB on GPIO12 (`rgb_led.c`).

## Bench bring-up — do in order, gate driver kept off (forced TZ) until each step passes
1. **Rails + clock**: power 3V3/5V/12V only (no motor). Confirm **SYSCLK = 100 MHz** (toggle a
   GPIO at a known divide, scope it) — proves `IMULT(20)`. If it reads 50 MHz the resonator
   assumption is wrong.
2. **Idle outputs**: verify EPWM1/2/3 outputs idle LOW and a forced OST holds them low through
   offset-cal; confirm `HAL_enablePWM` is the only thing that un-trips.
3. **ADC offsets**: PWM idle → all 3 currents read ~1.65 V (~count 2048); Udc tracks the bus
   through the 31.1× divider (apply a known bench voltage, check the reported V).
4. **[BENCH] Dead-band / short-pulse** at low bus before spinning; watch the half-bridge Vds for
   shoot-through; then close the current loop; then spin.
5. **Protection tests**: trip phase-C CMPSS3 OC, DC-bus CMPSS5 OV (~56 V), and the ISR software
   OC on phases A/B; confirm each forces all outputs low and latches `moduleOverCurrent`.
6. **CAN / encoder / RC-PWM / RGB**: DroneCAN node at 1 Mbit; MT6701 angle reads; RC-PWM
   1–2 ms maps to throttle; RGB status colors.

## [BENCH] Confirm / tune
- **JSM6288T dead time** — datasheet (`docs/JSM6288T.pdf`) confirms a per-phase HIN/LIN 6-input
  driver that DOES have built-in anti-shoot-through protection: an interlock (HIN=LIN=H → both
  outputs off, truth table p.8 / functional block "DeadTime & Control Logic") AND a ~200 ns
  internal dead time (DT min 100 / typ 200 / max 300 ns, p.5; waveform fig. 6-3). ton/toff
  ~120-250 ns. The MCU 500 ns dead-band (`HAL_PWM_DBRED_CNT/DBFED_CNT`, `hal.h`) is therefore
  EXTRA conservative margin, not the sole protection. [BENCH] scope the half-bridge Vds / gate
  waveforms and tune the MCU dead-band DOWN — the chip's ~200 ns is a floor, but the real
  non-overlap at the FET also depends on propagation/rise/fall, Qg/Coss and layout ringing, so
  verify on the scope rather than trusting the additive number (do not assume 0 is safe).
- **FET NVMFS5C612NL Vds rating** — this is the hard OV ceiling. Confirm the 12S OV
  (`HAL_BUS_OV_CMPSS_DACH` ≈ 56 V, and product `vbus_ov_set = 54`) sits safely below it.
- **OC thresholds** — product `oc_set_A = 30` and `ESC6288_ISR_OC_A = 60` are conservative
  bench values; raise toward the motor/shunt rating after validation.
- **Bus nominal / UV** — currently 48 V nominal, UV 18 V (bench-friendly); raise UV for flight.
- **MT6701 SSI** (`mt6701_ssi.c`) — **full 24-bit read + CRC implemented; bench-pending verify.**
  Per datasheet `docs/MT6701CT-STD.PDF` sec 6.8 (and the encoder daughterboard schematic
  `ef_encoder_mt6701.{NET,pdf}`): **POL1PHA0**; the 24-bit frame (14-bit angle + 4-bit Mg
  status + 6-bit CRC, poly X^6+X+1) is now clocked as two 16-bit words inside one **manual-CSN**
  window (GPIO11 re-muxed from SPISTE to GPIO), then decoded + CRC6-validated by the pure,
  host-tested `mt6701_decode_ssi()` (`src/encoder/mt6701.c`, golden-vector test). `MT6701_SSI_read()`
  returns *usable* = CRC ok & field normal & no loss-of-track; the product bridges it through
  `mt6701_update()` → `foc_raw_feedback_t.enc_*` → `esc_feedback_t`. **Remaining [BENCH]:**
  (1) confirm the SSI-vs-I²C EEPROM default of this MT6701CT-STD part (MODE=VDD selects the
  digital interface; the default within it is unverified); (2) confirm clock polarity/phase and
  the manual-CSN TL/TH timing on the scope; (3) tune `dir` / `zero_offset_counts` to the motor;
  (4) `auto_park` is deliberately left **disabled** until the encoder is validated with a prop.
- **RGB WS2812 timing** (`rgb_led.c`) — the bit-bang loop counts are approximate; scope GPIO12
  and tune `WS_*_LOOPS` to the WS2812B timing.
- **NTC → °C** — **implemented; bench-pending calibration.** The NCP18XH103 (ADCINC3 → ADCC SOC2)
  is converted by the pure, host-tested `ntc_counts_to_celsius()` (`src/common/ntc.c`, beta model)
  using the board divider in `board.h` (3V3 — NTC — [ADC] — R14 10k — GND, so NTC **high-side**).
  `product_main` feeds the result into `raw.temp_C` → `esc_feedback_t` → `esc_control`'s over-temp
  latch (`temp_ot_set=100 / clr=85`), which is now **live**. A dead sensor (open/short → near-rail
  count) reads back `BOARD_NTC_OPEN_TEMP_C` (150 °C) and trips the fault (fail-safe hot).
  **Remaining [BENCH]:** confirm R14 value + the VREFHI/3V3 rail, and trim `r25`/`beta` against the
  measured curve (apply known bench temperatures, compare reported °C).

## Rev-B hardware note (not fixable in firmware on this rev)
The phase-A and phase-B current-sense op-amp outputs land on **ADCIN A0/B15/C15 and A1**, which
have **no CMPSS comparator** on F28004x — so only phase C and the DC bus get hardware
cycle-by-cycle trips. This rev relies on software OC for A/B by design (user-approved). For a
rev B, route the IA/IB sense onto CMPSS-capable pads (e.g. B2/A4) to restore 3-phase hardware OC.

## Build / verify
```bash
BOARD=esc6288_revA MOTOR=am_4116_kva SRC_CHECK=1     bash build.sh   # pure src/ modules
BOARD=esc6288_revA                   CAN_CHECK=1     bash build.sh   # CAN bridge + comms
BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT_CHECK=1 bash build.sh   # product main + foc_bridge
BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT=1       bash build.sh   # full product link
BOARD=esc6288_revA LAB=all bash build.sh                              # 12-lab regression
bash tools/test/run.sh                                               # host tests (incl. src/ purity)
```
