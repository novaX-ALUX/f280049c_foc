# src/encoder/ — MT6701 absolute encoder

The MT6701 (14-bit magnetic absolute) drives the **prop-park** position loop.

It is split into two layers so the math can be host-tested before the hardware exists:

| File | Layer | Status |
|------|-------|--------|
| `mt6701.{h,c}` | **Pure angle processing** — raw 14-bit code → mechanical/electrical angle, unwrap, velocity, glitch/stale. No bus, no driverlib. | done, host-tested |
| `mt6701.{h,c}` | **Pure SSI frame decode** — `mt6701_crc6()` + `mt6701_decode_ssi()`: split a raw 24-bit SSI frame, CRC6-validate (poly X^6+X+1), decode Mg status. | done, host-tested |
| `boards/esc6288_revA/.../mt6701_ssi.*` | **Bus read** (driverlib) — clocks the 24-bit frame over SPIA with a manual CSN, decodes it, feeds `(raw14, valid)` to `mt6701_update()`. | implemented, **bench-confirmed** (esc6288 SPIA + HT0104, POL1PHA0 SSI, manual CSN, live 14-bit decode; 2026-06-30) |

The frame decode + CRC are pure and live here (host-tested against the independently
generated `tools/test/mt6701_golden.inc`). The thin driverlib read adapter lives board-side
(`boards/esc6288_revA/drivers/source/mt6701_ssi.c`) because the host-test purity gate forbids
driverlib under `src/`.

Data flow (esc6288): `MT6701_SSI_read()` → `mt6701_decode_ssi()` (CRC/field/track verdict) →
`mt6701_update()` → `mt6701_mech_rev` / `mt6701_vel_revps` / `mt6701_valid` / `mt6701_stale`
→ `foc_raw_feedback_t.enc_*` → `esc_feedback_t` (in `product/product_main.c`). Encoder-less
boards (launchxl) leave `enc_valid=false`, so `esc_control` never enters parking.

**Bench-confirmed on esc6288 (2026-06-30):** POL1PHA0 SSI clock polarity/phase, manual-CSN timing,
and SSI read mode (this MT6701CT-STD part powers up in SSI by default) — all confirmed working with
SPIA + HT0104 level shifter, valid live angle decode every frame. Remaining: motor-coupled `dir` /
`zero_offset_counts` tuning — see `boards/esc6288_revA/PORT_TODO.md`.
