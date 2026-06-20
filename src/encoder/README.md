# src/encoder/ — MT6701 absolute encoder

The MT6701 (14-bit magnetic absolute) drives the **prop-park** position loop.

It is split into two layers so the math can be host-tested before the hardware exists:

| File | Layer | Status |
|------|-------|--------|
| `mt6701.{h,c}` | **Pure angle processing** — raw 14-bit code → mechanical/electrical angle, unwrap, velocity, glitch/stale. No bus, no driverlib. | done, host-tested |
| `mt6701_ssi.*` | **Bus read** — MT6701 in SSI mode read over the SPI clock, fills raw codes into `mt6701_update()`. | **TODO (deferred)** |

## TODO: `mt6701_ssi.*` (bus read layer)
- On the esc6288 hardware the MT6701 is read over **SSI** (synchronous serial), not I²C.
  Read the 14-bit angle via the SPI peripheral clock; surface bus-level validity
  (framing/parity) as the `raw_valid` argument to `mt6701_update()`.
- Blocked on the encoder pin map in `boards/esc6288_revA/.../board.h` (not defined yet).
- Keep this layer thin: read raw code + validity only; all angle math stays in `mt6701.c`.

Until then no `mt6701_ssi.h` is added on purpose — declaring an unimplemented API would
create a dangling link dependency. The product `main` (also deferred) bridges the processed
outputs (`mt6701_mech_rev` / `mt6701_vel_revps` / `mt6701_valid` / `mt6701_stale`) into
`esc_feedback_t`.
