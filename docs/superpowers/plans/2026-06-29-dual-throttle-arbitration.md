# Dual-Throttle (CAN + RC-PWM) Arbitration Implementation Plan

> **Status (2026-06-29): IMPLEMENTED AND MERGED** (master `e652f55..f000dde`). This file is retained as the historical implementation plan/design record — do NOT run its unchecked TDD checklist against the current tree; the code already exists. The only remaining work is the Appendix bench-validation gate, required before enabling `ESC_ARB_CAN_PRIMARY`.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a pure, host-tested command-source arbiter that fuses the existing DroneCAN throttle with the (already-wired but unused) RC-PWM throttle on `esc6288_revA`, defaulting to CAN-only so the change ships inert until bench-validated.

**Architecture:** A new pure module `src/app/esc_arbiter.{h,c}` consumes two per-source samples each 1 ms tick (CAN, PWM) and emits a single `esc_command_t` plus the active source. It owns per-source health/staleness, a PWM low-dwell arm gate, a "PWM must track CAN before it may take over" lockout (defeats stuck-PWM), policy selection, and handoff slew. The board layer `rc_pwm.c` is extended with a raw `RC_PWM_read()` (fresh/overflow/width); `product_main.c` decodes that into a sample, calls the arbiter, and feeds the result into the unchanged `esc_control_step()`. The existing 0.5 s `esc_control` watchdog stays as the final coast backstop.

**Tech Stack:** C11, TI C2000Ware driverlib (board layer only), pure C for `src/`, host gcc unit tests (`tools/test/run.sh`).

## Global Constraints

- `src/` MUST stay free of SDK/board/driverlib headers (`driverlib.h`, `device.h`, `hal.h`, `user.h`) and `C2000Ware_MotorControl_SDK` — `tools/test/run.sh` greps for these and fails the build. Allowed: `stdint.h`, `stdbool.h`, `math.h`.
- Host test build flags are `-std=c11 -Wall -Wextra -Werror -O2`. Every internal helper must be `static` and used; every function parameter must be used. Exported (non-static) functions may be unused.
- `tools/test/run.sh` compiles **all** of `src/**/*.c` into **every** `test_*.c` binary (it globs `find "$SRC" -name '*.c'`). The new `esc_arbiter.c` must therefore compile cleanly with no `main()` and no unused-static warnings, even in unrelated test binaries.
- The active product target is `esc6288_revA`. The two LaunchPad boards must keep building; arbiter code is board-agnostic and compiles everywhere, but the RC-PWM source is esc6288-only (guarded by `#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)` in `product_main.c`).
- **Safety default:** the shipped arbiter policy MUST be `ESC_ARB_EXPLICIT_CAN` (PWM fully ignored). CAN-primary fallback is only enabled after the bench gate in the Appendix. Because of the SEQ CONTRACT (the arbiter bumps `out_seq` only on a fresh CAN frame and holds it otherwise), `esc_control`'s watchdog still ages from the last real CAN frame — so the shipped default is timing-identical to today's CAN-only failsafe, not merely "close".
- **Coast latency model:** the per-source `*_stale_s` windows govern *source selection* (when to consider fallback / declare no-source), NOT the coast timer. The motor still coasts `cmd_timeout_s` (0.5 s) after the last valid frame of the active source, via the unchanged `esc_control` watchdog. The timing constraint chain MUST hold per source: `frame_period  <  *_stale_s  <  cmd_timeout_s`. For PWM that is `~20 ms (50 Hz) < pwm_stale_s (60 ms) < 0.5 s` — `pwm_stale_s` must exceed a few pulse periods so a single dropped pulse does not deselect PWM, yet stay well under `cmd_timeout_s` so a fallback can engage before the coast fires. For CAN, `~10 ms (100 Hz) < can_stale_s (100 ms) < 0.5 s`.

## Design rationale — intended behaviors (do NOT "fix" these)

These are deliberate. They were reviewed; an implementer should preserve them, not tighten them away:

1. **`track_ok` is latched while CAN is unhealthy.** Once PWM has demonstrably tracked CAN and CAN is then lost, PWM becomes the authority and may command freely (the pilot's stick) — including moving away from the last CAN value. That is the entire point of an RC fallback. Eligibility is gated *before* the loss (it must have tracked while CAN was healthy) and by the arm gate; after handoff the PWM source is trusted. `track_ok` is re-cleared the moment PWM itself goes unhealthy (see test (g)), so a dropout cannot leave a stale-true latch authorizing a later blind takeover.
2. **PWM "armed" is sticky until signal loss, and an idle-low receiver can arm it.** A PWM source that has dwelled valid-low becomes armed and, if it ever becomes the selected source, emits `arm=true` at `throttle≈0`. `esc_control` then sits in `ARMED` at zero throttle (no torque; `RUN_TORQUE` still requires `throttle > throttle_run_thresh`). Armed-at-idle is the correct ready state for a hot standby — it is not a drive event. In the shipped `EXPLICIT_CAN` default this path is inert (PWM ignored). The receiver's own failsafe output (idle-low vs. hold-last vs. out-of-range) is validated at bench step 3 before `CAN_PRIMARY` is enabled.
- Preserve the safe-off invariant: this plan never touches `HAL_setupFaults`, trip-zone OST, or `apply_setpoint`'s coast branch. The arbiter only changes which command (or none) reaches `esc_control_step`.
- The CAN zero-frame arm handshake stays in `dronecan.c` unchanged; the arbiter consumes the already-handshaked `arm` bit. PWM gets its own independent low-dwell arm gate.
- No `Date.now()` / wall-clock; all timing is the caller's `dt_s` (0.001 f).

---

### Task 1: Arbiter types + pure PWM decode

**Files:**
- Create: `src/app/esc_arbiter.h`
- Create: `src/app/esc_arbiter.c`
- Test: `tools/test/test_esc_arbiter.c`

**Interfaces:**
- Consumes: `esc_command_t` from `src/common/esc_types.h`.
- Produces:
  - `typedef enum { ESC_SRC_NONE=0, ESC_SRC_CAN=1, ESC_SRC_PWM=2 } esc_src_id_t;`
  - `esc_src_sample_t { bool fresh; bool valid; float throttle; bool arm_req; }`
  - `esc_pwm_decode_cfg_t { float us_min, us_max, us_valid_min, us_valid_max; }`
  - `esc_src_sample_t esc_pwm_decode(const esc_pwm_decode_cfg_t *cfg, bool fresh, bool overflow, float width_us);`

- [ ] **Step 1: Write the failing test** — create `tools/test/test_esc_arbiter.c`

```c
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `bash tools/test/run.sh`
Expected: `BUILD FAIL test_esc_arbiter` (no `esc_arbiter.h`).

- [ ] **Step 3: Create the header** — `src/app/esc_arbiter.h`

```c
/*
 * esc_arbiter.h - Command-source arbitration (pure logic, no SDK).
 *
 * Fuses the DroneCAN throttle (esc_command_t, post arm-handshake) with an RC-PWM
 * throttle into a single esc_command_t + active source. Owns per-source staleness,
 * a PWM low-dwell arm gate, a "PWM must track CAN before it may take over" lockout
 * (defeats a stuck PWM line), policy selection, and handoff slew. It does NOT read
 * eCAP or CAN hardware: the product main hands it already-decoded samples each tick.
 *
 * Layering: product main -> esc_arbiter_step() -> esc_command_t -> esc_control_step().
 * When no source is healthy the arbiter emits have_cmd=false; the product main then
 * passes NULL to esc_control, whose existing watchdog coasts the stage.
 */
#ifndef ESC_ARBITER_H
#define ESC_ARBITER_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_types.h"

/* Which source currently owns the command. */
typedef enum {
    ESC_SRC_NONE = 0,
    ESC_SRC_CAN  = 1,
    ESC_SRC_PWM  = 2
} esc_src_id_t;

/* Selection policy. Default ships EXPLICIT_CAN (PWM ignored); the flight policy
 * CAN_PRIMARY is enabled only after the bench gate. */
typedef enum {
    ESC_ARB_EXPLICIT_CAN = 0,   /* CAN only; PWM ignored entirely (safe default) */
    ESC_ARB_EXPLICIT_PWM = 1,   /* PWM only (bench bring-up of the RC path) */
    ESC_ARB_CAN_PRIMARY  = 2     /* CAN primary, PWM tracked hot-standby fallback */
} esc_arb_policy_t;

/* Per-source sample, built by the product main each tick. */
typedef struct {
    bool  fresh;     /* a new frame/pulse arrived since last tick */
    bool  valid;     /* signal physically valid (in range / good frame) */
    float throttle;  /* [0,1], meaningful only if valid */
    bool  arm_req;   /* source arm intent (CAN: post-handshake arm; PWM: ignored) */
} esc_src_sample_t;

/* Pure RC-PWM pulse decode (us -> normalized sample). */
typedef struct {
    float us_min, us_max;            /* maps to [0,1]; e.g. 1000, 2000 */
    float us_valid_min, us_valid_max; /* reject window; e.g. 900, 2100 */
} esc_pwm_decode_cfg_t;

esc_src_sample_t esc_pwm_decode(const esc_pwm_decode_cfg_t *cfg,
                                bool fresh, bool overflow, float width_us);

/* Arbiter status bits (reported via esc_arbiter_result_t.status_bits). */
typedef enum {
    ESC_ARB_ST_NONE        = 0u,
    ESC_ARB_ST_PWM_ACTIVE  = 1u << 0,  /* PWM is the selected source */
    ESC_ARB_ST_NO_SOURCE   = 1u << 1,  /* no healthy source this tick */
    ESC_ARB_ST_PWM_LOCKOUT = 1u << 2,  /* PWM healthy+armed but not tracking -> ineligible */
    ESC_ARB_ST_HANDOFF     = 1u << 3   /* blending across a source change */
} esc_arb_status_t;

typedef struct {
    esc_arb_policy_t policy;
    float can_stale_s;            /* CAN unhealthy after this with no fresh+valid */
    float pwm_stale_s;            /* PWM unhealthy after this (~3 missed 50 Hz pulses) */
    float pwm_arm_low_s;          /* PWM must be valid-low this long before it may arm */
    float pwm_arm_throttle_eps;   /* throttle <= this counts as "low" */
    float track_tol;              /* |pwm-can| <= this counts as tracking */
    float track_required_s;       /* tracking must persist this long to enable fallback */
    float handoff_slew_per_s;     /* throttle slew limit across a source change */
    esc_pwm_decode_cfg_t pwm;     /* decode window (used by the product main) */
} esc_arbiter_cfg_t;

typedef struct {
    bool  seen;           /* a fresh+valid sample has EVER arrived (cold-boot guard) */
    float age_s;          /* time since last fresh+valid sample */
    bool  healthy;        /* seen AND age_s <= stale_s */
    float throttle;       /* last valid throttle, held while healthy */
    bool  arm_req;        /* last arm intent */
    float low_dwell_s;    /* PWM only: accumulated valid-low time (arming) */
    bool  armed;          /* PWM only: passed the low-dwell arm gate */
    float track_dwell_s;  /* PWM only: accumulated time tracking CAN */
    bool  track_ok;       /* PWM only: eligible as fallback */
} esc_src_state_t;

typedef struct {
    esc_arbiter_cfg_t cfg;
    esc_src_state_t   can;
    esc_src_state_t   pwm;
    esc_src_id_t      active;       /* current selection */
    uint32_t          out_seq;      /* SEQ CONTRACT: bumped ONLY when the selected source is
                                     * fresh+valid THIS tick; held (unchanged) while a healthy
                                     * source is being held with no new frame. esc_control's
                                     * watchdog is seq-change based, so this makes the failsafe
                                     * time from the last REAL frame -- identical to passing CAN
                                     * straight through as today. Never free-runs per tick. */
    float             out_throttle; /* slew-limited output throttle */
    bool              blending;     /* mid handoff slew */
} esc_arbiter_state_t;

typedef struct {
    bool          have_cmd;   /* false -> product main passes NULL to esc_control */
    esc_command_t cmd;        /* seq/throttle/arm, valid only if have_cmd */
    esc_src_id_t  active;     /* winning source (NONE if !have_cmd) */
    uint32_t      status_bits; /* esc_arb_status_t bitmask */
} esc_arbiter_result_t;

void esc_arbiter_init(esc_arbiter_state_t *st, const esc_arbiter_cfg_t *cfg);

/* One arbitration tick. can/pwm may carry fresh=false (no new sample). dt_s > 0. */
void esc_arbiter_step(esc_arbiter_state_t *st,
                      const esc_src_sample_t *can, const esc_src_sample_t *pwm,
                      float dt_s, esc_arbiter_result_t *res);

#endif /* ESC_ARBITER_H */
```

- [ ] **Step 4: Create the implementation with only `esc_pwm_decode`** — `src/app/esc_arbiter.c`

```c
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
```

- [ ] **Step 5: Run test to verify it passes**

Run: `bash tools/test/run.sh`
Expected: `PASS test_esc_arbiter` and all existing tests still pass.

- [ ] **Step 6: Commit**

```bash
git add src/app/esc_arbiter.h src/app/esc_arbiter.c tools/test/test_esc_arbiter.c
git commit -m "feat(arbiter): add command-source types + pure RC-PWM decode"
```

---

### Task 2: Source health, PWM arm gate, explicit policies

**Files:**
- Modify: `src/app/esc_arbiter.c` (add `esc_arbiter_init` + `esc_arbiter_step` for the two EXPLICIT policies)
- Test: `tools/test/test_esc_arbiter.c` (append cases)

**Interfaces:**
- Consumes: `esc_pwm_decode` (Task 1), the cfg/state/result structs (Task 1 header).
- Produces: `esc_arbiter_init`, `esc_arbiter_step` (EXPLICIT_CAN / EXPLICIT_PWM behavior; CAN_PRIMARY added in Task 3).

- [ ] **Step 1: Write the failing tests** — append before `CHECK_DONE();` in `tools/test/test_esc_arbiter.c`

```c
    /* ---- Task 2: health + explicit policies ---- */
    {
        esc_arbiter_cfg_t ac = {0};
        ac.can_stale_s = 0.10f;  ac.pwm_stale_s = 0.06f;
        ac.pwm_arm_low_s = 0.50f; ac.pwm_arm_throttle_eps = 0.05f;
        ac.track_tol = 0.10f;    ac.track_required_s = 0.50f;
        ac.handoff_slew_per_s = 2.0f;
        ac.pwm = pwm_cfg();

        const float dt = 0.001f;
        esc_src_sample_t can = {0}, pwm = {0};
        esc_arbiter_result_t r;

        /* EXPLICIT_CAN: a fresh CAN command is selected immediately, PWM ignored. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.4f; can.arm_req = true;
            pwm.fresh = true; pwm.valid = true; pwm.throttle = 0.9f; /* must be ignored */
            esc_arbiter_step(&st, &can, &pwm, dt, &r);
            CHECK(r.have_cmd);
            CHECK(r.active == ESC_SRC_CAN);
            CHECK_NEAR(r.cmd.throttle, 0.4f, 1e-4f);
            CHECK(r.cmd.arm);
            CHECK(r.cmd.seq != 0u);
        }

        /* CAN goes stale -> no source -> have_cmd false after can_stale_s. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.3f; can.arm_req = true;
            esc_arbiter_step(&st, &can, &pwm, dt, &r);  /* prime */
            esc_src_sample_t none = {0};                /* fresh=false */
            int i; bool coasted = false;
            for (i = 0; i < 200; i++) {                 /* 200 ms > 100 ms stale */
                esc_arbiter_step(&st, &none, &none, dt, &r);
                if (!r.have_cmd) { coasted = true; break; }
            }
            CHECK(coasted);
            CHECK(r.active == ESC_SRC_NONE);
            CHECK(r.status_bits & ESC_ARB_ST_NO_SOURCE);
        }

        /* EXPLICIT_PWM: PWM does NOT arm until it has been valid-low for pwm_arm_low_s. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t plo = {0};
            plo.fresh = true; plo.valid = true; plo.throttle = 0.0f;  /* low */
            int i;
            for (i = 0; i < 100; i++) {                 /* 100 ms < 500 ms dwell */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            }
            CHECK(!r.have_cmd);                         /* not armed yet -> no command */
            for (; i < 600; i++) {                      /* cross 500 ms */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            }
            CHECK(r.have_cmd);                          /* armed now */
            CHECK(r.active == ESC_SRC_PWM);
            CHECK(r.status_bits & ESC_ARB_ST_PWM_ACTIVE);
        }

        /* Stuck-high PWM at boot never sees low -> never arms -> never commands. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t phi = {0};
            phi.fresh = true; phi.valid = true; phi.throttle = 0.8f;  /* stuck high */
            int i;
            for (i = 0; i < 2000; i++) {                /* 2 s of stuck-high */
                esc_arbiter_step(&st, &(esc_src_sample_t){0}, &phi, dt, &r);
            }
            CHECK(!r.have_cmd);
            CHECK(r.active == ESC_SRC_NONE);
        }

        /* COLD BOOT (seen-gate): with NO CAN ever received, a zeroed arbiter must NOT emit
         * a phantom command. have_cmd stays false so the product passes NULL and
         * esc_control's "never received first frame is not a timeout" invariant holds. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t none = {0};
            int i;
            for (i = 0; i < 50; i++) {                  /* well within can_stale_s (100 ms) */
                esc_arbiter_step(&st, &none, &none, dt, &r);
                CHECK(!r.have_cmd);
                CHECK(r.active == ESC_SRC_NONE);
            }
        }

        /* SEQ CONTRACT: while CAN is healthy but no fresh frame, the emitted seq is HELD
         * (so esc_control's seq-change watchdog ages); a fresh frame bumps it. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_CAN;
            esc_arbiter_init(&st, &ac);
            can.fresh = true; can.valid = true; can.throttle = 0.3f; can.arm_req = true;
            esc_arbiter_step(&st, &can, &(esc_src_sample_t){0}, dt, &r);
            uint32_t s0 = r.cmd.seq;
            /* hold: no fresh CAN for several ticks (still within can_stale_s) */
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r);
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r);
            CHECK(r.have_cmd);
            CHECK(r.cmd.seq == s0);                     /* seq HELD while holding */
            CHECK_NEAR(r.cmd.throttle, 0.3f, 1e-4f);    /* last value held */
            /* a fresh frame bumps seq */
            esc_arbiter_step(&st, &can, &(esc_src_sample_t){0}, dt, &r);
            CHECK(r.cmd.seq != s0);
        }

        /* ARM-GATE RESET: PWM arms, loses signal (disarms), and must dwell low AGAIN
         * before it can re-arm -- a brief recovered pulse must not instantly re-arm. */
        {
            esc_arbiter_state_t st;
            ac.policy = ESC_ARB_EXPLICIT_PWM;
            esc_arbiter_init(&st, &ac);
            esc_src_sample_t plo = {0};
            plo.fresh = true; plo.valid = true; plo.throttle = 0.0f;
            int i;
            for (i = 0; i < 600; i++) { esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r); }
            CHECK(r.have_cmd);                          /* armed */
            /* signal lost long enough to go unhealthy (> pwm_stale_s) */
            for (i = 0; i < 100; i++) { esc_arbiter_step(&st, &(esc_src_sample_t){0}, &(esc_src_sample_t){0}, dt, &r); }
            CHECK(!r.have_cmd);                         /* disarmed by loss */
            /* one fresh low pulse: must NOT re-arm yet (dwell restarts) */
            esc_arbiter_step(&st, &(esc_src_sample_t){0}, &plo, dt, &r);
            CHECK(!r.have_cmd);
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `bash tools/test/run.sh`
Expected: `BUILD FAIL test_esc_arbiter` (undefined `esc_arbiter_init` / `esc_arbiter_step`).

- [ ] **Step 3: Implement init + the EXPLICIT step path** — append to `src/app/esc_arbiter.c`

```c
static void src_update(esc_src_state_t *s, const esc_src_sample_t *in,
                       float stale_s, float dt)
{
    if (in->fresh && in->valid) {
        s->seen    = true;            /* cold-boot guard: only a real frame makes us healthy */
        s->age_s   = 0.0f;
        s->throttle = clamp01(in->throttle);
        s->arm_req = in->arm_req;
    } else {
        s->age_s += dt;
    }
    /* NEVER healthy before the first real sample, else a zeroed age_s would make a
     * never-connected source look fresh for one stale window at boot (phantom command). */
    s->healthy = (s->seen && (s->age_s <= stale_s));
    if (!s->healthy) {
        s->throttle = 0.0f;
        s->arm_req  = false;
    }
}

static void pwm_update_arm(esc_src_state_t *p, const esc_arbiter_cfg_t *c, float dt)
{
    if (!p->healthy) {
        p->low_dwell_s = 0.0f;
        p->armed = false;            /* signal loss disarms */
        return;
    }
    if (p->throttle <= c->pwm_arm_throttle_eps) {
        p->low_dwell_s += dt;
        if (p->low_dwell_s >= c->pwm_arm_low_s) { p->armed = true; }
    } else if (!p->armed) {
        p->low_dwell_s = 0.0f;       /* must see sustained low before the FIRST arm */
    }
}

void esc_arbiter_init(esc_arbiter_state_t *st, const esc_arbiter_cfg_t *cfg)
{
    /* Word-wise zero to match the repo's pure-C idiom (product_build_esc_cfg); C28x has
     * no 8-bit storage, so do NOT iterate uint8_t bytes. sizeof of these structs is a
     * whole number of 16-bit words (all fields are >= 16-bit aligned). */
    uint16_t i;
    uint16_t *p = (uint16_t *)st;
    for (i = 0; i < (sizeof(*st) / sizeof(uint16_t)); i++) { p[i] = 0u; }
    st->cfg    = *cfg;
    st->active = ESC_SRC_NONE;
}

/* Emit the selected candidate with handoff slew. Shared by all policies.
 * fresh == the selected source delivered a fresh+valid sample THIS tick; out_seq is
 * bumped only then (see the SEQ CONTRACT on esc_arbiter_state_t.out_seq). */
static void emit(esc_arbiter_state_t *st, esc_src_id_t cand,
                 float target, bool arm, bool fresh, float dt, esc_arbiter_result_t *res)
{
    uint32_t bits = ESC_ARB_ST_NONE;

    if (cand != st->active && cand != ESC_SRC_NONE && st->active != ESC_SRC_NONE) {
        st->blending = true;         /* rate-limit only across a real source change */
    }
    if (cand == ESC_SRC_NONE) {
        st->out_throttle = 0.0f;
        st->blending = false;
    } else if (st->blending) {
        float step = st->cfg.handoff_slew_per_s * dt;
        if (target > st->out_throttle + step)      { st->out_throttle += step; }
        else if (target < st->out_throttle - step) { st->out_throttle -= step; }
        else { st->out_throttle = target; st->blending = false; }
        bits |= ESC_ARB_ST_HANDOFF;
    } else {
        st->out_throttle = target;
    }
    st->active = cand;

    /* Always fully initialize res->cmd (the product main copies it unconditionally), even
     * on NONE where it will not be consumed -- no indeterminate fields in safety code. */
    if (cand != ESC_SRC_NONE && fresh) { st->out_seq++; } /* held -> same seq -> esc_control ages */
    res->cmd.seq      = st->out_seq;
    res->cmd.throttle = (cand == ESC_SRC_NONE) ? 0.0f : st->out_throttle;
    res->cmd.arm      = (cand == ESC_SRC_NONE) ? false : arm;
    res->active       = cand;
    if (cand == ESC_SRC_NONE) {
        res->have_cmd = false;
        bits |= ESC_ARB_ST_NO_SOURCE;
    } else {
        res->have_cmd = true;
        if (cand == ESC_SRC_PWM) { bits |= ESC_ARB_ST_PWM_ACTIVE; }
    }
    res->status_bits = bits;
}

void esc_arbiter_step(esc_arbiter_state_t *st,
                      const esc_src_sample_t *can, const esc_src_sample_t *pwm,
                      float dt_s, esc_arbiter_result_t *res)
{
    const esc_arbiter_cfg_t *c = &st->cfg;
    esc_src_id_t cand = ESC_SRC_NONE;
    float target = 0.0f;
    bool  arm = false;

    src_update(&st->can, can, c->can_stale_s, dt_s);
    src_update(&st->pwm, pwm, c->pwm_stale_s, dt_s);
    pwm_update_arm(&st->pwm, c, dt_s);

    switch (c->policy) {
    case ESC_ARB_EXPLICIT_CAN:
        if (st->can.healthy) { cand = ESC_SRC_CAN; }
        break;
    case ESC_ARB_EXPLICIT_PWM:
        if (st->pwm.healthy && st->pwm.armed) { cand = ESC_SRC_PWM; }
        break;
    case ESC_ARB_CAN_PRIMARY:
    default:
        /* CAN_PRIMARY is completed in Task 3; treat as CAN-only until then. */
        if (st->can.healthy) { cand = ESC_SRC_CAN; }
        break;
    }

    if (cand == ESC_SRC_CAN)      { target = st->can.throttle; arm = st->can.arm_req; }
    else if (cand == ESC_SRC_PWM) { target = st->pwm.throttle; arm = st->pwm.armed; }

    {
        /* Fresh ONLY if the winning source delivered a new valid sample this tick. The
         * held case (healthy source, no new frame) re-emits the same seq so esc_control's
         * watchdog ages from the last real frame -- see the SEQ CONTRACT. */
        bool cand_fresh =
            (cand == ESC_SRC_CAN && can->fresh && can->valid) ||
            (cand == ESC_SRC_PWM && pwm->fresh && pwm->valid);
        emit(st, cand, target, arm, cand_fresh, dt_s, res);
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `bash tools/test/run.sh`
Expected: `PASS test_esc_arbiter` and all existing tests still pass.

- [ ] **Step 5: Commit**

```bash
git add src/app/esc_arbiter.c tools/test/test_esc_arbiter.c
git commit -m "feat(arbiter): source health, PWM low-dwell arm gate, explicit policies"
```

---

### Task 3: CAN_PRIMARY policy — tracking lockout, fallback, handoff

**Files:**
- Modify: `src/app/esc_arbiter.c` (add tracking dwell + CAN_PRIMARY selection)
- Test: `tools/test/test_esc_arbiter.c` (append cases)

**Interfaces:**
- Consumes: everything from Task 2.
- Produces: full `ESC_ARB_CAN_PRIMARY` behavior — PWM may take over only if it was healthy, armed, and tracking CAN within `track_tol` for `track_required_s` before CAN was lost.

- [ ] **Step 1: Write the failing tests** — append before `CHECK_DONE();`

```c
    /* ---- Task 3: CAN_PRIMARY fallback ---- */
    {
        esc_arbiter_cfg_t ac = {0};
        ac.policy = ESC_ARB_CAN_PRIMARY;
        ac.can_stale_s = 0.10f;  ac.pwm_stale_s = 0.06f;
        ac.pwm_arm_low_s = 0.20f; ac.pwm_arm_throttle_eps = 0.05f;
        ac.track_tol = 0.10f;    ac.track_required_s = 0.30f;
        ac.handoff_slew_per_s = 4.0f;
        ac.pwm = pwm_cfg();
        const float dt = 0.001f;
        esc_arbiter_result_t r;

        /* Helper sequences are inline below. */

        /* (a) CAN dies, PWM was armed + tracking -> fallback to PWM. */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            /* phase 1: both healthy, PWM arms low then both ramp together (tracking) */
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); } /* arm dwell */
            can.throttle = 0.50f; pwm.throttle = 0.48f;     /* agree within tol */
            for (i = 0; i < 400; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); } /* track dwell */
            CHECK(r.active == ESC_SRC_CAN);                 /* CAN still primary */
            /* phase 2: CAN stops, PWM continues */
            esc_src_sample_t cannone = {0};
            bool fellback = false;
            for (i = 0; i < 200; i++) {
                esc_arbiter_step(&st, &cannone, &pwm, dt, &r);
                if (r.active == ESC_SRC_PWM) { fellback = true; break; }
            }
            CHECK(fellback);
            CHECK(r.have_cmd);
            CHECK(r.status_bits & ESC_ARB_ST_PWM_ACTIVE);
        }

        /* (b) Stuck-mid PWM that NEVER tracked CAN -> no fallback when CAN dies. */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;   /* low so it can arm */
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); } /* arm */
            can.throttle = 0.60f; pwm.throttle = 0.20f;          /* diverge -> never tracks */
            for (i = 0; i < 500; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            CHECK(r.status_bits & ESC_ARB_ST_PWM_LOCKOUT);       /* armed but ineligible */
            esc_src_sample_t cannone = {0};
            bool fellback = false;
            for (i = 0; i < 300; i++) {
                esc_arbiter_step(&st, &cannone, &pwm, dt, &r);
                if (r.active == ESC_SRC_PWM) { fellback = true; break; }
            }
            CHECK(!fellback);                                    /* locked out */
            CHECK(!r.have_cmd);                                  /* both effectively gone */
        }

        /* (c) Both die AFTER being armed and running -> no command (the dangerous case). */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            can.throttle = 0.40f; pwm.throttle = 0.40f;          /* armed + running, tracking */
            for (i = 0; i < 400; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            CHECK(r.have_cmd && r.active == ESC_SRC_CAN);
            esc_src_sample_t none = {0};
            bool coasted = false;
            for (i = 0; i < 300; i++) {
                esc_arbiter_step(&st, &none, &none, dt, &r);
                if (!r.have_cmd) { coasted = true; break; }
            }
            CHECK(coasted);
            CHECK(r.active == ESC_SRC_NONE);
        }

        /* (f) PWM->CAN recovery: after a CAN-loss fallback, CAN returning is preferred
         *     again, with the handoff blend bounding the step back. */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            can.throttle = 0.50f; pwm.throttle = 0.45f;
            for (i = 0; i < 400; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            /* CAN drops -> fall to PWM */
            esc_src_sample_t cannone = {0};
            for (i = 0; i < 200 && r.active != ESC_SRC_PWM; i++) {
                esc_arbiter_step(&st, &cannone, &pwm, dt, &r);
            }
            CHECK(r.active == ESC_SRC_PWM);
            /* CAN returns -> must reclaim immediately (it is primary when healthy) */
            can.throttle = 0.50f;
            esc_arbiter_step(&st, &can, &pwm, dt, &r);
            CHECK(r.active == ESC_SRC_CAN);
        }

        /* (g) track_ok clears when PWM goes stale, ISOLATED from the arm gate: after the
         *     PWM drops out (clearing both track_ok and armed), it returns at idle-low and
         *     RE-ARMS, but with CAN dead it cannot re-track. So armed==true while
         *     track_ok==false -> the lockout (NOT the arm gate) is the sole blocker. This
         *     catches a regression where track_ok fails to clear on dropout. */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            can.throttle = 0.50f; pwm.throttle = 0.50f;
            for (i = 0; i < 400; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); } /* tracked */
            CHECK(st.pwm.track_ok);                        /* precondition: was eligible */
            /* PWM drops out long enough to go unhealthy -> track_ok AND armed clear */
            for (i = 0; i < 100; i++) { esc_arbiter_step(&st, &can, &(esc_src_sample_t){0}, dt, &r); }
            CHECK(!st.pwm.track_ok);
            CHECK(!st.pwm.armed);
            /* CAN dies; PWM returns at idle-low and re-arms (dwell), but cannot re-track */
            esc_src_sample_t cannone = {0};
            esc_src_sample_t plo = {0}; plo.fresh = plo.valid = true; plo.throttle = 0.0f;
            bool fellback = false;
            for (i = 0; i < 400; i++) {
                esc_arbiter_step(&st, &cannone, &plo, dt, &r);
                if (r.active == ESC_SRC_PWM) { fellback = true; break; }
            }
            CHECK(!fellback);                              /* never falls back */
            CHECK(st.pwm.armed);                           /* arm gate satisfied again ... */
            CHECK(!st.pwm.track_ok);                       /* ... so track_ok is the SOLE blocker */
            CHECK(r.status_bits & ESC_ARB_ST_PWM_LOCKOUT);
        }

        /* (e) Handoff slew: on a CAN->PWM switch the output ramps to PWM's value
         *     (0.08 below CAN, within track_tol) without stepping > slew*dt per tick. */
        {
            esc_arbiter_state_t st; esc_arbiter_init(&st, &ac);
            esc_src_sample_t can = {0}, pwm = {0};
            int i;
            pwm.fresh = pwm.valid = true; pwm.throttle = 0.0f;
            can.fresh = can.valid = true; can.throttle = 0.0f; can.arm_req = true;
            for (i = 0; i < 300; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            can.throttle = 0.50f; pwm.throttle = 0.42f;          /* track within 0.10 tol */
            for (i = 0; i < 400; i++) { esc_arbiter_step(&st, &can, &pwm, dt, &r); }
            float last = r.cmd.throttle;                         /* ~0.50 on CAN */
            CHECK_NEAR(last, 0.50f, 1e-4f);
            esc_src_sample_t cannone = {0};
            bool reached = false;
            for (i = 0; i < 200; i++) {                          /* CAN gone -> fall to PWM 0.42 */
                esc_arbiter_step(&st, &cannone, &pwm, dt, &r);
                if (r.have_cmd) {
                    float jump = r.cmd.throttle - last;          /* may be negative */
                    if (jump < 0.0f) { jump = -jump; }
                    CHECK(jump <= ac.handoff_slew_per_s * dt + 1e-5f);
                    last = r.cmd.throttle;
                    if (fabsf(last - 0.42f) < 1e-3f) { reached = true; }
                }
            }
            CHECK(reached);                                      /* slewed all the way to PWM */
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `bash tools/test/run.sh`
Expected: `FAIL test_esc_arbiter` — case (b) `ESC_ARB_ST_PWM_LOCKOUT` is never set and case (a) does not fall back (CAN_PRIMARY still behaves CAN-only from Task 2's placeholder).

- [ ] **Step 3: Add tracking dwell + real CAN_PRIMARY selection** — in `src/app/esc_arbiter.c`

Add this helper above `esc_arbiter_step`:

```c
static void pwm_update_track(esc_src_state_t *p, const esc_src_state_t *can,
                             const esc_arbiter_cfg_t *c, float dt)
{
    if (!p->healthy) {
        p->track_dwell_s = 0.0f;
        p->track_ok = false;          /* lost PWM -> ineligible */
        return;
    }
    if (can->healthy) {
        float d = p->throttle - can->throttle;
        if (d < 0.0f) { d = -d; }
        if (d <= c->track_tol) {
            p->track_dwell_s += dt;
            if (p->track_dwell_s >= c->track_required_s) { p->track_ok = true; }
        } else {
            p->track_dwell_s = 0.0f;
            p->track_ok = false;       /* diverged while CAN healthy -> lock out */
        }
    }
    /* CAN unhealthy but PWM healthy: HOLD track_ok latched so a previously-tracking
     * PWM stays eligible to take over. */
}
```

Call it in `esc_arbiter_step` right after `pwm_update_arm(...)`:

```c
    pwm_update_track(&st->pwm, &st->can, c, dt_s);
```

Replace the `ESC_ARB_CAN_PRIMARY` switch arm with the real policy:

```c
    case ESC_ARB_CAN_PRIMARY:
        if (st->can.healthy) {
            cand = ESC_SRC_CAN;
        } else if (st->pwm.healthy && st->pwm.armed && st->pwm.track_ok) {
            cand = ESC_SRC_PWM;
        }
        break;
```

Finally, surface the lockout reason. The `emit(...)` call (now inside the `cand_fresh` block from Task 2) writes `res->status_bits`, so OR the lockout bit in **after** that block returns. Replace the whole closing `cand_fresh` block from Task 2 with this version that appends the lockout bit:

```c
    {
        bool cand_fresh =
            (cand == ESC_SRC_CAN && can->fresh && can->valid) ||
            (cand == ESC_SRC_PWM && pwm->fresh && pwm->valid);
        emit(st, cand, target, arm, cand_fresh, dt_s, res);

        /* Report a PWM that is healthy+armed but not eligible to take over (would-be
         * fallback denied because it never tracked CAN). Only meaningful under CAN_PRIMARY:
         * under EXPLICIT_PWM tracking is not a gate, so this bit would mislead at bench. */
        if (c->policy == ESC_ARB_CAN_PRIMARY &&
            st->pwm.healthy && st->pwm.armed && !st->pwm.track_ok) {
            res->status_bits |= (uint32_t)ESC_ARB_ST_PWM_LOCKOUT;
        }
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `bash tools/test/run.sh`
Expected: `PASS test_esc_arbiter` and all existing tests still pass.

- [ ] **Step 5: Commit**

```bash
git add src/app/esc_arbiter.c tools/test/test_esc_arbiter.c
git commit -m "feat(arbiter): CAN_PRIMARY tracked-fallback with stuck-PWM lockout + handoff slew"
```

---

### Task 4: Board RC-PWM raw read

**Files:**
- Modify: `boards/esc6288_revA/drivers/include/rc_pwm.h`
- Modify: `boards/esc6288_revA/drivers/source/rc_pwm.c`

**Interfaces:**
- Consumes: existing eCAP setup in `RC_PWM_init()`.
- Produces:
  - `typedef struct { bool fresh; bool overflow; float width_us; } rc_pwm_sample_t;`
  - `void RC_PWM_read(rc_pwm_sample_t *out);` — one non-blocking eCAP poll, raw measurement only (no window logic; the pure `esc_pwm_decode` interprets it).

This task has no host test (board driverlib glue); it is verified by the on-target compile.

- [ ] **Step 1: Add the sample type + prototype** — `boards/esc6288_revA/drivers/include/rc_pwm.h`

Replace the existing `RC_PWM_getThrottle` declaration block with:

```c
#include <stdbool.h>

//! Raw RC-PWM capture for the arbiter's pure decode (esc_pwm_decode()).
typedef struct {
    bool  fresh;     //!< a new falling edge was captured since the last read
    bool  overflow;  //!< eCAP counter overflow (line stalled high/low -> lost signal)
    float width_us;  //!< last captured high-time in microseconds (valid only if fresh)
} rc_pwm_sample_t;

//! \brief Non-blocking eCAP1 poll. Reports fresh/overflow and the raw high-time in us.
//!        No range logic here -- esc_pwm_decode() owns the valid window + normalization.
void RC_PWM_read(rc_pwm_sample_t *out);
```

Keep `RC_PWM_init()` and `RC_PWM_getPulseWidth_counts()` as-is. The old float `RC_PWM_getThrottle()` may stay (still unused) or be removed; removing it is cleaner — delete its declaration here and its definition in the `.c`.

- [ ] **Step 2: Implement `RC_PWM_read`** — `boards/esc6288_revA/drivers/source/rc_pwm.c`

Replace the body of `RC_PWM_getThrottle()` (lines 57-89) with the new function (delete `RC_PWM_getThrottle`):

```c
void RC_PWM_read(rc_pwm_sample_t *out)
{
    uint32_t flags = ECAP_getInterruptSource(BOARD_RCPWM_ECAP_BASE);

    out->fresh    = false;
    out->overflow = false;
    out->width_us = 0.0f;

    /* Counter overflow => no edges for ~43 s of counts: the line stalled (lost signal). */
    if ((flags & ECAP_ISR_SOURCE_COUNTER_OVERFLOW) != 0U)
    {
        ECAP_clearInterrupt(BOARD_RCPWM_ECAP_BASE, RCPWM_ECAP_ALL_FLAGS);
        out->overflow = true;
        return;
    }

    /* Require a fresh falling edge (CEVT2) since the last successful read. */
    if ((flags & ECAP_ISR_SOURCE_CAPTURE_EVENT_2) == 0U)
    {
        return;   /* fresh=false: no new pulse */
    }

    {
        uint32_t cap2 = ECAP_getEventTimeStamp(BOARD_RCPWM_ECAP_BASE, ECAP_EVENT_2);
        ECAP_clearInterrupt(BOARD_RCPWM_ECAP_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
        out->fresh    = true;
        out->width_us = (float)cap2 / RCPWM_COUNTS_PER_US;
    }
}
```

The `RCPWM_MIN_US` / `RCPWM_MAX_US` / `RCPWM_VALID_*` macros are now unused in the `.c`; delete them (they live in `esc_arbiter_cfg` on the pure side) to keep `-Wall` clean for the board build. Keep `RCPWM_ECAP_ALL_FLAGS` and `RCPWM_COUNTS_PER_US` — `RC_PWM_read` still uses both.

Also update the now-stale init comment in `product/product_main.c` (~line 303) that lists `RC_PWM_getThrottle` as a live read API: change it to reference `RC_PWM_read` (the function it is replaced by). This is a comment-only edit, but leaving it would point an implementer at a deleted symbol.

- [ ] **Step 3: Verify the on-target compile (product link)**

Run: `BOARD=esc6288_revA MOTOR=am_4116_kv450 PRODUCT=1 bash build.sh`
Expected: build succeeds (board sources, including `rc_pwm.c`, compile and link). At this point `RC_PWM_read` is defined but not yet called — that is wired in Task 5.

- [ ] **Step 4: Commit**

```bash
git add boards/esc6288_revA/drivers/include/rc_pwm.h boards/esc6288_revA/drivers/source/rc_pwm.c
git commit -m "feat(esc6288 rc_pwm): expose raw RC_PWM_read sample for the arbiter"
```

---

### Task 5: Wire the arbiter into the product tick + observability

**Files:**
- Modify: `src/common/esc_types.h` (one reporting status bit)
- Modify: `product/product_main.c` (state, cfg builder, init, tick)

**Interfaces:**
- Consumes: `esc_arbiter_*` (Tasks 1-3), `RC_PWM_read` / `rc_pwm_sample_t` (Task 4).
- Produces: the live dual-source path. Ships with `policy = ESC_ARB_EXPLICIT_CAN`, so runtime behavior is identical to today.

- [ ] **Step 1: Add a reporting status bit** — `src/common/esc_types.h`

In `esc_status_t` (lines 50-57), add one bit:

```c
    ESC_ST_FAILSAFE_BRAKE     = 1u << 4,  /* link-loss failsafe, active brake */
    ESC_ST_SRC_PWM            = 1u << 5    /* RC-PWM is the active throttle source */
```

(Note the comma added after the `1u << 4` line.)

- [ ] **Step 2: Add arbiter includes + state** — `product/product_main.c`

Near the other `src/app` includes, add:

```c
#include "esc_arbiter.h"
```

Near the `static esc_command_t g_cmd; static bool g_have_cmd;` declarations (lines 142-143), add:

```c
static esc_arbiter_state_t g_arb;
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
static volatile esc_src_id_t g_arb_active;      /* debugger-visible: who owns throttle */
static volatile uint32_t     g_arb_status_bits; /* debugger-visible: esc_arb_status_t bits
                                                 * (PWM_ACTIVE / NO_SOURCE / PWM_LOCKOUT /
                                                 * HANDOFF) -- bench step 5/6 watch these,
                                                 * since esc_telemetry_t only carries SRC_PWM. */
#endif
```

- [ ] **Step 3: Build the arbiter config** — `product/product_main.c`

Add a builder next to `product_build_esc_cfg`:

```c
static void product_build_arb_cfg(esc_arbiter_cfg_t *a)
{
    uint16_t i;
    uint16_t *p = (uint16_t *)a;
    for (i = 0; i < (sizeof(*a) / sizeof(uint16_t)); i++) { p[i] = 0u; }

    /* SAFE DEFAULT: CAN only. RC-PWM is ignored until the bench gate flips this to
     * ESC_ARB_CAN_PRIMARY (see docs plan Appendix). Shipping behavior == CAN-only. */
    a->policy              = ESC_ARB_EXPLICIT_CAN;
    a->can_stale_s         = 0.10f;   /* ~10 missed 100 Hz RawCommands */
    a->pwm_stale_s         = 0.06f;   /* ~3 missed 50 Hz servo pulses  */
    a->pwm_arm_low_s       = 0.50f;   /* sustained idle before PWM may arm */
    a->pwm_arm_throttle_eps = 0.05f;
    a->track_tol           = 0.10f;   /* PWM within 10% of CAN counts as tracking */
    a->track_required_s    = 0.50f;
    a->handoff_slew_per_s  = 4.0f;    /* full-scale handoff over ~0.25 s */
    a->pwm.us_min = 1000.0f; a->pwm.us_max = 2000.0f;
    a->pwm.us_valid_min = 900.0f; a->pwm.us_valid_max = 2100.0f;
}
```

In `product_init()`, after `esc_control_init(...)` (line 274), add:

```c
    {
        esc_arbiter_cfg_t acfg;
        product_build_arb_cfg(&acfg);
        esc_arbiter_init(&g_arb, &acfg);
    }
```

- [ ] **Step 4: Replace the CAN-only drain with the arbiter** — `product/product_main.c`

In `product_tick_1ms()`, replace the block at lines 403-412 (the `while(can_bridge_read...)` drain that sets `g_cmd`/`g_have_cmd`) with:

```c
    // 1) drain CAN frames -> build the CAN source sample (last command addressed to us)
    {
        esc_src_sample_t can_s = {0};
        esc_src_sample_t pwm_s = {0};
        esc_arbiter_result_t ar;

        while (can_bridge_read(&rxf))
        {
            dronecan_on_rx(&g_dn, &rxf, &rr);
            if (rr.command_updated)
            {
                can_s.fresh    = true;
                can_s.valid    = true;
                can_s.throttle = rr.command.throttle;
                can_s.arm_req  = rr.command.arm;
            }
        }

#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
        // RC-PWM source: poll eCAP1, decode the raw high-time to a normalized sample.
        {
            rc_pwm_sample_t raw;
            RC_PWM_read(&raw);
            pwm_s = esc_pwm_decode(&g_arb.cfg.pwm, raw.fresh, raw.overflow, raw.width_us);
        }
#endif

        esc_arbiter_step(&g_arb, &can_s, &pwm_s, 0.001f, &ar);
        g_cmd      = ar.cmd;          // valid only if ar.have_cmd
        g_have_cmd = ar.have_cmd;
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
        g_arb_active      = ar.active;       // debugger / future telemetry
        g_arb_status_bits = ar.status_bits;  // PWM_ACTIVE / NO_SOURCE / PWM_LOCKOUT / HANDOFF
#endif
    }
```

The `esc_control_step()` call at line 456 already does `g_have_cmd ? &g_cmd : NULL`, so when the arbiter reports no source it cleanly passes NULL and the existing 0.5 s failsafe coasts the stage. No change to that call itself — but **delete the now-dead `g_have_cmd = false;` line immediately after it (line 457)**: the arbiter rebuilds `g_have_cmd` from `ar.have_cmd` at the top of every tick, so clearing it here is dead and misleading (it would imply `have_cmd` is only valid for one tick, hiding that the arbiter passes a held non-NULL command every tick).

- [ ] **Step 5: Report the active source in telemetry** — `product/product_main.c`

After the `esc_control_step(...)` call (line 456-457), add:

```c
#if (BUILD_BOARD_ID == BUILD_BOARD_ID_ESC6288_REVA)
    if (g_arb_active == ESC_SRC_PWM) { tel.status_bits |= (uint32_t)ESC_ST_SRC_PWM; }
#endif
```

**Observability note (do not over-claim this):** `tel` is a stack-local in `product_tick_1ms`, and the current DroneCAN `esc.Status` serializer (`build_esc_status()` in `src/comms/dronecan.c`, ~line 199) packs only voltage/current/temp/rpm/index — it does **not** put `status_bits` on the wire. So `ESC_ST_SRC_PWM` set here is **not yet externally observable**; it is a forward hook for when `status_bits` serialization is added. For the bench gate, source/lockout/handoff are observed via the debugger globals `g_arb_active` / `g_arb_status_bits` (Step 2), not over CAN. Wiring `status_bits` into the `esc.Status` frame is a separate, out-of-scope change.

- [ ] **Step 6: Verify host tests still pass (no `src/` regression)**

Run: `bash tools/test/run.sh`
Expected: all tests PASS (the `esc_types.h` bit + arbiter are pure; nothing else in `src/` changed).

- [ ] **Step 7: Verify the product source compiles**

Run: `BOARD=esc6288_revA MOTOR=am_4116_kv450 SRC_CHECK=1 bash build.sh`
Expected: PASS. If the build uses an explicit source list rather than a `src/app/*.c` glob, add `src/app/esc_arbiter.c` to that list in `build.sh` and re-run.

- [ ] **Step 8: Verify the product-glue zero-warning gate**

Run: `BOARD=esc6288_revA MOTOR=am_4116_kv450 PRODUCT_CHECK=1 bash build.sh`
Expected: PASS. `PRODUCT_CHECK` compile-checks the SDK-coupled product glue (`product_main.c` + `foc_bridge.c`) with warnings-as-errors; since this task edits the `product_main.c` tick path, run this gate explicitly (it is narrower than `PRODUCT=1` and catches product-glue warnings the link step may tolerate). Note: `PRODUCT_CHECK` does NOT compile `rc_pwm.c` or `esc_arbiter.c` — those are covered by Step 7 (`SRC_CHECK`) and Step 9 (`PRODUCT=1`).

- [ ] **Step 9: Verify the full product link**

Run: `BOARD=esc6288_revA MOTOR=am_4116_kv450 PRODUCT=1 bash build.sh`
Expected: PASS (arbiter + `RC_PWM_read` linked into the image).

- [ ] **Step 10: Verify the SDK-lab regression (other boards still build)**

Run: `BOARD=esc6288_revA LAB=all bash build.sh`
Expected: PASS.

- [ ] **Step 11: Commit**

```bash
git add src/common/esc_types.h product/product_main.c
git commit -m "feat(product): wire CAN+RC-PWM arbiter into the 1ms tick (CAN-only default)"
```

---

## Appendix: Bench validation gate (NOT a code task — required before enabling fallback)

The board is still at fab (`boards/esc6288_revA/PORT_TODO.md`). The shipped policy is `ESC_ARB_EXPLICIT_CAN`, so the arbiter is inert. **Do NOT change `a->policy` to `ESC_ARB_CAN_PRIMARY` until every item below passes, current-limited, with NO prop first and the prop last.** Watch in the debugger: `IdqSet_A`, `flagRunIdentAndOnLine`, `g_arb_active`, `g_arb_status_bits` (the arbiter bits — `status_bits` is not yet serialized over CAN, so read it here, not in `esc.Status`), and `EPwm{1,2,3}Regs.TZFLG.OST`.

1. Complete the existing pre-spin bring-up in `PORT_TODO.md` (CAN 1 Mbit DroneCAN node, MT6701, NTC, gate stage safe-off, OST invariant).
2. `EXPLICIT_PWM`: drive 900/1000/1500/2000/2100 µs and confirm `esc_pwm_decode` throttle + valid window; confirm no-signal (line high/low) → `overflow`/`!valid` → no command.
3. Receiver failsafe: pull the RC link and confirm the receiver outputs idle-low or out-of-range (and the ESC coasts), not a stuck mid pulse.
4. PWM arm gate: confirm the motor will not arm from PWM until ≥0.5 s of valid idle-low pulses; confirm a stuck-high or stuck-mid line at boot never arms.
5. `CAN_PRIMARY` tracking: with both sources live and PWM stick mirroring CAN, confirm `g_arb_active == ESC_SRC_CAN` and that PWM becomes eligible (no `ESC_ARB_ST_PWM_LOCKOUT`) only after ≥0.5 s of agreement.
6. Conflict lockout: command CAN and PWM to disagree by > `track_tol`; confirm PWM stays locked out and CAN wins.
7. CAN-loss fallback: with PWM tracking, kill CAN; confirm clean fallback to PWM within `can_stale_s`, with the handoff slew bounding the throttle step.
8. Stuck-PWM after fallback: while on PWM fallback, freeze the PWM line; confirm the arbiter deselects PWM within `pwm_stale_s` (`g_arb_active` → NONE, `g_arb_status_bits` → `NO_SOURCE`) and the motor then coasts within `cmd_timeout_s` of the last valid pulse (the `esc_control` watchdog, held throttle until then — verify this ride-through hold is acceptable for a spinning prop, and tighten `cmd_timeout_s` if not).
9. Both-dead: kill both; confirm coast via the existing `esc_control` failsafe within `cmd_timeout_s` of the last valid frame of whichever source was active.
10. Only after 2-9 pass with a current limit and no prop: repeat the critical cases with a prop, then raise `iq_max_A` per the normal bench schedule.

**Tuning follow-up:** under the SEQ CONTRACT the arbiter ages `esc_control` from the last real frame (held ticks re-emit the same seq), so total-loss coast latency is **~`cmd_timeout_s` (≈0.5 s)**, NOT `stale_s + cmd_timeout_s` — the `*_stale_s` window only governs how fast the arbiter switches/deselects sources, it does not add to the coast time. If ≈0.5 s is too slow for flight, lower `esc_control` `cmd_timeout_s` in `product_build_esc_cfg`. That is a separate, tested change — out of scope here.
