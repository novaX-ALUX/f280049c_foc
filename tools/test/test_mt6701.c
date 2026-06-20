#include "check.h"
#include "mt6701.h"

static mt6701_cfg_t base_cfg(void)
{
    mt6701_cfg_t c = {0};
    c.counts_per_rev      = 16384.0f;
    c.dir                 = 1;
    c.zero_offset_counts  = 0;
    c.pole_pairs          = 7;
    c.vel_iir_alpha       = 1.0f;   /* no smoothing for deterministic tests */
    c.max_delta_rev       = 0.25f;
    c.stale_limit_samples = 3;
    c.glitch_stale_samples= 3;
    return c;
}

int main(void)
{
    /* First valid frame -> baseline position, velocity exactly 0, no glitch. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 4096, true, 0.001f); /* 0.25 rev */
        CHECK(mt6701_valid(&s));
        CHECK_NEAR(mt6701_mech_rev(&s), 0.25f, 1e-4f);
        CHECK_NEAR(mt6701_vel_revps(&s), 0.0f, 1e-6f);
        CHECK(!mt6701_stale(&s));
    }

    /* Velocity sign/magnitude: +0.01 rev in 0.001 s = +10 rev/s. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        mt6701_update(&s, 164, true, 0.001f); /* ~0.01 rev */
        CHECK(mt6701_vel_revps(&s) > 0.0f);
        CHECK_NEAR(mt6701_vel_revps(&s), 10.0f, 0.5f);
    }
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 164, true, 0.001f);
        mt6701_update(&s, 0, true, 0.001f);   /* moving negative */
        CHECK(mt6701_vel_revps(&s) < 0.0f);
    }

    /* Wrap across 0 is NOT a glitch: 0.98 rev -> 0.01 rev is +0.03 rev forward. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 16056, true, 0.001f); /* ~0.98 rev */
        uint32_t g0 = s.glitch_count;
        mt6701_update(&s, 164, true, 0.001f);   /* ~0.01 rev */
        CHECK(s.glitch_count == g0);            /* no glitch counted */
        CHECK(mt6701_vel_revps(&s) > 0.0f);     /* forward, not a huge negative jump */
        CHECK_NEAR(mt6701_vel_revps(&s), 30.0f, 5.0f);
    }

    /* Multi-turn unwrap: three forward turns accumulate ~ +3 rev.
     * Step in 0.1-rev increments so each delta stays under the 0.25-rev glitch threshold. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        for (int turn = 0; turn < 3; ++turn) {
            for (int k = 1; k <= 10; ++k) {
                float p = (float)k * 0.1f;            /* 0.1 .. 1.0 (1.0 wraps to 0) */
                p -= floorf(p);
                uint16_t raw = (uint16_t)lroundf(p * 16384.0f);
                mt6701_update(&s, raw, true, 0.001f);
            }
        }
        CHECK_NEAR(mt6701_multiturn_rev(&s), 3.0f, 0.01f);
    }

    /* Glitch: single big jump is dropped and does NOT immediately go stale. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        mt6701_update(&s, 8192, true, 0.001f); /* +0.5 rev > 0.25 threshold -> glitch */
        CHECK(s.glitch_count == 1);
        CHECK(!mt6701_stale(&s));               /* single glitch is not stale */
        CHECK_NEAR(mt6701_mech_rev(&s), 0.0f, 1e-4f); /* position not updated */
    }

    /* Consecutive glitches -> stale after glitch_stale_samples. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);    /* accepted, position = 0 */
        /* Glitches don't update position, so each compares against 0; all are > 0.25 rev. */
        mt6701_update(&s, 8192, true, 0.001f); /* glitch run 1 */
        mt6701_update(&s, 8000, true, 0.001f); /* glitch run 2 */
        mt6701_update(&s, 8200, true, 0.001f); /* glitch run 3 -> stale */
        CHECK(mt6701_stale(&s));
    }

    /* raw > 0x3FFF treated as invalid; consecutive invalids -> stale. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        mt6701_update(&s, 0x4000, true, 0.001f); /* out of range -> invalid */
        CHECK(!mt6701_valid(&s));
        mt6701_update(&s, 0x5000, true, 0.001f);
        mt6701_update(&s, 0xFFFF, true, 0.001f);
        CHECK(mt6701_stale(&s));                 /* 3 consecutive invalid */
    }

    /* raw_valid=false also counts as invalid -> stale. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        mt6701_update(&s, 100, false, 0.001f);
        mt6701_update(&s, 100, false, 0.001f);
        mt6701_update(&s, 100, false, 0.001f);
        CHECK(mt6701_stale(&s));
    }

    /* dt_s <= 0 must not divide-by-zero; velocity held. */
    {
        mt6701_cfg_t c = base_cfg();
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 0, true, 0.001f);
        mt6701_update(&s, 164, true, 0.001f);
        float v = mt6701_vel_revps(&s);
        mt6701_update(&s, 328, true, 0.0f);   /* dt=0 */
        CHECK(isfinite(mt6701_vel_revps(&s)));
        CHECK_NEAR(mt6701_vel_revps(&s), v, 1e-6f);
        CHECK_NEAR(mt6701_mech_rev(&s), 328.0f / 16384.0f, 1e-4f); /* position still advances */
    }

    /* dir=-1 and zero offset: transform order raw->offset->dir->wrap. */
    {
        mt6701_cfg_t c = base_cfg();
        c.dir = -1;
        c.zero_offset_counts = 4096; /* 0.25 rev */
        mt6701_state_t s;
        mt6701_init(&s, &c);
        /* raw=8192 (0.5): (8192-4096)*-1 = -4096 -> wrap -0.25 -> 0.75 */
        mt6701_update(&s, 8192, true, 0.001f);
        CHECK_NEAR(mt6701_mech_rev(&s), 0.75f, 1e-4f);
    }

    /* Electrical angle wraps within [-pi, pi). */
    {
        mt6701_cfg_t c = base_cfg();
        c.pole_pairs = 7;
        mt6701_state_t s;
        mt6701_init(&s, &c);
        mt6701_update(&s, 8192, true, 0.001f); /* 0.5 rev * 7 = 3.5 -> frac 0.5 -> pi -> -pi range */
        float er = mt6701_elec_rad(&s);
        CHECK(er >= -3.1416f && er < 3.1416f);
    }

    CHECK_DONE();
}
