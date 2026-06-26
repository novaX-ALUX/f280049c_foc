#include "check.h"
#include "mt6701.h"
#include "mt6701_golden.inc"

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

    /* ---- SSI frame decode + CRC6 (datasheet 6.8.2) ---- */

    /* CRC6 anchored to a few hand-computed values (poly X^6+X+1, init 0, MSB-first). */
    {
        CHECK(mt6701_crc6(0u) == 0x00u);
        CHECK(mt6701_crc6(1u) == 0x03u);   /* only Mg[0] set */
        CHECK(mt6701_crc6(2u) == 0x06u);
        CHECK((mt6701_crc6(0xFFFFFFFFu) & ~0x3Fu) == 0u); /* result is always 6-bit */
    }

    /* Golden vectors: decode whole frames, compare every field against the
     * independently-generated tools/test/mt6701_golden.inc. */
    {
        int i;
        for (i = 0; i < MT6701_GOLDEN_N; ++i) {
            const mt6701_golden_t *g = &MT6701_GOLDEN[i];
            mt6701_frame_t f;
            bool ok = mt6701_decode_ssi(g->frame, &f);
            CHECK(ok == (g->crc_ok != 0u));
            CHECK(f.crc_ok == (g->crc_ok != 0u));
            CHECK(f.angle == g->angle);
            CHECK(f.mg == g->mg);
            CHECK(f.crc_rx == g->crc);
            CHECK(f.field_ok == (g->field_ok != 0u));
            CHECK(f.track_ok == (g->track_ok != 0u));
            CHECK(f.button == (g->button != 0u));
        }
    }

    /* Round-trip: build a frame with the module's own CRC, decode, expect valid;
     * flipping any single CRC bit must fail the check (CRC catches 1-bit errors). */
    {
        uint16_t angle = 0x2ABCu & 0x3FFFu;
        uint16_t mg = 0x0u;
        uint32_t data18 = ((uint32_t)angle << 4) | mg;
        uint16_t crc = mt6701_crc6(data18);
        uint32_t frame = ((uint32_t)angle << 10) | ((uint32_t)mg << 6) | crc;
        mt6701_frame_t f;
        int b;

        CHECK(mt6701_decode_ssi(frame, &f));
        CHECK(f.angle == angle);
        CHECK(f.crc_ok && f.field_ok && f.track_ok);

        for (b = 0; b < 6; ++b) {
            CHECK(!mt6701_decode_ssi(frame ^ (1uL << b), &f)); /* corrupt CRC bit b */
        }
    }

    /* decode_ssi tolerates a NULL out pointer (returns the CRC verdict only). */
    {
        uint32_t good = MT6701_GOLDEN[0].frame;       /* crc_ok vector */
        uint32_t bad  = MT6701_GOLDEN[MT6701_GOLDEN_N - 1].frame; /* forged-CRC vector */
        CHECK(mt6701_decode_ssi(good, 0));
        CHECK(!mt6701_decode_ssi(bad, 0));
    }

    /* ---- SSI word-pair alignment (mt6701_ssi_frame) ----
     * Real two-word captures from the LaunchXL-F280049C SPIB bench (CLK=GPIO22,
     * DO=GPIO31, CSN=GPIO34). The MT6701 emits one leading bit, so the frame is at
     * bits [30:7] of (w0:w1). mt6701_ssi_frame() must align to a CRC-valid frame;
     * the historical >>8 alignment (one bit early) must NOT pass CRC -- that off-by-one
     * was the bug that made the read silently "valid" only via the all-zero frame. */
    {
        struct { uint16_t w0, w1; uint32_t frame24; uint16_t angle; } v[] = {
            {0xE13E, 0x197F, 0xC27C32, 12447},
            {0xA04C, 0x1DFF, 0x40983B,  4134},
            {0xF14E, 0x1FFF, 0xE29C3F, 14503},
            {0xA980, 0x1E7F, 0x53003C,  5312},
            {0xE078, 0x17FF, 0xC0F02F, 12348},
            {0xA04A, 0x147F, 0x409428,  4133},
        };
        int n = (int)(sizeof(v) / sizeof(v[0]));
        int i;
        for (i = 0; i < n; ++i) {
            uint32_t frame = mt6701_ssi_frame(v[i].w0, v[i].w1);
            mt6701_frame_t f;
            CHECK(frame == v[i].frame24);                 /* exact alignment */
            CHECK(mt6701_decode_ssi(frame, &f));          /* CRC passes */
            CHECK(f.angle == v[i].angle);
            CHECK(f.field_ok && f.track_ok);              /* usable angle */

            /* The old (one-bit-early) assembly must fail CRC on this same capture. */
            uint32_t bad = (((uint32_t)v[i].w0 << 8) | ((uint32_t)v[i].w1 >> 8)) & 0xFFFFFFu;
            CHECK(!mt6701_decode_ssi(bad, 0));
        }
    }

    CHECK_DONE();
}
