#include "check.h"
#include "nvparam.h"
#include <math.h>

/* A mock Flash sector: a word array the "app" writes encoded records into and reads back.
 * The real target uses driverlib erase/program; here we only exercise the storage format. */
static uint16_t g_flash[NVPARAM_WORDS];

static void mock_flash_store(const nvparam_t *p)
{
    nvparam_encode(p, g_flash);          /* app would erase + program these words */
}
static nvparam_status_t mock_flash_load(nvparam_t *out)
{
    return nvparam_decode(g_flash, out); /* app would read these words back */
}

int main(void)
{
    /* defaults: DNA + unlearned park ref. */
    {
        nvparam_t p;
        nvparam_set_defaults(&p);
        CHECK(p.node_id == 0u);
        CHECK(!p.park_ref_valid);
        CHECK_NEAR(p.park_ref_target_rev, 0.0f, 0.0f);
    }

    /* roundtrip: a fully populated record survives encode -> decode unchanged. */
    {
        nvparam_t p, q;
        nvparam_set_defaults(&p);
        CHECK(nvparam_update_node_id(&p, 42u));
        CHECK(nvparam_update_park_ref(&p, true, 0.3125f));
        uint16_t buf[NVPARAM_WORDS];
        nvparam_encode(&p, buf);
        CHECK(buf[0] == NVPARAM_MAGIC);
        CHECK(buf[1] == NVPARAM_VERSION);
        CHECK(nvparam_decode(buf, &q) == NVPARAM_OK);
        CHECK(q.node_id == 42u);
        CHECK(q.park_ref_valid);
        CHECK_NEAR(q.park_ref_target_rev, 0.3125f, 1e-6f);
    }

    /* bad magic -> defaults + ERR_MAGIC. */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        uint16_t buf[NVPARAM_WORDS];
        nvparam_update_node_id(&p, 7u);
        nvparam_encode(&p, buf);
        buf[0] ^= 0xFFFFu;                          /* corrupt magic */
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_ERR_MAGIC);
        CHECK(q.node_id == 0u);                     /* defaults restored */
    }

    /* bad version -> defaults + ERR_VERSION. */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        uint16_t buf[NVPARAM_WORDS];
        nvparam_encode(&p, buf);
        buf[1] = NVPARAM_VERSION + 1u;
        buf[NVPARAM_WORDS - 1u] = nvparam_crc16(buf, NVPARAM_WORDS - 1u); /* valid CRC, bad ver */
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_ERR_VERSION);
        CHECK(q.node_id == 0u);
    }

    /* bad CRC -> defaults + ERR_CRC (any single-field tamper without CRC fixup). */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        uint16_t buf[NVPARAM_WORDS];
        nvparam_update_node_id(&p, 100u);
        nvparam_encode(&p, buf);
        buf[2] = 55u;                               /* change node_id, leave stale CRC */
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_ERR_CRC);
        CHECK(q.node_id == 0u);
    }

    /* node-id boundaries: 0,1,127 accepted; 128 and above rejected -> DNA. */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        CHECK(nvparam_update_node_id(&p, 0u)   && p.node_id == 0u);
        CHECK(nvparam_update_node_id(&p, 1u)   && p.node_id == 1u);
        CHECK(nvparam_update_node_id(&p, 127u) && p.node_id == 127u);
        CHECK(!nvparam_update_node_id(&p, 128u) && p.node_id == 0u);
        CHECK(!nvparam_update_node_id(&p, 65535u) && p.node_id == 0u);
    }

    /* a CRC-valid record carrying an out-of-range node id decodes as SANITIZED. */
    {
        uint16_t buf[NVPARAM_WORDS];
        buf[0] = NVPARAM_MAGIC; buf[1] = NVPARAM_VERSION;
        buf[2] = 200u;                              /* out of range on storage */
        buf[3] = 0u; buf[4] = 0u; buf[5] = 0u;
        buf[6] = nvparam_crc16(buf, NVPARAM_WORDS - 1u);
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_OK_SANITIZED);
        CHECK(q.node_id == 0u);
    }

    /* NaN / Inf park ref is rejected by the setter and sanitized on decode. */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        CHECK(!nvparam_update_park_ref(&p, true, NAN));
        CHECK(!p.park_ref_valid);
        CHECK(!nvparam_update_park_ref(&p, true, INFINITY));
        CHECK(!p.park_ref_valid);

        /* a CRC-valid record whose flag says valid but payload is NaN -> SANITIZED. */
        union { float f; uint32_t u; } cv; cv.f = NAN;
        uint16_t buf[NVPARAM_WORDS];
        buf[0] = NVPARAM_MAGIC; buf[1] = NVPARAM_VERSION; buf[2] = 0u;
        buf[3] = NVPARAM_FLAG_PARK_REF_VALID;
        buf[4] = (uint16_t)(cv.u & 0xFFFFu); buf[5] = (uint16_t)((cv.u >> 16) & 0xFFFFu);
        buf[6] = nvparam_crc16(buf, NVPARAM_WORDS - 1u);
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_OK_SANITIZED);
        CHECK(!q.park_ref_valid);
        CHECK_NEAR(q.park_ref_target_rev, 0.0f, 0.0f);
    }

    /* power-cycle through the mock flash: learn a park ref, store, "reboot", reload. */
    {
        nvparam_t boot;
        nvparam_set_defaults(&boot);
        mock_flash_store(&boot);                    /* factory: defaults written */
        CHECK(mock_flash_load(&boot) == NVPARAM_OK);
        CHECK(!boot.park_ref_valid);                /* first boot: unlearned */

        /* DNA allocates a node id, and park_ref raises a store request -> app persists. */
        nvparam_update_node_id(&boot, 73u);
        nvparam_update_park_ref(&boot, true, 0.871f);
        mock_flash_store(&boot);

        nvparam_t after;
        CHECK(mock_flash_load(&after) == NVPARAM_OK);
        CHECK(after.node_id == 73u);
        CHECK(after.park_ref_valid);
        CHECK_NEAR(after.park_ref_target_rev, 0.871f, 1e-6f);
    }

    /* CRC actually covers every field: changing one word changes the CRC. */
    {
        nvparam_t p; nvparam_set_defaults(&p);
        uint16_t a[NVPARAM_WORDS], b[NVPARAM_WORDS];
        nvparam_update_node_id(&p, 10u); nvparam_encode(&p, a);
        nvparam_update_node_id(&p, 11u); nvparam_encode(&p, b);
        CHECK(a[NVPARAM_WORDS - 1u] != b[NVPARAM_WORDS - 1u]);
    }

    /* a CRC-valid record flagged invalid but carrying negative zero (0x80000000) in the park-ref
     * float must be canonicalized to +0.0 and reported SANITIZED (not OK). */
    {
        uint16_t buf[NVPARAM_WORDS];
        buf[0] = NVPARAM_MAGIC; buf[1] = NVPARAM_VERSION; buf[2] = 0u;
        buf[3] = 0u;                 /* park_ref_valid = false */
        buf[4] = 0x0000u; buf[5] = 0x8000u;   /* -0.0f */
        buf[6] = nvparam_crc16(buf, NVPARAM_WORDS - 1u);
        nvparam_t q;
        CHECK(nvparam_decode(buf, &q) == NVPARAM_OK_SANITIZED);
        CHECK(!q.park_ref_valid);
        { union { float f; uint32_t u; } z; z.f = q.park_ref_target_rev;
          CHECK(z.u == 0u); }        /* canonical +0.0, not -0.0 */
    }

    CHECK_DONE();
}
