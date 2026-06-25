#include "nvparam.h"

/* float is finite when its exponent field is not all-ones (matches the Inf/NaN test in
 * dronecan_frame.c). Done from the bit pattern so it needs no math.h and is word-safe. */
static bool nvp_finite(float v)
{
    union { float f; uint32_t u; } x;
    x.f = v;
    return ((x.u >> 23) & 0xFFu) != 0xFFu;
}

void nvparam_set_defaults(nvparam_t *p)
{
    p->node_id             = 0u;     /* DNA */
    p->park_ref_valid      = false;
    p->park_ref_target_rev = 0.0f;
}

bool nvparam_validate(nvparam_t *p)
{
    bool clean = true;

    if (p->node_id > NVPARAM_NODE_ID_MAX) {
        p->node_id = 0u;             /* out of static range -> fall back to DNA */
        clean = false;
    }

    /* A park ref is only usable if it is both flagged valid AND finite. */
    if (p->park_ref_valid && !nvp_finite(p->park_ref_target_rev)) {
        p->park_ref_valid      = false;
        p->park_ref_target_rev = 0.0f;
        clean = false;
    }
    /* keep the invalid record canonical: zero the target. Use the bit pattern, not `!= 0.0f`,
     * so negative zero (0x80000000) is also caught (-0.0f compares equal to 0.0f). */
    if (!p->park_ref_valid) {
        union { float f; uint32_t u; } z; z.f = p->park_ref_target_rev;
        if (z.u != 0u) {
            p->park_ref_target_rev = 0.0f;
            clean = false;
        }
    }

    return clean;
}

bool nvparam_update_node_id(nvparam_t *p, uint16_t node_id)
{
    bool ok = (node_id <= NVPARAM_NODE_ID_MAX);
    p->node_id = node_id;
    (void)nvparam_validate(p);
    return ok;
}

bool nvparam_update_park_ref(nvparam_t *p, bool valid, float target_rev)
{
    bool ok = !valid || nvp_finite(target_rev);
    p->park_ref_valid      = valid;
    p->park_ref_target_rev = valid ? target_rev : 0.0f;
    (void)nvparam_validate(p);
    return ok;          /* accepted as-is only if a valid value was also finite */
}

uint16_t nvparam_crc16(const uint16_t *words, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i, b;

    for (i = 0u; i < n; i++) {
        /* feed high byte then low byte so the CRC is byte-stream equivalent without
         * ever relying on an 8-bit type (C28x has none). */
        uint16_t bytes[2];
        uint16_t k;
        bytes[0] = (uint16_t)((words[i] >> 8) & 0xFFu);
        bytes[1] = (uint16_t)(words[i] & 0xFFu);
        for (k = 0u; k < 2u; k++) {
            crc ^= (uint16_t)(bytes[k] << 8);
            for (b = 0u; b < 8u; b++) {
                if (crc & 0x8000u) {
                    crc = (uint16_t)((crc << 1) ^ 0x1021u);
                } else {
                    crc = (uint16_t)(crc << 1);
                }
            }
        }
    }
    return crc;
}

void nvparam_encode(const nvparam_t *p, uint16_t words[NVPARAM_WORDS])
{
    nvparam_t s = *p;
    union { float f; uint32_t u; } cv;

    (void)nvparam_validate(&s);      /* never serialize an out-of-range record */

    words[0] = NVPARAM_MAGIC;
    words[1] = NVPARAM_VERSION;
    words[2] = s.node_id;
    words[3] = s.park_ref_valid ? NVPARAM_FLAG_PARK_REF_VALID : 0u;

    cv.f = s.park_ref_target_rev;
    words[4] = (uint16_t)(cv.u & 0xFFFFu);
    words[5] = (uint16_t)((cv.u >> 16) & 0xFFFFu);

    words[6] = nvparam_crc16(words, NVPARAM_WORDS - 1u);
}

nvparam_status_t nvparam_decode(const uint16_t words[NVPARAM_WORDS], nvparam_t *out)
{
    union { float f; uint32_t u; } cv;

    if (words[0] != NVPARAM_MAGIC) {
        nvparam_set_defaults(out);
        return NVPARAM_ERR_MAGIC;
    }
    if (words[1] != NVPARAM_VERSION) {
        nvparam_set_defaults(out);
        return NVPARAM_ERR_VERSION;
    }
    if (words[6] != nvparam_crc16(words, NVPARAM_WORDS - 1u)) {
        nvparam_set_defaults(out);
        return NVPARAM_ERR_CRC;
    }

    out->node_id        = words[2];
    out->park_ref_valid = (words[3] & NVPARAM_FLAG_PARK_REF_VALID) != 0u;
    cv.u = ((uint32_t)words[5] << 16) | (uint32_t)words[4];
    out->park_ref_target_rev = cv.f;

    return nvparam_validate(out) ? NVPARAM_OK : NVPARAM_OK_SANITIZED;
}
