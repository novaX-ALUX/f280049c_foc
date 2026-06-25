#include "dronecan_param.h"
#include <stddef.h>

/* Value union tags (uavcan.protocol.param.Value, 5 members). */
#define VAL_EMPTY   0u
#define VAL_INT     1u
#define VAL_REAL    2u
#define VAL_BOOL    3u
#define VAL_STRING  4u
/* NumericValue union tags (3 members). */
#define NUM_EMPTY   0u
#define NUM_INT     1u
#define NUM_REAL    2u

typedef enum { PT_INT, PT_BOOL, PT_REAL } ptype_t;
typedef struct { const char *name; ptype_t type; } pdesc_t;

static const pdesc_t PARAMS[DRONECAN_PARAM_COUNT] = {
    { "node_id",             PT_INT  },
    { "park_ref_valid",      PT_BOOL },
    { "park_ref_target_rev", PT_REAL },
};

uint16_t dronecan_param_count(void) { return DRONECAN_PARAM_COUNT; }

const char *dronecan_param_name(uint16_t idx)
{
    return (idx < DRONECAN_PARAM_COUNT) ? PARAMS[idx].name : NULL;
}

/* ---- little helpers (one wire byte per uint16_t) ---- */
static uint32_t rd_u32(const uint16_t *b, uint16_t o)
{
    return  (uint32_t)(b[o] & 0xFFu)
         | ((uint32_t)(b[o + 1u] & 0xFFu) << 8)
         | ((uint32_t)(b[o + 2u] & 0xFFu) << 16)
         | ((uint32_t)(b[o + 3u] & 0xFFu) << 24);
}
static float u32_to_f(uint32_t u)
{
    union { float f; uint32_t u; } cv; cv.u = u; return cv.f;
}
static uint32_t f_to_u32(float v)
{
    union { float f; uint32_t u; } cv; cv.f = v; return cv.u;
}

static void put(uint16_t *o, uint16_t *pos, uint16_t cap, uint16_t v)
{
    if (*pos < cap) { o[(*pos)++] = (uint16_t)(v & 0xFFu); }
}
static void put_u32(uint16_t *o, uint16_t *pos, uint16_t cap, uint32_t u)
{
    put(o, pos, cap, (uint16_t)(u & 0xFFu));
    put(o, pos, cap, (uint16_t)((u >> 8) & 0xFFu));
    put(o, pos, cap, (uint16_t)((u >> 16) & 0xFFu));
    put(o, pos, cap, (uint16_t)((u >> 24) & 0xFFu));
}
/* int64 from a non-negative int32 (our params are all >= 0): low 4 bytes + zero sign extension. */
static void put_i64_nonneg(uint16_t *o, uint16_t *pos, uint16_t cap, uint32_t lo)
{
    put_u32(o, pos, cap, lo);
    put_u32(o, pos, cap, 0u);
}

/* Compare a request name (nlen wire bytes) against a NUL-terminated param name. */
static bool name_eq(const uint16_t *nb, uint16_t nlen, const char *s)
{
    uint16_t i;
    for (i = 0u; i < nlen; i++) {
        if (s[i] == '\0' || (uint16_t)(s[i] & 0xFF) != (uint16_t)(nb[i] & 0xFFu)) {
            return false;
        }
    }
    return s[nlen] == '\0';
}

/* Append a NumericValue (default/max/min): empty, or an int64, or a real32. */
static void put_numeric_int(uint16_t *o, uint16_t *pos, uint16_t cap, uint32_t v)
{
    put(o, pos, cap, NUM_INT); put_i64_nonneg(o, pos, cap, v);
}
static void put_numeric_real(uint16_t *o, uint16_t *pos, uint16_t cap, float v)
{
    put(o, pos, cap, NUM_REAL); put_u32(o, pos, cap, f_to_u32(v));
}
static void put_numeric_empty(uint16_t *o, uint16_t *pos, uint16_t cap)
{
    put(o, pos, cap, NUM_EMPTY);
}

uint16_t dronecan_param_build_response(nvparam_t *nv,
                                       const uint16_t *req, uint16_t req_len,
                                       uint16_t *out, uint16_t out_cap,
                                       bool *persist_needed)
{
    dronecan_payload_t hdr;            /* first 2 bytes carry index(13) + value tag(3) */
    uint16_t bitpos = 0u, i;
    uint16_t index, vtag;
    uint16_t voff;                     /* byte offset of the value payload (byte aligned) */
    uint16_t name_off, name_len;
    const uint16_t *name;
    int found = -1;
    bool have_set = false;

    if (persist_needed != NULL) { *persist_needed = false; }
    if (req == NULL || out == NULL || out_cap == 0u || req_len < 2u) { return 0u; }

    /* --- decode the 16-bit header: index(13) + value union tag(3) --- */
    dronecan_payload_init(&hdr);
    hdr.bytes[0] = (uint16_t)(req[0] & 0xFFu);
    hdr.bytes[1] = (uint16_t)(req[1] & 0xFFu);
    hdr.bit_len  = 16u;
    index = (uint16_t)dronecan_unpack_uint(&hdr, &bitpos, 13u);
    vtag  = (uint16_t)dronecan_unpack_uint(&hdr, &bitpos, 3u);
    voff  = 2u;                         /* value payload starts after the 2 header bytes */

    /* --- locate the value payload end / name start (all supported tags are byte aligned) --- */
    name_off = voff;
    switch (vtag) {
        case VAL_INT:    name_off = (uint16_t)(voff + 8u); break;   /* int64  */
        case VAL_REAL:   name_off = (uint16_t)(voff + 4u); break;   /* real32 */
        case VAL_BOOL:   name_off = (uint16_t)(voff + 1u); break;   /* uint8  */
        case VAL_STRING:                                            /* uint8[<=128]: len + bytes */
            if (voff < req_len) {
                name_off = (uint16_t)(voff + 1u + (req[voff] & 0xFFu));
            }
            break;
        case VAL_EMPTY:
        default:         name_off = voff; break;
    }
    /* A truncated request whose value field runs past the buffer is malformed: drop the value
     * (treat as a Get) so a Set never reads stale/past-end bytes. */
    if (name_off > req_len) { vtag = VAL_EMPTY; name_off = req_len; }
    name     = &req[name_off];
    name_len = (uint16_t)(req_len - name_off);
    if (name_len > DRONECAN_PARAM_NAME_MAX) { name_len = DRONECAN_PARAM_NAME_MAX; }

    /* --- select the parameter: non-empty name wins, else index --- */
    if (name_len > 0u) {
        for (i = 0u; i < DRONECAN_PARAM_COUNT; i++) {
            if (name_eq(name, name_len, PARAMS[i].name)) { found = (int)i; break; }
        }
    } else if (index < DRONECAN_PARAM_COUNT) {
        found = (int)index;
    }

    /* --- apply a Set (only when nv present, param known, and the value type matches) --- */
    if (nv != NULL && found >= 0 && vtag != VAL_EMPTY) {
        nvparam_t before = *nv;
        ptype_t pt = PARAMS[found].type;
        if (pt == PT_INT && vtag == VAL_INT) {
            uint32_t lo = rd_u32(req, voff);
            uint32_t hi = rd_u32(req, (uint16_t)(voff + 4u));
            /* funnel through nvparam: anything outside 0..127 (incl. negative / >32b) -> DNA(0) */
            uint16_t cand = (hi == 0u && lo <= NVPARAM_NODE_ID_MAX) ? (uint16_t)lo : 0xFFFFu;
            (void)nvparam_update_node_id(nv, cand);
            have_set = true;
        } else if (pt == PT_BOOL && vtag == VAL_BOOL) {
            bool b = (req[voff] & 0xFFu) != 0u;
            (void)nvparam_update_park_ref(nv, b, nv->park_ref_target_rev);
            have_set = true;
        } else if (pt == PT_REAL && vtag == VAL_REAL) {
            float f = u32_to_f(rd_u32(req, voff));
            (void)nvparam_update_park_ref(nv, nv->park_ref_valid, f);
            have_set = true;
        }
        if (have_set && persist_needed != NULL) {
            *persist_needed = (before.node_id != nv->node_id)
                           || (before.park_ref_valid != nv->park_ref_valid)
                           || (before.park_ref_target_rev != nv->park_ref_target_rev);
        }
    }

    /* --- serialize the response (byte-aligned union tags, per the golden) --- */
    {
        uint16_t pos = 0u;
        if (found < 0 || nv == NULL) {
            /* unknown / no face: empty value + empty numerics + empty name */
            put(out, &pos, out_cap, VAL_EMPTY);
            put_numeric_empty(out, &pos, out_cap);  /* default */
            put_numeric_empty(out, &pos, out_cap);  /* max */
            put_numeric_empty(out, &pos, out_cap);  /* min */
            return pos;
        }
        switch (PARAMS[found].type) {
            case PT_INT:   /* node_id: value/default/max/min as int64; advertise the 0..127 range */
                put(out, &pos, out_cap, VAL_INT); put_i64_nonneg(out, &pos, out_cap, nv->node_id);
                put_numeric_int(out, &pos, out_cap, 0u);    /* default */
                put_numeric_int(out, &pos, out_cap, NVPARAM_NODE_ID_MAX); /* max */
                put_numeric_int(out, &pos, out_cap, 0u);    /* min */
                break;
            case PT_BOOL:  /* park_ref_valid: boolean value, numerics empty (not a numeric type) */
                put(out, &pos, out_cap, VAL_BOOL);
                put(out, &pos, out_cap, nv->park_ref_valid ? 1u : 0u);
                put_numeric_empty(out, &pos, out_cap);
                put_numeric_empty(out, &pos, out_cap);
                put_numeric_empty(out, &pos, out_cap);
                break;
            case PT_REAL:  /* park_ref_target_rev: real value + real default 0, no min/max bound */
            default:
                put(out, &pos, out_cap, VAL_REAL);
                put_u32(out, &pos, out_cap, f_to_u32(nv->park_ref_target_rev));
                put_numeric_real(out, &pos, out_cap, 0.0f); /* default */
                put_numeric_empty(out, &pos, out_cap);      /* max */
                put_numeric_empty(out, &pos, out_cap);      /* min */
                break;
        }
        /* name (tail array, no length prefix) */
        {
            const char *nm = PARAMS[found].name;
            uint16_t k = 0u;
            while (nm[k] != '\0') { put(out, &pos, out_cap, (uint16_t)(nm[k] & 0xFF)); k++; }
        }
        return pos;
    }
}
