#include "dronecan_frame.h"

#define BYTE_MASK 0xFFu

void dronecan_frame_sanitize(dronecan_frame_t *f)
{
    uint16_t i;
    f->extended = true;
    for (i = 0; i < 8; ++i) {
        f->data[i] = (i < f->dlc) ? (f->data[i] & BYTE_MASK) : 0u;
    }
}

/* ---- CAN ID fields ---- */
uint16_t dronecan_id_priority(uint32_t id)      { return (uint16_t)((id >> 24) & 0x1Fu); }
uint16_t dronecan_id_source(uint32_t id)        { return (uint16_t)(id & 0x7Fu); }
bool     dronecan_id_is_service(uint32_t id)    { return ((id >> 7) & 1u) != 0u; }
uint16_t dronecan_id_msg_dtid(uint32_t id)      { return (uint16_t)((id >> 8) & 0xFFFFu); }
uint16_t dronecan_id_discriminator(uint32_t id) { return (uint16_t)((id >> 10) & 0x3FFFu); }

uint32_t dronecan_msg_id(uint16_t priority, uint16_t dtid, uint16_t source)
{
    return (((uint32_t)(priority & 0x1Fu)) << 24)
         | (((uint32_t)dtid) << 8)
         | ((uint32_t)(source & 0x7Fu));
}

uint32_t dronecan_anon_id(uint16_t priority, uint16_t dtid, uint16_t discriminator)
{
    /* Anonymous message frame: discriminator[23:10], low 2 bits of DTID [9:8], source = 0. */
    return (((uint32_t)(priority & 0x1Fu)) << 24)
         | (((uint32_t)(discriminator & 0x3FFFu)) << 10)
         | (((uint32_t)(dtid & 0x3u)) << 8);
}

/* ---- tail byte ---- */
uint16_t dronecan_tail_encode(bool sot, bool eot, bool toggle, uint16_t transfer_id)
{
    uint16_t t = (uint16_t)(transfer_id & 0x1Fu);
    if (sot)    { t |= 0x80u; }
    if (eot)    { t |= 0x40u; }
    if (toggle) { t |= 0x20u; }
    return t;
}

dronecan_tail_t dronecan_tail_decode(uint16_t tail)
{
    dronecan_tail_t t;
    t.sot         = (tail & 0x80u) != 0u;
    t.eot         = (tail & 0x40u) != 0u;
    t.toggle      = (tail & 0x20u) != 0u;
    t.transfer_id = (uint16_t)(tail & 0x1Fu);
    return t;
}

/* ---- payload bit buffer ---- */
void dronecan_payload_init(dronecan_payload_t *p)
{
    uint16_t i;
    for (i = 0; i < DRONECAN_PAYLOAD_MAX; ++i) {
        p->bytes[i] = 0u;
    }
    p->bit_len = 0u;
}

uint16_t dronecan_payload_bytelen(const dronecan_payload_t *p)
{
    return (uint16_t)((p->bit_len + 7u) >> 3);
}

/* Append the low n bits of val, MSB-first (bit n-1 first). */
static void append_msb(dronecan_payload_t *p, uint32_t val, uint16_t n)
{
    int16_t i;
    for (i = (int16_t)n - 1; i >= 0; --i) {
        uint16_t bytepos = (uint16_t)(p->bit_len >> 3);
        uint16_t off     = (uint16_t)(p->bit_len & 7u);
        uint16_t bit     = (uint16_t)((val >> (uint16_t)i) & 1u);
        if (bytepos >= DRONECAN_PAYLOAD_MAX) {
            return; /* overflow guard; callers stay within DRONECAN_PAYLOAD_MAX */
        }
        if (off == 0u) {
            p->bytes[bytepos] = 0u;
        }
        p->bytes[bytepos] = (uint16_t)(p->bytes[bytepos] | (bit << (7u - off)));
        p->bit_len++;
    }
}

/* Read n bits MSB-first from *bitpos (advanced). */
static uint32_t read_msb(const dronecan_payload_t *p, uint16_t *bitpos, uint16_t n)
{
    uint32_t v = 0u;
    uint16_t k;
    for (k = 0; k < n; ++k) {
        uint16_t bytepos = (uint16_t)(*bitpos >> 3);
        uint16_t off     = (uint16_t)(*bitpos & 7u);
        uint16_t bit     = 0u;
        if (bytepos < DRONECAN_PAYLOAD_MAX) {
            bit = (uint16_t)((p->bytes[bytepos] >> (7u - off)) & 1u);
        }
        v = (v << 1) | bit;
        (*bitpos)++;
    }
    return v;
}

void dronecan_pack_uint(dronecan_payload_t *p, uint32_t value, uint16_t nbits)
{
    uint16_t rem = nbits;
    while (rem >= 8u) {
        append_msb(p, value & BYTE_MASK, 8u);
        value >>= 8;
        rem = (uint16_t)(rem - 8u);
    }
    if (rem > 0u) {
        append_msb(p, value & ((1u << rem) - 1u), rem);
    }
}

void dronecan_pack_int(dronecan_payload_t *p, int32_t value, uint16_t nbits)
{
    uint32_t mask = (nbits >= 32u) ? 0xFFFFFFFFu : ((1u << nbits) - 1u);
    dronecan_pack_uint(p, ((uint32_t)value) & mask, nbits);
}

void dronecan_pack_bytes(dronecan_payload_t *p, const uint16_t *src, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; ++i) {
        dronecan_pack_uint(p, src[i] & BYTE_MASK, 8u);
    }
}

void dronecan_pack_float16(dronecan_payload_t *p, float v)
{
    dronecan_pack_uint(p, dronecan_float32_to_float16(v), 16u);
}

uint32_t dronecan_unpack_uint(const dronecan_payload_t *p, uint16_t *bitpos, uint16_t nbits)
{
    uint32_t value = 0u;
    uint16_t shift = 0u;
    uint16_t rem = nbits;
    while (rem >= 8u) {
        value |= (uint32_t)read_msb(p, bitpos, 8u) << shift;
        shift = (uint16_t)(shift + 8u);
        rem = (uint16_t)(rem - 8u);
    }
    if (rem > 0u) {
        value |= (uint32_t)read_msb(p, bitpos, rem) << shift;
    }
    return value;
}

int32_t dronecan_unpack_int(const dronecan_payload_t *p, uint16_t *bitpos, uint16_t nbits)
{
    uint32_t v = dronecan_unpack_uint(p, bitpos, nbits);
    if (nbits < 32u && (v & (1u << (nbits - 1u))) != 0u) {
        v |= ~((1u << nbits) - 1u); /* sign extend */
    }
    return (int32_t)v;
}

float dronecan_unpack_float16(const dronecan_payload_t *p, uint16_t *bitpos)
{
    return dronecan_float16_to_float32((uint16_t)dronecan_unpack_uint(p, bitpos, 16u));
}

/* ---- float16 (IEEE binary16), round-to-nearest-even ---- */
uint16_t dronecan_float32_to_float16(float v)
{
    union { float f; uint32_t u; } x;
    uint32_t sign, exp, man, u;
    x.f = v;
    u = x.u;
    sign = (u >> 16) & 0x8000u;
    exp  = (u >> 23) & 0xFFu;
    man  = u & 0x7FFFFFu;

    if (exp == 0xFFu) {                         /* Inf / NaN */
        return (uint16_t)(sign | 0x7C00u | (man ? 0x0200u : 0u));
    }
    {
        int32_t e = (int32_t)exp - 127 + 15;    /* rebias to float16 */
        if (e >= 0x1F) {                        /* overflow -> Inf */
            return (uint16_t)(sign | 0x7C00u);
        }
        if (e <= 0) {                           /* subnormal / underflow */
            if (e < -10) {
                return (uint16_t)sign;
            }
            man |= 0x800000u;                   /* implicit leading 1 */
            {
                uint16_t shift = (uint16_t)(14 - e);
                uint32_t half = man >> shift;
                uint32_t rem  = man & ((1u << shift) - 1u);
                uint32_t halfway = 1u << (shift - 1u);
                if (rem > halfway || (rem == halfway && (half & 1u))) {
                    half++;                      /* round to nearest even */
                }
                return (uint16_t)(sign | half);
            }
        }
        {
            uint16_t h = (uint16_t)(sign | ((uint32_t)e << 10) | (man >> 13));
            uint32_t rem = man & 0x1FFFu;
            uint32_t halfway = 0x1000u;
            if (rem > halfway || (rem == halfway && (h & 1u))) {
                h++;                             /* may carry into exponent, which is correct */
            }
            return h;
        }
    }
}

float dronecan_float16_to_float32(uint16_t h)
{
    union { float f; uint32_t u; } x;
    uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
    uint32_t exp  = (h >> 10) & 0x1Fu;
    uint32_t man  = h & 0x3FFu;

    if (exp == 0u) {
        if (man == 0u) {
            x.u = sign;                          /* +/-0 */
        } else {
            /* subnormal: normalize */
            int32_t e = -1;
            do {
                e++;
                man <<= 1;
            } while ((man & 0x400u) == 0u);
            man &= 0x3FFu;
            x.u = sign | ((uint32_t)(127 - 15 - e) << 23) | (man << 13);
        }
    } else if (exp == 0x1Fu) {
        x.u = sign | 0x7F800000u | (man << 13);  /* Inf / NaN */
    } else {
        x.u = sign | ((exp - 15u + 127u) << 23) | (man << 13);
    }
    return x.f;
}

/* ---- CRC ---- */
uint16_t dronecan_crc16(const uint16_t *bytes, uint16_t n, uint16_t init)
{
    uint16_t crc = init;
    uint16_t i, b;
    for (i = 0; i < n; ++i) {
        crc ^= (uint16_t)((bytes[i] & BYTE_MASK) << 8);
        for (b = 0; b < 8u; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t dronecan_transfer_crc_seed(uint32_t sig_lo, uint32_t sig_hi)
{
    /* Feed the 64-bit signature as little-endian wire bytes into CRC16 from 0xFFFF. */
    uint16_t sig[8];
    sig[0] = (uint16_t)(sig_lo & 0xFFu);
    sig[1] = (uint16_t)((sig_lo >> 8) & 0xFFu);
    sig[2] = (uint16_t)((sig_lo >> 16) & 0xFFu);
    sig[3] = (uint16_t)((sig_lo >> 24) & 0xFFu);
    sig[4] = (uint16_t)(sig_hi & 0xFFu);
    sig[5] = (uint16_t)((sig_hi >> 8) & 0xFFu);
    sig[6] = (uint16_t)((sig_hi >> 16) & 0xFFu);
    sig[7] = (uint16_t)((sig_hi >> 24) & 0xFFu);
    return dronecan_crc16(sig, 8u, 0xFFFFu);
}
