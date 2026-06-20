#include "dronecan.h"
#include "dronecan_ids.h"
#include <stddef.h>

#define ARM_ZERO_FRAMES_DEFAULT 10u

static uint16_t valid_node_id(uint16_t n)
{
    return (n >= 1u && n <= 127u) ? n : 0u;
}

void dronecan_init(dronecan_t *dn, const dronecan_cfg_t *cfg)
{
    dn->cfg = *cfg;

    /* cfg sanitation (fail safe, never silently dangerous) */
    if (dn->cfg.esc_index > DRONECAN_ESC_INDEX_MAX) {
        dn->cfg.esc_index = 0u;
    }
    if (dn->cfg.arm_zero_frames == 0u) {
        dn->cfg.arm_zero_frames = ARM_ZERO_FRAMES_DEFAULT;
    }
    dn->cfg.node_id = valid_node_id(dn->cfg.node_id);
    if (dn->cfg.node_status_period_ms == 0u) { dn->cfg.node_status_period_ms = 1000u; }
    if (dn->cfg.esc_status_period_ms == 0u)  { dn->cfg.esc_status_period_ms = 100u; }
    if (dn->cfg.dna_request_period_ms == 0u) { dn->cfg.dna_request_period_ms = 1000u; }

    dn->node_id       = dn->cfg.node_id;   /* static id if configured, else 0 (DNA) */
    dn->node_id_dirty = false;
    dn->seq           = 0u;
    dn->zero_run      = 0u;
    dn->armed         = false;

    dn->sched_primed        = false;
    dn->last_node_status_ms = 0u;
    dn->last_esc_status_ms  = 0u;
    dn->tid_node_status     = 0u;
    dn->tid_esc_status      = 0u;
}

uint16_t dronecan_node_id(const dronecan_t *dn)      { return dn->node_id; }
bool     dronecan_node_id_dirty(const dronecan_t *dn){ return dn->node_id_dirty; }
void     dronecan_clear_node_id_dirty(dronecan_t *dn){ dn->node_id_dirty = false; }

/* Copy a CAN frame's data bytes (sans tail) into a payload buffer, masking to 8 bits. */
static void payload_from_frame(dronecan_payload_t *p, const dronecan_frame_t *f, uint16_t nbytes)
{
    uint16_t i;
    dronecan_payload_init(p);
    for (i = 0; i < nbytes && i < DRONECAN_PAYLOAD_MAX; ++i) {
        p->bytes[i] = (uint16_t)(f->data[i] & 0xFFu);
    }
    p->bit_len = (uint16_t)(nbytes * 8u);
}

static void handle_raw_command(dronecan_t *dn, const dronecan_frame_t *f, dronecan_rx_result_t *res)
{
    /* Single-frame only: SOT=EOT=1, toggle=0. Multi-frame RawCommand is skipped. */
    dronecan_tail_t tail;
    uint16_t payload_bytes, count, bitpos;
    int32_t raw14;
    dronecan_payload_t p;

    if (f->dlc < 1u) {
        return;
    }
    tail = dronecan_tail_decode(f->data[f->dlc - 1u]);
    if (!tail.sot || !tail.eot || tail.toggle) {
        return; /* multi-frame / illegal tail */
    }

    payload_bytes = (uint16_t)(f->dlc - 1u);
    count = (uint16_t)((payload_bytes * 8u) / 14u);  /* int14[] element count */
    if (dn->cfg.esc_index >= count) {
        return; /* no component for us -> do not update command/seq */
    }

    payload_from_frame(&p, f, payload_bytes);
    bitpos = (uint16_t)(dn->cfg.esc_index * 14u);
    raw14 = dronecan_unpack_int(&p, &bitpos, 14u);

    /* Zero-frame arming handshake. */
    {
        bool is_zero = (raw14 <= 0);
        if (!dn->armed) {
            if (is_zero) {
                dn->zero_run++;
                if (dn->zero_run >= dn->cfg.arm_zero_frames) {
                    dn->armed = true;
                }
            } else {
                dn->zero_run = 0u; /* nonzero before handshake resets the counter */
            }
        }
    }

    dn->seq++;
    res->command.seq = dn->seq;
    if (dn->armed) {
        float thr = (raw14 <= 0) ? 0.0f : ((float)raw14 / (float)DRONECAN_RAWCMD_FULLSCALE);
        if (thr > 1.0f) {
            thr = 1.0f;
        }
        res->command.throttle = thr;
        res->command.arm = true;
    } else {
        res->command.throttle = 0.0f;
        res->command.arm = false;
    }
    res->command_updated = true;
}

/* ---- TX builders ---- */

static int32_t saturate_int(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* Build the single NodeStatus frame for the given uptime/health. */
static void build_node_status(dronecan_t *dn, uint32_t now_ms, const esc_telemetry_t *tel,
                              dronecan_frame_t *f)
{
    dronecan_payload_t p;
    uint16_t plen, i;
    uint16_t health = (tel->hard_fault_bits != 0u) ? DRONECAN_HEALTH_ERROR : DRONECAN_HEALTH_OK;
    uint16_t vendor = (uint16_t)(tel->hard_fault_bits & 0xFFFFu);

    dronecan_payload_init(&p);
    dronecan_pack_uint(&p, now_ms / 1000u, 32u);          /* uptime_sec */
    dronecan_pack_uint(&p, health, 2u);                   /* health */
    dronecan_pack_uint(&p, DRONECAN_MODE_OPERATIONAL, 3u);/* mode */
    dronecan_pack_uint(&p, 0u, 3u);                       /* sub_mode */
    dronecan_pack_uint(&p, vendor, 16u);                  /* vendor_specific_status_code */
    plen = dronecan_payload_bytelen(&p);

    f->extended = true;
    f->id = dronecan_msg_id(DRONECAN_PRIO_NODE_STATUS, DRONECAN_DTID_NODE_STATUS, dn->node_id);
    for (i = 0; i < plen; ++i) {
        f->data[i] = p.bytes[i];
    }
    f->data[plen] = dronecan_tail_encode(true, true, false, dn->tid_node_status);
    f->dlc = (uint16_t)(plen + 1u);
}

/* Build the 3-frame esc.Status transfer. out3 must have room for 3 frames. */
static void build_esc_status(dronecan_t *dn, const esc_telemetry_t *tel, dronecan_frame_t *out3)
{
    dronecan_payload_t p;
    uint16_t plen, seed, crc, total, off, fidx, i;
    uint16_t buf[2 + DRONECAN_PAYLOAD_MAX];
    uint32_t id;
    int32_t rpm;

    dronecan_payload_init(&p);
    dronecan_pack_uint(&p, 0u, 32u);                                       /* error_count = 0 */
    dronecan_pack_float16(&p, tel->vbus_V);                               /* voltage (V) */
    dronecan_pack_float16(&p, tel->current_A);                            /* current (A) */
    dronecan_pack_float16(&p, tel->temp_C + DRONECAN_KELVIN_OFFSET);      /* temperature (K) */
    rpm = saturate_int((int32_t)tel->rpm, -131072, 131071);
    dronecan_pack_int(&p, rpm, 18u);                                      /* rpm (int18) */
    dronecan_pack_uint(&p, 0u, 7u);                                       /* power_rating_pct = 0 */
    dronecan_pack_uint(&p, dn->cfg.esc_index, 5u);                        /* esc_index */
    plen = dronecan_payload_bytelen(&p);

    /* Multi-frame transfer CRC over the serialized payload (signature-seeded), CRC at front. */
    seed = dronecan_transfer_crc_seed(DRONECAN_ESC_STATUS_SIG_LO, DRONECAN_ESC_STATUS_SIG_HI);
    crc = dronecan_crc16(p.bytes, plen, seed);
    buf[0] = (uint16_t)(crc & 0xFFu);
    buf[1] = (uint16_t)((crc >> 8) & 0xFFu);
    for (i = 0; i < plen; ++i) {
        buf[2 + i] = p.bytes[i];
    }
    total = (uint16_t)(plen + 2u);

    id = dronecan_msg_id(DRONECAN_PRIO_ESC_STATUS, DRONECAN_DTID_ESC_STATUS, dn->node_id);
    off = 0u;
    fidx = 0u;
    while (off < total) {
        uint16_t remn = (uint16_t)(total - off);
        uint16_t chunk = (remn >= 7) ? (uint16_t)7 : remn;
        dronecan_frame_t *f = &out3[fidx];
        bool sot = (fidx == 0u);
        bool eot = ((off + chunk) >= total);
        bool toggle = ((fidx & 1u) != 0u);
        f->extended = true;
        f->id = id;
        for (i = 0; i < chunk; ++i) {
            f->data[i] = buf[off + i];
        }
        f->data[chunk] = dronecan_tail_encode(sot, eot, toggle, dn->tid_esc_status);
        f->dlc = (uint16_t)(chunk + 1u);
        off = (uint16_t)(off + chunk);
        fidx++;
    }
}

int dronecan_tick(dronecan_t *dn, uint32_t now_ms, const esc_telemetry_t *tel,
                  dronecan_frame_t *out, int cap)
{
    int n = 0;

    if (dn == NULL) {
        return 0;
    }
    if (out == NULL || cap <= 0) {
        return 0;
    }

    if (dn->node_id == 0u) {
        /* Unallocated: DNA requests only (implemented in a later step); no telemetry needed. */
        return 0;
    }
    if (tel == NULL) {
        return 0; /* allocated but no telemetry -> emit nothing */
    }

    /* First eligible tick after the node id is known sends immediately. */
    if (!dn->sched_primed) {
        dn->last_node_status_ms = now_ms - dn->cfg.node_status_period_ms;
        dn->last_esc_status_ms  = now_ms - dn->cfg.esc_status_period_ms;
        dn->sched_primed = true;
    }

    /* Priority: NodeStatus before esc.Status. */
    if ((uint32_t)(now_ms - dn->last_node_status_ms) >= dn->cfg.node_status_period_ms) {
        if (n < cap) {
            build_node_status(dn, now_ms, tel, &out[n]);
            n++;
            dn->last_node_status_ms = now_ms;
            dn->tid_node_status = (uint16_t)((dn->tid_node_status + 1u) & 0x1Fu);
        }
        /* else: no slot -> stays due, timer/tid untouched */
    }

    if ((uint32_t)(now_ms - dn->last_esc_status_ms) >= dn->cfg.esc_status_period_ms) {
        if ((cap - n) >= 3) {
            build_esc_status(dn, tel, &out[n]);
            n += 3;
            dn->last_esc_status_ms = now_ms;
            dn->tid_esc_status = (uint16_t)((dn->tid_esc_status + 1u) & 0x1Fu);
        }
        /* else: not enough room for the whole transfer -> stays due */
    }

    return n;
}

void dronecan_on_rx(dronecan_t *dn, const dronecan_frame_t *f, dronecan_rx_result_t *res)
{
    uint16_t dtid;

    res->command_updated = false;

    if (dn == NULL || f == NULL) {
        return;
    }
    if (!f->extended || dronecan_id_is_service(f->id)) {
        return; /* DroneCAN messages are extended, non-service */
    }

    dtid = dronecan_id_msg_dtid(f->id);
    if (dtid == DRONECAN_DTID_RAW_COMMAND) {
        if (dronecan_id_source(f->id) == 0u) {
            return; /* RawCommand must come from a real (non-anonymous) node */
        }
        handle_raw_command(dn, f, res);
    }
    /* DNA allocation replies (dtid == DRONECAN_DTID_ALLOCATION) handled in a later step. */
}
