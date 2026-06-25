#include "dronecan.h"
#include "dronecan_ids.h"
#include "dronecan_param.h"
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

    dn->gni_pending = false;
    dn->gni_dst     = 0u;
    dn->gni_tid     = 0u;
    dn->gni_prio    = 0u;

    dn->gs_pending   = false;
    dn->gs_dst       = 0u;
    dn->gs_tid       = 0u;
    dn->gs_prio      = 0u;
    dn->gs_resp_len  = 0u;
    dn->gs_persist   = false;
    dn->gs_rx_active = false;
    dn->gs_rx_src    = 0u;
    dn->gs_rx_tid    = 0u;
    dn->gs_rx_toggle = false;
    dn->gs_rx_frames = 0u;
    dn->gs_rx_len    = 0u;

    dn->tid_alloc         = 0u;
    dn->dna_primed        = false;
    dn->dna_t0            = 0u;
    dn->dna_req_sent      = false;
    dn->dna_last_req_ms   = 0u;
    dn->dna_confirmed_len = 0u;
    dn->dna_rx_active     = false;
    dn->dna_rx_src        = 0u;
    dn->dna_rx_tid        = 0u;
    dn->dna_rx_toggle     = false;
    dn->dna_rx_frames     = 0u;
    dn->dna_rx_len        = 0u;
}

uint16_t dronecan_node_id(const dronecan_t *dn)      { return dn->node_id; }
bool     dronecan_node_id_dirty(const dronecan_t *dn){ return dn->node_id_dirty; }
void     dronecan_clear_node_id_dirty(dronecan_t *dn){ dn->node_id_dirty = false; }
bool     dronecan_param_dirty(const dronecan_t *dn)  { return dn->gs_persist; }
void     dronecan_clear_param_dirty(dronecan_t *dn)  { dn->gs_persist = false; }

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

/* Serialize the 7-byte uavcan.protocol.NodeStatus body (shared by the NodeStatus message
 * and the status field embedded at the front of a GetNodeInfo response). */
static void pack_node_status_body(dronecan_payload_t *p, uint32_t now_ms, const esc_telemetry_t *tel)
{
    uint16_t health = (tel->hard_fault_bits != 0u) ? DRONECAN_HEALTH_ERROR : DRONECAN_HEALTH_OK;
    uint16_t vendor = (uint16_t)(tel->hard_fault_bits & 0xFFFFu);

    dronecan_pack_uint(p, now_ms / 1000u, 32u);           /* uptime_sec */
    dronecan_pack_uint(p, health, 2u);                    /* health */
    dronecan_pack_uint(p, DRONECAN_MODE_OPERATIONAL, 3u); /* mode */
    dronecan_pack_uint(p, 0u, 3u);                        /* sub_mode */
    dronecan_pack_uint(p, vendor, 16u);                   /* vendor_specific_status_code */
}

/* Build the single NodeStatus frame for the given uptime/health. */
static void build_node_status(dronecan_t *dn, uint32_t now_ms, const esc_telemetry_t *tel,
                              dronecan_frame_t *f)
{
    dronecan_payload_t p;
    uint16_t plen, i;

    dronecan_payload_init(&p);
    pack_node_status_body(&p, now_ms, tel);
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

/* Build the anonymous DNA allocation request for the current stage (next unique-id chunk). */
static void build_dna_request(dronecan_t *dn, dronecan_frame_t *f)
{
    dronecan_payload_t p;
    uint16_t start = dn->dna_confirmed_len;
    uint16_t remn = (uint16_t)((start <= 16u) ? (16u - start) : 0u);
    uint16_t chunk = (remn >= 6u) ? (uint16_t)6 : remn;
    bool first = (start == 0u);
    uint16_t plen, disc, i;

    dronecan_payload_init(&p);
    dronecan_pack_uint(&p, 0u, 7u);                       /* node_id = 0 (requesting) */
    dronecan_pack_uint(&p, first ? 1u : 0u, 1u);          /* first_part_of_unique_id */
    dronecan_pack_bytes(&p, &dn->cfg.unique_id[start], chunk);
    plen = dronecan_payload_bytelen(&p);

    disc = (uint16_t)(dronecan_crc16(p.bytes, plen, 0xFFFFu) & 0x3FFFu);
    f->extended = true;
    f->id = dronecan_anon_id(DRONECAN_PRIO_ALLOCATION, DRONECAN_DTID_ALLOCATION, disc);
    for (i = 0; i < plen; ++i) {
        f->data[i] = p.bytes[i];
    }
    f->data[plen] = dronecan_tail_encode(true, true, false, dn->tid_alloc);
    f->dlc = (uint16_t)(plen + 1u);
}

/* Append val's low nbytes little-endian into buf at *pos. */
static void buf_put_le(uint16_t *buf, uint16_t *pos, uint32_t val, uint16_t nbytes)
{
    uint16_t i;
    for (i = 0; i < nbytes; ++i) {
        buf[(*pos)++] = (uint16_t)((val >> (uint16_t)(8u * i)) & 0xFFu);
    }
}

/*
 * Build the GetNodeInfo response transfer (multi-frame) into out[0..cap). Returns the frame
 * count, or 0 if the caller cannot hold the whole transfer (it then stays pending). The whole
 * uavcan.protocol.GetNodeInfo.Response is byte-aligned, so it is assembled as raw wire bytes:
 *   status (NodeStatus, 7) | software_version (15) | hardware_version (2 + uid[16] + cert_len=0)
 *   | name (tail array, no length prefix).
 * A service response echoes the request's transfer-id (gni_tid) and priority (gni_prio) and is
 * addressed back to the requester (gni_dst); the source is our node id.
 */
static int build_node_info(dronecan_t *dn, uint32_t now_ms, const esc_telemetry_t *tel,
                           dronecan_frame_t *out, int cap)
{
    dronecan_payload_t ns;
    uint16_t buf[2u + 7u + 15u + 19u + DRONECAN_NODE_NAME_MAX]; /* [0..1]=transfer CRC, [2..]=payload */
    uint16_t pos = 0u, nslen, i, seed, crc, total, off, fidx, flags;
    int need;
    uint32_t id;
    const char *nm = dn->cfg.node_name;

    /* status: 7-byte NodeStatus body */
    dronecan_payload_init(&ns);
    pack_node_status_body(&ns, now_ms, tel);
    nslen = dronecan_payload_bytelen(&ns);
    for (i = 0; i < nslen; ++i) { buf[2u + pos++] = ns.bytes[i]; }

    /* software_version: major, minor, optional_field_flags, vcs_commit(u32), image_crc(u64) */
    flags = (dn->cfg.sw_vcs_commit != 0u) ? 1u : 0u; /* bit0 = VCS_COMMIT present; image_crc unset */
    buf[2u + pos++] = (uint16_t)(dn->cfg.sw_version_major & 0xFFu);
    buf[2u + pos++] = (uint16_t)(dn->cfg.sw_version_minor & 0xFFu);
    buf[2u + pos++] = (uint16_t)(flags & 0xFFu);
    buf_put_le(&buf[2], &pos, dn->cfg.sw_vcs_commit, 4u);
    for (i = 0; i < 8u; ++i) { buf[2u + pos++] = 0u; } /* image_crc = 0 */

    /* hardware_version: major, minor, unique_id[16], certificate_of_authenticity (len=0) */
    buf[2u + pos++] = (uint16_t)(dn->cfg.hw_version_major & 0xFFu);
    buf[2u + pos++] = (uint16_t)(dn->cfg.hw_version_minor & 0xFFu);
    for (i = 0; i < 16u; ++i) { buf[2u + pos++] = (uint16_t)(dn->cfg.unique_id[i] & 0xFFu); }
    buf[2u + pos++] = 0u; /* certificate_of_authenticity length (uint8) */

    /* name: tail array (last field) -> no length prefix, just the bytes */
    if (nm != NULL) {
        uint16_t k = 0u;
        while (nm[k] != '\0' && k < DRONECAN_NODE_NAME_MAX) {
            buf[2u + pos++] = (uint16_t)(nm[k] & 0xFFu);
            k++;
        }
    }

    /* multi-frame transfer CRC over the serialized payload (signature-seeded), CRC at front */
    seed = dronecan_transfer_crc_seed(DRONECAN_GET_NODE_INFO_SIG_LO, DRONECAN_GET_NODE_INFO_SIG_HI);
    crc = dronecan_crc16(&buf[2], pos, seed);
    buf[0] = (uint16_t)(crc & 0xFFu);
    buf[1] = (uint16_t)((crc >> 8) & 0xFFu);
    total = (uint16_t)(pos + 2u);

    need = (int)((total + 6u) / 7u); /* 7 payload bytes per frame (8 - tail) */
    if (cap < need) {
        return 0; /* all-or-nothing: stay pending until the whole transfer fits */
    }

    id = dronecan_svc_id(dn->gni_prio, DRONECAN_STID_GET_NODE_INFO, false /* response */,
                         dn->gni_dst, dn->node_id);
    off = 0u;
    fidx = 0u;
    while (off < total) {
        uint16_t remn = (uint16_t)(total - off);
        uint16_t chunk = (remn >= 7u) ? (uint16_t)7 : remn;
        dronecan_frame_t *f = &out[fidx];
        bool sot = (fidx == 0u);
        bool eot = ((off + chunk) >= total);
        bool toggle = ((fidx & 1u) != 0u);
        f->extended = true;
        f->id = id;
        for (i = 0; i < chunk; ++i) {
            f->data[i] = buf[off + i];
        }
        f->data[chunk] = dronecan_tail_encode(sot, eot, toggle, dn->gni_tid);
        f->dlc = (uint16_t)(chunk + 1u);
        off = (uint16_t)(off + chunk);
        fidx++;
    }
    return (int)fidx;
}

/*
 * Frame the queued GetSet response payload (dn->gs_resp) back to the requester. Single-frame
 * (<=7 payload bytes) carries no transfer CRC; multi-frame prepends the 2-byte signature-seeded
 * CRC, exactly like build_node_info. All-or-nothing: returns 0 (stay pending) if cap is short.
 */
static int build_get_set_response(dronecan_t *dn, dronecan_frame_t *out, int cap)
{
    uint16_t buf[2u + DRONECAN_PARAM_RESP_MAX];
    uint16_t total, off, fidx, i;
    int need;
    uint32_t id;
    bool multi = (dn->gs_resp_len > 7u);

    if (multi) {
        uint16_t seed = dronecan_transfer_crc_seed(DRONECAN_GET_SET_SIG_LO, DRONECAN_GET_SET_SIG_HI);
        uint16_t crc  = dronecan_crc16(dn->gs_resp, dn->gs_resp_len, seed);
        buf[0] = (uint16_t)(crc & 0xFFu);
        buf[1] = (uint16_t)((crc >> 8) & 0xFFu);
        for (i = 0; i < dn->gs_resp_len; ++i) { buf[2u + i] = dn->gs_resp[i]; }
        total = (uint16_t)(dn->gs_resp_len + 2u);
    } else {
        for (i = 0; i < dn->gs_resp_len; ++i) { buf[i] = dn->gs_resp[i]; }
        total = dn->gs_resp_len;
    }

    need = (int)((total + 6u) / 7u);
    if (cap < need) {
        return 0;
    }

    id = dronecan_svc_id(dn->gs_prio, DRONECAN_STID_GET_SET, false /* response */,
                         dn->gs_dst, dn->node_id);
    off = 0u;
    fidx = 0u;
    while (off < total) {
        uint16_t remn = (uint16_t)(total - off);
        uint16_t chunk = (remn >= 7u) ? (uint16_t)7 : remn;
        dronecan_frame_t *f = &out[fidx];
        bool sot = (fidx == 0u);
        bool eot = ((off + chunk) >= total);
        bool toggle = ((fidx & 1u) != 0u);
        f->extended = true;
        f->id = id;
        for (i = 0; i < chunk; ++i) { f->data[i] = buf[off + i]; }
        f->data[chunk] = dronecan_tail_encode(sot, eot, toggle, dn->gs_tid);
        f->dlc = (uint16_t)(chunk + 1u);
        off = (uint16_t)(off + chunk);
        fidx++;
    }
    return (int)fidx;
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
        /* Unallocated: emit DNA allocation requests (anonymous). No telemetry needed. */
        if (!dn->dna_primed) {
            dn->dna_primed = true;
            dn->dna_t0 = now_ms;
        }
        if ((uint32_t)(now_ms - dn->dna_t0) < dn->cfg.dna_start_delay_ms) {
            return 0; /* still within the start delay */
        }
        if (!dn->dna_req_sent
            || (uint32_t)(now_ms - dn->dna_last_req_ms) >= dn->cfg.dna_request_period_ms) {
            build_dna_request(dn, &out[0]);
            dn->dna_last_req_ms = now_ms;
            dn->dna_req_sent = true;
            dn->tid_alloc = (uint16_t)((dn->tid_alloc + 1u) & 0x1Fu);
            return 1;
        }
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

    /* Highest priority: answer a pending GetNodeInfo request. All-or-nothing -- if the whole
     * multi-frame response does not fit in the remaining slots it stays pending for a later tick
     * (so a caller with a large enough cap enumerates promptly). */
    if (dn->gni_pending) {
        int got = build_node_info(dn, now_ms, tel, &out[n], cap - n);
        if (got > 0) {
            n += got;
            dn->gni_pending = false;
        }
    }

    /* Answer a pending GetSet (param) request. All-or-nothing like GetNodeInfo. */
    if (dn->gs_pending) {
        int got = build_get_set_response(dn, &out[n], cap - n);
        if (got > 0) {
            n += got;
            dn->gs_pending = false;
        }
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

/* Reassemble and process an incoming Allocation transfer (from the allocator). */
static void handle_allocation(dronecan_t *dn, const dronecan_frame_t *f)
{
    dronecan_tail_t tail;
    uint16_t src, paylen, i, start_byte, n, bp, node_id, uid_len;
    dronecan_payload_t p;

    if (dn->node_id != 0u || f->dlc < 1u) {
        return; /* already allocated, or no tail */
    }
    src = dronecan_id_source(f->id);
    if (src == 0u) {
        return; /* responses come from a real allocator, not anonymous */
    }
    tail = dronecan_tail_decode(f->data[f->dlc - 1u]);
    paylen = (uint16_t)(f->dlc - 1u);

    if (tail.sot && tail.toggle) {
        return; /* first frame must have toggle=0 (tail is not covered by the transfer CRC) */
    }

    if (tail.sot) {
        dn->dna_rx_active = true;
        dn->dna_rx_src    = src;
        dn->dna_rx_tid    = tail.transfer_id;
        dn->dna_rx_len    = 0u;
        dn->dna_rx_frames = 0u;
        dn->dna_rx_toggle = false;
    } else {
        if (!dn->dna_rx_active || dn->dna_rx_src != src || dn->dna_rx_tid != tail.transfer_id) {
            return; /* not part of the active transfer */
        }
        if (tail.toggle != dn->dna_rx_toggle) {
            return; /* toggle mismatch -> drop */
        }
    }

    for (i = 0; i < paylen && dn->dna_rx_len < DRONECAN_PAYLOAD_MAX; ++i) {
        dn->dna_rx_buf[dn->dna_rx_len++] = (uint16_t)(f->data[i] & 0xFFu);
    }
    dn->dna_rx_frames++;
    dn->dna_rx_toggle = !dn->dna_rx_toggle;

    if (!tail.eot) {
        return; /* wait for the rest of the transfer */
    }
    dn->dna_rx_active = false;

    /* Multi-frame transfers carry a 2-byte transfer CRC at the front; single frame does not. */
    start_byte = (dn->dna_rx_frames > 1u) ? 2u : 0u;
    if (dn->dna_rx_len < (uint16_t)(start_byte + 1u)) {
        return;
    }
    if (start_byte == 2u) {
        /* Validate the multi-frame transfer CRC before trusting the payload. */
        uint16_t seed = dronecan_transfer_crc_seed(DRONECAN_ALLOCATION_SIG_LO,
                                                   DRONECAN_ALLOCATION_SIG_HI);
        uint16_t want = (uint16_t)(dn->dna_rx_buf[0] | (dn->dna_rx_buf[1] << 8));
        uint16_t got  = dronecan_crc16(&dn->dna_rx_buf[2],
                                       (uint16_t)(dn->dna_rx_len - 2u), seed);
        if (got != want) {
            return; /* corrupted / mismatched transfer -> drop */
        }
    }

    dronecan_payload_init(&p);
    n = 0u;
    for (i = start_byte; i < dn->dna_rx_len; ++i) {
        p.bytes[n++] = dn->dna_rx_buf[i];
    }
    p.bit_len = (uint16_t)(n * 8u);

    bp = 0u;
    node_id = (uint16_t)dronecan_unpack_uint(&p, &bp, 7u);
    (void)dronecan_unpack_uint(&p, &bp, 1u);           /* first_part_of_unique_id */
    uid_len = (uint16_t)(n - 1u);                      /* remaining bytes = echoed unique id */

    /* The echoed unique id must be a prefix of ours, else this allocation is for another node. */
    for (i = 0; i < uid_len && i < 16u; ++i) {
        uint16_t b = (uint16_t)dronecan_unpack_uint(&p, &bp, 8u);
        if (b != (uint16_t)(dn->cfg.unique_id[i] & 0xFFu)) {
            return;
        }
    }

    if (node_id != 0u) {
        dn->node_id = node_id;       /* allocated */
        dn->node_id_dirty = true;    /* product main persists it to NV */
    } else if (uid_len > dn->dna_confirmed_len) {
        dn->dna_confirmed_len = uid_len;
        dn->dna_req_sent = false;    /* echo confirmed -> send the next chunk on the next tick */
    }
}

/* Record a pending GetNodeInfo response for a request addressed to us. The response is emitted
 * by dronecan_tick (the sole frame emitter), echoing the request transfer-id/priority. */
static void handle_get_node_info(dronecan_t *dn, const dronecan_frame_t *f)
{
    dronecan_tail_t tail;

    if (dn->node_id == 0u) {
        return; /* unallocated: no source id to answer with yet */
    }
    if (!dronecan_id_is_request(f->id)) {
        return; /* a response (someone else's), not a request -> ignore */
    }
    if (dronecan_id_source(f->id) == 0u || dronecan_id_dest(f->id) != dn->node_id || f->dlc < 1u) {
        return; /* must be from a real node, addressed to us, and carry a tail byte */
    }
    tail = dronecan_tail_decode(f->data[f->dlc - 1u]);
    dn->gni_dst     = dronecan_id_source(f->id);
    dn->gni_tid     = tail.transfer_id;          /* response echoes the request transfer id */
    dn->gni_prio    = dronecan_id_priority(f->id);
    dn->gni_pending = true;
}

/*
 * Reassemble an incoming param.GetSet REQUEST addressed to us and, on the last frame, decode +
 * apply it (dronecan_param_build_response writes nvparam via the #4 rules) and queue the response
 * for dronecan_tick. Mirrors handle_allocation's reassembly: single-frame has no transfer CRC,
 * multi-frame carries a 2-byte signature-seeded CRC at the front.
 */
static void handle_get_set(dronecan_t *dn, const dronecan_frame_t *f)
{
    dronecan_tail_t tail;
    uint16_t src, paylen, i, start_byte;
    bool persist = false;

    if (dn->node_id == 0u) {
        return; /* unallocated: no source id to answer with yet */
    }
    if (!dronecan_id_is_request(f->id)) {
        return; /* a response, not a request */
    }
    src = dronecan_id_source(f->id);
    if (src == 0u || dronecan_id_dest(f->id) != dn->node_id || f->dlc < 1u) {
        return; /* must be from a real node, addressed to us, with a tail byte */
    }
    tail   = dronecan_tail_decode(f->data[f->dlc - 1u]);
    paylen = (uint16_t)(f->dlc - 1u);

    if (tail.sot && tail.toggle) {
        return; /* first frame must have toggle=0 */
    }
    if (tail.sot) {
        dn->gs_rx_active = true;
        dn->gs_rx_src    = src;
        dn->gs_rx_tid    = tail.transfer_id;
        dn->gs_rx_len    = 0u;
        dn->gs_rx_frames = 0u;
        dn->gs_rx_toggle = false;
        dn->gs_prio      = dronecan_id_priority(f->id);  /* echoed in the response */
    } else {
        if (!dn->gs_rx_active || dn->gs_rx_src != src || dn->gs_rx_tid != tail.transfer_id) {
            return; /* not part of the active transfer */
        }
        if (tail.toggle != dn->gs_rx_toggle) {
            return; /* toggle mismatch -> drop */
        }
    }

    for (i = 0; i < paylen && dn->gs_rx_len < DRONECAN_PARAM_REQ_MAX; ++i) {
        dn->gs_rx_buf[dn->gs_rx_len++] = (uint16_t)(f->data[i] & 0xFFu);
    }
    dn->gs_rx_frames++;
    dn->gs_rx_toggle = !dn->gs_rx_toggle;

    if (!tail.eot) {
        return; /* wait for the rest */
    }
    dn->gs_rx_active = false;

    /* Multi-frame transfers carry a 2-byte transfer CRC at the front; single frame does not. */
    start_byte = (dn->gs_rx_frames > 1u) ? 2u : 0u;
    if (dn->gs_rx_len < (uint16_t)(start_byte + 2u)) {
        return; /* too short to be a GetSet request (need at least index + value tag) */
    }
    if (start_byte == 2u) {
        uint16_t seed = dronecan_transfer_crc_seed(DRONECAN_GET_SET_SIG_LO, DRONECAN_GET_SET_SIG_HI);
        uint16_t want = (uint16_t)(dn->gs_rx_buf[0] | (dn->gs_rx_buf[1] << 8));
        uint16_t got  = dronecan_crc16(&dn->gs_rx_buf[2], (uint16_t)(dn->gs_rx_len - 2u), seed);
        if (got != want) {
            return; /* corrupted transfer -> drop */
        }
    }

    dn->gs_resp_len = dronecan_param_build_response(
        dn->cfg.nvparam, &dn->gs_rx_buf[start_byte],
        (uint16_t)(dn->gs_rx_len - start_byte),
        dn->gs_resp, DRONECAN_PARAM_RESP_MAX, &persist);
    if (dn->gs_resp_len == 0u) {
        return; /* nothing to send (bad args) */
    }
    dn->gs_dst     = src;
    dn->gs_tid     = tail.transfer_id;   /* response echoes the request transfer id */
    dn->gs_pending = true;
    if (persist) {
        dn->gs_persist = true;           /* a Set changed nvparam -> app persists to Flash */
    }
}

void dronecan_on_rx(dronecan_t *dn, const dronecan_frame_t *f, dronecan_rx_result_t *res)
{
    uint16_t dtid;

    if (res == NULL) {
        return;
    }
    res->command_updated = false;

    if (dn == NULL || f == NULL) {
        return;
    }
    if (f->dlc > 8u) {
        return; /* malformed: more data bytes than a CAN frame can hold */
    }
    if (!f->extended) {
        return; /* DroneCAN is always extended-id */
    }
    if (dronecan_id_is_service(f->id)) {
        /* Services carry no RawCommand; we answer GetNodeInfo and param.GetSet. */
        uint16_t stid = dronecan_id_svc_type(f->id);
        if (stid == DRONECAN_STID_GET_NODE_INFO) {
            handle_get_node_info(dn, f);
        } else if (stid == DRONECAN_STID_GET_SET) {
            handle_get_set(dn, f);
        }
        return;
    }

    dtid = dronecan_id_msg_dtid(f->id);
    if (dtid == DRONECAN_DTID_RAW_COMMAND) {
        if (dn->node_id == 0u) {
            /* Not yet allocated: we cannot publish NodeStatus/esc.Status, so we must not
             * arm or drive either. Drop RawCommand entirely -- do NOT advance the zero-frame
             * handshake or seq -- so arming starts fresh once DNA assigns a node id. */
            return;
        }
        if (dronecan_id_source(f->id) == 0u) {
            return; /* RawCommand must come from a real (non-anonymous) node */
        }
        handle_raw_command(dn, f, res);
    } else if (dtid == DRONECAN_DTID_ALLOCATION) {
        handle_allocation(dn, f);
    }
}
