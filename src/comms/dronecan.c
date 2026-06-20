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
