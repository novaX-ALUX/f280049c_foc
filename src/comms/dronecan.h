/*
 * dronecan.h - Hand-rolled DroneCAN v0 protocol core (frame-level, pure, host-tested).
 *
 * Consumes/produces raw CAN frame DTOs and bridges them to the product app types:
 *   RX RawCommand  -> esc_command_t   (throttle/arm via a zero-frame arming handshake)
 *   TX esc.Status / NodeStatus  <- esc_telemetry_t
 *   DNA allocatee  (dynamic node-id)
 *
 * No CAN peripheral, no driverlib, no product main: this is the protocol logic only. Command
 * timeout is NOT handled here (the app's esc_control owns it); comms only delivers fresh frames.
 */
#ifndef DRONECAN_H
#define DRONECAN_H

#include "esc_types.h"
#include "dronecan_frame.h"
#include "dronecan_param.h"   /* nvparam_t + GetSet codec sizing for the param face */

/* Max GetNodeInfo node name length we will emit (DSDL allows <=80). Bounds the response
 * frame count; the TX caller (dronecan_tick cap) must hold the whole multi-frame transfer. */
#define DRONECAN_NODE_NAME_MAX 32

typedef struct {
    uint16_t esc_index;            /* 0..19; our index into the RawCommand array */
    uint16_t unique_id[16];        /* 16-byte node unique id (low 8 bits each) for DNA + GetNodeInfo */
    uint16_t arm_zero_frames;      /* consecutive zero commands required to arm (0 -> default) */
    uint16_t node_id;              /* 1..127 static; anything else -> dynamic (DNA) */
    uint32_t node_status_period_ms;/* 0 -> default 1000 */
    uint32_t esc_status_period_ms; /* 0 -> default 100 */
    uint32_t dna_request_period_ms;/* 0 -> default 1000 */
    uint32_t dna_start_delay_ms;   /* delay before the first allocation request */

    /* GetNodeInfo response identity (so the node enumerates on ArduPilot / yakut / DroneCAN GUI). */
    uint16_t hw_version_major;     /* HardwareVersion.major */
    uint16_t hw_version_minor;     /* HardwareVersion.minor */
    uint16_t sw_version_major;     /* SoftwareVersion.major */
    uint16_t sw_version_minor;     /* SoftwareVersion.minor */
    uint32_t sw_vcs_commit;        /* SoftwareVersion.vcs_commit; 0 -> optional VCS flag clear */
    const char *node_name;         /* reverse-DNS node name (<=DRONECAN_NODE_NAME_MAX, static
                                    * lifetime; the cfg copy keeps the pointer); NULL -> empty */

    /* GetSet parameter face: the in-RAM nvparam mirror this node exposes/edits over DroneCAN.
     * NULL disables the param face (GetSet requests then get the canonical empty response).
     * The pointer must outlive the dronecan_t; writes go through nvparam_update_* (#4 rules). */
    nvparam_t *nvparam;
} dronecan_cfg_t;

typedef struct {
    bool          command_updated;
    esc_command_t command;
} dronecan_rx_result_t;

typedef struct {
    dronecan_cfg_t cfg;

    uint16_t node_id;        /* effective node id (0 until DNA allocates) */
    bool     node_id_dirty;  /* a freshly allocated id awaits NV persist (set by DNA) */

    /* RawCommand RX + arming */
    uint32_t seq;            /* esc_command_t.seq, ++ on every accepted RawCommand for us */
    uint16_t zero_run;       /* consecutive zero commands seen before arming */
    bool     armed;          /* zero-frame handshake passed */

    /* TX scheduling */
    bool     sched_primed;       /* timers seeded so the first eligible tick sends */
    uint32_t last_node_status_ms;
    uint32_t last_esc_status_ms;
    uint16_t tid_node_status;    /* 5-bit transfer ids */
    uint16_t tid_esc_status;

    /* Pending GetNodeInfo response (echoes the request's transfer-id/priority back to it) */
    bool     gni_pending;
    uint16_t gni_dst;            /* requester node id (response destination) */
    uint16_t gni_tid;            /* request transfer id, echoed in the response */
    uint16_t gni_prio;           /* request priority, echoed in the response */

    /* GetSet (param.GetSet service): pending response + incoming request reassembly. */
    bool     gs_pending;         /* a serialized response payload awaits TX */
    uint16_t gs_dst, gs_tid, gs_prio;  /* requester id / echoed transfer id / echoed priority */
    uint16_t gs_resp[DRONECAN_PARAM_RESP_MAX];
    uint16_t gs_resp_len;
    bool     gs_persist;         /* a Set changed nvparam -> caller persists (sticky) */
    bool     gs_rx_active;
    uint16_t gs_rx_src, gs_rx_tid;
    bool     gs_rx_toggle;
    uint16_t gs_rx_frames, gs_rx_len;
    uint16_t gs_rx_buf[DRONECAN_PARAM_REQ_MAX];

    /* DNA (dynamic node-id allocatee) */
    uint16_t tid_alloc;
    bool     dna_primed;         /* dna_t0 captured */
    uint32_t dna_t0;             /* time of first tick (start-delay origin) */
    bool     dna_req_sent;       /* a request for the current stage is outstanding */
    uint32_t dna_last_req_ms;
    uint16_t dna_confirmed_len;  /* unique-id bytes the allocator has echoed (0/6/12) */
    /* incoming Allocation reassembly */
    bool     dna_rx_active;
    uint16_t dna_rx_src;
    uint16_t dna_rx_tid;
    bool     dna_rx_toggle;
    uint16_t dna_rx_frames;
    uint16_t dna_rx_len;
    uint16_t dna_rx_buf[DRONECAN_PAYLOAD_MAX];
} dronecan_t;

void dronecan_init(dronecan_t *dn, const dronecan_cfg_t *cfg);

/* Feed one received CAN frame. Updates res->command_updated/command for a RawCommand
 * addressed to our esc_index; advances DNA on allocation replies; ignores everything else. */
void dronecan_on_rx(dronecan_t *dn, const dronecan_frame_t *f, dronecan_rx_result_t *res);

/*
 * Emit any due TX frames into out[0..cap). Returns the number written.
 * - A transfer is all-or-nothing: esc.Status needs 3 free slots, single-frame messages need 1;
 *   if there is no room the transfer stays due (timers / transfer-id are NOT advanced).
 * - Priority: unallocated -> DNA only; allocated -> NodeStatus before esc.Status.
 * - tel may be NULL only while unallocated (DNA needs no telemetry); once allocated a NULL tel
 *   suppresses NodeStatus and Status. Bad args (dn NULL, or out NULL / cap<=0) -> returns 0.
 */
int dronecan_tick(dronecan_t *dn, uint32_t now_ms, const esc_telemetry_t *tel,
                  dronecan_frame_t *out, int cap);

uint16_t dronecan_node_id(const dronecan_t *dn);
bool     dronecan_node_id_dirty(const dronecan_t *dn);
void     dronecan_clear_node_id_dirty(dronecan_t *dn);

/* A GetSet write changed the nvparam mirror and should be persisted to Flash by the app. */
bool     dronecan_param_dirty(const dronecan_t *dn);
void     dronecan_clear_param_dirty(dronecan_t *dn);

#endif /* DRONECAN_H */
