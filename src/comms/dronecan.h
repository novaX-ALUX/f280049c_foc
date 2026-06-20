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

typedef struct {
    uint16_t esc_index;            /* 0..19; our index into the RawCommand array */
    uint16_t unique_id[16];        /* 16-byte node unique id (low 8 bits each) for DNA */
    uint16_t arm_zero_frames;      /* consecutive zero commands required to arm (0 -> default) */
    uint16_t node_id;              /* 1..127 static; anything else -> dynamic (DNA) */
    uint32_t node_status_period_ms;/* 0 -> default 1000 */
    uint32_t esc_status_period_ms; /* 0 -> default 100 */
    uint32_t dna_request_period_ms;/* 0 -> default 1000 */
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
} dronecan_t;

void dronecan_init(dronecan_t *dn, const dronecan_cfg_t *cfg);

/* Feed one received CAN frame. Updates res->command_updated/command for a RawCommand
 * addressed to our esc_index; advances DNA on allocation replies; ignores everything else. */
void dronecan_on_rx(dronecan_t *dn, const dronecan_frame_t *f, dronecan_rx_result_t *res);

uint16_t dronecan_node_id(const dronecan_t *dn);
bool     dronecan_node_id_dirty(const dronecan_t *dn);
void     dronecan_clear_node_id_dirty(dronecan_t *dn);

#endif /* DRONECAN_H */
