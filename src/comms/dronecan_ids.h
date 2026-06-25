/*
 * dronecan_ids.h - DroneCAN/UAVCAN v0 protocol constants for the esc6288 node.
 *
 * Hand-rolled minimal DroneCAN stack (no libcanard: C28x CHAR_BIT==16 breaks libcanard's
 * uint8_t DSDL packing). Values are the canonical message contract used by ArduPilot and the
 * legacy esc_drv8300 firmware; all wire encodings are validated by golden frames.
 */
#ifndef DRONECAN_IDS_H
#define DRONECAN_IDS_H

/* Data type IDs (DTID) */
#define DRONECAN_DTID_NODE_STATUS     341u    /* uavcan.protocol.NodeStatus */
#define DRONECAN_DTID_ESC_STATUS      1034u   /* uavcan.equipment.esc.Status */
#define DRONECAN_DTID_RAW_COMMAND     1030u   /* uavcan.equipment.esc.RawCommand */
#define DRONECAN_DTID_ALLOCATION      1u      /* uavcan.protocol.dynamic_node_id.Allocation */

/* Service type IDs (STID) */
#define DRONECAN_STID_GET_NODE_INFO   1u      /* uavcan.protocol.GetNodeInfo (service) */
#define DRONECAN_STID_GET_SET         11u     /* uavcan.protocol.param.GetSet (service) */

/* Transfer priorities (lower = higher priority) */
#define DRONECAN_PRIO_NODE_STATUS     16u
#define DRONECAN_PRIO_ESC_STATUS      16u
#define DRONECAN_PRIO_ALLOCATION      30u

/* 64-bit data type signatures (transfer-CRC seed source), split for the C28x. */
#define DRONECAN_ESC_STATUS_SIG_LO    0xA2FBB254u
#define DRONECAN_ESC_STATUS_SIG_HI    0xA9AF28AEu
#define DRONECAN_ALLOCATION_SIG_LO    0x20A11D40u
#define DRONECAN_ALLOCATION_SIG_HI    0x0B2A8126u
#define DRONECAN_GET_NODE_INFO_SIG_LO 0x21C46A9Eu  /* sig 0xEE468A8121C46A9E */
#define DRONECAN_GET_NODE_INFO_SIG_HI 0xEE468A81u
#define DRONECAN_GET_SET_SIG_LO       0x39D1A4D5u  /* sig 0xA7B622F939D1A4D5 */
#define DRONECAN_GET_SET_SIG_HI       0xA7B622F9u

/* uavcan.protocol.NodeStatus health / mode */
#define DRONECAN_HEALTH_OK            0u
#define DRONECAN_HEALTH_WARNING       1u
#define DRONECAN_HEALTH_ERROR         2u
#define DRONECAN_HEALTH_CRITICAL      3u
#define DRONECAN_MODE_OPERATIONAL     0u

/* esc.RawCommand */
#define DRONECAN_RAWCMD_FULLSCALE     8191    /* int14 positive full scale */
#define DRONECAN_RAWCMD_MAX_ESC       20      /* int14[<=20] */
#define DRONECAN_ESC_INDEX_MAX        19u     /* valid esc_index 0..19 */

/* Celsius <-> Kelvin (esc.Status temperature is in Kelvin) */
#define DRONECAN_KELVIN_OFFSET        273.15f

#endif /* DRONECAN_IDS_H */
