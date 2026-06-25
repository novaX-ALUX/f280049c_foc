/*
 * dronecan_param.h - uavcan.protocol.param.GetSet codec for the bring-up parameter face.
 *
 * A MINIMAL remote-access face over the local nvparam contract (src/app/nvparam). DroneCAN is
 * only the transport: every write is funnelled through nvparam_update_* so the validation rules
 * (#4) stay the single source of truth -- this module never re-defines what a legal value is.
 *
 * Exposed parameters (first bring-up batch):
 *   [0] "node_id"             integer 0..127 (0 = DNA)        -> nvparam.node_id
 *   [1] "park_ref_valid"      boolean                          -> nvparam.park_ref_valid
 *   [2] "park_ref_target_rev" real (mechanical revolutions)    -> nvparam.park_ref_target_rev
 *
 * Wire format follows the pydronecan reference serializer (the repo's interop anchor); the
 * GetSet REQUEST uses a 3-bit Value union tag while the RESPONSE emits byte-aligned (8-bit)
 * union tags -- an asymmetry observed from pydronecan 1.0.27 and locked by golden vectors
 * (tools/test/dronecan_param_golden.inc). Interop with the actual flight controller is a
 * bench item.
 *
 * Pure: byte buffers in, byte buffers out (one wire byte per uint16_t, C28x convention). The
 * transport (multi-frame reassembly, transfer CRC, framing) lives in dronecan.c.
 */
#ifndef DRONECAN_PARAM_H
#define DRONECAN_PARAM_H

#include "dronecan_frame.h"
#include "nvparam.h"

#define DRONECAN_PARAM_COUNT     3u
#define DRONECAN_PARAM_NAME_MAX  20u   /* "park_ref_target_rev" = 19 chars + slack */
#define DRONECAN_PARAM_REQ_MAX   48u   /* largest GetSet request we accept (value + name) */
#define DRONECAN_PARAM_RESP_MAX  64u   /* largest response we emit (node_id: ~43 bytes) */

/* Number of exposed parameters / a parameter's name (NULL if idx out of range). */
uint16_t    dronecan_param_count(void);
const char *dronecan_param_name(uint16_t idx);

/*
 * Decode a reassembled GetSet request payload (req[0..req_len), one wire byte per element),
 * apply any Set through nvparam_update_* on *nv, and serialize the GetSet response payload into
 * out[0..ret). Selection: a non-empty request name wins; otherwise the request index is used;
 * an unknown name/index yields the canonical empty response (empty value + empty name).
 *
 * Returns the response byte length (<= out_cap), or 0 on bad args. *persist_needed is set true
 * iff a Set actually changed *nv (the caller then persists nvparam to Flash). nv may be NULL
 * (no param face): then every request gets the empty response and *persist_needed stays false.
 */
uint16_t dronecan_param_build_response(nvparam_t *nv,
                                       const uint16_t *req, uint16_t req_len,
                                       uint16_t *out, uint16_t out_cap,
                                       bool *persist_needed);

#endif /* DRONECAN_PARAM_H */
