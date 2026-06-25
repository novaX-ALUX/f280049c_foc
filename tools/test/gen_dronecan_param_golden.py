#!/usr/bin/env python3
"""
Golden generator for the GetSet parameter face (src/comms/dronecan_param + transport).

Emits tools/test/dronecan_param_golden.inc (checked in) from pydronecan's reference
serializer, the repo's interop anchor. Two layers:
  - CODEC: GetSet request payload -> expected GetSet response payload (pre-framing bytes),
    so test_dronecan_param checks the pure DSDL (de)serialization byte-for-byte.
  - TRANSPORT: full request frame(s) in -> full response frame(s) out, so test_dronecan
    checks multi-frame reassembly, transfer CRC, and framing end-to-end.

Run:  python3 tools/test/gen_dronecan_param_golden.py > tools/test/dronecan_param_golden.inc
Requires pydronecan 1.0.27.
"""
import sys
import dronecan
from dronecan import uavcan
from dronecan.transport import Transfer

PYVER = dronecan.__version__
P = uavcan.protocol.param
OUR_NODE = 25
REQUESTER = 42


# A "value spec" is a (kind, x) tuple applied to a nested union field in place; pydronecan
# rejects whole-union assignment, so we mutate the target union's member attribute directly.
def apply_value(union, spec):
    if spec is None:
        return
    kind, x = spec
    setattr(union, kind, x)


def vint(x):   return ("integer_value", x)
def vreal(x):  return ("real_value", x)
def vbool(x):  return ("boolean_value", x)
def nint(x):   return ("integer_value", x)
def nreal(x):  return ("real_value", x)


def req_payload(index=0, name="", value=None):
    r = P.GetSet.Request(index=index)
    if name:
        r.name = name
    apply_value(r.value, value)
    t = Transfer(payload=r, source_node_id=REQUESTER, dest_node_id=OUR_NODE,
                 transfer_id=1, transfer_priority=16, service_not_message=True,
                 request_not_response=True)
    return list(bytes(t.payload))


def resp_payload(value=None, default=None, vmax=None, vmin=None, name=""):
    r = P.GetSet.Response()
    apply_value(r.value, value)
    apply_value(r.default_value, default)
    apply_value(r.max_value, vmax)
    apply_value(r.min_value, vmin)
    if name:
        r.name = name
    t = Transfer(payload=r, source_node_id=OUR_NODE, dest_node_id=REQUESTER,
                 transfer_id=1, transfer_priority=16, service_not_message=True,
                 request_not_response=False)
    return list(bytes(t.payload))


def carr(bs):
    return "{" + ", ".join("0x%02X" % (b & 0xFF) for b in bs) + "}"


def pad(bs, n):
    return list(bs) + [0] * (n - len(bs))


# Response payloads for our params, parameterized by current value (post-set readback state).
def resp_node_id(cur):
    return resp_payload(vint(cur), nint(0), nint(127), nint(0), "node_id")


def resp_park_valid(cur):
    return resp_payload(vbool(1 if cur else 0), name="park_ref_valid")


def resp_park_target(cur):
    return resp_payload(vreal(cur), nreal(0.0), name="park_ref_target_rev")


def resp_empty():
    return resp_payload()


REQMAX, RESPMAX = 48, 64


def main():
    out = []
    p = out.append
    p("/* dronecan_param_golden.inc -- GENERATED, do not edit by hand.")
    p(" *   generator : tools/test/gen_dronecan_param_golden.py")
    p(" *   pydronecan: %s   (GetSet dtid=11, sig=0x%016X)" %
      (PYVER, P.GetSet.get_data_type_signature()))
    p(" *   command   : python3 tools/test/gen_dronecan_param_golden.py > tools/test/dronecan_param_golden.inc")
    p(" * One wire byte per uint16_t element (C28x convention).")
    p(" */")
    p("")

    # ---- CODEC layer: request payload -> response payload ----
    # Each case: a starting nvparam state (set up in C), the request, and the response pydronecan
    # produces for the resulting (post-set) state. The C test owns the nvparam asserts.
    codec = [
        # name, request payload, response payload
        ("get_nodeid_byname_0",  req_payload(name="node_id"),                 resp_node_id(0)),
        ("get_nodeid_byidx_42",  req_payload(index=0),                        resp_node_id(42)),
        ("set_nodeid_25",        req_payload(name="node_id", value=vint(25)), resp_node_id(25)),
        ("set_nodeid_bad_200",   req_payload(name="node_id", value=vint(200)),resp_node_id(0)),
        ("get_parkvalid",        req_payload(name="park_ref_valid"),          resp_park_valid(0)),
        ("set_parkvalid_1",      req_payload(name="park_ref_valid", value=vbool(1)), resp_park_valid(1)),
        ("set_parktgt_025",      req_payload(name="park_ref_target_rev", value=vreal(0.25)), resp_park_target(0.25)),
        ("set_parktgt_nan",      req_payload(name="park_ref_target_rev", value=vreal(float("nan"))), resp_park_target(0.0)),
        ("get_unknown_name",     req_payload(name="bogus"),                   resp_empty()),
        ("get_unknown_idx_99",   req_payload(index=99),                       resp_empty()),
    ]
    p("/* ===== GetSet CODEC golden (request payload -> response payload) ===== */")
    p("typedef struct { const char *name; uint16_t req_len; uint16_t req[%u];" % REQMAX)
    p("                 uint16_t resp_len; uint16_t resp[%u]; } gold_gs_codec_t;" % RESPMAX)
    p("static const gold_gs_codec_t GOLD_GS_CODEC[] = {")
    for name, rq, rs in codec:
        assert len(rq) <= REQMAX and len(rs) <= RESPMAX, name
        p("  /* %s: req %ub -> resp %ub */" % (name, len(rq), len(rs)))
        p("  { \"%s\", %u, %s, %u, %s }," %
          (name, len(rq), carr(pad(rq, REQMAX)), len(rs), carr(pad(rs, RESPMAX))))
    p("};")
    p("")

    # ---- TRANSPORT layer: full frames in -> full frames out ----
    def req_frames(index=0, name="", value=None, tid=3, prio=16):
        r = P.GetSet.Request(index=index)
        if name:
            r.name = name
        apply_value(r.value, value)
        t = Transfer(payload=r, source_node_id=REQUESTER, dest_node_id=OUR_NODE,
                     transfer_id=tid, transfer_priority=prio, service_not_message=True,
                     request_not_response=True)
        return t.to_frames()

    def resp_frames(value=None, default=None, vmax=None, vmin=None, name="", tid=3, prio=16):
        r = P.GetSet.Response()
        apply_value(r.value, value)
        apply_value(r.default_value, default)
        apply_value(r.max_value, vmax)
        apply_value(r.min_value, vmin)
        if name:
            r.name = name
        t = Transfer(payload=r, source_node_id=OUR_NODE, dest_node_id=REQUESTER,
                     transfer_id=tid, transfer_priority=prio, service_not_message=True,
                     request_not_response=False)
        return t.to_frames()

    cases = [
        # name, request frames, response frames, tid, prio
        ("tx_get_idx0_nodeid",
         req_frames(index=0, tid=3, prio=16),
         resp_frames(vint(0), nint(0), nint(127), nint(0), "node_id", tid=3, prio=16)),
        ("tx_set_nodeid_25",
         req_frames(name="node_id", value=vint(25), tid=7, prio=20),
         resp_frames(vint(25), nint(0), nint(127), nint(0), "node_id", tid=7, prio=20)),
    ]
    p("/* ===== GetSet TRANSPORT golden (request frames in -> response frames out) ===== */")
    p("typedef struct { const char *name; uint16_t requester; uint16_t our_node; uint16_t tid;")
    p("                 uint16_t prio; uint16_t n_req; uint32_t req_id[3]; uint16_t req_dlc[3];")
    p("                 uint16_t req_data[3][8]; uint16_t n_resp; uint32_t resp_id[8];")
    p("                 uint16_t resp_dlc[8]; uint16_t resp_data[8][8]; } gold_gs_tx_t;")
    p("static const gold_gs_tx_t GOLD_GS_TX[] = {")
    for name, rf, sf, in [(c[0], c[1], c[2]) for c in cases]:
        tid = 3 if "idx0" in name else 7
        prio = 16 if "idx0" in name else 20
        assert len(rf) <= 3 and len(sf) <= 8, name
        rids, rdlcs, rdatas = [], [], []
        for i in range(3):
            if i < len(rf):
                bs = bytes(rf[i].bytes)
                rids.append("0x%08X" % rf[i].message_id); rdlcs.append(str(len(bs)))
                rdatas.append(carr(bs))
            else:
                rids.append("0"); rdlcs.append("0"); rdatas.append("{0}")
        sids, sdlcs, sdatas = [], [], []
        for i in range(8):
            if i < len(sf):
                bs = bytes(sf[i].bytes)
                sids.append("0x%08X" % sf[i].message_id); sdlcs.append(str(len(bs)))
                sdatas.append(carr(bs))
            else:
                sids.append("0"); sdlcs.append("0"); sdatas.append("{0}")
        p("  /* %s: req %u frame(s) -> resp %u frame(s) */" % (name, len(rf), len(sf)))
        p("  { \"%s\", %u, %u, %u, %u, %u," % (name, REQUESTER, OUR_NODE, tid, prio, len(rf)))
        p("    {%s}, {%s}, {%s, %s, %s}," %
          (", ".join(rids), ", ".join(rdlcs), rdatas[0], rdatas[1], rdatas[2]))
        p("    %u," % len(sf))
        p("    {%s}," % ", ".join(sids))
        p("    {%s}," % ", ".join(sdlcs))
        p("    {%s} }," % ", ".join(sdatas))
    p("};")
    p("")

    sys.stdout.write("\n".join(out).rstrip("\n") + "\n")


if __name__ == "__main__":
    main()
