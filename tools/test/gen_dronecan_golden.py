#!/usr/bin/env python3
"""
Golden-frame generator for the src/comms DroneCAN core.

Produces tools/test/dronecan_golden.inc (checked in) from pydronecan's reference
serializer, so the C tests compare WHOLE frames byte-for-byte against the canonical
wire format instead of trusting a self-consistent roundtrip. The generated .inc is
the anchor every protocol decision is validated against.

Run:  python3 tools/test/gen_dronecan_golden.py > tools/test/dronecan_golden.inc

Notes:
- Output arrays use uint16_t (C28x has no uint8_t); each element is one wire byte (low 8 bits).
- Frame.bytes from pydronecan already includes the DroneCAN tail byte; dlc counts it.
- Requires pydronecan 1.0.27.
"""
import sys
import dronecan
from dronecan import uavcan
from dronecan.transport import Transfer

PYDRONECAN_VER = dronecan.__version__
UNIQUE_ID = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
             0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF]


def crc16_ccitt_false(data, init=0xFFFF):
    """Plain CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) -- matches the C core's crc16."""
    crc = init
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frames_of(payload, source_node_id, transfer_id, priority=16, service=False):
    t = Transfer(payload=payload, source_node_id=source_node_id,
                 transfer_id=transfer_id, transfer_priority=priority,
                 service_not_message=service)
    if source_node_id == 0:
        # Anonymous (DNA) frame: firmware sets discriminator = crc16(payload) & 0x3FFF.
        t.discriminator = crc16_ccitt_false(bytes(t.payload)) & 0x3FFF
    return t.to_frames()


def c_bytes(bs):
    return "{" + ", ".join("0x%02X" % b for b in bs) + "}"


def emit_frame_fields(prefix, f):
    bs = bytes(f.bytes)
    return ("%s_id=0x%08X, %s_dlc=%u, %s_data=%s"
            % (prefix, f.message_id, prefix, len(bs), prefix, c_bytes(bs)))


def main():
    out = []
    p = out.append

    p("/* dronecan_golden.inc -- GENERATED, do not edit by hand.")
    p(" *")
    p(" * Provenance:")
    p(" *   generator : tools/test/gen_dronecan_golden.py")
    p(" *   pydronecan: %s" % PYDRONECAN_VER)
    p(" *   command   : python3 tools/test/gen_dronecan_golden.py > tools/test/dronecan_golden.inc")
    p(" *   unique_id : %s" % c_bytes(UNIQUE_ID))
    p(" *")
    p(" * Each *_data[] element is ONE wire byte (low 8 bits) in a uint16_t, matching")
    p(" * the C28x dronecan_frame_t.data[] convention. dlc includes the DroneCAN tail byte.")
    p(" * Frames come from pydronecan's reference serializer (the interop anchor).")
    p(" */")
    p("")

    # ---- RawCommand RX cases (frame -> decoded int14 at esc_index) ----
    # case: name, cmd[], source, transfer_id, then probe several esc_index values.
    raw_cases = [
        ("raw_1esc", [5000], 42, 1),
        ("raw_4esc", [0, 8191, -100, 4096], 42, 5),
        ("raw_5esc_multiframe", [1, 2, 3, 4, 5], 42, 7),
    ]
    p("/* ===== RawCommand (DTID 1030) RX golden ===== */")
    p("typedef struct { const char *name; uint32_t id; uint16_t dlc; uint16_t data[8];")
    p("                 uint16_t n_frames; uint16_t cmd_count; } gold_raw_t;")
    p("static const gold_raw_t GOLD_RAW[] = {")
    for name, cmd, src, tid in raw_cases:
        fr = frames_of(uavcan.equipment.esc.RawCommand(cmd=cmd), src, tid)
        f0 = fr[0]
        bs = bytes(f0.bytes)
        p("  /* %s: cmd=%s src=%u tid=%u frames=%u */" % (name, cmd, src, tid, len(fr)))
        p("  { \"%s\", 0x%08X, %u, %s, %u, %u }," %
          (name, f0.message_id, len(bs), c_bytes(bs), len(fr), len(cmd)))
    p("};")
    p("")
    # Also dump the decoded expectation table (esc_index -> raw14) for the single-frame cases.
    p("/* expected decode: GOLD_RAW index, esc_index, expected raw14, present(1/0) */")
    p("typedef struct { uint16_t raw_idx; uint16_t esc_index; int16_t raw14; uint16_t present; } gold_raw_dec_t;")
    p("static const gold_raw_dec_t GOLD_RAW_DEC[] = {")
    # raw_1esc (index 0)
    p("  { 0, 0, 5000, 1 }, { 0, 1, 0, 0 },")
    # raw_4esc (index 1)
    p("  { 1, 0, 0, 1 }, { 1, 1, 8191, 1 }, { 1, 2, -100, 1 }, { 1, 3, 4096, 1 }, { 1, 4, 0, 0 },")
    p("};")
    p("")

    # ---- esc.Status TX golden (multi-frame) ----
    p("/* ===== esc.Status (DTID 1034) TX golden ===== */")
    status_cases = [
        ("status_nominal", dict(error_count=0, voltage=24.0, current=1.5,
                                temperature=300.0, rpm=1234, power_rating_pct=0,
                                esc_index=0), 42, 3),
        ("status_idx7", dict(error_count=0, voltage=12.5, current=-2.0,
                             temperature=263.15, rpm=-500, power_rating_pct=0,
                             esc_index=7), 42, 4),
    ]
    p("typedef struct { const char *name; float voltage, current, temp_K; int32_t rpm;")
    p("                 uint16_t esc_index; uint32_t src; uint16_t tid;")
    p("                 uint16_t n; uint32_t id[3]; uint16_t dlc[3]; uint16_t data[3][8]; } gold_status_t;")
    p("static const gold_status_t GOLD_STATUS[] = {")
    for name, kw, src, tid in status_cases:
        fr = frames_of(uavcan.equipment.esc.Status(**kw), src, tid)
        assert len(fr) <= 3, "status expected <=3 frames, got %d" % len(fr)
        ids = []
        dlcs = []
        datas = []
        for i in range(3):
            if i < len(fr):
                bs = bytes(fr[i].bytes)
                ids.append("0x%08X" % fr[i].message_id)
                dlcs.append(str(len(bs)))
                datas.append(c_bytes(bs))
            else:
                ids.append("0")
                dlcs.append("0")
                datas.append("{0}")
        p("  /* %s: %s */" % (name, kw))
        p("  { \"%s\", %ff, %ff, %ff, %d, %u, %u, %u, %u," %
          (name, kw["voltage"], kw["current"], kw["temperature"], kw["rpm"],
           kw["esc_index"], src, tid, len(fr)))
        p("    {%s}, {%s}, {%s, %s, %s} }," %
          (", ".join(ids), ", ".join(dlcs), datas[0], datas[1], datas[2]))
    p("};")
    p("")

    # ---- NodeStatus TX golden ----
    p("/* ===== NodeStatus (DTID 341) TX golden ===== */")
    ns_cases = [
        ("ns_ok", dict(uptime_sec=7, health=0, mode=0, sub_mode=0,
                       vendor_specific_status_code=0), 42, 2),
        ("ns_fault_ov", dict(uptime_sec=123, health=2, mode=0, sub_mode=0,
                             vendor_specific_status_code=0x0002), 42, 6),
    ]
    p("typedef struct { const char *name; uint32_t uptime; uint16_t health; uint16_t vendor;")
    p("                 uint32_t src; uint16_t tid; uint32_t id; uint16_t dlc; uint16_t data[8]; } gold_ns_t;")
    p("static const gold_ns_t GOLD_NS[] = {")
    for name, kw, src, tid in ns_cases:
        fr = frames_of(uavcan.protocol.NodeStatus(**kw), src, tid)
        assert len(fr) == 1
        bs = bytes(fr[0].bytes)
        p("  /* %s: %s */" % (name, kw))
        p("  { \"%s\", %u, %u, 0x%04X, %u, %u, 0x%08X, %u, %s }," %
          (name, kw["uptime_sec"], kw["health"], kw["vendor_specific_status_code"],
           src, tid, fr[0].message_id, len(bs), c_bytes(bs)))
    p("};")
    p("")

    # ---- DNA allocation golden (anonymous request frames + a response) ----
    p("/* ===== DNA (DTID 1) golden ===== */")
    p("typedef struct { const char *name; uint16_t first_part; uint16_t uid_len;")
    p("                 uint16_t uid[16]; uint32_t tid; uint32_t id; uint16_t dlc; uint16_t data[8]; } gold_dna_t;")
    dna_cases = [
        ("dna_req_stage1", True, UNIQUE_ID[0:6], 0),
        ("dna_req_stage2", False, UNIQUE_ID[6:12], 1),
        ("dna_req_stage3", False, UNIQUE_ID[12:16], 2),
    ]
    p("static const gold_dna_t GOLD_DNA_REQ[] = {")
    for name, first, uid, tid in dna_cases:
        msg = uavcan.protocol.dynamic_node_id.Allocation(
            node_id=0, first_part_of_unique_id=first, unique_id=uid)
        fr = frames_of(msg, 0, tid, priority=30)  # anonymous source, DNA priority 30
        assert len(fr) == 1, "%s: expected single anon frame, got %d" % (name, len(fr))
        bs = bytes(fr[0].bytes)
        uid_arr = uid + [0] * (16 - len(uid))
        p("  /* %s: first=%s uid=%s */" % (name, first, [hex(x) for x in uid]))
        p("  { \"%s\", %u, %u, %s, %u, 0x%08X, %u, %s }," %
          (name, 1 if first else 0, len(uid), c_bytes(uid_arr), tid,
           fr[0].message_id, len(bs), c_bytes(bs)))
    p("};")
    p("")
    # Allocator responses (full frame sequences): two echoes then the final assignment.
    p("typedef struct { const char *name; uint16_t exp_node_id; uint16_t exp_uid_len;")
    p("                 uint16_t n; uint32_t id[3]; uint16_t dlc[3]; uint16_t data[3][8]; } gold_dna_resp_t;")
    resp_cases = [
        ("dna_echo_s1", 0, UNIQUE_ID[0:6], 100, 0),    # echo first 6 bytes (single frame)
        ("dna_echo_s2", 0, UNIQUE_ID[0:12], 100, 1),   # echo first 12 bytes (multi-frame)
        ("dna_final",   25, UNIQUE_ID[0:16], 100, 2),  # assign node id 25 (multi-frame)
    ]
    p("static const gold_dna_resp_t GOLD_DNA_RESP[] = {")
    for name, node_id, uid, src, tid in resp_cases:
        msg = uavcan.protocol.dynamic_node_id.Allocation(
            node_id=node_id, first_part_of_unique_id=(len(uid) <= 6), unique_id=uid)
        fr = frames_of(msg, src, tid)
        assert len(fr) <= 3, "%s: %d frames" % (name, len(fr))
        ids, dlcs, datas = [], [], []
        for i in range(3):
            if i < len(fr):
                bs = bytes(fr[i].bytes)
                ids.append("0x%08X" % fr[i].message_id)
                dlcs.append(str(len(bs)))
                datas.append(c_bytes(bs))
            else:
                ids.append("0"); dlcs.append("0"); datas.append("{0}")
        p("  /* %s: node_id=%u uid_len=%u src=%u tid=%u frames=%u */" %
          (name, node_id, len(uid), src, tid, len(fr)))
        p("  { \"%s\", %u, %u, %u, {%s}, {%s}, {%s, %s, %s} }," %
          (name, node_id, len(uid), len(fr), ", ".join(ids), ", ".join(dlcs),
           datas[0], datas[1], datas[2]))
    p("};")
    p("")

    sys.stdout.write("\n".join(out).rstrip("\n") + "\n")


if __name__ == "__main__":
    main()
