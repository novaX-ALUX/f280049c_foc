#include "check.h"
#include "dronecan_frame.h"
#include "dronecan_ids.h"
#include "dronecan.h"
#include <string.h>

/* Golden frames generated from pydronecan 1.0.27 (the interop anchor). */
#include "dronecan_golden.inc"

/* Pack an int14[] array DSDL-style (RawCommand body) into a payload buffer. */
static void pack_raw_cmd(dronecan_payload_t *p, const int16_t *cmd, uint16_t n)
{
    uint16_t i;
    dronecan_payload_init(p);
    for (i = 0; i < n; ++i) {
        dronecan_pack_int(p, cmd[i], 14u);
    }
}

static dronecan_frame_t frame_from_raw(const gold_raw_t *g)
{
    dronecan_frame_t f;
    uint16_t i;
    f.extended = true;
    f.id = g->id;
    f.dlc = g->dlc;
    for (i = 0; i < 8; ++i) {
        f.data[i] = (i < g->dlc) ? g->data[i] : 0u;
    }
    return f;
}

/* Build a single-frame RawCommand of `count` identical int14 values (count<=4 -> single frame). */
static dronecan_frame_t make_rawN(int16_t v, uint16_t count, uint16_t src)
{
    dronecan_frame_t f;
    dronecan_payload_t p;
    uint16_t blen, i;
    memset(&f, 0, sizeof f);
    dronecan_payload_init(&p);
    for (i = 0; i < count; ++i) {
        dronecan_pack_int(&p, v, 14u);
    }
    blen = dronecan_payload_bytelen(&p);
    f.extended = true;
    f.id = dronecan_msg_id(16u, DRONECAN_DTID_RAW_COMMAND, src);
    for (i = 0; i < blen && i < 7u; ++i) {
        f.data[i] = p.bytes[i];
    }
    f.data[blen] = dronecan_tail_encode(true, true, false, 0u);
    f.dlc = (uint16_t)(blen + 1u);
    return f;
}

/* Single-element RawCommand (addresses esc_index 0 only). */
static dronecan_frame_t make_raw1(int16_t v, uint16_t src)
{
    return make_rawN(v, 1u, src);
}

static dronecan_cfg_t raw_cfg(uint16_t esc_index)
{
    dronecan_cfg_t c;
    memset(&c, 0, sizeof c);
    c.esc_index = esc_index;
    c.node_id = 25u; /* static; DNA not under test here */
    return c;
}

static void arm_node(dronecan_t *dn)
{
    dronecan_rx_result_t r;
    int i;
    for (i = 0; i < 10; ++i) {       /* default arm_zero_frames = 10 */
        dronecan_frame_t f = make_rawN(0, 4u, 42u); /* 4 elems -> covers esc_index 0..3 */
        dronecan_on_rx(dn, &f, &r);
    }
}

static int frame_eq(const dronecan_frame_t *f, uint32_t id, uint16_t dlc, const uint16_t *data)
{
    uint16_t i;
    if (f->id != id || f->dlc != dlc) {
        return 0;
    }
    for (i = 0; i < dlc; ++i) {
        if ((f->data[i] & 0xFFu) != (data[i] & 0xFFu)) {
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    /* ---- CAN ID field accessors (build + extract) ---- */
    {
        uint32_t id = dronecan_msg_id(16u, DRONECAN_DTID_RAW_COMMAND, 42u);
        CHECK(id == 0x1004062Au);
        CHECK(dronecan_id_priority(id) == 16u);
        CHECK(dronecan_id_msg_dtid(id) == DRONECAN_DTID_RAW_COMMAND);
        CHECK(dronecan_id_source(id) == 42u);
        CHECK(!dronecan_id_is_service(id));

        CHECK(dronecan_msg_id(16u, DRONECAN_DTID_ESC_STATUS, 42u) == 0x10040A2Au);
        CHECK(dronecan_msg_id(16u, DRONECAN_DTID_NODE_STATUS, 42u) == 0x1001552Au);
    }

    /* ---- anonymous (DNA) ID: discriminator extracted from golden round-trips ---- */
    {
        uint16_t disc = dronecan_id_discriminator(GOLD_DNA_REQ[0].id);
        CHECK(GOLD_DNA_REQ[0].id == 0x1EFE5500u);
        CHECK(dronecan_anon_id(30u, DRONECAN_DTID_ALLOCATION, disc) == GOLD_DNA_REQ[0].id);
        CHECK(dronecan_id_source(GOLD_DNA_REQ[0].id) == 0u); /* anonymous */
    }

    /* ---- tail byte decode against golden frames ---- */
    {
        /* raw_4esc single frame: tail = last data byte, SOT=EOT=1 toggle=0 tid=5 */
        const gold_raw_t *g = &GOLD_RAW[1];
        dronecan_tail_t t = dronecan_tail_decode(g->data[g->dlc - 1]);
        CHECK(t.sot && t.eot && !t.toggle);
        CHECK(t.transfer_id == 5u);
        CHECK(dronecan_tail_encode(true, true, false, 5u) == g->data[g->dlc - 1]);

        /* Status frame 1 (middle): SOT=0, EOT=0, toggle=1 */
        const gold_status_t *s = &GOLD_STATUS[0];
        dronecan_tail_t t1 = dronecan_tail_decode(s->data[1][s->dlc[1] - 1]);
        CHECK(!t1.sot && !t1.eot && t1.toggle);
    }

    /* ---- DSDL bit packing: reproduce golden RawCommand payloads byte-for-byte ---- */
    {
        int16_t c1[1] = {5000};
        dronecan_payload_t p;
        pack_raw_cmd(&p, c1, 1);
        const gold_raw_t *g = &GOLD_RAW[0];
        uint16_t blen = dronecan_payload_bytelen(&p);
        CHECK(blen == (uint16_t)(g->dlc - 1)); /* golden dlc includes tail */
        uint16_t i;
        for (i = 0; i < blen; ++i) {
            CHECK(p.bytes[i] == g->data[i]);
        }
    }
    {
        int16_t c4[4] = {0, 8191, -100, 4096};
        dronecan_payload_t p;
        pack_raw_cmd(&p, c4, 4);
        const gold_raw_t *g = &GOLD_RAW[1];
        uint16_t blen = dronecan_payload_bytelen(&p);
        CHECK(blen == (uint16_t)(g->dlc - 1));
        uint16_t i;
        for (i = 0; i < blen; ++i) {
            CHECK(p.bytes[i] == g->data[i]);
        }
    }

    /* ---- unpack round-trip: decode the int14 array back ---- */
    {
        int16_t c4[4] = {0, 8191, -100, 4096};
        dronecan_payload_t p;
        pack_raw_cmd(&p, c4, 4);
        uint16_t bp = 0u;
        uint16_t i;
        for (i = 0; i < 4; ++i) {
            int32_t v = dronecan_unpack_int(&p, &bp, 14u);
            CHECK(v == c4[i]);
        }
    }

    /* ---- transfer CRC: seed from esc.Status signature reproduces the golden frame CRC ---- */
    {
        /* Reconstruct the 14-byte serialized Status payload from golden frame fragments:
         * frame0[2..6] (after the 2 CRC bytes), frame1[0..6], frame2[0..1]. */
        const gold_status_t *s = &GOLD_STATUS[0];
        uint16_t payload[14];
        uint16_t n = 0, i;
        for (i = 2; i < (uint16_t)(s->dlc[0] - 1); ++i) payload[n++] = s->data[0][i];
        for (i = 0; i < (uint16_t)(s->dlc[1] - 1); ++i) payload[n++] = s->data[1][i];
        for (i = 0; i < (uint16_t)(s->dlc[2] - 1); ++i) payload[n++] = s->data[2][i];
        CHECK(n == 14u);
        uint16_t seed = dronecan_transfer_crc_seed(DRONECAN_ESC_STATUS_SIG_LO,
                                                   DRONECAN_ESC_STATUS_SIG_HI);
        uint16_t crc = dronecan_crc16(payload, n, seed);
        uint16_t golden_crc = (uint16_t)(s->data[0][0] | (s->data[0][1] << 8)); /* CRC lo,hi at front */
        CHECK(crc == golden_crc);
        CHECK(crc == 0x1DF1u);
    }

    /* ---- CRC16-CCITT-FALSE known vector: "123456789" -> 0x29B1 ---- */
    {
        uint16_t msg[9] = {'1','2','3','4','5','6','7','8','9'};
        CHECK(dronecan_crc16(msg, 9u, 0xFFFFu) == 0x29B1u);
    }

    /* ---- float16 round-trip + edge cases ---- */
    {
        CHECK(dronecan_float32_to_float16(0.0f) == 0x0000u);
        CHECK_NEAR(dronecan_float16_to_float32(dronecan_float32_to_float16(1.0f)), 1.0f, 1e-3f);
        CHECK_NEAR(dronecan_float16_to_float32(dronecan_float32_to_float16(24.0f)), 24.0f, 1e-2f);
        CHECK_NEAR(dronecan_float16_to_float32(dronecan_float32_to_float16(-2.0f)), -2.0f, 1e-3f);
        /* negative (current) keeps sign */
        CHECK(dronecan_float32_to_float16(-2.0f) & 0x8000u);
        /* overflow saturates to +Inf (exp all ones, mantissa 0) */
        CHECK(dronecan_float32_to_float16(1.0e9f) == 0x7C00u);
    }

    /* ---- RX: golden RawCommand decode (esc_index present/OOB) via GOLD_RAW_DEC ---- */
    {
        uint16_t k;
        for (k = 0; k < sizeof(GOLD_RAW_DEC) / sizeof(GOLD_RAW_DEC[0]); ++k) {
            const gold_raw_dec_t *d = &GOLD_RAW_DEC[k];
            dronecan_t dn;
            dronecan_cfg_t c = raw_cfg(d->esc_index);
            dronecan_rx_result_t r;
            dronecan_frame_t f;
            dronecan_init(&dn, &c);
            arm_node(&dn);                       /* pass handshake so throttle reflects raw14 */
            f = frame_from_raw(&GOLD_RAW[d->raw_idx]);
            dronecan_on_rx(&dn, &f, &r);
            CHECK(r.command_updated == (d->present != 0u));
            if (d->present) {
                float expect = (d->raw14 <= 0) ? 0.0f
                             : ((float)d->raw14 / (float)DRONECAN_RAWCMD_FULLSCALE);
                CHECK_NEAR(r.command.throttle, expect, 1e-4f);
                CHECK(r.command.arm);
            }
        }
    }

    /* ---- RX: multi-frame RawCommand (first frame, EOT=0) is skipped ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        dronecan_frame_t f = frame_from_raw(&GOLD_RAW[2]); /* raw_5esc_multiframe, 2 frames */
        dronecan_init(&dn, &c);
        dronecan_on_rx(&dn, &f, &r);
        CHECK(!r.command_updated);
    }

    /* ---- RX: arming handshake + reset rule ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        int i;
        dronecan_init(&dn, &c);

        /* Nonzero before handshake: accepted (seq++) but arm=false, throttle=0. */
        dronecan_frame_t nz = make_raw1(5000, 42u);
        dronecan_on_rx(&dn, &nz, &r);
        CHECK(r.command_updated);
        CHECK(!r.command.arm);
        CHECK_NEAR(r.command.throttle, 0.0f, 1e-9f);

        /* 5 zeros, then a nonzero must RESET the counter (no early arm). */
        for (i = 0; i < 5; ++i) { dronecan_frame_t z = make_raw1(0, 42u); dronecan_on_rx(&dn, &z, &r); }
        { dronecan_frame_t nz2 = make_raw1(4000, 42u); dronecan_on_rx(&dn, &nz2, &r); CHECK(!r.command.arm); }
        for (i = 0; i < 9; ++i) { dronecan_frame_t z = make_raw1(0, 42u); dronecan_on_rx(&dn, &z, &r); CHECK(!r.command.arm); }
        { dronecan_frame_t z = make_raw1(0, 42u); dronecan_on_rx(&dn, &z, &r); CHECK(r.command.arm); } /* 10th zero */

        /* Now a real command rides through. */
        { dronecan_frame_t go = make_raw1(8191, 42u); dronecan_on_rx(&dn, &go, &r);
          CHECK(r.command.arm); CHECK_NEAR(r.command.throttle, 1.0f, 1e-4f); }
    }

    /* ---- RX: unallocated node (node_id==0, DNA pending) must drop RawCommand entirely ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        int i;
        c.node_id = 0u;            /* DNA mode: not yet allocated */
        dronecan_init(&dn, &c);
        CHECK(dn.node_id == 0u);

        /* Spam 12 zeros + a nonzero: none may arm, advance the zero-run counter, or bump seq. */
        for (i = 0; i < 12; ++i) {
            dronecan_frame_t z = make_raw1(0, 42u);
            dronecan_on_rx(&dn, &z, &r);
            CHECK(!r.command_updated);
        }
        { dronecan_frame_t nz = make_raw1(5000, 42u); dronecan_on_rx(&dn, &nz, &r);
          CHECK(!r.command_updated); }
        CHECK(!dn.armed);
        CHECK(dn.zero_run == 0u);
        CHECK(dn.seq == 0u);

        /* Allocation completes -> arming must start FRESH: exactly arm_zero_frames zeros to arm. */
        dn.node_id = 25u;
        for (i = 0; i < 9; ++i) { dronecan_frame_t z = make_raw1(0, 42u);
          dronecan_on_rx(&dn, &z, &r); CHECK(r.command_updated); CHECK(!r.command.arm); }
        { dronecan_frame_t z = make_raw1(0, 42u); dronecan_on_rx(&dn, &z, &r);
          CHECK(r.command.arm); }  /* 10th post-allocation zero arms */
    }

    /* ---- RX: seq increments on every accepted frame, even constant throttle ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        uint32_t s1, s2;
        dronecan_init(&dn, &c);
        arm_node(&dn);
        { dronecan_frame_t g = make_raw1(3000, 42u); dronecan_on_rx(&dn, &g, &r); s1 = r.command.seq; }
        { dronecan_frame_t g = make_raw1(3000, 42u); dronecan_on_rx(&dn, &g, &r); s2 = r.command.seq; }
        CHECK(s2 == s1 + 1u);
    }

    /* ---- RX: ignore list ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        dronecan_init(&dn, &c);
        arm_node(&dn);

        /* non-extended */
        { dronecan_frame_t f = make_raw1(5000, 42u); f.extended = false;
          dronecan_on_rx(&dn, &f, &r); CHECK(!r.command_updated); }
        /* service frame (bit 7 set) */
        { dronecan_frame_t f = make_raw1(5000, 42u); f.id |= (1u << 7);
          dronecan_on_rx(&dn, &f, &r); CHECK(!r.command_updated); }
        /* wrong DTID */
        { dronecan_frame_t f = make_raw1(5000, 42u);
          f.id = dronecan_msg_id(16u, DRONECAN_DTID_ESC_STATUS, 42u);
          dronecan_on_rx(&dn, &f, &r); CHECK(!r.command_updated); }
        /* anonymous source (0) */
        { dronecan_frame_t f = make_raw1(5000, 0u);
          dronecan_on_rx(&dn, &f, &r); CHECK(!r.command_updated); }
        /* bad tail (toggle set on single frame) */
        { dronecan_frame_t f = make_raw1(5000, 42u);
          f.data[f.dlc - 1] = dronecan_tail_encode(true, true, true, 0u);
          dronecan_on_rx(&dn, &f, &r); CHECK(!r.command_updated); }
    }

    /* ---- RX: high-byte pollution on data[] is masked before decode ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(1u);
        dronecan_rx_result_t r, rclean;
        dronecan_frame_t f, fclean;
        uint16_t i;
        dronecan_init(&dn, &c);
        arm_node(&dn);
        fclean = frame_from_raw(&GOLD_RAW[1]); /* raw_4esc */
        dronecan_on_rx(&dn, &fclean, &rclean);
        dronecan_init(&dn, &c);
        arm_node(&dn);
        f = frame_from_raw(&GOLD_RAW[1]);
        for (i = 0; i < f.dlc; ++i) { f.data[i] |= 0xFF00u; } /* pollute high bits */
        dronecan_on_rx(&dn, &f, &r);
        CHECK(r.command_updated && rclean.command_updated);
        CHECK_NEAR(r.command.throttle, rclean.command.throttle, 1e-6f);
    }

    /* ---- cfg sanitation: illegal esc_index defaults to 0, arm_zero_frames 0 -> 10 ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        memset(&c, 0, sizeof c);
        c.esc_index = 31u;        /* illegal */
        c.arm_zero_frames = 0u;   /* must become 10 */
        c.node_id = 25u;
        dronecan_init(&dn, &c);
        CHECK(dn.cfg.esc_index == 0u);
        CHECK(dn.cfg.arm_zero_frames == 10u);
        /* esc_index 20/31/65535 never read OOB: a 4-ESC frame has no component for them */
        { dronecan_cfg_t c2 = raw_cfg(20u); dronecan_rx_result_t r; dronecan_frame_t f;
          dronecan_init(&dn, &c2); CHECK(dn.cfg.esc_index == 0u); /* 20 -> 0 */
          f = frame_from_raw(&GOLD_RAW[1]); dronecan_on_rx(&dn, &f, &r);
          CHECK(r.command_updated); /* index 0 present in a 4-ESC frame */ }
    }

    /* ---- TX: NodeStatus golden (whole frame) ---- */
    {
        uint16_t k;
        for (k = 0; k < sizeof(GOLD_NS) / sizeof(GOLD_NS[0]); ++k) {
            const gold_ns_t *g = &GOLD_NS[k];
            dronecan_t dn;
            dronecan_cfg_t c;
            esc_telemetry_t tel;
            dronecan_frame_t out[4];
            int n;
            memset(&c, 0, sizeof c);
            memset(&tel, 0, sizeof tel);
            c.node_id = (uint16_t)g->src;
            dronecan_init(&dn, &c);
            dn.tid_node_status = (uint16_t)g->tid;
            tel.hard_fault_bits = (g->health == DRONECAN_HEALTH_ERROR) ? g->vendor : 0u;
            /* cap=1: only the single-frame NodeStatus fits; Status (needs 3) is skipped. */
            n = dronecan_tick(&dn, g->uptime * 1000u, &tel, out, 1);
            CHECK(n == 1);
            CHECK(frame_eq(&out[0], g->id, g->dlc, g->data));
        }
    }

    /* ---- TX: esc.Status golden (3-frame transfer, CRC/tail/toggle/fields) ---- */
    {
        uint16_t k;
        for (k = 0; k < sizeof(GOLD_STATUS) / sizeof(GOLD_STATUS[0]); ++k) {
            const gold_status_t *g = &GOLD_STATUS[k];
            dronecan_t dn;
            dronecan_cfg_t c;
            esc_telemetry_t tel;
            dronecan_frame_t out[8];
            int n;
            memset(&c, 0, sizeof c);
            memset(&tel, 0, sizeof tel);
            c.node_id = (uint16_t)g->src;
            c.esc_index = g->esc_index;
            dronecan_init(&dn, &c);
            dn.tid_esc_status = (uint16_t)g->tid;
            tel.vbus_V    = g->voltage;
            tel.current_A = g->current;
            tel.temp_C    = g->temp_K - DRONECAN_KELVIN_OFFSET;
            tel.rpm       = (float)g->rpm;
            tel.hard_fault_bits = 0u;
            /* cap large: NodeStatus emits first (out[0]), Status at out[1..3]. */
            n = dronecan_tick(&dn, 0u, &tel, out, 8);
            CHECK(n == 4);
            CHECK(frame_eq(&out[1], g->id[0], g->dlc[0], g->data[0]));
            CHECK(frame_eq(&out[2], g->id[1], g->dlc[1], g->data[1]));
            CHECK(frame_eq(&out[3], g->id[2], g->dlc[2], g->data[2]));
        }
    }

    /* ---- TX scheduling: transfer atomicity, priority, no swallowed period ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        esc_telemetry_t tel;
        dronecan_frame_t out[8];
        int n;
        memset(&c, 0, sizeof c);
        memset(&tel, 0, sizeof tel);
        c.node_id = 25u;
        dronecan_init(&dn, &c);

        /* cap=2: NodeStatus fits (1), Status needs 3 -> skipped, stays due. */
        n = dronecan_tick(&dn, 1000u, &tel, out, 2);
        CHECK(n == 1);
        CHECK(dronecan_id_msg_dtid(out[0].id) == DRONECAN_DTID_NODE_STATUS); /* priority */
        /* Status still due at the same time -> a roomy tick emits exactly the 3-frame transfer. */
        n = dronecan_tick(&dn, 1000u, &tel, out, 8);
        CHECK(n == 3);
        CHECK(dronecan_id_msg_dtid(out[0].id) == DRONECAN_DTID_ESC_STATUS);
        {
            dronecan_tail_t t0 = dronecan_tail_decode(out[0].data[out[0].dlc - 1]);
            dronecan_tail_t t2 = dronecan_tail_decode(out[2].data[out[2].dlc - 1]);
            CHECK(t0.sot && !t0.eot && !t0.toggle);
            CHECK(!t2.sot && t2.eot && !t2.toggle); /* toggle 0->1->0 */
            CHECK(t0.transfer_id == t2.transfer_id);
        }
        /* Nothing due immediately after. */
        n = dronecan_tick(&dn, 1000u, &tel, out, 8);
        CHECK(n == 0);
    }

    /* ---- TX: uint32 now_ms wrap-around still fires on time ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        esc_telemetry_t tel;
        dronecan_frame_t out[8];
        int n;
        memset(&c, 0, sizeof c);
        memset(&tel, 0, sizeof tel);
        c.node_id = 25u;
        dronecan_init(&dn, &c);
        dn.sched_primed = true;
        /* last = now - 2000 (wraps below 0): elapsed 2000ms exceeds both periods. */
        dn.last_node_status_ms = (uint32_t)(0x00000100u - 2000u);
        dn.last_esc_status_ms  = (uint32_t)(0x00000100u - 2000u);
        n = dronecan_tick(&dn, 0x00000100u, &tel, out, 8);
        CHECK(n == 4); /* NodeStatus (period 1000) + Status (period 100) both due across wrap */
    }

    /* ---- TX bad-arg + DNA-mode + tel==NULL ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        esc_telemetry_t tel;
        dronecan_frame_t out[8];
        memset(&c, 0, sizeof c);
        memset(&tel, 0, sizeof tel);
        c.node_id = 25u;
        dronecan_init(&dn, &c);
        CHECK(dronecan_tick(NULL, 0u, &tel, out, 8) == 0);
        CHECK(dronecan_tick(&dn, 0u, &tel, NULL, 8) == 0);
        CHECK(dronecan_tick(&dn, 0u, &tel, out, 0) == 0);
        CHECK(dronecan_tick(&dn, 0u, NULL, out, 8) == 0); /* allocated + no telemetry */
        /* DNA mode (node_id 0): emits an anonymous allocation request (tel ignored). */
        { dronecan_cfg_t cd; dronecan_t dd; memset(&cd, 0, sizeof cd); cd.node_id = 0u;
          dronecan_init(&dd, &cd);
          CHECK(dronecan_tick(&dd, 0u, &tel, out, 8) == 1);
          CHECK(dronecan_id_source(out[0].id) == 0u); /* anonymous */
          CHECK(dronecan_id_priority(out[0].id) == DRONECAN_PRIO_ALLOCATION); }
    }

    /* ---- DNA: three-stage allocation, golden requests + responses -> ALLOCATED ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        dronecan_frame_t out[4];
        dronecan_rx_result_t rr;
        int n, i;
        memset(&c, 0, sizeof c);
        c.node_id = 0u;                 /* dynamic -> DNA */
        for (i = 0; i < 16; ++i) {
            c.unique_id[i] = (uint16_t)(0xA0 + i);
        }
        dronecan_init(&dn, &c);
        CHECK(dronecan_node_id(&dn) == 0u);

        /* tick -> stage-1 request (golden). */
        n = dronecan_tick(&dn, 0u, NULL, out, 4);
        CHECK(n == 1);
        CHECK(frame_eq(&out[0], GOLD_DNA_REQ[0].id, GOLD_DNA_REQ[0].dlc, GOLD_DNA_REQ[0].data));

        /* allocator echoes first 6 bytes (single frame) -> advances. */
        {
            const gold_dna_resp_t *g = &GOLD_DNA_RESP[0];
            dronecan_frame_t f = { g->id[0], g->dlc[0], {0}, true };
            for (i = 0; i < g->dlc[0]; ++i) f.data[i] = g->data[0][i];
            dronecan_on_rx(&dn, &f, &rr);
        }
        CHECK(dn.dna_confirmed_len == 6u);

        /* tick -> stage-2 request (golden, tid 1). */
        n = dronecan_tick(&dn, 100u, NULL, out, 4);
        CHECK(n == 1);
        CHECK(frame_eq(&out[0], GOLD_DNA_REQ[1].id, GOLD_DNA_REQ[1].dlc, GOLD_DNA_REQ[1].data));

        /* allocator echoes first 12 bytes (multi-frame) -> advances. */
        {
            const gold_dna_resp_t *g = &GOLD_DNA_RESP[1];
            uint16_t fr;
            for (fr = 0; fr < g->n; ++fr) {
                dronecan_frame_t f = { g->id[fr], g->dlc[fr], {0}, true };
                for (i = 0; i < g->dlc[fr]; ++i) f.data[i] = g->data[fr][i];
                dronecan_on_rx(&dn, &f, &rr);
            }
        }
        CHECK(dn.dna_confirmed_len == 12u);

        /* tick -> stage-3 request (golden, tid 2). */
        n = dronecan_tick(&dn, 200u, NULL, out, 4);
        CHECK(n == 1);
        CHECK(frame_eq(&out[0], GOLD_DNA_REQ[2].id, GOLD_DNA_REQ[2].dlc, GOLD_DNA_REQ[2].data));

        /* allocator assigns node id 25 (multi-frame, full uid) -> ALLOCATED + dirty. */
        {
            const gold_dna_resp_t *g = &GOLD_DNA_RESP[2];
            uint16_t fr;
            for (fr = 0; fr < g->n; ++fr) {
                dronecan_frame_t f = { g->id[fr], g->dlc[fr], {0}, true };
                for (i = 0; i < g->dlc[fr]; ++i) f.data[i] = g->data[fr][i];
                dronecan_on_rx(&dn, &f, &rr);
            }
        }
        CHECK(dronecan_node_id(&dn) == 25u);
        CHECK(dronecan_node_id_dirty(&dn));
        dronecan_clear_node_id_dirty(&dn);
        CHECK(!dronecan_node_id_dirty(&dn));

        /* now allocated -> tick emits NodeStatus (first eligible tick), not DNA. */
        {
            esc_telemetry_t tel;
            memset(&tel, 0, sizeof tel);
            n = dronecan_tick(&dn, 1000u, &tel, out, 1);
            CHECK(n == 1);
            CHECK(dronecan_id_msg_dtid(out[0].id) == DRONECAN_DTID_NODE_STATUS);
            CHECK(dronecan_id_source(out[0].id) == 25u);
        }
    }

    /* ---- RX: dlc > 8 is rejected (no read past data[8]) ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_rx_result_t r;
        dronecan_frame_t f;
        dronecan_init(&dn, &c);
        arm_node(&dn);
        f = make_raw1(5000, 42u);
        f.dlc = 9u;                 /* malformed */
        dronecan_on_rx(&dn, &f, &r);
        CHECK(!r.command_updated);
    }

    /* ---- DNA: multi-frame response with a corrupted transfer CRC is dropped ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        dronecan_rx_result_t rr;
        const gold_dna_resp_t *g = &GOLD_DNA_RESP[2]; /* final, multi-frame */
        uint16_t fr;
        int i;
        memset(&c, 0, sizeof c);
        c.node_id = 0u;
        for (i = 0; i < 16; ++i) c.unique_id[i] = (uint16_t)(0xA0 + i);
        dronecan_init(&dn, &c);
        for (fr = 0; fr < g->n; ++fr) {
            dronecan_frame_t f = { g->id[fr], g->dlc[fr], {0}, true };
            for (i = 0; i < g->dlc[fr]; ++i) f.data[i] = g->data[fr][i];
            if (fr == 0) { f.data[0] ^= 0xFFu; } /* corrupt the transfer CRC */
            dronecan_on_rx(&dn, &f, &rr);
        }
        CHECK(dronecan_node_id(&dn) == 0u); /* bad CRC -> not allocated */
    }

    /* ---- DNA: multi-frame response whose SOT frame has toggle=1 is dropped ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        dronecan_rx_result_t rr;
        const gold_dna_resp_t *g = &GOLD_DNA_RESP[2]; /* final, multi-frame */
        uint16_t fr;
        int i;
        memset(&c, 0, sizeof c);
        c.node_id = 0u;
        for (i = 0; i < 16; ++i) c.unique_id[i] = (uint16_t)(0xA0 + i);
        dronecan_init(&dn, &c);
        for (fr = 0; fr < g->n; ++fr) {
            dronecan_frame_t f = { g->id[fr], g->dlc[fr], {0}, true };
            for (i = 0; i < g->dlc[fr]; ++i) f.data[i] = g->data[fr][i];
            if (fr == 0) { f.data[f.dlc - 1] |= 0x20u; } /* force toggle=1 on the SOT frame */
            dronecan_on_rx(&dn, &f, &rr);
        }
        CHECK(dronecan_node_id(&dn) == 0u); /* bad SOT toggle -> not allocated */
    }

    /* ---- RX: res == NULL must not crash ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c = raw_cfg(0u);
        dronecan_frame_t f = make_raw1(5000, 42u);
        dronecan_init(&dn, &c);
        dronecan_on_rx(&dn, &f, NULL); /* strictly non-null contract, but must be defensive */
        CHECK(1); /* reached here without dereferencing NULL */
    }

    /* ---- DNA: an allocation for a DIFFERENT unique id is ignored ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        dronecan_rx_result_t rr;
        const gold_dna_resp_t *g = &GOLD_DNA_RESP[2];
        dronecan_frame_t f;
        int i;
        memset(&c, 0, sizeof c);
        c.node_id = 0u;
        for (i = 0; i < 16; ++i) c.unique_id[i] = 0x10u + (uint16_t)i; /* not the golden uid */
        dronecan_init(&dn, &c);
        { uint16_t fr; for (fr = 0; fr < g->n; ++fr) {
            dronecan_frame_t ff = { g->id[fr], g->dlc[fr], {0}, true };
            for (i = 0; i < g->dlc[fr]; ++i) ff.data[i] = g->data[fr][i];
            dronecan_on_rx(&dn, &ff, &rr);
        } }
        (void)f;
        CHECK(dronecan_node_id(&dn) == 0u); /* not ours -> stays unallocated */
    }

    /* ---- DNA: start delay holds the first request ---- */
    {
        dronecan_t dn;
        dronecan_cfg_t c;
        dronecan_frame_t out[2];
        int n, i;
        memset(&c, 0, sizeof c);
        c.node_id = 0u;
        c.dna_start_delay_ms = 500u;
        for (i = 0; i < 16; ++i) c.unique_id[i] = (uint16_t)(0xA0 + i);
        dronecan_init(&dn, &c);
        n = dronecan_tick(&dn, 0u, NULL, out, 2);     /* t0 captured, still in delay */
        CHECK(n == 0);
        n = dronecan_tick(&dn, 499u, NULL, out, 2);   /* still */
        CHECK(n == 0);
        n = dronecan_tick(&dn, 500u, NULL, out, 2);   /* delay elapsed -> request */
        CHECK(n == 1);
        CHECK(dronecan_id_discriminator(out[0].id) == dronecan_id_discriminator(GOLD_DNA_REQ[0].id));
    }

    /* ---- frame sanitize masks high bits and zeros the tail ---- */
    {
        dronecan_frame_t f;
        f.dlc = 3u;
        f.extended = false;
        f.data[0] = 0x1FFu; f.data[1] = 0xA5u; f.data[2] = 0x2C3u;
        f.data[3] = 0xBEEF; f.data[7] = 0xBEEF;
        dronecan_frame_sanitize(&f);
        CHECK(f.data[0] == 0xFFu);
        CHECK(f.data[1] == 0xA5u);
        CHECK(f.data[2] == 0xC3u);
        CHECK(f.data[3] == 0u);
        CHECK(f.data[7] == 0u);
        CHECK(!f.extended); /* sanitize must not change frame type */
    }

    CHECK_DONE();
}
