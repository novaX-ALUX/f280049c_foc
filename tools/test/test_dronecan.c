#include "check.h"
#include "dronecan_frame.h"
#include "dronecan_ids.h"
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

    /* ---- frame sanitize masks high bits and zeros the tail ---- */
    {
        dronecan_frame_t f;
        f.dlc = 3u;
        f.data[0] = 0x1FFu; f.data[1] = 0xA5u; f.data[2] = 0x2C3u;
        f.data[3] = 0xBEEF; f.data[7] = 0xBEEF;
        dronecan_frame_sanitize(&f);
        CHECK(f.data[0] == 0xFFu);
        CHECK(f.data[1] == 0xA5u);
        CHECK(f.data[2] == 0xC3u);
        CHECK(f.data[3] == 0u);
        CHECK(f.data[7] == 0u);
        CHECK(f.extended);
    }

    CHECK_DONE();
}
