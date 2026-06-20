/* dronecan_golden.inc -- GENERATED, do not edit by hand.
 *
 * Provenance:
 *   generator : tools/test/gen_dronecan_golden.py
 *   pydronecan: 1.0.27
 *   command   : python3 tools/test/gen_dronecan_golden.py > tools/test/dronecan_golden.inc
 *   unique_id : {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF}
 *
 * Each *_data[] element is ONE wire byte (low 8 bits) in a uint16_t, matching
 * the C28x dronecan_frame_t.data[] convention. dlc includes the DroneCAN tail byte.
 * Frames come from pydronecan's reference serializer (the interop anchor).
 */

/* ===== RawCommand (DTID 1030) RX golden ===== */
typedef struct { const char *name; uint32_t id; uint16_t dlc; uint16_t data[8];
                 uint16_t n_frames; uint16_t cmd_count; } gold_raw_t;
static const gold_raw_t GOLD_RAW[] = {
  /* raw_1esc: cmd=[5000] src=42 tid=1 frames=1 */
  { "raw_1esc", 0x1004062A, 3, {0x88, 0x4C, 0xC1}, 1, 1 },
  /* raw_4esc: cmd=[0, 8191, -100, 4096] src=42 tid=5 frames=1 */
  { "raw_4esc", 0x1004062A, 8, {0x00, 0x03, 0xFD, 0xF9, 0xCF, 0xC0, 0x10, 0xC5}, 1, 4 },
  /* raw_5esc_multiframe: cmd=[1, 2, 3, 4, 5] src=42 tid=7 frames=2 */
  { "raw_5esc_multiframe", 0x1004062A, 8, {0xDE, 0x83, 0x01, 0x00, 0x08, 0x00, 0x30, 0x87}, 2, 5 },
};

/* expected decode: GOLD_RAW index, esc_index, expected raw14, present(1/0) */
typedef struct { uint16_t raw_idx; uint16_t esc_index; int16_t raw14; uint16_t present; } gold_raw_dec_t;
static const gold_raw_dec_t GOLD_RAW_DEC[] = {
  { 0, 0, 5000, 1 }, { 0, 1, 0, 0 },
  { 1, 0, 0, 1 }, { 1, 1, 8191, 1 }, { 1, 2, -100, 1 }, { 1, 3, 4096, 1 }, { 1, 4, 0, 0 },
};

/* ===== esc.Status (DTID 1034) TX golden ===== */
typedef struct { const char *name; float voltage, current, temp_K; int32_t rpm;
                 uint16_t esc_index; uint32_t src; uint16_t tid;
                 uint16_t n; uint32_t id[3]; uint16_t dlc[3]; uint16_t data[3][8]; } gold_status_t;
static const gold_status_t GOLD_STATUS[] = {
  /* status_nominal: {'error_count': 0, 'voltage': 24.0, 'current': 1.5, 'temperature': 300.0, 'rpm': 1234, 'power_rating_pct': 0, 'esc_index': 0} */
  { "status_nominal", 24.000000f, 1.500000f, 300.000000f, 1234, 0, 42, 3, 3,
    {0x10040A2A, 0x10040A2A, 0x10040A2A}, {8, 8, 3}, {{0xF1, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83}, {0x4E, 0x00, 0x3E, 0xB0, 0x5C, 0xD2, 0x04, 0x23}, {0x00, 0x00, 0x43}} },
  /* status_idx7: {'error_count': 0, 'voltage': 12.5, 'current': -2.0, 'temperature': 263.15, 'rpm': -500, 'power_rating_pct': 0, 'esc_index': 7} */
  { "status_idx7", 12.500000f, -2.000000f, 263.150000f, -500, 7, 42, 4, 3,
    {0x10040A2A, 0x10040A2A, 0x10040A2A}, {8, 8, 3}, {{0x5C, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x40, 0x84}, {0x4A, 0x00, 0xC0, 0x1D, 0x5C, 0x0C, 0xFE, 0x24}, {0xC0, 0x1C, 0x44}} },
};

/* ===== NodeStatus (DTID 341) TX golden ===== */
typedef struct { const char *name; uint32_t uptime; uint16_t health; uint16_t vendor;
                 uint32_t src; uint16_t tid; uint32_t id; uint16_t dlc; uint16_t data[8]; } gold_ns_t;
static const gold_ns_t GOLD_NS[] = {
  /* ns_ok: {'uptime_sec': 7, 'health': 0, 'mode': 0, 'sub_mode': 0, 'vendor_specific_status_code': 0} */
  { "ns_ok", 7, 0, 0x0000, 42, 2, 0x1001552A, 8, {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2} },
  /* ns_fault_ov: {'uptime_sec': 123, 'health': 2, 'mode': 0, 'sub_mode': 0, 'vendor_specific_status_code': 2} */
  { "ns_fault_ov", 123, 2, 0x0002, 42, 6, 0x1001552A, 8, {0x7B, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0xC6} },
};

/* ===== DNA (DTID 1) golden ===== */
typedef struct { const char *name; uint16_t first_part; uint16_t uid_len;
                 uint16_t uid[16]; uint32_t tid; uint32_t id; uint16_t dlc; uint16_t data[8]; } gold_dna_t;
static const gold_dna_t GOLD_DNA_REQ[] = {
  /* dna_req_stage1: first=True uid=['0xa0', '0xa1', '0xa2', '0xa3', '0xa4', '0xa5'] */
  { "dna_req_stage1", 1, 6, {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0, 0x1EFE5500, 8, {0x01, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xC0} },
  /* dna_req_stage2: first=False uid=['0xa6', '0xa7', '0xa8', '0xa9', '0xaa', '0xab'] */
  { "dna_req_stage2", 0, 6, {0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 1, 0x1EA8ED00, 8, {0x00, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xC1} },
  /* dna_req_stage3: first=False uid=['0xac', '0xad', '0xae', '0xaf'] */
  { "dna_req_stage3", 0, 4, {0xAC, 0xAD, 0xAE, 0xAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 2, 0x1EC7C500, 6, {0x00, 0xAC, 0xAD, 0xAE, 0xAF, 0xC2} },
};

typedef struct { const char *name; uint16_t exp_node_id; uint16_t exp_uid_len;
                 uint16_t n; uint32_t id[3]; uint16_t dlc[3]; uint16_t data[3][8]; } gold_dna_resp_t;
static const gold_dna_resp_t GOLD_DNA_RESP[] = {
  /* dna_echo_s1: node_id=0 uid_len=6 src=100 tid=0 frames=1 */
  { "dna_echo_s1", 0, 6, 1, {0x10000164, 0, 0}, {8, 0, 0}, {{0x01, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xC0}, {0}, {0}} },
  /* dna_echo_s2: node_id=0 uid_len=12 src=100 tid=1 frames=3 */
  { "dna_echo_s2", 0, 12, 3, {0x10000164, 0x10000164, 0x10000164}, {8, 8, 2}, {{0x2A, 0x08, 0x00, 0xA0, 0xA1, 0xA2, 0xA3, 0x81}, {0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0x21}, {0xAB, 0x41}} },
  /* dna_final: node_id=25 uid_len=16 src=100 tid=2 frames=3 */
  { "dna_final", 25, 16, 3, {0x10000164, 0x10000164, 0x10000164}, {8, 8, 6}, {{0xFE, 0xE7, 0x32, 0xA0, 0xA1, 0xA2, 0xA3, 0x82}, {0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0x22}, {0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0x42}} },
};

