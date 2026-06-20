/*
 * dronecan_frame.h - CAN frame DTO + DSDL bit/CRC/float16 primitives (pure, host-tested).
 *
 * C28x note: the minimum addressable unit is 16 bits, so there is no usable uint8_t. Every
 * "wire byte" lives in the low 8 bits of a uint16_t. dronecan_frame_t.data[] and the payload
 * buffer both follow this convention; helpers mask to 0xFF so the high bits never leak onto
 * the bus or into decoding.
 *
 * Bit packing follows UAVCAN v0 little-endian DSDL: a primitive of N bits is emitted as full
 * bytes low-first (MSB-first within each byte), then the remaining high bits MSB-first. The
 * exact layout is validated byte-for-byte by golden frames (tools/test/dronecan_golden.inc);
 * see "bit order validated by golden vectors".
 */
#ifndef DRONECAN_FRAME_H
#define DRONECAN_FRAME_H

#include <stdint.h>
#include <stdbool.h>

/* ---- CAN frame DTO ---- */
typedef struct {
    uint32_t id;        /* 29-bit extended CAN identifier */
    uint16_t dlc;       /* number of data bytes incl. the DroneCAN tail byte (0..8) */
    uint16_t data[8];   /* one wire byte per element (low 8 bits) */
    bool     extended;  /* always true for DroneCAN */
} dronecan_frame_t;

/* Mask data[] to 8 bits over [0,dlc) and zero the remainder. Byte hygiene only; does NOT
 * change f->extended (frame-type validation is the RX dispatcher's responsibility). */
void dronecan_frame_sanitize(dronecan_frame_t *f);

/* ---- CAN ID field accessors ---- */
uint16_t dronecan_id_priority(uint32_t id);        /* bits [28:24] */
uint16_t dronecan_id_source(uint32_t id);          /* bits [6:0], 0 = anonymous */
bool     dronecan_id_is_service(uint32_t id);      /* bit [7] */
uint16_t dronecan_id_msg_dtid(uint32_t id);        /* message frame DTID, bits [23:8] */
uint16_t dronecan_id_discriminator(uint32_t id);   /* anonymous frame, bits [23:10] */

uint32_t dronecan_msg_id(uint16_t priority, uint16_t dtid, uint16_t source);
uint32_t dronecan_anon_id(uint16_t priority, uint16_t dtid, uint16_t discriminator);

/* ---- DroneCAN tail byte ---- */
typedef struct {
    bool     sot;          /* start of transfer */
    bool     eot;          /* end of transfer */
    bool     toggle;
    uint16_t transfer_id;  /* 5 bits */
} dronecan_tail_t;

uint16_t        dronecan_tail_encode(bool sot, bool eot, bool toggle, uint16_t transfer_id);
dronecan_tail_t dronecan_tail_decode(uint16_t tail);

/* ---- DSDL payload bit buffer (wire bytes in uint16) ---- */
#define DRONECAN_PAYLOAD_MAX 24   /* bytes; largest handled message fits well under this */

typedef struct {
    uint16_t bytes[DRONECAN_PAYLOAD_MAX];
    uint16_t bit_len;
} dronecan_payload_t;

void dronecan_payload_init(dronecan_payload_t *p);
uint16_t dronecan_payload_bytelen(const dronecan_payload_t *p);

/* Pack/serialize (append). */
void dronecan_pack_uint(dronecan_payload_t *p, uint32_t value, uint16_t nbits);
void dronecan_pack_int(dronecan_payload_t *p, int32_t value, uint16_t nbits);
void dronecan_pack_float16(dronecan_payload_t *p, float v);
/* Append raw bytes (e.g. unique_id), each low 8 bits. */
void dronecan_pack_bytes(dronecan_payload_t *p, const uint16_t *src, uint16_t n);

/* Unpack/deserialize at *bitpos (advanced in place). */
uint32_t dronecan_unpack_uint(const dronecan_payload_t *p, uint16_t *bitpos, uint16_t nbits);
int32_t  dronecan_unpack_int(const dronecan_payload_t *p, uint16_t *bitpos, uint16_t nbits);
float    dronecan_unpack_float16(const dronecan_payload_t *p, uint16_t *bitpos);

/* ---- float16 (IEEE binary16) ---- */
uint16_t dronecan_float32_to_float16(float v);
float    dronecan_float16_to_float32(uint16_t h);

/* ---- CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) ---- */
uint16_t dronecan_crc16(const uint16_t *bytes, uint16_t n, uint16_t init);
/* Transfer-CRC seed: feed the 64-bit data type signature as little-endian wire bytes. */
uint16_t dronecan_transfer_crc_seed(uint32_t sig_lo, uint32_t sig_hi);

#endif /* DRONECAN_FRAME_H */
