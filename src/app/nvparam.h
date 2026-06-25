/*
 * nvparam.h - non-volatile parameter record: serialize / validate (pure logic).
 *
 * Holds the small set of fields the ESC must survive a power cycle:
 *   - node_id           : DroneCAN node id (0 = dynamic/DNA, 1..127 = static)
 *   - park_ref_valid    : has a parked-prop reference angle been learned?
 *   - park_ref_target_rev: the learned reference (mechanical revolutions)
 *
 * This module is the STORAGE FORMAT only -- it never erases or programs Flash. The
 * app / product main owns the actual driverlib erase/program; it asks this module to
 * encode the record into a word buffer (to write) and to decode + validate a buffer it
 * read back. Keeping the format pure makes the magic/version/CRC/range rules host-testable.
 *
 * WORD-ORIENTED ON PURPOSE: the C28x has no 8-bit type (char is 16-bit), so the record is
 * an array of uint16_t words and the CRC/float (de)serialization is done with explicit
 * shifts -- never byte addressing -- so host (gcc) and target (C28x) agree bit-for-bit.
 */
#ifndef NVPARAM_H
#define NVPARAM_H

#include <stdint.h>
#include <stdbool.h>

/* On-storage layout (uint16_t words):
 *   [0] magic   [1] version   [2] node_id   [3] flags
 *   [4] park_ref low16   [5] park_ref high16   [6] CRC16 over words[0..5]
 */
#define NVPARAM_WORDS      7u
#define NVPARAM_MAGIC      0x4E50u   /* 'N','P' -- novaX param record           */
#define NVPARAM_VERSION    0x0001u   /* bump when the layout changes            */
#define NVPARAM_NODE_ID_MAX 127u     /* DroneCAN static id range is 1..127      */

#define NVPARAM_FLAG_PARK_REF_VALID  0x0001u

typedef enum {
    NVPARAM_OK = 0,        /* structurally valid AND every field already in range  */
    NVPARAM_OK_SANITIZED,  /* structurally valid but a field was out of range and  */
                           /*   was reset to its default (caller should re-store)  */
    NVPARAM_ERR_MAGIC,     /* wrong/blank magic  -> defaults returned              */
    NVPARAM_ERR_VERSION,   /* unknown version    -> defaults returned              */
    NVPARAM_ERR_CRC,       /* CRC mismatch       -> defaults returned              */
} nvparam_status_t;

typedef struct {
    uint16_t node_id;            /* 0 = DNA, 1..127 = static                       */
    bool     park_ref_valid;
    float    park_ref_target_rev;
} nvparam_t;

/* Power-on defaults: node_id = 0 (DNA), park ref invalid. */
void nvparam_set_defaults(nvparam_t *p);

/*
 * In-place range check / sanitize. node_id > 127 -> 0 (DNA); a non-finite (NaN/Inf)
 * park ref, or a "valid" park ref that is non-finite, -> invalid + 0. Returns true if
 * the record was already clean (nothing changed), false if a field had to be reset.
 */
bool nvparam_validate(nvparam_t *p);

/* Convenience setters that sanitize on the way in. Return true if the value was accepted
 * as-is, false if it was out of range and a default/sanitized value was stored instead. */
bool nvparam_update_node_id(nvparam_t *p, uint16_t node_id);
bool nvparam_update_park_ref(nvparam_t *p, bool valid, float target_rev);

/* Serialize p into exactly NVPARAM_WORDS words (magic/version/fields/CRC). The input is
 * sanitized first so a malformed in-RAM record can never be written as a valid one. */
void nvparam_encode(const nvparam_t *p, uint16_t words[NVPARAM_WORDS]);

/*
 * Decode + validate NVPARAM_WORDS words read back from storage. On any structural failure
 * (magic/version/CRC) *out is filled with defaults and the matching error code is returned.
 * On structural success the fields are sanitized: NVPARAM_OK if they were already in range,
 * NVPARAM_OK_SANITIZED if one had to be reset.
 */
nvparam_status_t nvparam_decode(const uint16_t words[NVPARAM_WORDS], nvparam_t *out);

/* CRC16-CCITT (poly 0x1021, init 0xFFFF) over n words, high byte then low byte of each
 * word. Exposed for tests; deterministic on host and target (no byte addressing). */
uint16_t nvparam_crc16(const uint16_t *words, uint16_t n);

#endif /* NVPARAM_H */
