/*
 * dronecan_fifo.h - Single-producer/single-consumer ring buffer of dronecan_frame_t.
 *
 * Pure logic (no driverlib), host-tested. This is the queue between the CAN peripheral
 * bridge and the DroneCAN core: an RX instance (ISR produces, main consumes) and a TX
 * instance (main produces, ISR consumes). Lock-free SPSC -- safe with one producer and one
 * consumer because the producer only advances head and the consumer only advances tail; the
 * uint16 index writes are atomic on the C28x. Full -> drop the new frame (never overwrite)
 * and bump a diagnostic counter.
 */
#ifndef DRONECAN_FIFO_H
#define DRONECAN_FIFO_H

#include "dronecan_frame.h"

#define DRONECAN_FIFO_CAP 16u   /* power of two; usable capacity is CAP-1 */

typedef struct {
    volatile uint16_t head;     /* producer index */
    volatile uint16_t tail;     /* consumer index */
    volatile uint32_t dropped;  /* frames dropped on full (producer writes, consumer reads for diag) */
    dronecan_frame_t buf[DRONECAN_FIFO_CAP];
} dronecan_fifo_t;

void dronecan_fifo_init(dronecan_fifo_t *q);

/* Producer: copy frame in. Returns false (and bumps dropped) when full. */
bool dronecan_fifo_push(dronecan_fifo_t *q, const dronecan_frame_t *f);

/* Consumer: copy next frame out. Returns false when empty. */
bool dronecan_fifo_pop(dronecan_fifo_t *q, dronecan_frame_t *f);

bool     dronecan_fifo_empty(const dronecan_fifo_t *q);
bool     dronecan_fifo_full(const dronecan_fifo_t *q);
uint16_t dronecan_fifo_count(const dronecan_fifo_t *q);

#endif /* DRONECAN_FIFO_H */
