#include "dronecan_fifo.h"

#define FIFO_MASK (DRONECAN_FIFO_CAP - 1u)

void dronecan_fifo_init(dronecan_fifo_t *q)
{
    q->head    = 0u;
    q->tail    = 0u;
    q->dropped = 0u;
}

bool dronecan_fifo_push(dronecan_fifo_t *q, const dronecan_frame_t *f)
{
    uint16_t h   = q->head;
    uint16_t nxt = (uint16_t)((h + 1u) & FIFO_MASK);
    if (nxt == q->tail) {
        q->dropped++;     /* full -> drop the new frame */
        return false;
    }
    q->buf[h] = *f;       /* write the frame data ... */
    q->head = nxt;        /* ... then publish it (release) */
    return true;
}

bool dronecan_fifo_pop(dronecan_fifo_t *q, dronecan_frame_t *f)
{
    uint16_t t = q->tail;
    if (t == q->head) {
        return false;     /* empty */
    }
    *f = q->buf[t];       /* read the frame ... */
    q->tail = (uint16_t)((t + 1u) & FIFO_MASK);  /* ... then free the slot */
    return true;
}

bool dronecan_fifo_empty(const dronecan_fifo_t *q)
{
    return q->head == q->tail;
}

bool dronecan_fifo_full(const dronecan_fifo_t *q)
{
    return (uint16_t)((q->head + 1u) & FIFO_MASK) == q->tail;
}

uint16_t dronecan_fifo_count(const dronecan_fifo_t *q)
{
    return (uint16_t)((q->head - q->tail) & FIFO_MASK);
}
