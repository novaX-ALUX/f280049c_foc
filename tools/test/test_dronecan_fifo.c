#include "check.h"
#include "dronecan_fifo.h"
#include <string.h>

static dronecan_frame_t mkframe(uint32_t id)
{
    dronecan_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.dlc = 1u;
    f.data[0] = (uint16_t)(id & 0xFFu);
    f.extended = true;
    return f;
}

int main(void)
{
    /* init -> empty, not full, count 0 */
    {
        dronecan_fifo_t q;
        dronecan_fifo_init(&q);
        CHECK(dronecan_fifo_empty(&q));
        CHECK(!dronecan_fifo_full(&q));
        CHECK(dronecan_fifo_count(&q) == 0u);
        CHECK(q.dropped == 0u);
        dronecan_frame_t out;
        CHECK(!dronecan_fifo_pop(&q, &out)); /* pop empty */
    }

    /* FIFO order */
    {
        dronecan_fifo_t q;
        dronecan_frame_t out;
        dronecan_fifo_init(&q);
        CHECK(dronecan_fifo_push(&q, &(dronecan_frame_t){.id = 10}));
        CHECK(dronecan_fifo_push(&q, &(dronecan_frame_t){.id = 20}));
        CHECK(dronecan_fifo_push(&q, &(dronecan_frame_t){.id = 30}));
        CHECK(dronecan_fifo_count(&q) == 3u);
        CHECK(dronecan_fifo_pop(&q, &out) && out.id == 10u);
        CHECK(dronecan_fifo_pop(&q, &out) && out.id == 20u);
        CHECK(dronecan_fifo_pop(&q, &out) && out.id == 30u);
        CHECK(dronecan_fifo_empty(&q));
    }

    /* fill to full (CAP-1), next push drops + counter increments */
    {
        dronecan_fifo_t q;
        uint16_t i;
        dronecan_fifo_init(&q);
        for (i = 0; i < DRONECAN_FIFO_CAP - 1u; ++i) {
            dronecan_frame_t f = mkframe(i);
            CHECK(dronecan_fifo_push(&q, &f));
        }
        CHECK(dronecan_fifo_full(&q));
        CHECK(dronecan_fifo_count(&q) == DRONECAN_FIFO_CAP - 1u);
        {
            dronecan_frame_t f = mkframe(999);
            CHECK(!dronecan_fifo_push(&q, &f)); /* full -> drop */
            CHECK(q.dropped == 1u);
        }
        /* values come back in order and unchanged */
        for (i = 0; i < DRONECAN_FIFO_CAP - 1u; ++i) {
            dronecan_frame_t out;
            CHECK(dronecan_fifo_pop(&q, &out) && out.id == i);
        }
        CHECK(dronecan_fifo_empty(&q));
    }

    /* wrap-around: many push/pop cycles exceed CAP */
    {
        dronecan_fifo_t q;
        uint32_t i;
        dronecan_fifo_init(&q);
        for (i = 0; i < 100u; ++i) {
            dronecan_frame_t f = mkframe(i);
            dronecan_frame_t out;
            CHECK(dronecan_fifo_push(&q, &f));
            CHECK(dronecan_fifo_pop(&q, &out) && out.id == i);
        }
        CHECK(dronecan_fifo_empty(&q));
        CHECK(q.dropped == 0u);
    }

    CHECK_DONE();
}
