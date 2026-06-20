#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "dronecan_frame.h"
#include "dronecan_fifo.h"
#include "can_bridge.h"
#include <stddef.h>

#define CAN_BRIDGE_RX_OBJ  1U   /* RX wildcard mailbox */
#define CAN_BRIDGE_TX_OBJ  2U   /* TX mailbox (id reconfigured per frame) */

static dronecan_fifo_t  s_rx;
static dronecan_fifo_t  s_tx;
static volatile bool    s_tx_in_flight;

void can_bridge_init(void)
{
    dronecan_fifo_init(&s_rx);
    dronecan_fifo_init(&s_tx);
    s_tx_in_flight = false;

    /* CAN GPIO mux: TX push-pull (pull-up), RX input. */
    GPIO_setPinConfig(BOARD_CAN_TX_PINCFG);
    GPIO_setDirectionMode(BOARD_CAN_TX_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(BOARD_CAN_TX_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(BOARD_CAN_RX_PINCFG);
    GPIO_setDirectionMode(BOARD_CAN_RX_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(BOARD_CAN_RX_GPIO, GPIO_PIN_TYPE_STD);

    CAN_initModule(BOARD_CAN_BASE);
    CAN_setBitRate(BOARD_CAN_BASE, DEVICE_SYSCLK_FREQ, BOARD_CAN_BITRATE, 10U);

    /* RX: single wildcard mailbox accepting any 29-bit extended id (mask 0 + EXT filter).
     * Software (dronecan_on_rx) does the protocol-level filtering. */
    CAN_setupMessageObject(BOARD_CAN_BASE, CAN_BRIDGE_RX_OBJ, 0U,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0U,
                           CAN_MSG_OBJ_RX_INT_ENABLE | CAN_MSG_OBJ_USE_EXT_FILTER, 8U);
    /* TX: one mailbox, its id is rewritten per frame in tx_load_next(). */
    CAN_setupMessageObject(BOARD_CAN_BASE, CAN_BRIDGE_TX_OBJ, 0U,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0U,
                           CAN_MSG_OBJ_TX_INT_ENABLE, 8U);

    CAN_startModule(BOARD_CAN_BASE);
}

void can_bridge_enable_ints(void)
{
    Interrupt_register(BOARD_CAN_INT, &can_bridge_rxISR);
    CAN_enableInterrupt(BOARD_CAN_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    Interrupt_enable(BOARD_CAN_INT);
    Interrupt_enableInCPU(INTERRUPT_CPU_INT9);
    CAN_enableGlobalInterrupt(BOARD_CAN_BASE, CAN_GLOBAL_INT_CANINT0);
}

/*
 * Load the next queued TX frame into the single TX mailbox and send it.
 * Caller MUST hold the critical section (INT_CANA0 disabled) or be inside the ISR -- the
 * tx_in_flight flag + the shared TX mailbox are not protected by the (SPSC) FIFO itself.
 * CAN_setupMessageObject returns before its IF1 transfer finishes, so wait for IF1 BUSY to
 * clear before CAN_sendMessage rewrites IF1. setupMessageObject re-applies TX_INT_ENABLE
 * (the TXIE bit) every time, so the TX-complete interrupt keeps firing.
 */
static void tx_load_next(void)
{
    dronecan_frame_t f;

    if (s_tx_in_flight) {
        return;
    }
    if (!dronecan_fifo_pop(&s_tx, &f)) {
        return;
    }
    s_tx_in_flight = true;
    CAN_setupMessageObject(BOARD_CAN_BASE, CAN_BRIDGE_TX_OBJ, f.id,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0U,
                           CAN_MSG_OBJ_TX_INT_ENABLE, f.dlc);
    while ((HWREGH(BOARD_CAN_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) != 0U) {
    }
    CAN_sendMessage(BOARD_CAN_BASE, CAN_BRIDGE_TX_OBJ, f.dlc, f.data);
}

void can_bridge_tx_pump(void)
{
    Interrupt_disable(BOARD_CAN_INT);
    tx_load_next();
    Interrupt_enable(BOARD_CAN_INT);
}

bool can_bridge_write(const dronecan_frame_t *f)
{
    bool ok;

    if (f == NULL || !f->extended || f->dlc > 8U) {
        return false; /* reject malformed before it reaches the bus */
    }
    Interrupt_disable(BOARD_CAN_INT);
    ok = dronecan_fifo_push(&s_tx, f);
    tx_load_next();
    Interrupt_enable(BOARD_CAN_INT);
    return ok;
}

bool can_bridge_read(dronecan_frame_t *f)
{
    return dronecan_fifo_pop(&s_rx, f); /* main is the sole RX consumer (SPSC) */
}

uint32_t can_bridge_rx_dropped(void) { return s_rx.dropped; }
uint32_t can_bridge_tx_dropped(void) { return s_tx.dropped; }

#pragma CODE_SECTION(can_bridge_rxISR, ".TI.ramfunc");
__interrupt void can_bridge_rxISR(void)
{
    uint32_t cause = CAN_getInterruptCause(BOARD_CAN_BASE);

    if (cause == CAN_INT_INT0ID_STATUS) {
        (void)CAN_getStatus(BOARD_CAN_BASE); /* reading ES clears the status interrupt */
    } else if (cause == CAN_BRIDGE_RX_OBJ) {
        CAN_MsgFrameType ftype;
        uint32_t rid;
        uint16_t tmp[16];   /* >= max raw DLC (4-bit, up to 15) so driverlib's DLC-sized copy
                             * can never overflow our buffer. */
        if (CAN_readMessageWithID(BOARD_CAN_BASE, CAN_BRIDGE_RX_OBJ, &ftype, &rid, tmp)) {
            uint16_t raw_dlc = (uint16_t)(HWREGH(BOARD_CAN_BASE + CAN_O_IF2MCTL)
                                          & CAN_IF2MCTL_DLC_M);
            /* Validate BEFORE trusting the length: extended frame + real DLC. Discard bad
             * frames (do not clamp -- clamping would disguise a corrupt frame as 8 bytes). */
            if ((ftype == CAN_MSG_FRAME_EXT) && (raw_dlc <= 8U)) {
                dronecan_frame_t f;
                uint16_t i;
                f.id = rid;
                f.dlc = raw_dlc;
                f.extended = true;
                for (i = 0U; i < 8U; ++i) {
                    f.data[i] = (i < raw_dlc) ? (tmp[i] & 0xFFU) : 0U;
                }
                (void)dronecan_fifo_push(&s_rx, &f);
            }
        }
        /* All RX paths (incl. discard) clear the message-object pending flag (CLRINTPND), not
         * just NEWDAT, or a bad frame would wedge the CANA0 interrupt. */
        CAN_clearInterruptStatus(BOARD_CAN_BASE, CAN_BRIDGE_RX_OBJ);
    } else if (cause == CAN_BRIDGE_TX_OBJ) {
        CAN_clearInterruptStatus(BOARD_CAN_BASE, CAN_BRIDGE_TX_OBJ);
        s_tx_in_flight = false;
        tx_load_next(); /* in ISR context (atomic w.r.t. main): send the next queued frame */
    }

    CAN_clearGlobalInterruptStatus(BOARD_CAN_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
