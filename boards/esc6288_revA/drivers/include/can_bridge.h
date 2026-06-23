/*
 * can_bridge.h - CANA peripheral bridge: 29-bit extended CAN frame <-> dronecan_frame_t.
 *
 * driverlib glue for the DroneCAN transport on esc6288_revA (CANA, 1 Mbit, GPIO37 TX /
 * GPIO35 RX via the SIT1042 transceiver). It owns one RX and one TX dronecan_fifo and an
 * INT_CANA0 ISR. This layer is hardware-only (driverlib) -- it is NOT host-tested; the
 * queue logic (dronecan_fifo) and the protocol core (dronecan.*) are the host-tested pure
 * layers. The bridge is self-contained: the product main calls can_bridge_init() +
 * can_bridge_enable_ints(); it is not wired into the labs. Pins come from board.h
 * (BOARD_CAN_*), so this file is identical to the launchxl bridge.
 */
#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "dronecan_frame.h"

/* Configure CANA (GPIO mux, bit rate, RX wildcard + TX mailboxes) and the FIFOs. */
void can_bridge_init(void);

/* Register + enable the INT_CANA0 interrupt. Call after can_bridge_init(). */
void can_bridge_enable_ints(void);

/* CANA0 ISR (RX receive + TX-complete). Registered by can_bridge_enable_ints(). */
__interrupt void can_bridge_rxISR(void);

/* Main-side: pop one received frame (false if none). */
bool can_bridge_read(dronecan_frame_t *f);

/* Main-side: queue a frame for transmit (false if rejected/full). */
bool can_bridge_write(const dronecan_frame_t *f);

/* Main-side: kick the TX mailbox if idle (also done from write/ISR). */
void can_bridge_tx_pump(void);

uint32_t can_bridge_rx_dropped(void);
uint32_t can_bridge_tx_dropped(void);

#endif /* CAN_BRIDGE_H */
