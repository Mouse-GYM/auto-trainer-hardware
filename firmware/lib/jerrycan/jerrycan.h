#pragma once

#include <zephyr/kernel.h>

#include "jerrycan_types.h"

// Run the main service loop for JerryCAN - RX callbacks will be processed in the context of the calling thread
// Returns 0 on success, -EAGAIN if no message is available, or another negative error code on failure
int jerrycan_run(k_timeout_t timeout);

// Register a callback to be called when a message of the specified type is received
void jerrycan_register_rx_callback(jerrycan_rx_callback_t *callback);

// Called from the CAN ISR for every ESTOP frame addressed to this node or broadcast, with whether it engages (true)
// or releases the e-stop. ESTOP frames bypass the RX queue and jerrycan_run(), so a full queue or a busy loop can
// neither drop nor delay them; the handler must not block.
typedef void (*jerrycan_estop_handler_t)(bool engage);

void jerrycan_set_estop_handler(jerrycan_estop_handler_t handler);

// Send a CAN message
int jerrycan_tx(jerrycan_msg_t *msg, k_timeout_t timeout);

void jerrycan_send_ack(uint8_t uuid, int error_code);
