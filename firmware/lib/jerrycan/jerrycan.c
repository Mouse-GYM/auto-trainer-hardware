#include "jerrycan.h"

#include <generic_gpios.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>

static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *gpio_dev = DEVICE_DT_GET_ANY(ll_generic_gpios);

LOG_MODULE_REGISTER(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

static sys_slist_t can_rx_callbacks_list;

CAN_MSGQ_DEFINE(jerrycan_rx_msgq, 10);
K_MSGQ_DEFINE(jerrycan_tx_msgq, sizeof(jerrycan_msg_t), 150, 4);

// CAN_ID[10:0] = { MsgID[5:0], DeviceType[2:0], ModuleAddr[1:0] }
// The combination of DeviceType and ModuleAddr uniquely identifies this device (NODE_ID)
#define NODE_ID_MASK 0x1F
static uint8_t can_node_id;

// How long a transmit may wait for a free TX mailbox, and then for the frame to go out. A frame nobody acknowledges
// (no other node, a wiring fault, bus-off) never completes; without these bounds the first such frame would block
// the loop that also handles every received command. A healthy bus sends a frame in well under a millisecond.
#define CAN_TX_MAILBOX_TIMEOUT K_MSEC(50)
#define CAN_TX_DONE_TIMEOUT K_MSEC(50)

static K_SEM_DEFINE(can_tx_done_sem, 0, 1);

static inline uint16_t can_id(jerrycan_cmd_type_t msg_type) { return (uint16_t)msg_type << 5 | can_node_id; }

static void can_tx_done(const struct device *dev, int error, void *user_data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    k_sem_give(user_data);
}

/**
 * Send `frame` and wait, within bounds, for it to complete, so frames still go out one at a time and in order.
 *
 * @return 0 once sent, -EAGAIN if the bus didn't take it in time, or the driver's error.
 */
static int jerrycan_send_frame(const struct can_frame *frame) {
    // A frame that timed out earlier may complete now; don't count that as this one.
    k_sem_reset(&can_tx_done_sem);

    const int ret = can_send(can_dev, frame, CAN_TX_MAILBOX_TIMEOUT, can_tx_done, &can_tx_done_sem);
    if (ret != 0) {
        return ret;
    }

    return k_sem_take(&can_tx_done_sem, CAN_TX_DONE_TIMEOUT) == 0 ? 0 : -EAGAIN;
}

static int get_can_node_id(uint8_t *can_node_id_out) {
    // Read the DeviceType bits to make sure this is a Pellet Module
    uint32_t device_type = 0;
    int ret = ll_generic_gpio_read(gpio_dev, 0x3, &device_type);
    if (ret != 0) {
        LOG_ERR("Failed to read the device type pins: %d", ret);
        return ret;
    }

    // Read the NodeID to program up the CAN_ID properly
    uint32_t node_id = 0;
    ret = ll_generic_gpio_read(gpio_dev, 0xC, &node_id);
    if (ret != 0) {
        LOG_ERR("Failed to read the node ID pins: %d", ret);
        return ret;
    }
    node_id = (node_id >> 2) & 0x3;

    // Print out the device type and node ID
    LOG_INF("CAN_ID: 0x%X", (node_id << 2) | device_type);

    *can_node_id_out = ((node_id << 2) | device_type) & NODE_ID_MASK;
    return 0;
}

// Return the payload size for a given message type
static uint8_t jerrycan_msg_get_payload_size(jerrycan_cmd_type_t msg_type) {
    static const uint8_t jerrycan_size_map[] = {
        [JERRYCAN_CMD_ESTOP] = sizeof(jerrycan_cmd_estop_t),
        [JERRYCAN_CMD_HEARTBEAT] = sizeof(jerrycan_cmd_heartbeat_t),
        [JERRYCAN_CMD_STATUS] = sizeof(jerrycan_cmd_status_t),
        [JERRYCAN_CMD_STEPPER_MOVE] = sizeof(jerrycan_cmd_stepper_move_t),
        [JERRYCAN_CMD_SERVO_MOVE] = sizeof(jerrycan_cmd_servo_move_t),
        [JERRYCAN_CMD_SERVO_ATTACH] = sizeof(jerrycan_cmd_servo_attach_t),
        [JERRYCAN_CMD_SERVO_DETACH] = sizeof(jerrycan_cmd_servo_detach_t),
        [JERRYCAN_CMD_STEPPER_HOME] = sizeof(jerrycan_cmd_stepper_home_t),
        [JERRYCAN_CMD_STEPPER_FAULT_CLEAR] = sizeof(jerrycan_cmd_stepper_fault_clear_t),
        [JERRYCAN_CMD_CFG_WRITE] = sizeof(jerrycan_cmd_cfg_t),
        [JERRYCAN_CMD_CFG_RESPONSE] = sizeof(jerrycan_cmd_cfg_t),
        [JERRYCAN_CMD_CFG_READ] = sizeof(jerrycan_cmd_cfg_t),
        [JERRYCAN_CMD_STEPPER_STATUS] = sizeof(jerrycan_cmd_stepper_status_t),
        [JERRYCAN_CMD_SERVO_STATUS] = sizeof(jerrycan_cmd_servo_status_t),
        [JERRYCAN_CMD_PRESSURE_READ] = sizeof(jerrycan_cmd_pressure_read_t),
        [JERRYCAN_CMD_TEMP_HUM_READ] = sizeof(jerrycan_cmd_temp_hum_read_t),
        [JERRYCAN_CMD_GPIO_READ] = sizeof(jerrycan_cmd_gpio_read_t),
        [JERRYCAN_CMD_GPIO_WRITE] = sizeof(jerrycan_cmd_gpio_write_t),
        [JERRYCAN_CMD_TONE] = sizeof(jerrycan_cmd_tone_t),
        [JERRYCAN_CMD_ANALOG_OUT] = sizeof(jerrycan_cmd_analog_out_t),
        [JERRYCAN_CMD_LOAD_CELL_READ] = sizeof(jerrycan_cmd_load_cell_read_t),
        [JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_BEGIN] = sizeof(jerrycan_cmd_audio_data_cmd_t),
        [JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT] = sizeof(jerrycan_cmd_audio_data_t),
        [JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END] = sizeof(jerrycan_cmd_audio_data_cmd_t),
        [JERRYCAN_CMD_LOAD_CELL_TARE] = sizeof(jerrycan_cmd_load_cell_tare_t),
        [JERRYCAN_CMD_RGB_LED] = sizeof(jerrycan_cmd_rgb_led_t),
        [JERRYCAN_CMD_DOOR_SENSOR] = sizeof(jerrycan_cmd_door_closed_t),
        [JERRYCAN_CMD_BOOTLOADER_COMMAND] = sizeof(jerrycan_cmd_bootloader_command_t),
        [JERRYCAN_CMD_BOOTLOADER_RESPONSE] = sizeof(jerrycan_cmd_bootloader_response_t),
        [JERRYCAN_CMD_BOOTLOADER_DATA] = sizeof(jerrycan_cmd_bootloader_data_t),
        [JERRYCAN_CMD_DELAY] = sizeof(jerrycan_cmd_delay_t),
        [JERRYCAN_CMD_FIXED_XYZ] = sizeof(jerrycan_cmd_fixed_xyz),
        [JERRYCAN_RSP_ACK] = sizeof(jerrycan_rsp_ack_t),
    };

    if (msg_type > JERRYCAN_CMD_MAX || msg_type < JERRYCAN_CMD_MIN) {
        return 0;
    }

    return jerrycan_size_map[msg_type];
}

// Add a jerrycan message to the TX queue- these messages will be processed by the main jerrycan_run() loop
// This allows any thread or interrupt to generate an outgoing CAN message which will then be handled by the main task
int jerrycan_tx(jerrycan_msg_t *msg, k_timeout_t timeout) {
    static size_t failureCount = 0;

    // Place message in queue
    const int ret = k_msgq_put(&jerrycan_tx_msgq, msg, timeout);
    if (ret != 0) {
        if (!failureCount++) {
            LOG_ERR("jerrycan_tx failed to place message in queue: %d", ret);
        }
    } else {
        if (failureCount) {
            LOG_ERR("jerrycan transmissions restored. %d lost packets.", failureCount);
        }
        failureCount = 0;
    }

    return ret;
}

// Consolidated function for TX and RX handling
int jerrycan_run(k_timeout_t timeout) {
    static jerrycan_msg_t msg;
    static struct can_frame frame;
    static bool tx_stalled;
    int ret;

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    // Defined in modules/bootloader.c, which is only built with MCUboot
    void confirm_image();
    confirm_image();
#endif

    static struct k_poll_event events[2] = {
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &jerrycan_tx_msgq, 1),
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &jerrycan_rx_msgq,
                                        2)};

    // Poll for events in the RX and TX queues
    ret = k_poll(events, sizeof(events) / sizeof(struct k_poll_event), timeout);
    if (ret != 0) {
        if (ret == -EAGAIN) {
            LOG_DBG("JerryCAN timed out while polling for events");
        } else {
            LOG_ERR("JerryCAN failed to poll for events: %d", ret);
        }
        return ret;
    }

    // TX processing
    if (events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
        while (k_msgq_get(&jerrycan_tx_msgq, &msg, K_NO_WAIT) == 0) {
            uint8_t payload_size = jerrycan_msg_get_payload_size(msg.type);
            frame.id = can_id(msg.type);

            // Enable CAN FD frames and Bit Rate Switching
            frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

            // Copy the payload into the CAN frame
            memcpy(frame.data, msg.payload, payload_size);
            if (payload_size < sizeof(msg.payload)) {
                memcpy(frame.data + payload_size, &msg.uuid, sizeof(msg.uuid));
                payload_size += sizeof(msg.uuid);
            }

            frame.dlc = can_bytes_to_dlc(payload_size);

            // For payload sizes > 8, DLC no longer maps 1:1 to the number of bytes in the payload
            // If the actual payload size is less than the number of bytes indicated by DLC, pad the rest with 0
            // const uint8_t dlc_bytes = can_dlc_to_bytes(frame.dlc);
            // memset(&frame.data[payload_size], 0, dlc_bytes - payload_size);

            ret = jerrycan_send_frame(&frame);
            if (ret == -EAGAIN || ret == -ENETUNREACH || ret == -ENETDOWN) {
                // The bus isn't taking frames (timed out, bus-off, or controller stopped). Drop the backlog rather
                // than waiting on each queued frame in turn, and go on to the received commands.
                const uint32_t dropped = k_msgq_num_used_get(&jerrycan_tx_msgq);
                k_msgq_purge(&jerrycan_tx_msgq);
                if (!tx_stalled) {
                    LOG_ERR("CAN transmit timed out; dropped %u queued frames", dropped);
                }
                tx_stalled = true;
                break;
            }
            if (ret != 0) {
                LOG_ERR("Failed to send CAN frame: %d", ret);
            } else if (tx_stalled) {
                LOG_INF("CAN transmit restored");
                tx_stalled = false;
            }
        }
    }

    // RX processing
    if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
        while (k_msgq_get(&jerrycan_rx_msgq, &frame, K_NO_WAIT) == 0) {
            msg.type = frame.id >> 5;
            uint8_t msg_len = jerrycan_msg_get_payload_size(msg.type);
            memcpy(msg.payload, frame.data, msg_len);
            if (msg_len < sizeof(msg.payload)) {
                memcpy(&msg.uuid, frame.data + msg_len, sizeof(msg.uuid));
            }

            sys_snode_t *snode;
            SYS_SLIST_FOR_EACH_NODE(&can_rx_callbacks_list, snode) {
                jerrycan_rx_callback_t *callback = CONTAINER_OF(snode, jerrycan_rx_callback_t, node);
                if (callback->filter_msg_type == msg.type) {
                    int error = callback->func(&msg);
                    if ((error != COMMAND_NOT_COMPLETE) && (error != SEND_NO_ACKNOWLEDGEMENT)) {
                        jerrycan_send_ack(msg.uuid, error);
                    }
                }
            }
        }
    }

    // Reset event states
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;

    return ret;
}

void jerrycan_send_ack(const uint8_t uuid, const int error_code) {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_RSP_ACK,
        .ack =
            {
                .error = error_code,
            },
        .uuid = uuid,
    };

    jerrycan_tx(&msg, K_NO_WAIT);
}

void jerrycan_register_rx_callback(jerrycan_rx_callback_t *callback) {
    sys_slist_append(&can_rx_callbacks_list, &callback->node);
}

static int jerrycan_init() {
    int ret;

    // Initialize the linked list that will hold the callbacks to be called on RX frame
    sys_slist_init(&can_rx_callbacks_list);

    // Read this device type and address from GPIOS. Without them the address would be arbitrary, so don't join
    // the bus at all.
    ret = get_can_node_id(&can_node_id);
    if (ret != 0) {
        return ret;
    }

    // Ensure the CAN device is ready
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    ret = can_set_mode(can_dev, CAN_MODE_FD);
    if (ret < 0) {
        LOG_ERR("Failed to set CAN bus to FD Mode");
        return ret;
    }

    // Set up the arbitration-phase bitrate for the CAN device to be 1 Mbps
    ret = can_set_bitrate(can_dev, 1000000);
    if (ret != 0) {
        LOG_ERR("Failed to set CAN bitrate: %d", ret);
        return ret;
    }

    // Set up the data-phase bitrate for the CAN device to be 5 Mbps
    ret = can_set_bitrate_data(can_dev, 5000000);
    if (ret != 0) {
        LOG_ERR("Failed to set CAN data bitrate: %d", ret);
        return ret;
    }

    // Start the CAN device
    ret = can_start(can_dev);
    if (ret) {
        LOG_ERR("Failed to start CAN device: %d", ret);
        return ret;
    }

    // Add the filter for messages address to this specific device
    const struct can_filter jerrycan_rx_filter = {
        .flags = 0,
        .id = can_node_id,
        .mask = NODE_ID_MASK,
    };

    ret = can_add_rx_filter_msgq(can_dev, &jerrycan_rx_msgq, &jerrycan_rx_filter);
    if (ret < 0) {
        LOG_ERR("Failed to add CAN filter: %d", ret);
        return ret;
    }

    // Add the filter for broadcast messages (NodeID = 0x1F)
    const struct can_filter jerrycan_broadcast_filter = {
        .flags = 0,
        .id = NODE_ID_MASK,
        .mask = NODE_ID_MASK,
    };

    ret = can_add_rx_filter_msgq(can_dev, &jerrycan_rx_msgq, &jerrycan_broadcast_filter);
    if (ret < 0) {
        LOG_ERR("Failed to add CAN filter: %d", ret);
        return ret;
    }

    return 0;
}

SYS_INIT(jerrycan_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
