/**
 * @file uptime.c
 * @brief JerryCAN Uptime Message Handling
 *
 */

#include <zephyr/device.h>

#include <zephyr/logging/log.h>

#include "jerrycan.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

#define DT_DRV_COMPAT ll_uptime

/* Number of enabled uptime instances found in the device tree */
// #define UPTIME_COUNT DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)


static int jerrycan_send_uptime(int64_t msecs, uuid_t uuid, bool is_boot) {
    jerrycan_msg_t response = {
        .type = JERRYCAN_CMD_UPTIME,
        .uuid = uuid,
        .uptime = {.msecs = msecs, .is_boot = is_boot}
    };
    return jerrycan_tx(&response, K_NO_WAIT);
}


static int jerrycan_uptime_write_handler(const jerrycan_msg_t *msg) {
    int ret = jerrycan_send_uptime(k_uptime_get(), msg->uuid, false);
    if (ret == 0) {
        return SEND_NO_ACKNOWLEDGEMENT;
    }
    return ret;
}

static jerrycan_rx_callback_t uptime_callback = {
    .filter_msg_type = JERRYCAN_CMD_UPTIME,
    .func = jerrycan_uptime_write_handler,
};


static void jerrycan_uptime_timer_tx() {
    jerrycan_send_uptime(k_uptime_get(), 0, false);
}

K_TIMER_DEFINE(jerrycan_uptime_timer, jerrycan_uptime_timer_tx, NULL);


static int jerrycan_uptime_init() {

    jerrycan_send_uptime(k_uptime_get(), 0, true);

    jerrycan_register_rx_callback(&uptime_callback);

    k_timer_start(&jerrycan_uptime_timer, K_MSEC(500), K_SECONDS(1));

    return 0;
}

SYS_INIT(jerrycan_uptime_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
