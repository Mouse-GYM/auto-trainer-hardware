#include <app_version.h>
#include <zephyr/logging/log.h>

#include "jerrycan.h"

LOG_MODULE_REGISTER(app);

int main() {
    LOG_INF("Autotrainer Magnet Module v%s", APP_VERSION_STRING);

    while (true) {
        jerrycan_run(K_FOREVER);
    }
}
