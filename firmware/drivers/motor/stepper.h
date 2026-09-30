#pragma once

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "motor_callbacks.h"
#include "motor_common.h"

typedef ll_motor_events_t ll_stepper_events_t;
typedef ll_motor_event_callback_t ll_stepper_event_callback_t;
typedef ll_motor_cb_t ll_stepper_cb_t;

typedef enum {
    LL_STEPPER_DIR_FORWARD,
    LL_STEPPER_DIR_BACKWARD,
} ll_stepper_dir_t;

int ll_queue_stepper_positions(const struct device *dev, uint32_t *positions, size_t len, k_timeout_t timeout);
int ll_stepper_register_callback(const struct device *dev, ll_stepper_cb_t *cb);
int ll_stepper_set_direction(const struct device *dev, ll_stepper_dir_t dir);
int ll_stepper_enable(const struct device *dev);
int ll_stepper_disable(const struct device *dev);
/*
 * Stop a move partway: stop the DMA and the STEP output as `ll_stepper_disable` does, stop the timer the way a
 * completed move leaves it, and drop any blocks still queued so the next move starts from an empty queue. No
 * DMA_QUEUE_EMPTY event is raised. Re-enable with `ll_stepper_enable` before the next move.
 */
int ll_stepper_abort(const struct device *dev);
bool ll_stepper_is_enabled(const struct device *dev);
int ll_stepper_dma_stop(const struct device *dev);
int ll_stepper_get_limit_switch_state(const struct device *dev);
