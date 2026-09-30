#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "motor_callbacks.h"

typedef ll_motor_events_t ll_servo_events_t;
typedef ll_motor_event_callback_t ll_servo_event_callback_t;
typedef ll_motor_cb_t ll_servo_cb_t;

int ll_queue_servo_positions(const struct device *dev, uint32_t *positions, size_t len, k_timeout_t timeout);
int ll_servo_register_callback(const struct device *dev, ll_servo_cb_t *cb);
int ll_servo_enable(const struct device *dev, bool enable);
int ll_servo_dma_stop(const struct device *dev);
/*
 * Stop a move partway: stop the DMA and drop any blocks still queued, so the next move starts from an empty queue.
 * The PWM keeps running with the last pulse width, so the servo holds where it is. No DMA_QUEUE_EMPTY event is
 * raised.
 */
int ll_servo_abort(const struct device *dev);
