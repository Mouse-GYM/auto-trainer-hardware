#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "motor_callbacks.h"

// Timer auto-reload for a 20 ms servo frame at the 0.5 us count every servo timer uses (`st,prescaler = 84`).
// A CCR above this never compares inside the period, so it is also the ceiling on any pulse the driver can
// produce — see `SERVO_MAX_PULSE_DURATION_US`.
#define SERVO_TIMER_PERIOD_COUNTS 40000U

typedef ll_motor_events_t ll_servo_events_t;
typedef ll_motor_event_callback_t ll_servo_event_callback_t;
typedef ll_motor_cb_t ll_servo_cb_t;

int ll_queue_servo_positions(const struct device *dev, uint32_t *positions, size_t len, k_timeout_t timeout);
int ll_servo_register_callback(const struct device *dev, ll_servo_cb_t *cb);
int ll_servo_enable(const struct device *dev, bool enable);
int ll_servo_dma_stop(const struct device *dev);
