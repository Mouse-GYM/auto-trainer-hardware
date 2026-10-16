#pragma once
#include <zephyr/device.h>

/* Returns the last read load value in millivolts as an int */
int16_t ll_load_cell_get_load_mv(const struct device *dev);

/* Returns the last read load value in millivolts as a float */
float ll_load_cell_get_load_mv_float(const struct device *dev);

/* Called from the system workqueue with the calibration result: 0, or the -errno it failed with. */
typedef void (*ll_load_cell_tare_cb_t)(int result, void *user_data);

/**
 * Tare the load cell. The calibration runs on the system workqueue; `cb`, which may be NULL, gets its result.
 *
 * @retval 0 if the tare was queued.
 * @retval -EBUSY if a tare is already pending; its callback is unchanged.
 * @retval -errno if the work couldn't be submitted.
 */
int ll_load_cell_tare(const struct device *dev, ll_load_cell_tare_cb_t cb, void *user_data);
