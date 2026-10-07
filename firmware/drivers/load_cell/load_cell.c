#include "load_cell.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "nau7802.h"

#define DT_DRV_COMPAT ll_load_cell

LOG_MODULE_REGISTER(ll_load_cell, CONFIG_LL_LOAD_CELL_LOG_LEVEL);

typedef struct {
    nau7802_chs_t adc_channel;
    nau7802_vldo_t ldo_voltage;
    nau7802_gains_t gain_index;
    int gain;
    nau7802_crs_t conversion_rate;
    struct gpio_dt_spec drdy_pin;
} ll_load_cell_cfg_t;

typedef struct {
    const struct i2c_dt_spec i2c;
    struct k_work read_i2c_work;
    struct k_work tare_i2c_work;
    struct k_work drdy_failed_work;
    struct gpio_callback drdy_cb;
    struct k_timer *drdy_failed_timer;
    int32_t load;
    nau7802_crs_t conversion_rate;
} ll_load_cell_data_t;

static inline int k_work_error_handler(int error) {
    switch (error) {
        case 0: /* fall-through */
            /* work was already submitted to a queue */
        case 1: /* fall-through */
            /* work was not submitted and has been queued to queue */
        case 2:
            /* work was running and has been queued to the queue that was running it */
            break;
        case -EBUSY:
            LOG_ERR(
                "Failed to submit read_i2c_work to the system workqueue: work item is cancelling; or queue is "
                "draining; or queue is plugged");
            break;
        case -EINVAL:
            LOG_ERR(
                "Failed to submit read_i2c_work to the system workqueue: queue is null and the work item has never "
                "been run");
            break;
        case -ENODEV:
            LOG_ERR("Failed to submit read_i2c_work to the system workqueue: queue has not been started");
            break;
        default:
            LOG_ERR("Failed to submit read_i2c_work to the system workqueue: Unknown error - %d", error);
            error = -EINVAL;
            break;
    }

    return error;
}

/* ISR called on DRDY rising-edge interrupt to submit I2C work item to the system workqueue */
static void ll_load_cell_drdy_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    ll_load_cell_data_t *data = CONTAINER_OF(cb, ll_load_cell_data_t, drdy_cb);

    k_timer_start(data->drdy_failed_timer, K_SECONDS(1), K_NO_WAIT);

    /* Submit I2C work item to the system workqueue */
    int ret = k_work_submit(&data->read_i2c_work);
    k_work_error_handler(ret);
}

static void ll_load_cell_drdy_expired(struct k_timer *timer) {
    LOG_WRN("DRDY Stopped generating data ready.");

    const struct device *dev = k_timer_user_data_get(timer);
    ll_load_cell_data_t *data = dev->data;

    k_work_submit(&data->drdy_failed_work);
}

/* I2C work item handler function for performing conversion result read transaction */
static void ll_load_cell_read_i2c_work_handler(struct k_work *work) {
    ll_load_cell_data_t *data = CONTAINER_OF(work, ll_load_cell_data_t, read_i2c_work);

    int32_t sample;
    if (nau7802_read_conversion_result(&data->i2c, &sample) != 0) {
        LOG_ERR("Error reading conversion result");
        sample = INT24_MIN;
    }

    data->load = sample;
}

static void ll_load_cell_tare_i2c_work_handler(struct k_work *work) {
    ll_load_cell_data_t *data = CONTAINER_OF(work, ll_load_cell_data_t, tare_i2c_work);

    /* Set calibration mode to system */
    int ret = nau7802_set_calibration_mode(&data->i2c, OFFSET_CALIBRATION_SYSTEM);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error Setting NUA7802 calibration mode to system - %d", ret);
        return;
    }

    /* Perform calibration */
    ret = nau7802_calibrate(&data->i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error performing NUA7802 system calibration - %d", ret);
        return;
    }
}

/* I2C work item handler function for performing conversion result read transaction */
static void ll_load_cell_drdy_failed_work_handler(struct k_work *work) {
    ll_load_cell_data_t *data = CONTAINER_OF(work, ll_load_cell_data_t, drdy_failed_work);
    const struct i2c_dt_spec *i2c = &data->i2c;

    // Sometimes the calibration sequence causses the NAU7802 to go into a non-functional
    // state. But power-cycling the on-chip electronics and re-starting the ADC conversions,
    // the chip becomes functional again. This is always done to ensure the chip is in working
    // order after the calibration.
    nau7802_power_sequence(i2c);

    nau7802_set_conversion_rate(i2c, data->conversion_rate);

    nau7802_start_adc(i2c, false);
    k_sleep(K_MSEC(10));
    nau7802_start_adc(i2c, true);

    k_timer_start(data->drdy_failed_timer, K_SECONDS(1), K_NO_WAIT);
}

/* Returns the last read load value in millivolts as an int */
int16_t ll_load_cell_get_load_mv(const struct device *dev) {
    const ll_load_cell_cfg_t *cfg = dev->config;
    ll_load_cell_data_t *data = dev->data;

    return (int16_t)nau7802_counts_to_mv(data->load, cfg->ldo_voltage, cfg->gain);
}

/* Returns the last read load value in millivolts as a float */
float ll_load_cell_get_load_mv_float(const struct device *dev) {
    const ll_load_cell_cfg_t *cfg = dev->config;
    ll_load_cell_data_t *data = dev->data;

    return nau7802_counts_to_mv(data->load, cfg->ldo_voltage, cfg->gain);
}

/* Tares the load cell */
int ll_load_cell_tare(const struct device *dev) {
    ll_load_cell_data_t *data = dev->data;

    /* Submit I2C work item to the system workqueue */
    const int ret = k_work_submit(&data->tare_i2c_work);
    return k_work_error_handler(ret);
}

/* Initialize NAU7802 24-bit ADC */
static int ll_load_cell_nau7802_init(const struct device *dev) {
    const ll_load_cell_cfg_t *cfg = dev->config;
    ll_load_cell_data_t *data = dev->data;
    const struct i2c_dt_spec *i2c = &data->i2c;
    int ret;

    /* Power Up NUA7802 */
    ret = nau7802_power_up(i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error powering up NUA7802 - %d", ret);
        return ret;
    }

    /* Configure NUA7802 LDO to output the specified voltage */
    ret = nau7802_set_ldo_voltage(i2c, cfg->ldo_voltage);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error setting NUA7802 LDO voltage - %d", ret);
        return ret;
    }

    /* Enable NUA7802 LDO */
    ret = nau7802_enable_ldo(i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error enabling NUA7802 LDO - %d", ret);
        return ret;
    }

    /* Configure NUA7802 Gain */
    ret = nau7802_set_gain(i2c, cfg->gain_index);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error setting NUA7802 Gain - %d", ret);
        return ret;
    }

    /* Select NUA7802 external clock source */
    ret = nau7802_set_clock_source(i2c, NAU7802_OSCS_EXTERNAL);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error setting NUA7802 clock source - %d", ret);
        return ret;
    }

    /* Disable NUA7802 bandwidth chopper */
    ret = nau7802_disable_bw_chopper(i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error disabling NUA7802 bandwidth chopper - %d", ret);
        return ret;
    }

    /* Configure NUA7802 ADC Channel */
    ret = nau7802_set_adc_channel(i2c, cfg->adc_channel);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error setting NUA7802 ADC Channel - %d", ret);
        return ret;
    }

    /* Configure NUA7802 conversion rate */
    ret = nau7802_set_conversion_rate(i2c, cfg->conversion_rate);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error setting NUA7802 conversion rate - %d", ret);
        return ret;
    }

    /* Disable NUA7802 I2C Weak Pullup (strong pullup disabled by default)*/
    ret = nau7802_disable_weak_pullup(i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error disabling NUA7802 I2C weak pullup - %d", ret);
        return ret;
    }

    /*********************/
    /* Calibrate NUA7802 */
    /*********************/

    /* Internal Calibration */

    /* Set calibration mode to internal */
    ret = nau7802_set_calibration_mode(i2c, OFFSET_CALIBRATION_INTERNAL);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error Setting NUA7802 calibration mode to internal - %d", ret);
        return ret;
    }

    /* Perform calibration */
    ret = nau7802_calibrate(i2c);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error performing NUA7802 internal calibration - %d", ret);
        return ret;
    }

    /* Start NUA7802 ADC Conversion */
    ret = nau7802_start_adc(i2c, true);
    if (ret != 0) {
        LOG_ERR("Failed to initialize NUA7802: Error starting NUA7802 ADC conversion - %d", ret);
        return ret;
    }

    return ret;
}

/* Initialize Load Cell Driver */
static int ll_load_cell_init(const struct device *dev) {
    const ll_load_cell_cfg_t *cfg = dev->config;
    ll_load_cell_data_t *data = dev->data;
    int ret;
    LOG_INF("Initializing load cell driver...");

    /* Confirm that I2C bus is ready */
    if (!device_is_ready(data->i2c.bus)) {
        LOG_ERR("Failed to initialize Load Cell: I2C bus device is not ready");
        return -ENODEV;
    }

    /* Configure DRDY pin as input */
    ret = gpio_pin_configure_dt(&cfg->drdy_pin, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to initialize Load Cell: Error configuring DRDY pin - %d", ret);
        return ret;
    }

    /* Add DRDY interrupt callback to DRDY pin */
    gpio_init_callback(&data->drdy_cb, ll_load_cell_drdy_isr, BIT(cfg->drdy_pin.pin));
    ret = gpio_add_callback_dt(&cfg->drdy_pin, &data->drdy_cb);
    if (ret != 0) {
        LOG_ERR("Failed to initialize Load Cell: Error adding callback to DRDY pin - %d", ret);
        return ret;
    }

    /* Configure rising-edge interrupt DRDY pin */
    ret = gpio_pin_interrupt_configure_dt(&cfg->drdy_pin, GPIO_INT_ENABLE | GPIO_INT_EDGE_RISING);
    if (ret != 0) {
        LOG_ERR("Failed to initialize Load Cell: Error to configuring interrupt for DRDY pin - %d", ret);
        return ret;
    }

    k_work_init(&data->read_i2c_work, ll_load_cell_read_i2c_work_handler);
    k_work_init(&data->tare_i2c_work, ll_load_cell_tare_i2c_work_handler);
    k_work_init(&data->drdy_failed_work, ll_load_cell_drdy_failed_work_handler);

    /* Initialize NAU7802 */
    ret = ll_load_cell_nau7802_init(dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize Load Cell: Error initializing NAU7802 - %d", ret);
        return ret;
    }

    k_timer_user_data_set(data->drdy_failed_timer, (void *)dev);
    k_timer_start(data->drdy_failed_timer, K_SECONDS(1), K_NO_WAIT);

    LOG_INF("Load cell driver initialized successfully");

    return ret;
}

/* Maps the given enum index to the corresponding enum value */
/* clang-format off */
#define CONVERSION_RATE_FROM_ENUM_IDX(__idx__)           \
    ((__idx__ == 0) ? NAU7802_CONVERSION_RATE_10SPS      \
    : ((__idx__ == 1) ? NAU7802_CONVERSION_RATE_20SPS    \
    : ((__idx__ == 2) ? NAU7802_CONVERSION_RATE_40SPS    \
    : ((__idx__ == 3) ? NAU7802_CONVERSION_RATE_80SPS    \
    : ((__idx__ == 4) ? NAU7802_CONVERSION_RATE_320SPS   \
                    : -1)))))
/* clang-format on */

#define LOAD_CELL_INST(idx)                                                                                       \
    K_TIMER_DEFINE(drdy_timer_##idx, ll_load_cell_drdy_expired, NULL);                                            \
    BUILD_ASSERT(DT_REG_ADDR(DT_INST_PARENT(idx)) != NAU7802_I2CADDR,                                             \
                 "`reg` property does not match NAU7802 I2C Address (0x2A)");                                     \
    static const ll_load_cell_cfg_t load_cell_cfg_##idx = {                                                       \
        .adc_channel = (nau7802_chs_t)DT_INST_ENUM_IDX(idx, adc_channel),                                         \
        .ldo_voltage = (nau7802_vldo_t)DT_INST_ENUM_IDX(idx, ldo_voltage),                                        \
        .gain_index = (nau7802_gains_t)DT_INST_ENUM_IDX(idx, gain),                                               \
        .gain = DT_INST_PROP(idx, gain),                                                                          \
        .conversion_rate = CONVERSION_RATE_FROM_ENUM_IDX(DT_INST_ENUM_IDX(idx, conversion_rate)),                 \
        .drdy_pin = GPIO_DT_SPEC_INST_GET(idx, drdy_gpios),                                                       \
    };                                                                                                            \
    static ll_load_cell_data_t load_cell_data_##idx = {                                                           \
        .i2c = I2C_DT_SPEC_INST_GET(idx),                                                                         \
        .drdy_cb =                                                                                                \
            {                                                                                                     \
                .handler = ll_load_cell_drdy_isr,                                                                 \
                .pin_mask = 1 << DT_INST_GPIO_PIN(idx, drdy_gpios),                                               \
            },                                                                                                    \
        .drdy_failed_timer = &drdy_timer_##idx,                                                                   \
        .load = 0.0,                                                                                              \
        .conversion_rate = CONVERSION_RATE_FROM_ENUM_IDX(DT_INST_ENUM_IDX(idx, conversion_rate)),                 \
    };                                                                                                            \
                                                                                                                  \
    DEVICE_DT_INST_DEFINE(idx, ll_load_cell_init, NULL, &load_cell_data_##idx, &load_cell_cfg_##idx, POST_KERNEL, \
                          LL_LOAD_CELL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(LOAD_CELL_INST)
