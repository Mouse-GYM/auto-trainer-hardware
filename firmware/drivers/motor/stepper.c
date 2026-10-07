#include "stepper.h"

#include <stm32_ll_tim.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/dma/stm32_dma.h>
#include <zephyr/logging/log.h>

#include "motor_common.h"

#define DT_DRV_COMPAT ll_stepper

LOG_MODULE_DECLARE(ll_motor, CONFIG_LL_MOTOR_LOG_LEVEL);

int ll_queue_stepper_positions(const struct device *dev, uint32_t *positions, size_t len, k_timeout_t timeout) {
    const ll_motor_data_t *data = dev->data;
    if (!atomic_get(&data->output_enabled)) {
        return -ESHUTDOWN;
    }

    return ll_motor_queue_data(dev, positions, len, timeout);
}

int ll_stepper_register_callback(const struct device *dev, ll_stepper_cb_t *cb) {
    return ll_motor_register_callback(dev, cb);
}

int ll_stepper_set_direction(const struct device *dev, ll_stepper_dir_t dir) {
    const ll_motor_cfg_t *cfg = dev->config;
    int ret;

    if (cfg->dir_pin.port == NULL) {
        return -ENOTSUP;
    }

    // FIXME: Not sure on the polarity for the DIR pin, but can be changed in the device tree if needed.
    // Should we make this a setting? Maybe punt that to higher level logic?
    switch (dir) {
        case LL_STEPPER_DIR_FORWARD:
            ret = gpio_pin_set(cfg->dir_pin.port, cfg->dir_pin.pin, 0);
            break;
        case LL_STEPPER_DIR_BACKWARD:
            ret = gpio_pin_set(cfg->dir_pin.port, cfg->dir_pin.pin, 1);
            break;
        default:
            return -EINVAL;
    }

    return ret;
}

static int ll_stepper_init(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    ll_motor_data_t *data = dev->data;
    int ret;

    // Configure the DIR pin
    if (cfg->dir_pin.port != NULL) {
        ret = gpio_pin_configure_dt(&cfg->dir_pin, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure DIR pin: %d", ret);
            return ret;
        }
    }

    // Perform common motor initialization stuff
    ret = ll_motor_init(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize stepper: %d", ret);
        return ret;
    }

    // Set up the output compare and pulse generator
    LL_TIM_OC_InitTypeDef output_chan_init;
    LL_TIM_OC_StructInit(&output_chan_init);
    output_chan_init.OCMode = LL_TIM_OCMODE_PULSE_ON_COMPARE;
    output_chan_init.CompareValue = 0;

    if (LL_TIM_OC_Init(cfg->timer, cfg->channel, &output_chan_init) != SUCCESS) {
        LOG_ERR("Failed to initialize output channel");
        return -EIO;
    }

    LL_TIM_OC_SetPulseWidth(cfg->timer, 170);  // 1us - Pulse width is based on the kernel clock...

    // Enable the channel
    LL_TIM_CC_EnableChannel(cfg->timer, cfg->channel);
    atomic_set(&data->output_enabled, true);

    return 0;
}

int ll_stepper_disable(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    ll_motor_data_t *data = dev->data;

    k_spinlock_key_t key = k_spin_lock(&data->output_lock);
    atomic_clear(&data->output_enabled);
    LL_TIM_CC_DisableChannel(cfg->timer, cfg->channel);
    LL_TIM_DisableCounter(cfg->timer);
    k_spin_unlock(&data->output_lock, key);

    int ret = ll_stepper_dma_stop(dev);
    k_msgq_purge(cfg->msgq);

    return ret;
}

int ll_stepper_enable(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    ll_motor_data_t *data = dev->data;

    k_spinlock_key_t key = k_spin_lock(&data->output_lock);
    atomic_set(&data->output_enabled, true);
    LL_TIM_CC_EnableChannel(cfg->timer, cfg->channel);
    k_spin_unlock(&data->output_lock, key);

    return 0;
}

bool ll_stepper_is_enabled(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;

    uint32_t ret = LL_TIM_CC_IsEnabledChannel(cfg->timer, cfg->channel);

    return ret == 1;
}

int ll_stepper_dma_stop(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    const ll_motor_data_t *data = dev->data;
    return dma_stop(cfg->dma_dev, data->dma_channel);
}

int ll_stepper_get_limit_switch_state(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;

    int ret = gpio_pin_get_dt(&cfg->limit_switch_pin);

    if (ret < 0) {
        LOG_ERR("Failed to get limit switch state: %d", ret);
        return 0;  // Default to limit switch low on read fail?
    }

    return ret;
}

#define STEPPER_INST(idx)                                                                                 \
    BUILD_ASSERT(IS_TIM_PULSEONCOMPARE_INSTANCE(TIM(idx)),                                                \
                 "Stepper driver requires a timer with pulse-on-compare mode");                           \
    BUILD_ASSERT(IS_TIM_PULSEONCOMPARE_CHANNEL(CHANNEL_NUM_TO_LL_MAP(DT_INST_PROP(idx, pwm_channel))),    \
                 "Stepper driver requires a timer channel that supports pulse-on-compare mode");          \
                                                                                                          \
    PINCTRL_DT_INST_DEFINE(idx);                                                                          \
                                                                                                          \
    K_MSGQ_DEFINE(stepper_msgq##idx, sizeof(ll_motorq_msg_t), 16, 4);                                     \
                                                                                                          \
    static const ll_motor_cfg_t stepper_cfg##idx = {                                                      \
        .timer = TIM(idx),                                                                                \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                                      \
        .clk = DT_INST_CLK(idx, timer),                                                                   \
        .prescaler = DT_PROP(TIMER(idx), st_prescaler),                                                   \
        .channel = channel_to_ll_map[DT_INST_PROP(idx, pwm_channel) - 1],                                 \
        .dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(idx, tx)),                                                \
        .msgq = &stepper_msgq##idx,                                                                       \
        .timer_dma_reg = LL_TIM_DMABURST_BASEADDR_ARR,                                                    \
        .stop_on_dma_complete = true,                                                                     \
        .limit_switch_pin = GPIO_DT_SPEC_INST_GET_OR(idx, limit_switch_gpios, {0}),                       \
        .dir_pin = GPIO_DT_SPEC_INST_GET(idx, dir_gpios),                                                 \
        .stepper_driver_device = COND_CODE_1(DT_NODE_HAS_PROP(DT_INST(idx, ll_stepper), driver_dev),      \
                                             DEVICE_DT_GET(DT_INST_PROP(idx, driver_dev)), NULL),         \
        .motor_id = idx,                                                                                  \
    };                                                                                                    \
                                                                                                          \
    static ll_motor_data_t stepper_data##idx = {                                                          \
        .position = 0,                                                                                    \
        .dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),                                       \
        .dma_cfg =                                                                                        \
            {                                                                                             \
                .block_count = 2,                                                                         \
                .dma_slot = STM32_DMA_SLOT(idx, tx, slot),                                                \
                .channel_direction = STM32_DMA_CONFIG_DIRECTION(STM32_DMA_MEMORY_TO_PERIPH),              \
                .source_data_size = 4,                                                                    \
                .dest_data_size = 4,                                                                      \
                .source_burst_length = 1,                                                                 \
                .dest_burst_length = 1,                                                                   \
                .channel_priority = STM32_DMA_CONFIG_PRIORITY(STM32_DMA_CHANNEL_CONFIG(idx, tx)),         \
                .dma_callback = ll_motor_dma_tx_callback,                                                 \
            },                                                                                            \
        .limit_switch_cb =                                                                                \
            {                                                                                             \
                .node = {NULL},                                                                           \
                .handler = NULL,                                                                          \
                .pin_mask = 0,                                                                            \
            },                                                                                            \
        .motor_device = NULL,                                                                             \
    };                                                                                                    \
                                                                                                          \
    DEVICE_DT_INST_DEFINE(idx, ll_stepper_init, NULL, &stepper_data##idx, &stepper_cfg##idx, POST_KERNEL, \
                          CONFIG_LL_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(STEPPER_INST)
