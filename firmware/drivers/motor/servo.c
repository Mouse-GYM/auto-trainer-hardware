#include "servo.h"

#include <stm32_ll_tim.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/dma/stm32_dma.h>
#include <zephyr/logging/log.h>

#include "motor_common.h"

#define DT_DRV_COMPAT ll_servo

LOG_MODULE_DECLARE(ll_motor, CONFIG_LL_MOTOR_LOG_LEVEL);

static const uint32_t channel_to_ccr_map[] = {LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_BASEADDR_CCR2,
                                              LL_TIM_DMABURST_BASEADDR_CCR3, LL_TIM_DMABURST_BASEADDR_CCR4,
                                              LL_TIM_DMABURST_BASEADDR_CCR5, LL_TIM_DMABURST_BASEADDR_CCR6};

int ll_queue_servo_positions(const struct device *dev, uint32_t *positions, size_t len, k_timeout_t timeout) {
    return ll_motor_queue_data(dev, positions, len, timeout);
}

int ll_servo_register_callback(const struct device *dev, ll_servo_cb_t *cb) {
    return ll_motor_register_callback(dev, cb);
}

int ll_servo_enable(const struct device *dev, bool enable) {
    const ll_motor_cfg_t *cfg = dev->config;

    if (enable) {
        LL_TIM_EnableCounter(cfg->timer);
    } else {
        LL_TIM_DisableCounter(cfg->timer);
    }

    return 0;
}

static int ll_servo_init(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    ll_motor_data_t *data = dev->data;

    // Perform common motor initialization stuff
    int ret = ll_motor_init(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize stepper: %d", ret);
        return ret;
    }

    // Set the Auto Reload Register to be a 20ms period (assuming 2MHz clock)
    LL_TIM_SetAutoReload(cfg->timer, SERVO_TIMER_PERIOD_COUNTS);

    LL_TIM_OC_InitTypeDef output_chan_init;
    LL_TIM_OC_StructInit(&output_chan_init);
    output_chan_init.OCMode = LL_TIM_OCMODE_PWM1;
    output_chan_init.CompareValue = data->position;

    if (LL_TIM_OC_Init(cfg->timer, cfg->channel, &output_chan_init) != SUCCESS) {
        LOG_ERR("Failed to initialize output channel");
        return -EIO;
    }

    LL_TIM_OC_EnablePreload(cfg->timer, cfg->channel);

    // Enable the channel and timer
    LL_TIM_CC_EnableChannel(cfg->timer, cfg->channel);

    return 0;
}

int ll_servo_dma_stop(const struct device *dev) {
    const ll_motor_cfg_t *cfg = dev->config;
    const ll_motor_data_t *data = dev->data;
    return dma_stop(cfg->dma_dev, data->dma_channel);
}

#define SERVO_INST(idx)                                                                             \
    PINCTRL_DT_INST_DEFINE(idx);                                                                    \
                                                                                                    \
    K_MSGQ_DEFINE(servo_msgq##idx, sizeof(ll_motorq_msg_t), 16, 4);                                 \
                                                                                                    \
    static const ll_motor_cfg_t servo_cfg##idx = {                                                  \
        .timer = TIM(idx),                                                                          \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                                \
        .clk = DT_INST_CLK(idx, timer),                                                             \
        .prescaler = DT_PROP(TIMER(idx), st_prescaler),                                             \
        .channel = channel_to_ll_map[DT_INST_PROP(idx, pwm_channel) - 1],                           \
        .dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(idx, tx)),                                          \
        .msgq = &servo_msgq##idx,                                                                   \
        .timer_dma_reg = channel_to_ccr_map[DT_INST_PROP(idx, pwm_channel) - 1],                    \
        .stop_on_dma_complete = false,                                                              \
        .limit_switch_pin = GPIO_DT_SPEC_INST_GET_OR(idx, limit_switch_gpios, {0}),                 \
        .motor_id = idx,                                                                            \
    };                                                                                              \
                                                                                                    \
    static ll_motor_data_t servo_data##idx = {                                                      \
        .position = 1000,                                                                           \
        .dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),                                 \
        .dma_cfg =                                                                                  \
            {                                                                                       \
                .block_count = 2,                                                                   \
                .dma_slot = STM32_DMA_SLOT(idx, tx, slot),                                          \
                .channel_direction = STM32_DMA_CONFIG_DIRECTION(STM32_DMA_MEMORY_TO_PERIPH),        \
                .source_data_size = 4,                                                              \
                .dest_data_size = 4,                                                                \
                .source_burst_length = 1,                                                           \
                .dest_burst_length = 1,                                                             \
                .channel_priority = STM32_DMA_CONFIG_PRIORITY(STM32_DMA_CHANNEL_CONFIG(idx, tx)),   \
                .dma_callback = ll_motor_dma_tx_callback,                                           \
            },                                                                                      \
        .limit_switch_cb =                                                                          \
            {                                                                                       \
                .node = {NULL},                                                                     \
                .handler = NULL,                                                                    \
                .pin_mask = 0,                                                                      \
            },                                                                                      \
        .motor_device = NULL,                                                                       \
    };                                                                                              \
                                                                                                    \
    DEVICE_DT_INST_DEFINE(idx, ll_servo_init, NULL, &servo_data##idx, &servo_cfg##idx, POST_KERNEL, \
                          CONFIG_LL_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SERVO_INST)
