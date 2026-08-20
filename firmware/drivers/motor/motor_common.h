#pragma once

#include <stdint.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>

#include "motor_callbacks.h"

typedef struct {
    uint32_t *buf;
    size_t buf_size;
} ll_motorq_msg_t;

// Helpers for getting the timer instance from the DT into STM32 LL HAL friendly format
#define TIMER(idx) DT_INST_PARENT(idx)
#define TIM(idx) ((TIM_TypeDef *)DT_REG_ADDR(TIMER(idx)))

#define DT_INST_CLK(index, inst) {.bus = DT_CLOCKS_CELL(TIMER(index), bus), .enr = DT_CLOCKS_CELL(TIMER(index), bits)}

static const uint32_t channel_to_ll_map[] = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3,
                                             LL_TIM_CHANNEL_CH4, LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6};

// This macro is odd, but how the LL HAL defines the channels is a bit odd...
/* clang-format off */
#define CHANNEL_NUM_TO_LL_MAP(__CHANNEL__)  \
    ((__CHANNEL__ == 1)   ? TIM_CHANNEL_1   \
    : ((__CHANNEL__ == 2) ? TIM_CHANNEL_2   \
    : ((__CHANNEL__ == 3) ? TIM_CHANNEL_3   \
    : ((__CHANNEL__ == 4) ? TIM_CHANNEL_4   \
    : ((__CHANNEL__ == 5) ? TIM_CHANNEL_5   \
    : ((__CHANNEL__ == 6) ? TIM_CHANNEL_6   \
                         : -1))))))
/* clang-format on */

typedef struct {
    int32_t position;
    struct dma_config dma_cfg;
    int dma_channel;
    atomic_t output_enabled;
    struct k_spinlock output_lock;
    sys_slist_t callbacks;
    const struct device *motor_device;  // Horrible kludge but necessary to overcome deficiencies with GPIO driver.
    struct gpio_callback limit_switch_cb;
} ll_motor_data_t;

typedef struct {
    TIM_TypeDef *timer;
    const struct pinctrl_dev_config *pcfg;
    struct stm32_pclken clk;
    uint32_t prescaler;
    uint32_t channel;
    const struct device *dma_dev;
    struct k_msgq *msgq;
    uint32_t timer_dma_reg;
    bool stop_on_dma_complete;
    struct gpio_dt_spec limit_switch_pin;
    struct gpio_dt_spec dir_pin;  // Used for the stepper driver
    const struct device *stepper_driver_device;
    uint8_t motor_id;
} ll_motor_cfg_t;

static inline int ll_motor_timer_enable_clock(const struct stm32_pclken *timer_clk) {
    // Enable the timer clock
    const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
    if (!device_is_ready(clk)) {
        return -ENODEV;
    }

    return clock_control_on(clk, (clock_control_subsys_t *)timer_clk);
}

int ll_common_start_dma(struct dma_config *dma_cfg);
void ll_motor_dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status);
int ll_motor_queue_get_num_used(const struct device *dev);
int ll_motor_queue_data(const struct device *dev, uint32_t *buf, size_t len, k_timeout_t timeout);
int ll_motor_start_dma(const struct device *dev);
int ll_motor_init(const struct device *dev);
int ll_motor_register_callback(const struct device *dev, ll_motor_cb_t *cb);
uint8_t ll_motor_get_id(const struct device *dev);
