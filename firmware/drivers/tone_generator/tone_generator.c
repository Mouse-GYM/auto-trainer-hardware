#include "tone_generator.h"

#include <stm32_ll_dac.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/dma/stm32_dma.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT ll_tone_generator

LOG_MODULE_REGISTER(ll_tone_generator, CONFIG_LL_TONE_GENERATOR_LOG_LEVEL);

/* 512-Sample 12-bit Sine Wave */
static const uint32_t sine_wave[512] = {
    0x800, 0x819, 0x832, 0x84B, 0x864, 0x87D, 0x896, 0x8AF, 0x8C8, 0x8E1, 0x8FA, 0x913, 0x92C, 0x945, 0x95D, 0x976,
    0x98F, 0x9A7, 0x9C0, 0x9D8, 0x9F1, 0xA09, 0xA21, 0xA3A, 0xA52, 0xA6A, 0xA82, 0xA99, 0xAB1, 0xAC9, 0xAE0, 0xAF8,
    0xB0F, 0xB26, 0xB3D, 0xB54, 0xB6B, 0xB81, 0xB98, 0xBAE, 0xBC4, 0xBDB, 0xBF0, 0xC06, 0xC1C, 0xC31, 0xC47, 0xC5C,
    0xC71, 0xC86, 0xC9A, 0xCAF, 0xCC3, 0xCD7, 0xCEB, 0xCFF, 0xD12, 0xD25, 0xD39, 0xD4B, 0xD5E, 0xD71, 0xD83, 0xD95,
    0xDA7, 0xDB9, 0xDCA, 0xDDB, 0xDEC, 0xDFD, 0xE0E, 0xE1E, 0xE2E, 0xE3E, 0xE4D, 0xE5D, 0xE6C, 0xE7B, 0xE89, 0xE97,
    0xEA6, 0xEB3, 0xEC1, 0xECE, 0xEDB, 0xEE8, 0xEF5, 0xF01, 0xF0D, 0xF18, 0xF24, 0xF2F, 0xF3A, 0xF45, 0xF4F, 0xF59,
    0xF63, 0xF6C, 0xF75, 0xF7E, 0xF87, 0xF8F, 0xF97, 0xF9F, 0xFA6, 0xFAE, 0xFB4, 0xFBB, 0xFC1, 0xFC7, 0xFCD, 0xFD2,
    0xFD7, 0xFDC, 0xFE0, 0xFE5, 0xFE8, 0xFEC, 0xFEF, 0xFF2, 0xFF5, 0xFF7, 0xFF9, 0xFFB, 0xFFC, 0xFFD, 0xFFE, 0xFFE,
    0xFFF, 0xFFE, 0xFFE, 0xFFD, 0xFFC, 0xFFB, 0xFF9, 0xFF7, 0xFF5, 0xFF2, 0xFEF, 0xFEC, 0xFE8, 0xFE5, 0xFE0, 0xFDC,
    0xFD7, 0xFD2, 0xFCD, 0xFC7, 0xFC1, 0xFBB, 0xFB4, 0xFAE, 0xFA6, 0xF9F, 0xF97, 0xF8F, 0xF87, 0xF7E, 0xF75, 0xF6C,
    0xF63, 0xF59, 0xF4F, 0xF45, 0xF3A, 0xF2F, 0xF24, 0xF18, 0xF0D, 0xF01, 0xEF5, 0xEE8, 0xEDB, 0xECE, 0xEC1, 0xEB3,
    0xEA6, 0xE97, 0xE89, 0xE7B, 0xE6C, 0xE5D, 0xE4D, 0xE3E, 0xE2E, 0xE1E, 0xE0E, 0xDFD, 0xDEC, 0xDDB, 0xDCA, 0xDB9,
    0xDA7, 0xD95, 0xD83, 0xD71, 0xD5E, 0xD4B, 0xD39, 0xD25, 0xD12, 0xCFF, 0xCEB, 0xCD7, 0xCC3, 0xCAF, 0xC9A, 0xC86,
    0xC71, 0xC5C, 0xC47, 0xC31, 0xC1C, 0xC06, 0xBF0, 0xBDB, 0xBC4, 0xBAE, 0xB98, 0xB81, 0xB6B, 0xB54, 0xB3D, 0xB26,
    0xB0F, 0xAF8, 0xAE0, 0xAC9, 0xAB1, 0xA99, 0xA82, 0xA6A, 0xA52, 0xA3A, 0xA21, 0xA09, 0x9F1, 0x9D8, 0x9C0, 0x9A7,
    0x98F, 0x976, 0x95D, 0x945, 0x92C, 0x913, 0x8FA, 0x8E1, 0x8C8, 0x8AF, 0x896, 0x87D, 0x864, 0x84B, 0x832, 0x819,
    0x800, 0x7E6, 0x7CD, 0x7B4, 0x79B, 0x782, 0x769, 0x750, 0x737, 0x71E, 0x705, 0x6EC, 0x6D3, 0x6BA, 0x6A2, 0x689,
    0x670, 0x658, 0x63F, 0x627, 0x60E, 0x5F6, 0x5DE, 0x5C5, 0x5AD, 0x595, 0x57D, 0x566, 0x54E, 0x536, 0x51F, 0x507,
    0x4F0, 0x4D9, 0x4C2, 0x4AB, 0x494, 0x47E, 0x467, 0x451, 0x43B, 0x424, 0x40F, 0x3F9, 0x3E3, 0x3CE, 0x3B8, 0x3A3,
    0x38E, 0x379, 0x365, 0x350, 0x33C, 0x328, 0x314, 0x300, 0x2ED, 0x2DA, 0x2C6, 0x2B4, 0x2A1, 0x28E, 0x27C, 0x26A,
    0x258, 0x246, 0x235, 0x224, 0x213, 0x202, 0x1F1, 0x1E1, 0x1D1, 0x1C1, 0x1B2, 0x1A2, 0x193, 0x184, 0x176, 0x168,
    0x159, 0x14C, 0x13E, 0x131, 0x124, 0x117, 0x10A, 0x0FE, 0x0F2, 0x0E7, 0x0DB, 0x0D0, 0x0C5, 0x0BA, 0x0B0, 0x0A6,
    0x09C, 0x093, 0x08A, 0x081, 0x078, 0x070, 0x068, 0x060, 0x059, 0x051, 0x04B, 0x044, 0x03E, 0x038, 0x032, 0x02D,
    0x028, 0x023, 0x01F, 0x01A, 0x017, 0x013, 0x010, 0x00D, 0x00A, 0x008, 0x006, 0x004, 0x003, 0x002, 0x001, 0x001,
    0x001, 0x001, 0x001, 0x002, 0x003, 0x004, 0x006, 0x008, 0x00A, 0x00D, 0x010, 0x013, 0x017, 0x01A, 0x01F, 0x023,
    0x028, 0x02D, 0x032, 0x038, 0x03E, 0x044, 0x04B, 0x051, 0x059, 0x060, 0x068, 0x070, 0x078, 0x081, 0x08A, 0x093,
    0x09C, 0x0A6, 0x0B0, 0x0BA, 0x0C5, 0x0D0, 0x0DB, 0x0E7, 0x0F2, 0x0FE, 0x10A, 0x117, 0x124, 0x131, 0x13E, 0x14C,
    0x159, 0x168, 0x176, 0x184, 0x193, 0x1A2, 0x1B2, 0x1C1, 0x1D1, 0x1E1, 0x1F1, 0x202, 0x213, 0x224, 0x235, 0x246,
    0x258, 0x26A, 0x27C, 0x28E, 0x2A1, 0x2B4, 0x2C6, 0x2DA, 0x2ED, 0x300, 0x314, 0x328, 0x33C, 0x350, 0x365, 0x379,
    0x38E, 0x3A3, 0x3B8, 0x3CE, 0x3E3, 0x3F9, 0x40F, 0x424, 0x43B, 0x451, 0x467, 0x47E, 0x494, 0x4AB, 0x4C2, 0x4D9,
    0x4F0, 0x507, 0x51F, 0x536, 0x54E, 0x566, 0x57D, 0x595, 0x5AD, 0x5C5, 0x5DE, 0x5F6, 0x60E, 0x627, 0x63F, 0x658,
    0x670, 0x689, 0x6A2, 0x6BA, 0x6D3, 0x6EC, 0x705, 0x71E, 0x737, 0x750, 0x769, 0x782, 0x79B, 0x7B4, 0x7CD, 0x7E6};

/* Macro to map Channel Number to LL DAC Channel Constant*/
#define DAC_CHANNEL_NUM_TO_LL_MAP(ch) ((ch) == 1 ? LL_DAC_CHANNEL_1 : LL_DAC_CHANNEL_2)

#define GET_LL_DAC_TIM_TRIGGER_SOURCE(__DACx__, __TIMx__)                                 \
    (((__DACx__) == DAC1 && (__TIMx__) == TIM2)             ? LL_DAC_TRIG_EXT_TIM2_TRGO   \
     : ((__DACx__) == DAC1 && (__TIMx__) == TIM3)           ? LL_DAC_TRIG_EXT_TIM3_TRGO   \
     : ((__DACx__) == DAC1 && (__TIMx__) == TIM4)           ? LL_DAC_TRIG_EXT_TIM4_TRGO   \
     : ((__DACx__) == DAC1 && (__TIMx__) == TIM6)           ? LL_DAC_TRIG_EXT_TIM6_TRGO   \
     : ((__DACx__) == DAC1 && (__TIMx__) == TIM7)           ? LL_DAC_TRIG_EXT_TIM7_TRGO   \
     : ((__DACx__) == DAC1 && (__TIMx__) == TIM15)          ? LL_DAC_TRIG_EXT_TIM15_TRGO  \
     : ((__DACx__) == DAC1 && (__TIMx__) == (void *)HRTIM1) ? LL_DAC_TRIG_EXT_HRTIM_TRGO1 \
     : ((__DACx__) == DAC2 && (__TIMx__) == TIM2)           ? LL_DAC_TRIG_EXT_TIM2_TRGO   \
     : ((__DACx__) == DAC2 && (__TIMx__) == TIM4)           ? LL_DAC_TRIG_EXT_TIM4_TRGO   \
     : ((__DACx__) == DAC2 && (__TIMx__) == TIM6)           ? LL_DAC_TRIG_EXT_TIM6_TRGO   \
     : ((__DACx__) == DAC2 && (__TIMx__) == TIM15)          ? LL_DAC_TRIG_EXT_TIM15_TRGO  \
     : ((__DACx__) == DAC2 && (__TIMx__) == (void *)HRTIM1) ? LL_DAC_TRIG_EXT_HRTIM_TRGO2 \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM1)           ? LL_DAC_TRIG_EXT_TIM1_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM8)           ? LL_DAC_TRIG_EXT_TIM8_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM2)           ? LL_DAC_TRIG_EXT_TIM2_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM3)           ? LL_DAC_TRIG_EXT_TIM3_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM4)           ? LL_DAC_TRIG_EXT_TIM4_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == TIM6)           ? LL_DAC_TRIG_EXT_TIM6_TRGO   \
     : ((__DACx__) == DAC3 && (__TIMx__) == (void *)HRTIM1) ? LL_DAC_TRIG_EXT_HRTIM_TRGO3 \
     : ((__DACx__) == DAC4 && (__TIMx__) == (void *)HRTIM1) ? LL_DAC_TRIG_EXT_HRTIM_TRGO1 \
                                                            : -1)

/* Macro to acquire 12-bit right-aligned data holding register for the given DAC instance and channel */
#define GET_DACx_DHR12Rx_REGISTER(__DACx__, __CHANNELx__) \
    ((__CHANNELx__) == LL_DAC_CHANNEL_1 ? &(((DAC_TypeDef *)__DACx__)->DHR12R1) : &(((DAC_TypeDef *)__DACx__)->DHR12R2))

typedef struct {
    TIM_TypeDef *sample_rate_timer;
    DAC_TypeDef *dac;
    struct stm32_pclken tim_clk;
    struct stm32_pclken dac_clk;
    const struct device *dma_dev;
    uint32_t dac_channel;
    uint32_t dac_resolution;
    uint32_t dma_channel;
    const struct gpio_dt_spec enable_pin;
} ll_tone_generator_cfg_t;

typedef struct {
    bool initialized;
    bool enabled;
    unsigned int frequency_hz;
    struct dma_config dma_cfg;
    struct k_timer duration_timer;
} ll_tone_generator_data_t;

/* Callback to handle DMA transfer error */
static void dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel, int status) {
    if (status < 0) {
        LOG_ERR("DMA encountered error and entered interrupt with status: %d", status);
        ll_tone_generator_abort_tone((const struct device *)user_data);
    }
}

/*  Kernel Timer Expirey Callback function */
static void duration_expiry_cb(struct k_timer *timer) {
    LOG_DBG("Duration Timer Expired");
    ll_tone_generator_abort_tone((const struct device *)timer->user_data);
}

/* Enable the provided clock */
static inline int ll_tone_generator_enable_clock(const struct stm32_pclken *clk) {
    const struct device *rcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
    if (!device_is_ready(rcc)) {
        return -ENODEV;
    }
    return clock_control_on(rcc, (clock_control_subsys_t *)clk);
}

/* DMA Initialization */
static int ll_tone_generator_dma_init(const struct device *dev) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    ll_tone_generator_data_t *data = dev->data;
    int ret;

    if (!cfg->dma_dev) {
        LOG_ERR("dma_dev is NULL");
        return -ENODEV;
    }

    ret = dma_config(cfg->dma_dev, cfg->dma_channel, &data->dma_cfg);
    if (ret != 0) {
        LOG_ERR("Failed to configure DMA: %d", ret);
        return ret;
    }

    return ret;
}

/* TIM Initialization */
static int ll_tone_generator_tim_init(const struct device *dev) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    int ret;

    ret = ll_tone_generator_enable_clock(&cfg->tim_clk);
    if (ret != 0) {
        LOG_ERR("Failed to enable sample rate timer clock: %d", ret);
        return ret;
    }

    /* Declare TIM initialization structure */
    LL_TIM_InitTypeDef tim_init = {0};

    /* Initialize the TIM initializationn structure */
    LL_TIM_StructInit(&tim_init);

    /*
    Populate the TIM initialization structure.
    Sticking with smallest prescaler to allow for maximum resolution
    */
    tim_init.Prescaler = 0x0;
    tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
    tim_init.Autoreload = 0x0001;
    tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    /* Enable TIM Clock */
    ret = LL_TIM_Init(cfg->sample_rate_timer, &tim_init);
    if (ret != 0) {
        LOG_ERR("Failed to initialize TIM: %d", ret);
        return ret;
    }

    /* Enable Auto-Reload Preload (Buffer the auto-reload register) */
    LL_TIM_EnableARRPreload(cfg->sample_rate_timer);

    /* Set trigger to output on UPDATE event */
    LL_TIM_SetTriggerOutput(cfg->sample_rate_timer, LL_TIM_TRGO_UPDATE);

    /* Disable Master/Slave mode */
    LL_TIM_DisableMasterSlaveMode(cfg->sample_rate_timer);

    return ret;
}

/* DAC Initialization */
static int ll_tone_generator_dac_init(const struct device *dev) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    int ret;

    /* Enable DAC Clock */
    ret = ll_tone_generator_enable_clock(&cfg->dac_clk);
    if (ret != 0) {
        LOG_ERR("Failed to enable DAC clock: %d", ret);
        return ret;
    }

    /* Declare DAC initialization structure */
    LL_DAC_InitTypeDef dac_init = {0};

    /* Initialize the DAC initializationn structure */
    LL_DAC_StructInit(&dac_init);

    /* Populate the DAC initialization structure. */
    dac_init.TriggerSource = GET_LL_DAC_TIM_TRIGGER_SOURCE(
        cfg->dac, cfg->sample_rate_timer);  // Determine trigger source from chosen DAC and timer

    dac_init.TriggerSource2 =
        LL_DAC_TRIG_SOFTWARE;  // Set secondary trigger source to software to allow for clearing of output
    dac_init.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;  // No auto generation of waveform occuring
    dac_init.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;     // Buffer output to prevent glitches in output waveform
    dac_init.OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO;  // Connect DAC output to GPIO
    dac_init.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;         // Output mode is NORMAL

    ret = LL_DAC_Init(cfg->dac, cfg->dac_channel, &dac_init);
    if (ret != 0) {
        LOG_ERR("Failed to Initialize DAC: %d", ret);
        return ret;
    }

    return ret;
}

/* Tone Generator Initialization Function */
static int ll_tone_generator_init(const struct device *dev) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    ll_tone_generator_data_t *data = dev->data;
    int ret;

    LOG_INF("Initializing Tone Generator...");

    ret = gpio_pin_configure_dt(&cfg->enable_pin, GPIO_OUTPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure enable pin - %d", ret);
    }

    ret = gpio_pin_set_dt(&cfg->enable_pin, 0);
    if (ret != 0) {
        LOG_ERR("Failed to clear enable pin - %d", ret);
    }

    /* Initialize the duration timer */
    k_timer_init(&data->duration_timer, duration_expiry_cb, NULL);

    /* Place tone generator in dma_cfg user_data so it can be used to handle errors appropriately */
    data->dma_cfg.user_data = (void *)dev;

    /* Place tone generator in k_timer user_data so it can be passed to ll_tone_generator_abort_tone() */
    data->duration_timer.user_data = (void *)dev;

    /* Initialize the sample rate timer TIM peripheral */
    ret = ll_tone_generator_tim_init(dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize sample rate timer: %d", ret);
        return ret;
    }

    /* Initialize the DAC peripheral */
    ret = ll_tone_generator_dac_init(dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize DAC: %d", ret);
        return ret;
    }

    /* Intialize the DMA peripheral */
    ret = ll_tone_generator_dma_init(dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize DMA: %d", ret);
        return ret;
    }

    LOG_INF("Tone Generator Initialized!");
    data->initialized = true;
    return ret;
}

/**
 * Generates a tone with the given frequency and duration on the provided tone generator.
 */
int ll_tone_generator_play_tone(const struct device *dev, unsigned int frequency_hz, unsigned int duration_ms) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    ll_tone_generator_data_t *data = dev->data;
    int ret;

    /* Ensure that the tone generator has been initialized */
    if (!data->initialized) {
        LOG_ERR("Tone Generator must be initialized before play tone can be called");
        return -1;
    }

    /* If a tone is actively being played, stop it before continuing */
    if (data->enabled) {
        ret = ll_tone_generator_abort_tone(dev);
        if (ret != 0) {
            LOG_ERR("Error playing tone: Failed to abort active tone - %d", ret);
            return ret;
        }
    }

    /* Ensure that the specified frequency lies within the commandable range */
    if (frequency_hz < TONE_GENERATOR_MIN_FREQUENCY || frequency_hz > TONE_GENERATOR_MAX_FREQUENCY) {
        LOG_ERR("Invalid frequency <%d> - must reside within the commandable range [%d, %d]", frequency_hz,
                TONE_GENERATOR_MIN_FREQUENCY, TONE_GENERATOR_MAX_FREQUENCY);
        // Frequency 0 would divide by zero in the reload calculation below, which traps and halts the module.
        return -EINVAL;
    }

    /* Enable the audio amplifier */
    ret = gpio_pin_set_dt(&cfg->enable_pin, 1);
    if (ret != 0) {
        LOG_ERR("Failed to set enable pin - %d", ret);
    }

    /* Start the DMA transfer */
    ret = dma_start(cfg->dma_dev, cfg->dma_channel);
    if (ret != 0) {
        LOG_ERR("Failed to start DMA: %d", ret);
        return ret;
    }

    /* Calculate the period corresponding to the given frequency, and write to sample rate timer */
    LL_TIM_SetAutoReload(
        cfg->sample_rate_timer,
        __LL_TIM_CALC_ARR(SystemCoreClock, 0x0, frequency_hz * (sizeof(sine_wave) / sizeof(sine_wave[0]))));

    /* Enable counter */
    LL_TIM_EnableCounter(cfg->sample_rate_timer);

    /* Enable DAC Trigger */
    LL_DAC_EnableTrigger(cfg->dac, cfg->dac_channel);

    /* Enable DAC channel DMA request */
    LL_DAC_EnableDMAReq(cfg->dac, cfg->dac_channel);

    /* Enable DAC */
    LL_DAC_Enable(cfg->dac, cfg->dac_channel);

    /* Start the duration timer to expire after the given duration in milliseconds in one-shot mode */
    k_timer_start(&data->duration_timer, K_MSEC(duration_ms), K_NO_WAIT);

    data->frequency_hz = frequency_hz;

    /* Set the enabled flag to true */
    data->enabled = true;

    LOG_INF("Playing tone at %uHz for %ums", frequency_hz, duration_ms);

    return ret;
}

/**
 * Generates a tone with the given frequency and duration on the provided
 * tone generator. Blocks for the duration of the tone.
 */
int ll_tone_generator_play_tone_blocking(const struct device *dev, unsigned int frequency_hz,
                                         unsigned int duration_ms) {
    int ret;

    /* Start playing tone */
    ret = ll_tone_generator_play_tone(dev, frequency_hz, duration_ms);

    /* Sleep for duration of tone */
    k_msleep(duration_ms);

    return ret;
}

/* Halts the tone generation proccess on the given tone generator */
int ll_tone_generator_abort_tone(const struct device *dev) {
    const ll_tone_generator_cfg_t *cfg = dev->config;
    ll_tone_generator_data_t *data = dev->data;
    int ret;

    if (!data->initialized) {
        LOG_ERR("Tone Generator must be initialized before abort tone can be called");
    }

    /* Ensure that the duration time has been stopped - does nothing if already expired */
    k_timer_stop(&data->duration_timer);

    /* Stop the DMA transfer */
    ret = dma_stop(cfg->dma_dev, cfg->dma_channel);
    if (ret != 0) {
        LOG_ERR("Failed to stop DMA: %d", ret);
        return ret;
    }

    /* Disable the sample rate timer */
    LL_TIM_DisableCounter(cfg->sample_rate_timer);

    /* Disable the DAC */
    LL_DAC_Disable(cfg->dac, cfg->dac_channel);

    /* Disable the audio amplifier */
    ret = gpio_pin_set_dt(&cfg->enable_pin, 0);
    if (ret != 0) {
        LOG_ERR("Failed to set clear pin - %d", ret);
    }

    data->frequency_hz = 0;
    data->enabled = false;

    return ret;
}

/* Returns the amount time in ms remaining for the current tone */
uint32_t ll_tone_generator_get_time_remaining(const struct device *dev) {
    ll_tone_generator_data_t *data = dev->data;

    return k_timer_remaining_get(&data->duration_timer);
}

/* Returns the frequency in Hz of the current tone */
uint32_t ll_tone_generator_get_frequency(const struct device *dev) {
    ll_tone_generator_data_t *data = dev->data;

    return data->frequency_hz;
}

#define TONE_GENERATOR_INST(idx)                                                                                    \
    static struct dma_block_config blk_cfg_##idx = {                                                                \
        .block_size = sizeof(sine_wave),                                                                            \
        .source_address = (uint32_t)sine_wave,                                                                      \
        .dest_address = (uint32_t)GET_DACx_DHR12Rx_REGISTER(                                                        \
            DT_REG_ADDR(DT_INST_PROP(idx, dac)), DAC_CHANNEL_NUM_TO_LL_MAP(DT_INST_PROP(idx, dac_channel))),        \
        .source_addr_adj = DMA_ADDR_ADJ_INCREMENT,                                                                  \
        .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,                                                                    \
        .source_reload_en = true,                                                                                   \
        .dest_reload_en = true,                                                                                     \
    };                                                                                                              \
                                                                                                                    \
    static const ll_tone_generator_cfg_t tone_generator_cfg_##idx = {                                               \
        .sample_rate_timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PROP(idx, sample_rate_timer)),                      \
        .tim_clk = (struct stm32_pclken){.bus = DT_CLOCKS_CELL(DT_INST_PROP(idx, sample_rate_timer), bus),          \
                                         .enr = DT_CLOCKS_CELL(DT_INST_PROP(idx, sample_rate_timer), bits)},        \
        .dac = (DAC_TypeDef *)DT_REG_ADDR(DT_INST_PROP(idx, dac)),                                                  \
        .dac_clk = (struct stm32_pclken){.bus = DT_CLOCKS_CELL(DT_INST_PROP(idx, dac), bus),                        \
                                         .enr = DT_CLOCKS_CELL(DT_INST_PROP(idx, dac), bits)},                      \
        .dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(idx)),                                                           \
        .dac_channel = DAC_CHANNEL_NUM_TO_LL_MAP(DT_INST_PROP(idx, dac_channel)),                                   \
        .dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),                                                 \
        .enable_pin = GPIO_DT_SPEC_INST_GET(idx, enable_gpios),                                                     \
    };                                                                                                              \
                                                                                                                    \
    static ll_tone_generator_data_t tone_generator_data_##idx = {                                                   \
        .initialized = false,                                                                                       \
        .enabled = false,                                                                                           \
        .dma_cfg =                                                                                                  \
            {                                                                                                       \
                .block_count = 1,                                                                                   \
                .dma_slot = STM32_DMA_SLOT(idx, tx, slot),                                                          \
                .channel_direction = STM32_DMA_CONFIG_DIRECTION(STM32_DMA_MEMORY_TO_PERIPH),                        \
                .source_data_size = 4,                                                                              \
                .dest_data_size = 4,                                                                                \
                .source_burst_length = 1,                                                                           \
                .dest_burst_length = 1,                                                                             \
                .channel_priority = STM32_DMA_CONFIG_PRIORITY(STM32_DMA_CHANNEL_CONFIG(idx, tx)),                   \
                .head_block = &blk_cfg_##idx,                                                                       \
                .complete_callback_en = false,                                                                      \
                .error_callback_dis = true,                                                                         \
                .dma_callback = dma_callback,                                                                       \
                .user_data = NULL,                                                                                  \
                .cyclic = true,                                                                                     \
            },                                                                                                      \
        .frequency_hz = 0,                                                                                          \
    };                                                                                                              \
                                                                                                                    \
    DEVICE_DT_INST_DEFINE(idx, ll_tone_generator_init, NULL, &tone_generator_data_##idx, &tone_generator_cfg_##idx, \
                          POST_KERNEL, CONFIG_LL_TONE_GENERATOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TONE_GENERATOR_INST)
