#include "nau7802.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nau7802_chip, CONFIG_LL_LOAD_CELL_LOG_LEVEL);

/* Bounds on the status-bit polling loops, so a chip that ACKs but never sets PUR or clears CALS can't hang boot or
 * the system workqueue. Calibration runs over several conversions, so it gets enough time for the slowest rate. */
#define NAU7802_POWER_UP_TIMEOUT_MS 500
#define NAU7802_CALIBRATION_TIMEOUT_MS 2000
#define NAU7802_POLL_INTERVAL_MS 1

/* Read the specified NAU7802 Register into the given NAU7802 Register Structure */
static int nau7802_read(const struct i2c_dt_spec *i2c, nau7802_reg_address_t reg, nau7802_reg_t *value) {
    return i2c_reg_read_byte_dt(i2c, reg, &value->byte);
}

/* Write the given NAU7802 Register Structure into the specified NAU7802 Register */
static int nau7802_write(const struct i2c_dt_spec *i2c, nau7802_reg_address_t reg, nau7802_reg_t value) {
    return i2c_reg_write_byte_dt(i2c, reg, value.byte);
}

/* Mapping of nau7802_vldo_t to associated voltage in millivolts as a float */
static const float nau7802_vldo_to_mv_float[] = {
    [NAU7802_VLDO_4V5] = 4500.0f, [NAU7802_VLDO_4V2] = 4200.0f, [NAU7802_VLDO_3V9] = 3900.0f,
    [NAU7802_VLDO_3V6] = 3600.0f, [NAU7802_VLDO_3V3] = 3300.0f, [NAU7802_VLDO_3V0] = 3000.0f,
    [NAU7802_VLDO_2V7] = 2700.0f, [NAU7802_VLDO_2V4] = 2400.0f,
};

/* Convert from signed 24-bit ADC counts to millivolts as a float */
float nau7802_counts_to_mv(int32_t const raw_counts, const uint32_t vldo_index, const int32_t gain) {
    float mv = (float)raw_counts / (float)INT24_MAX;
    mv *= nau7802_vldo_to_mv_float[vldo_index];
    mv /= gain;

    return mv;
}

int nau7802_power_sequence(const struct i2c_dt_spec *i2c) {
    nau7802_pu_ctrl_reg_t pu_ctrl;

    int ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    pu_ctrl.pud = 0;  // Power off Digital Circuitry
    pu_ctrl.pua = 0;  // Power off Analog Circuitry

    ret = nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    k_sleep(K_MSEC(10));

    pu_ctrl.pud = 1;  // Power On Digital Circuitry
    pu_ctrl.pua = 1;  // Power On Analog Circuitry

    ret = nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Wait until power up */
    const int64_t deadline = k_uptime_get() + NAU7802_POWER_UP_TIMEOUT_MS;
    do {
        ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
        if (ret != 0) {
            return ret;
        }
        if (pu_ctrl.pur) {
            return 0;
        }
        k_msleep(NAU7802_POLL_INTERVAL_MS);
    } while (k_uptime_get() < deadline);

    LOG_ERR("Timed out waiting for power-up ready");
    return -ETIMEDOUT;
}

int nau7802_power_up(const struct i2c_dt_spec *i2c) {
    nau7802_pu_ctrl_reg_t pu_ctrl;
    int ret;

    /*******************/
    /* Reset Registers */
    /*******************/

    /* Read current value of PU_CTRL register */
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Set reset registers bit */
    pu_ctrl.rr = 1;

    /* Write updated value to PU_CTRL register */
    ret = nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /*************************/
    /* Clear Reset Registers */
    /*************************/

    /* Read current value of PU_CTRL register */
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Clear reset registers bit */
    pu_ctrl.rr = 0;

    /* Write updated value to PU_CTRL register */
    ret = nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    return nau7802_power_sequence(i2c);
}

int nau7802_set_adc_channel(const struct i2c_dt_spec *i2c, nau7802_chs_t channel) {
    nau7802_ctrl2_reg_t ctrl2;
    int ret;

    /* Read current value of the CTRL2 register */
    ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
    if (ret != 0) {
        return ret;
    }

    /* Set the channel select bit */
    ctrl2.chs = channel;

    /* Write the updated value to the CTRL2 register */
    return nau7802_write(i2c, NAU7802_CTRL2, (nau7802_reg_t)ctrl2);
}

int nau7802_set_ldo_voltage(const struct i2c_dt_spec *i2c, nau7802_vldo_t ldo_voltage) {
    nau7802_ctrl1_reg_t ctrl1;
    int ret;

    /* Read current value of the CTRL1 register */
    ret = nau7802_read(i2c, NAU7802_CTRL1, (nau7802_reg_t *)&ctrl1);
    if (ret != 0) {
        return ret;
    }

    /* Set the channel select bit */
    ctrl1.vldo = ldo_voltage;

    /* Write the updated value to the CTRL1 register */
    return nau7802_write(i2c, NAU7802_CTRL1, (nau7802_reg_t)ctrl1);
}

int nau7802_enable_ldo(const struct i2c_dt_spec *i2c) {
    nau7802_pu_ctrl_reg_t pu_ctrl;
    int ret;

    /* Read current value of PU_CTRL register */
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Set AVDD source select bit to use internal LDO */
    pu_ctrl.avdds = 1;

    /* Write updated value to PU_CTRL register */
    return nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
}

int nau7802_set_clock_source(const struct i2c_dt_spec *i2c, nau7802_oscs_t clock_source) {
    nau7802_pu_ctrl_reg_t pu_ctrl;
    int ret;

    /* Read current value of PU_CTRL register */
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Set system clock source select bit */
    pu_ctrl.oscs = clock_source;

    /* Write updated value to PU_CTRL register */
    return nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
}

int nau7802_set_gain(const struct i2c_dt_spec *i2c, nau7802_gains_t gain) {
    nau7802_ctrl1_reg_t ctrl1;
    int ret;

    /* Read current value of the CTRL1 register */
    ret = nau7802_read(i2c, NAU7802_CTRL1, (nau7802_reg_t *)&ctrl1);
    if (ret != 0) {
        return ret;
    }

    /* Set the gain selection bits */
    ctrl1.gains = gain;

    /* Write the updated value to the CTRL1 register */
    return nau7802_write(i2c, NAU7802_CTRL1, (nau7802_reg_t)ctrl1);
}

int nau7802_disable_bw_chopper(const struct i2c_dt_spec *i2c) {
    nau7802_reg_t reg;
    int ret;

    /* Read current value of the ADC_REG register */
    ret = nau7802_read(i2c, NAU7802_ADC_REG, &reg);
    if (ret != 0) {
        return ret;
    }

    /* Disable CLK_CHP by setting both bits of REG_CHPS */
    reg.adc_reg.reg_chps = 0b11;

    /* Write the updated value to the ADC_REG register */
    ret = nau7802_write(i2c, NAU7802_ADC_REG, reg);
    if (ret != 0) {
        return ret;
    }

    /* Read current value of the PGA_REG register */
    ret = nau7802_read(i2c, NAU7802_PGA_REG, &reg);
    if (ret != 0) {
        return ret;
    }

    /* Set PGA Chopper Disabled bit */
    reg.pga_reg.pgachpdis = 1;

    /* Write the updated value to the PGA_REG register */
    return nau7802_write(i2c, NAU7802_PGA_REG, reg);
}

int nau7802_set_conversion_rate(const struct i2c_dt_spec *i2c, nau7802_crs_t conversion_rate) {
    nau7802_ctrl2_reg_t ctrl2;
    int ret;

    /* Read current value of the CTRL2 register */
    ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
    if (ret != 0) {
        return ret;
    }

    /* Set the conversion rate bits */
    ctrl2.crs = conversion_rate;

    /* Write the updated value to the CTRL2 register */
    return nau7802_write(i2c, NAU7802_CTRL2, (nau7802_reg_t)ctrl2);
}

int nau7802_disable_weak_pullup(const struct i2c_dt_spec *i2c) {
    nau7802_i2c_ctrl_reg_t i2c_ctrl;
    int ret;

    /* Read current value of the I2C_CTRL register */
    ret = nau7802_read(i2c, NAU7802_I2C_CTRL, (nau7802_reg_t *)&i2c_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Set the weak pull-up disable bit */
    i2c_ctrl.wpd = 1;

    /* Write the updated value to the I2C_CTRL register */
    return nau7802_write(i2c, NAU7802_I2C_CTRL, (nau7802_reg_t)i2c_ctrl);
}

int nau7802_start_adc(const struct i2c_dt_spec *i2c, bool enable) {
    nau7802_pu_ctrl_reg_t pu_ctrl;
    int ret;

    /* Read current value of PU_CTRL register */
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) {
        return ret;
    }

    /* Set cycle start bit */
    pu_ctrl.cs = enable ? 1 : 0;

    /* Write updated value to PU_CTRL register */
    return nau7802_write(i2c, NAU7802_PU_CTRL, (nau7802_reg_t)pu_ctrl);
}

int nau7802_read_conversion_result(const struct i2c_dt_spec *i2c, int32_t *result) {
    nau7802_adc_out_reg_t adc_out;
    int ret;

    /* Perform a burst read of the ADC_OUT[2:0] registers into value */
    ret = i2c_burst_read_dt(i2c, NAU7802_ADC_B2, adc_out.bytes, sizeof(nau7802_adc_out_reg_t));
    if (ret != 0) {
        *result = INT24_MIN;
    } else {
        // Reads big endian data out. MSB goes in bytes[0], LSB goes in bytes[2]
        uint32_t raw24 =
            (uint32_t)adc_out.bytes[0] << 16 | (uint32_t)adc_out.bytes[1] << 8 | (uint32_t)adc_out.bytes[2];

        // Sign-extend 24-bit to 32-bit
        if (raw24 & 0x800000) {
            raw24 |= 0xFF000000;
        }
        *result = (int32_t)raw24;
    }

    return ret;
}

int nau7802_set_calibration_mode(const struct i2c_dt_spec *i2c, nau7802_calmod_t mode) {
    nau7802_ctrl2_reg_t ctrl2;
    int ret;

    /* Read current value of the CTRL2 register */
    ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
    if (ret != 0) {
        return ret;
    }

    /* Set the calibration mode bits */
    ctrl2.calmod = mode;

    /* Write the updated value to the CTRL2 register */
    return nau7802_write(i2c, NAU7802_CTRL2, (nau7802_reg_t)ctrl2);
}

int nau7802_calibrate(const struct i2c_dt_spec *i2c) {
    nau7802_ctrl2_reg_t ctrl2;
    int ret;

    nau7802_start_adc(i2c, false);  // Disable while calibration is being performed

    /*********************/
    /* Start calibration */
    /*********************/

    /* Read current value of CTRL2 register */
    ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
    if (ret != 0) {
        return ret;
    };

    /* Set start calibration bit */
    ctrl2.cals = 1;

    /* Write updated value to CTRL2 register */
    ret = nau7802_write(i2c, NAU7802_CTRL2, (nau7802_reg_t)ctrl2);
    if (ret != 0) {
        return ret;
    };

    /**********************************/
    /* Wait for calibration to finish */
    /**********************************/

    const int64_t deadline = k_uptime_get() + NAU7802_CALIBRATION_TIMEOUT_MS;
    while (true) {
        /* Read current value of CTRL2 register */
        ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
        if (ret != 0) {
            return ret;
        }
        /* Exit loop when CALS bit goes low */
        if (!ctrl2.cals) {
            break;
        }
        if (k_uptime_get() >= deadline) {
            LOG_ERR("Timed out waiting for calibration to finish");
            ret = -ETIMEDOUT;
            break;
        }
        k_msleep(NAU7802_POLL_INTERVAL_MS);
    }

    /* If calibration error occurred return -EIO */
    if (ret == 0 && ctrl2.cal_err) {
        ret = -EIO;
    }

    nau7802_start_adc(i2c, true);

    return ret;
}
