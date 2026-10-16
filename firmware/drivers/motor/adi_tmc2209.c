#include "adi_tmc2209.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util_macro.h>

#ifndef IS_BIT_SET
#define IS_BIT_SET(value, bit) ((((value) >> (bit)) & (0x1)) != 0)
#endif

#include "adi_tmc2209_types.h"

#define DT_DRV_COMPAT adi_tmc2209

LOG_MODULE_REGISTER(adi_tmc2209, CONFIG_ADI_TMC2209_DEBUG_LEVEL);

typedef struct adi_tmc2209_config {
    const struct device *uart_dev;
    uint8_t address;  // 0, 1, 2, or 3
} adi_tmc2209_config_t;

typedef struct adi_tmc2209_data {
    uint8_t writes_sent;  // Write datagrams sent, modulo 256, to compare against IFCNT.
    // Last values written to the registers checked after init.
    adi_tmc2209_reg_t written_gconf;
    adi_tmc2209_reg_t written_chopconf;
    adi_tmc2209_reg_t written_ihold_irun;
    adi_tmc2209_init_check_t init_check;
    uint8_t enabled_toff;  // Last nonzero CHOPCONF.toff written; `adi_tmc2209_output_enable` restores it.
} adi_tmc2209_data_t;

#define SYNC_NIBBLE 0x05U
#define UART_READ_TRIES 10000U
#define SEND_DELAY 2  // ((N div 2) * 2 + 1) * 8 clock cycles
#define HOST_ADDR 0xFFU

#define SLEEP_DELAY() k_sleep(K_MSEC(10))

/* Every driver shares the one single-wire UART, and the motor monitor polls DRV_STATUS from its own thread while
 * the CAN thread may be writing CHOPCONF. This keeps datagrams from interleaving, and read-modify-write sequences
 * hold it across all their transfers. k_mutex is recursive for its owner, so those can call the locked
 * read/write. Never take it from an ISR. */
static K_MUTEX_DEFINE(adi_tmc2209_bus_lock);

/**
 * @returns crc of `data[0 : size]` (Polynomial is x^8 + x^2 + x + 1.)
 */
static uint8_t calculate_crc(const uint8_t *const data, const size_t size) {
    uint8_t crc = 0;

    for (size_t i = 0; i < size; i++) {
        uint8_t byte = data[i];

        for (size_t j = 0; j < CHAR_BIT * sizeof(uint8_t); j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            byte >>= 1;
        }
    }

    return crc;
}

/**
 * Write the crc of `data[0 : size - 1]` to `data[size - 1]`.
 *
 * @param data must have at least `size` bytes.
 * @param size must be at least 2.
 *
 * @retval 0 on success
 * @retval -EINVAL if `size` is less than 2.
 */
static int store_crc(uint8_t *data, const size_t size) {
    if (size < 2) {
        return -EINVAL;
    }

    data[size - 1] = calculate_crc(data, size - 1);
    return 0;
}

/**
 * Check that the crc of `data[0 : size - 1]` matches `data[size - 1]`.
 *
 * @retval `true` on success.
 * @retval `false` on failure.
 */
static bool check_crc(const uint8_t *data, const size_t size) {
    if (size < 2) {
        return false;
    }

    return calculate_crc(data, size - 1) == data[size - 1];
}

#define NO_BYTE_WAITING (-1)
static int read_single_line_uart(const struct device *dev, uint8_t *buf, const size_t size) {
    const adi_tmc2209_config_t *config = dev->config;

    for (size_t i = 0; i < size; i++) {
        uint8_t byte;
        int ret;
        int j = 0;
        do {
            if (j > UART_READ_TRIES) {
                LOG_DBG("Timed out reading byte %d", i);
                return -ETIMEDOUT;
            }
            ret = uart_poll_in(config->uart_dev, &byte);
            j++;
        } while (ret == NO_BYTE_WAITING);

        if (ret != 0) {
            LOG_DBG("Failed to read byte %d: %d", i, ret);
            return ret;
        }

        buf[i] = byte;
    }

    return 0;
}

static int write_single_line_uart_and_flush_read(const struct device *dev, const uint8_t *buf, const size_t size) {
    /* Write the datagram to the UART. The driver will echo the datagram back to us so we go ahead
     * and read it to flush it from the buffer.
     */
    const adi_tmc2209_config_t *config = dev->config;
    uint8_t rx_buf[WR_PACKET_LENGTH];
    if (size > sizeof(rx_buf)) {
        LOG_ERR("Datagram too large (%d) for rx buffer (%d).", size, sizeof(rx_buf));
        return -EINVAL;
    }

    for (size_t i = 0; i < size; i++) {
        uart_poll_out(config->uart_dev, buf[i]);
    }

    return read_single_line_uart(dev, rx_buf, size);
}

/**
 * Write `data` to `reg_address` on this device. Data is 4 bytes long!
 *
 * @retval 0 on success,
 * @retval -errno on error.
 */
static int adi_tmc2209_write(const struct device *dev, const uint8_t reg_address, adi_tmc2209_reg_t data) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_data_t *dev_data = dev->data;

    k_mutex_lock(&adi_tmc2209_bus_lock, K_FOREVER);

    // Record the value for the post-init check before it's byte-swapped below.
    switch (reg_address) {
        case REG_GCONF:
            dev_data->written_gconf = data;
            break;
        case REG_CHOPCONF:
            dev_data->written_chopconf = data;
            if (data.chopconf.toff != 0) {
                dev_data->enabled_toff = data.chopconf.toff;
            }
            break;
        case REG_IHOLD_IRUN:
            dev_data->written_ihold_irun = data;
            break;
        default:
            break;
    }
    dev_data->writes_sent++;

    // Convert from host to network byte order
    data.as_uint32 = sys_cpu_to_be32(data.as_uint32);

    write_datagram_t datagram = {{
        .sync = SYNC_NIBBLE,
        .reserved = 0,
        .address = config->address,
        .reg_address = reg_address,
        .rw = WRITE,
        .data = data,
        .crc = 0,  // appease warnings
    }};
    SLEEP_DELAY();
    store_crc(datagram.raw, sizeof(datagram.raw));
    // This only confirms the datagram went out on the wire (its echo came back); the IC doesn't acknowledge writes.
    const int ret = write_single_line_uart_and_flush_read(dev, datagram.raw, sizeof(datagram.raw));

    k_mutex_unlock(&adi_tmc2209_bus_lock);

    return ret;
}

/**
 * The request/reply exchange of `adi_tmc2209_read`; the caller holds `adi_tmc2209_bus_lock`.
 */
static int adi_tmc2209_read_datagram(const struct device *dev, const uint8_t reg_address, adi_tmc2209_reg_t *data) {
    const adi_tmc2209_config_t *config = dev->config;

    if (data == NULL) {
        return -EINVAL;
    }

    read_datagram_t datagram = {{
        .sync = SYNC_NIBBLE,
        .reserved = 0x0,
        .address = config->address,
        .reg_address = reg_address,
        .rw = READ,
        .crc = 0,  // appease warnings
    }};

    store_crc(datagram.raw, sizeof(datagram.raw));
    SLEEP_DELAY();
    write_single_line_uart_and_flush_read(dev, datagram.raw, sizeof(datagram.raw));

    read_reply_datagram_t reply_datagram = {0};

    const int ret = read_single_line_uart(dev, reply_datagram.raw, sizeof(reply_datagram.raw));

    if (ret != 0) {
        LOG_ERR("Failed (%d) to read reply datagram for %d. Bytes: %02X %02X %02X %02X %02X %02X %02X %02X", ret,
                config->address, reply_datagram.raw[0], reply_datagram.raw[1], reply_datagram.raw[2],
                reply_datagram.raw[3], reply_datagram.raw[4], reply_datagram.raw[5], reply_datagram.raw[6],
                reply_datagram.raw[7]);
        return ret;
    }

    if (!check_crc(reply_datagram.raw, sizeof(reply_datagram.raw))) {
        LOG_ERR("CRC check failed on reply datagram");
        return -EIO;
    }

    if (reply_datagram.fields.sync != SYNC_NIBBLE || reply_datagram.fields.address != HOST_ADDR ||
        reply_datagram.fields.reg_address != reg_address) {
        LOG_ERR("Unexpected data in reply datagram");
        return -EIO;
    }

    // Convert the endianness of the response data
    data->as_uint32 = sys_be32_to_cpu(reply_datagram.fields.data.as_uint32);

    return 0;
}

/**
 * Read `data` from `reg_address` on `device`.
 *
 * @param dev Device handle; must not be NULL
 * @param reg_address
 * @param data must be 4 bytes long.
 *
 * @retval 0 on success
 * @retval -errno on error.
 */
static int adi_tmc2209_read(const struct device *dev, const uint8_t reg_address, adi_tmc2209_reg_t *data) {
    k_mutex_lock(&adi_tmc2209_bus_lock, K_FOREVER);
    const int ret = adi_tmc2209_read_datagram(dev, reg_address, data);
    k_mutex_unlock(&adi_tmc2209_bus_lock);

    return ret;
}

/**
 * Set the OTP to match the values in `otp_data`. It is only possible to set currently unset
 * bits due to the IC's specifications. If there are bits set in the OTP when read which are
 * not set in `otp_data`, except the otp_fclktrim bits (see below),
 * it will not attempt to set any other bits and return -ENOTSUP. If the OTP data is already
 * matching, it will return 0 and take no further action.
 *
 * EXCEPTIONS TO THE ABOVE: The otp_fclktrim bits are used by the factory and will not be set
 * by this driver under any circumstances. If `otp_data.otp_fclktrim` is not 0, this
 * function will return -EPERM.
 *
 * @param dev
 * @param otp_data data to match to
 * @return 0 if the OTP is already set to the value, no further action is taken,
 *         1 if the OTP has been set successfully to match,
 *         -EIO on any error from called IO functions (it will log the return value),
 *              do not assume all bits have been set!
 *         -EAGAIN if, when rereading the OTP, it has not been set properly,
 *         -ENOTSUP if there is a bit unset in `otp_fields` which is set in the read OTP,
 *         -EPERM if `otp_data.otp_fclktrim` is not 0.
 */
static int adi_tmc2209_set_otp(const struct device *dev, const struct OTP_READ_data_fields otp_data) {
    const size_t BYTES_USED = 3U;
    const size_t BITS_PER_BYTE = 8;  // Not using `unsigned char`s, so CHAR_BIT is inappropriate.
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t current_otp = {0};
    const adi_tmc2209_reg_t new_otp = {.otp_read = otp_data};

    if (otp_data.otp_fclktrim_DO_NOT_USE != 0) {
        return -EPERM;
    }

    int ret = adi_tmc2209_read(dev, REG_OTP_READ, &current_otp);
    if (ret < 0) {
        LOG_ERR("[Device : %d] Error reading current OTP: %d, can't continue.", config->address, ret);
        return -EIO;
    }

    LOG_INF("[Device : %d] Current OTP: 0x%08X", config->address, current_otp.as_uint32);
    current_otp.otp_read.otp_fclktrim_DO_NOT_USE = 0;  // Unset this to avoid trying to program it

    const uint32_t old_otp_val = current_otp.as_uint32;
    const uint32_t new_otp_val = new_otp.as_uint32;

    if (old_otp_val == new_otp_val) {
        // Nothing to do
        return 0;
    }

    // Verify that no bits are set in the current otp that are not in the new otp
    for (size_t bit = 0; bit < BYTES_USED * BITS_PER_BYTE; bit++) {
        if (IS_BIT_SET(old_otp_val, bit) && !IS_BIT_SET(new_otp_val, bit)) {
            LOG_ERR("Bit %d is set in old OTP but unset in new OTP", bit);
            return -ENOTSUP;
        }
    }  // Is there a bit-twiddling hack for this? Maybe (old_otp_val & ~new_otp_val) == 0?

    for (size_t byte = 0; byte < BYTES_USED; byte++) {
        const uint8_t old_otp_byte = (old_otp_val >> (byte * 8)) & 0xFF;
        const uint8_t new_otp_byte = (new_otp_val >> (byte * 8)) & 0xFF;
        if (new_otp_byte != old_otp_byte) {
            for (size_t bit = 0; bit < 8; bit++) {
                if (IS_BIT_SET(new_otp_byte, bit)) {
                    if (!IS_BIT_SET(old_otp_byte, bit)) {
                        __ASSERT(!(byte == 0 && bit < 5),
                                 "ATTEMPTED to set fclktrim bytes in OTP, program configuration error.");
                        const adi_tmc2209_reg_t write_reg = {
                            .otp_program =
                                {
                                    .otpbit = bit,
                                    .otpbyte = byte,
                                    .otpmagic = OTP_MAGIC,
                                },
                        };
                        ret = adi_tmc2209_write(dev, REG_OTP_PROG, write_reg);
                        if (ret < 0) {
                            LOG_ERR("[Dev: %d] IO Failure when setting OTP: %d", config->address, ret);
                            return -EIO;
                        }
                    }  // else nothing to do here
                } else {
                    // Out of an abundance of caution
                    __ASSERT(BIT_IS_SET(new_otp_byte, bit), "Attempt to unset OTP bit which is already set");
                }
            }
        }
    }

    ret = adi_tmc2209_read(dev, REG_OTP_READ, &current_otp);
    if (ret < 0) {
        LOG_ERR("[Device : %d] Error re-reading OTP after set: %d", config->address, ret);
        return -EAGAIN;  // Is this more appropriate than -EIO?
    }
    current_otp.otp_read.otp_fclktrim_DO_NOT_USE = 0;

    if (current_otp.as_uint32 != new_otp_val) {
        LOG_ERR("[Device : %d] Current OTP: 0x%08X, attempted new OTP: 0x%08X, do not match when re-reading.",
                config->address, current_otp.as_uint32, new_otp_val);
        return -EAGAIN;
    }

    LOG_INF("Successfully updated OTP");

    return 1;
}

/**
 * Read the OTP, bitwise-or it with the mask provided and set the new OTP to that. Useful if you just
 * want to set one bit or register.
 *
 * @param dev Device to set OTP for
 * @param OTP_READ_data_fields MASK for the OTP_READ register
 * @return value from adi_tmc2209_set_otp
 */
__attribute__((unused)) static int adi_tmc2209_set_otp_or(const struct device *dev,
                                                          struct OTP_READ_data_fields otp_data_mask) {
    const adi_tmc2209_config_t *config = dev->config;
    const adi_tmc2209_reg_t reg_mask = {.otp_read = otp_data_mask};
    adi_tmc2209_reg_t current_otp = {0};
    const int ret = adi_tmc2209_read(dev, REG_OTP_READ, &current_otp);
    if (ret < 0) {
        LOG_ERR("[Device : %d] Error reading current OTP: %d", config->address, ret);
        return -EIO;
    }

    adi_tmc2209_reg_t new_otp = {.as_uint32 = current_otp.as_uint32 | reg_mask.as_uint32};
    new_otp.otp_read.otp_fclktrim_DO_NOT_USE = 0;
    return adi_tmc2209_set_otp(dev, new_otp.otp_read);
}

/**
 * Configure GCONF to use:
 * * internal voltage reference,
 * * external sense resistors,
 * * StealthChop,
 * * UART on the PDN_UART pin,
 * * microstep resolution using mres rather than MS1 and MS2, and
 * * no multistep filtering.
 *
 * Use the existing configurations for shaft direction, index_otpw, index_stop.
 *
 * Fails if the test_mode bit is set.
 *
 * @param dev Device to configure
 * @return 0 on success, -errno on error
 */
static int adi_tmc2209_set_default_gconf_stealthchop(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t reg = {0};
    int ret = adi_tmc2209_read(dev, REG_GCONF, &reg);
    if (ret != 0) {
        LOG_ERR("[Dev: %d], couldn't read GCONF! %d", config->address, ret);
        // There may be garbage in `reg`
        reg.as_uint32 = 0;
    }

    if (reg.gconf.test_mode_DO_NOT_USE) {
        LOG_ERR("[Dev: %d], GCONF test mode enabled, not supported!", config->address);
        return -ENOTSUP;
    }

    reg.gconf.i_scale_analog = 0;
    reg.gconf.internal_Rsense = 0;
    reg.gconf.en_spreadcycle = 0;
    reg.gconf.pdn_disable = 1;
    reg.gconf.mstep_reg_select = 1;
    reg.gconf.multistep_filt = 0;
    ret = adi_tmc2209_write(dev, REG_GCONF, reg);
    return ret;
}

int adi_tmc2209_set_ihold_irun(const struct device *dev, const uint8_t hold_current, const uint8_t run_current,
                               const uint8_t hold_delay) {
    if (hold_current > 32 || hold_current < 1 || run_current > 32 || run_current < 1 || hold_delay > 15) {
        LOG_ERR("Invalid arguments for setting IHOLD_IRUN register.");
        return -EINVAL;
    }

    adi_tmc2209_reg_t val = {0};

    val.ihold_irun.ihold = hold_current - 1;
    val.ihold_irun.irun = run_current - 1;
    val.ihold_irun.iholddelay = hold_delay;
    return adi_tmc2209_write(dev, REG_IHOLD_IRUN, val);
}

/**
 * The read-modify-write of `adi_tmc2209_set_microstep`; the caller holds `adi_tmc2209_bus_lock`.
 */
static int adi_tmc2209_set_mres(const struct device *dev, const uint32_t steps_per_fullstep) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t val = {0};
    int ret = adi_tmc2209_read(dev, REG_CHOPCONF, &val);

    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read chopconf data", config->address, ret);
        return -EIO;
    }

    uint8_t mres = 0;

    switch (steps_per_fullstep) {
        case 1:
            mres = 8;
            break;
        case 2:
            mres = 7;
            break;
        case 4:
            mres = 6;
            break;
        case 8:
            mres = 5;
            break;
        case 16:
            mres = 4;
            break;
        case 32:
            mres = 3;
            break;
        case 64:
            mres = 2;
            break;
        case 128:
            mres = 1;
            break;
        case 256:
            mres = 0;
            break;
        default:
            LOG_ERR("[Dev: %d] Invalid microstep: %u", config->address, steps_per_fullstep);
            return -EINVAL;
    }

    val.chopconf.mres = mres;
    ret = adi_tmc2209_write(dev, REG_CHOPCONF, val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to write chopconf", config->address, ret);
        return ret;
    }

    // The planner's step size must match the IC's, so confirm the IC took it.
    ret = adi_tmc2209_read(dev, REG_CHOPCONF, &val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read back chopconf", config->address, ret);
        return ret;
    }
    if (val.chopconf.mres != mres) {
        LOG_ERR("[Dev: %d] mres read back %d, wrote %d", config->address, val.chopconf.mres, mres);
        return -EIO;
    }

    return 0;
}

int adi_tmc2209_set_microstep(const struct device *dev, const uint32_t steps_per_fullstep) {
    k_mutex_lock(&adi_tmc2209_bus_lock, K_FOREVER);
    const int ret = adi_tmc2209_set_mres(dev, steps_per_fullstep);
    k_mutex_unlock(&adi_tmc2209_bus_lock);

    return ret;
}

int adi_tmc2209_read_drv_status(const struct device *dev, adi_tmc2209_reg_t *drv_status) {
    return adi_tmc2209_read(dev, REG_DRV_STATUS, drv_status);
}

/**
 * Set CHOPCONF.toff, leaving every other CHOPCONF field as the IC reports it, and confirm it by reading it back.
 * The read, write and readback hold the bus lock together so a concurrent CHOPCONF update can't interleave.
 *
 * @return 0 on success, -EIO if the readback doesn't match, -errno if the IC can't be read.
 */
static int adi_tmc2209_set_toff(const struct device *dev, const uint8_t toff) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t reg = {0};

    k_mutex_lock(&adi_tmc2209_bus_lock, K_FOREVER);
    int ret = adi_tmc2209_read(dev, REG_CHOPCONF, &reg);
    if (ret == 0) {
        reg.chopconf.toff = toff;
        ret = adi_tmc2209_write(dev, REG_CHOPCONF, reg);
    }
    if (ret == 0) {
        ret = adi_tmc2209_read(dev, REG_CHOPCONF, &reg);
    }
    if (ret == 0 && reg.chopconf.toff != toff) {
        ret = -EIO;
    }
    k_mutex_unlock(&adi_tmc2209_bus_lock);

    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set toff to %d; CHOPCONF read 0x%08X", config->address, ret, toff,
                reg.as_uint32);
    } else {
        LOG_INF("[Dev: %d] toff set to %d", config->address, toff);
    }

    return ret;
}

int adi_tmc2209_output_disable(const struct device *dev) { return adi_tmc2209_set_toff(dev, 0); }

int adi_tmc2209_output_enable(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    const adi_tmc2209_data_t *dev_data = dev->data;

    if (dev_data->enabled_toff == 0) {
        LOG_ERR("[Dev: %d] No enabled toff recorded to restore", config->address);
        return -EINVAL;
    }

    return adi_tmc2209_set_toff(dev, dev_data->enabled_toff);
}

/**
 * Set CHOPCONF with our settings for our modes, to wit:
 * * Do not use double-edged step impulses (`dedge`). It is incompatible with the
 *   mode of the STM32 timer peripheral we are using.
 * * Use full (256-microstep) interpolation.
 * * Use half-scale vsense for greater current dynamic range.
 * * Set hend, hstrt, toff, tbl to values which work in the enclosure.
 *
 * @param dev
 * @return
 */
static int adi_tmc2209_set_chopconf_stealthchop(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t val = {0};
    int ret = adi_tmc2209_read(dev, REG_CHOPCONF, &val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read chopconf data", config->address, ret);
        val.as_uint32 = 0x10000053;  // reset default
    }

    val.chopconf.dedge = 0;
    val.chopconf.intpol = 1;
    val.chopconf.vsense = 1;
    val.chopconf.tbl = 3;
    val.chopconf.hend = 0;
    val.chopconf.hstrt = 4;
    val.chopconf.toff = 2;
    ret = adi_tmc2209_write(dev, REG_CHOPCONF, val);
    return ret;
}

/**
 * Set the PWMCONF register for our application.
 * * Set it to freewheel if we haven't already.
 * * Set pwm_freq to 1 per datasheet suggestion for internal clock (p. 40).
 * * Set autograd and autoscale to yes but fill in PWM_GRAD and PWM_OFS with known good values
 *   to use until the driver can get enough data to adapt.
 *
 * @param dev device to configure
 * @return 0 on success, -errno on error
 */
static int adi_tmc2209_set_pwmconf_stealthchop(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t val = {0};

    int ret = adi_tmc2209_read(dev, REG_PWMCONF, &val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read pwmconf", config->address, ret);
        val.as_uint32 = 0xC10D0024;  // Reset default
    }

    val.pwmconf.pwm_autograd = 1;
    val.pwmconf.pwm_autoscale = 1;
    val.pwmconf.freewheel = 1;
    val.pwmconf.pwm_freq = 1;
    val.pwmconf.pwm_grad = 0x76;
    val.pwmconf.pwm_ofs = 0xff;

    ret = adi_tmc2209_write(dev, REG_PWMCONF, val);
    return ret;
}

/**
 * Disable coolstep on `dev`. It is unreliable on these motors.
 *
 * @return 0 on success, -errno on failure
 */
static int adi_tmc2209_coolstep_disable(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_reg_t reg = {0};
    int ret = adi_tmc2209_read(dev, REG_COOLCONF, &reg);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read coolstep", config->address, ret);
        reg.as_uint32 = 0;
    }
    reg.coolconf.semin = 0;
    ret = adi_tmc2209_write(dev, REG_COOLCONF, reg);
    return ret;
}

// Bits of GCONF and CHOPCONF that read back as written; the rest are reserved.
#define GCONF_RW_MASK 0x000003FFU
#define CHOPCONF_RW_MASK 0xFF0387FFU

/**
 * Read `reg_address` into `read` and compare the bits in `rw_mask` against `written`.
 *
 * @return 0 if they match, otherwise the ADI_TMC2209_FAULT_* bit describing the failure.
 */
static uint32_t adi_tmc2209_check_readback(const struct device *dev, const char *name, const uint8_t reg_address,
                                           const uint32_t rw_mask, const adi_tmc2209_reg_t written,
                                           adi_tmc2209_reg_t *read) {
    const adi_tmc2209_config_t *config = dev->config;

    read->as_uint32 = 0;
    const int ret = adi_tmc2209_read(dev, reg_address, read);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read back %s", config->address, ret, name);
        return ADI_TMC2209_FAULT_UART_READ;
    }

    if ((read->as_uint32 & rw_mask) != (written.as_uint32 & rw_mask)) {
        LOG_ERR("[Dev: %d] %s read back 0x%08X, wrote 0x%08X", config->address, name, read->as_uint32,
                written.as_uint32);
        return ADI_TMC2209_FAULT_READBACK;
    }

    LOG_INF("[Dev: %d] %s 0x%08X, matches write", config->address, name, read->as_uint32);
    return 0;
}

/**
 * Confirm the configuration written by `adi_tmc2209_init` reached the IC, log it, and store the
 * result for `adi_tmc2209_get_init_check`.
 * * IFCNT counts write datagrams the IC accepted (valid CRC, its address); reads don't change it.
 *   It must have advanced by exactly the number of writes sent since `ifcnt_before` was read.
 * * GCONF and CHOPCONF are read back and compared with the last values written.
 * * IHOLD_IRUN is write-only, so its read is logged but not compared; IFCNT confirms that write.
 *
 * @param ifcnt_before_ret return value of the IFCNT read taken before the configuration writes
 * @param ifcnt_before value of that read
 * @param writes_before `writes_sent` at the time of that read
 */
static void adi_tmc2209_verify_init(const struct device *dev, const int ifcnt_before_ret,
                                    const adi_tmc2209_reg_t ifcnt_before, const uint8_t writes_before) {
    const adi_tmc2209_config_t *config = dev->config;
    adi_tmc2209_data_t *dev_data = dev->data;
    adi_tmc2209_init_check_t *check = &dev_data->init_check;

    check->faults = 0;
    check->writes_sent = dev_data->writes_sent - writes_before;
    check->ifcnt_before = ifcnt_before.ifcnt & 0xFF;

    adi_tmc2209_reg_t ifcnt_after = {0};
    const int ifcnt_after_ret = adi_tmc2209_read(dev, REG_IFCNT, &ifcnt_after);
    check->ifcnt_after = ifcnt_after.ifcnt & 0xFF;
    if (ifcnt_before_ret < 0 || ifcnt_after_ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d, %d) to read IFCNT, can't confirm %u writes", config->address,
                ifcnt_before_ret, ifcnt_after_ret, check->writes_sent);
        check->faults |= ADI_TMC2209_FAULT_UART_READ;
    } else {
        const uint8_t advanced = check->ifcnt_after - check->ifcnt_before;
        if (advanced != check->writes_sent) {
            LOG_ERR("[Dev: %d] IFCNT %u -> %u advanced by %u, but %u writes were sent", config->address,
                    check->ifcnt_before, check->ifcnt_after, advanced, check->writes_sent);
            check->faults |= ADI_TMC2209_FAULT_WRITE_LOST;
        } else {
            LOG_INF("[Dev: %d] IFCNT %u -> %u, all %u writes accepted", config->address, check->ifcnt_before,
                    check->ifcnt_after, check->writes_sent);
        }
    }

    check->gconf_written = dev_data->written_gconf;
    check->faults |= adi_tmc2209_check_readback(dev, "GCONF", REG_GCONF, GCONF_RW_MASK, check->gconf_written,
                                                &check->gconf_read);

    check->chopconf_written = dev_data->written_chopconf;
    check->faults |= adi_tmc2209_check_readback(dev, "CHOPCONF", REG_CHOPCONF, CHOPCONF_RW_MASK,
                                                check->chopconf_written, &check->chopconf_read);
    LOG_INF("[Dev: %d] CHOPCONF vsense %d, mres %d, toff %d, tbl %d", config->address,
            check->chopconf_read.chopconf.vsense, check->chopconf_read.chopconf.mres,
            check->chopconf_read.chopconf.toff, check->chopconf_read.chopconf.tbl);

    check->ihold_irun_written = dev_data->written_ihold_irun;
    check->ihold_irun_read.as_uint32 = 0;
    const int ihold_irun_ret = adi_tmc2209_read(dev, REG_IHOLD_IRUN, &check->ihold_irun_read);
    LOG_INF("[Dev: %d] IHOLD_IRUN wrote 0x%08X (irun %d, ihold %d, iholddelay %d); write-only, read gave 0x%08X (%d)",
            config->address, check->ihold_irun_written.as_uint32, check->ihold_irun_written.ihold_irun.irun,
            check->ihold_irun_written.ihold_irun.ihold, check->ihold_irun_written.ihold_irun.iholddelay,
            check->ihold_irun_read.as_uint32, ihold_irun_ret);

    if (check->faults != 0) {
        LOG_ERR("[Dev: %d] Configuration check failed, faults 0x%X", config->address, check->faults);
    }
}

const adi_tmc2209_init_check_t *adi_tmc2209_get_init_check(const struct device *dev) {
    const adi_tmc2209_data_t *dev_data = dev->data;
    return &dev_data->init_check;
}

/**
 * Configure this driver according to our motors' characteristics. Some general notes:
 * * We are using the internal clock which is factory trimmed to 12MHz.
 * * We want to use StealthChop by default but SpreadCycle when we go fast enough.
 *
 * @param dev
 * @return 0 on success, -errno on error
 */
static int adi_tmc2209_init(const struct device *dev) {
    const adi_tmc2209_config_t *config = dev->config;
    const adi_tmc2209_data_t *dev_data = dev->data;

    struct OTP_READ_data_fields otp_to_prog = {0};
    otp_to_prog.otp_ihold = 1;
    adi_tmc2209_set_otp(dev, otp_to_prog);

    // Snapshot the interface counter; every write from here on is checked against it at the end.
    adi_tmc2209_reg_t ifcnt_before = {0};
    const int ifcnt_before_ret = adi_tmc2209_read(dev, REG_IFCNT, &ifcnt_before);
    const uint8_t writes_before = dev_data->writes_sent;

    adi_tmc2209_reg_t val = {0};

    val.as_uint32 = 0;
    // Clear reset and print diagnostics from GSTAT.
    int ret = adi_tmc2209_read(dev, REG_GSTAT, &val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to read gstat data", config->address, ret);
    } else {
        LOG_DBG("[Dev: %d] GSTAT flags reset [%d] drv_err [%d] uv_cp [%d]", config->address, val.gstat.reset,
                val.gstat.drv_err, val.gstat.uv_cp);

        // Write back to clear the status flags.
        ret = adi_tmc2209_write(dev, REG_GSTAT, val);
        if (ret < 0) {
            LOG_ERR("Failed (%d) to write gstat data to reset", ret);
        }
    }

    // Configure SEND_DELAY to avoid issues with other drivers.
    val.as_uint32 = 0;
    val.nodeconf.send_delay = SEND_DELAY;
    adi_tmc2209_write(dev, REG_NODECONF, val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to write nodeconf", config->address, ret);
    }

    // Configure GCONF next to avoid PDN_UART pin weirdness.
    ret = adi_tmc2209_set_default_gconf_stealthchop(dev);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set default GCONF", config->address, ret);
    }

    // Set IHOLD and IRUN according to our motor's characteristics.
    // The actuators are rated 0.49 A per phase, taken here as the peak of the sine current. With vsense on and
    // 180 mOhm sense resistors, I_rms = irun/32 * 0.18 V / (0.18 + 0.02) Ohm / sqrt(2) = irun/32 * 0.636 A.
    // 17 => 0.338 A RMS, 0.478 A peak: the largest setting that keeps the peak within the rating.
    const uint8_t default_irun = 17;
    const uint8_t default_ihold = 1;        // Use as little current as possible in standstill to reduce heating.
    const uint8_t default_iholddelay = 15;  // Use the greatest time because otherwise it's choppy at low speed.
    ret = adi_tmc2209_set_ihold_irun(dev, default_ihold, default_irun, default_iholddelay);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set irun", config->address, ret);
    }

    // Set TPOWERDOOWN lower (to save current/heating) but still high enough to use autoconf
    // for both SpreadCycle and StealthChop.
    val.as_uint32 = 4;
    ret = adi_tmc2209_write(dev, REG_TPOWERDOWN, val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set TPOWERDOWN", config->address, ret);
    }

    // Empirical testing shows that SpreadCycle works better than StealthChop at these speeds.
    val.as_uint32 = 1500;
    ret = adi_tmc2209_write(dev, REG_TPWMTHRS, val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to write TPWMTHRS -- SpreadCycle may become enabled", config->address, ret);
    }

    // Set single-stepping by default.
    ret = adi_tmc2209_set_microstep(dev, 1);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set microstep", config->address, ret);
    }

    ret = adi_tmc2209_set_chopconf_stealthchop(dev);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set chopconf", config->address, ret);
    }

    ret = adi_tmc2209_coolstep_disable(dev);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to disable coolstep", config->address, ret);
    }

    // Disable StallGuard. At low velocities, it's extremely unreliable and is my
    // best guess as to the problems we were seeing in the enclosures before. At high
    // velocities, we tend to pass through resonances that make the values go all over
    // the place. We are using SpreadCycle there anyways, so just turn this off.
    val.as_uint32 = 0;
    ret = adi_tmc2209_write(dev, REG_SGTHRS, val);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to write SGTHRS", config->address, ret);
    }

    ret = adi_tmc2209_set_pwmconf_stealthchop(dev);
    if (ret < 0) {
        LOG_ERR("[Dev: %d] Failed (%d) to set pwmconf", config->address, ret);
    }

    adi_tmc2209_verify_init(dev, ifcnt_before_ret, ifcnt_before, writes_before);

    return ret;
}

static struct adi_tmc2209_driver_api adi_tmc2209_driver_api = {
    .write = adi_tmc2209_write,
    .read = adi_tmc2209_read,
};

#define ADI_TMC2209_INIT(n)                                                                                       \
    static const adi_tmc2209_config_t adi_tmc2209_config_##n = {                                                  \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                                                \
        .address = DT_INST_REG_ADDR(n),                                                                           \
    };                                                                                                            \
    static adi_tmc2209_data_t adi_tmc2209_data_##n;                                                               \
    DEVICE_DT_INST_DEFINE(n, adi_tmc2209_init, NULL, &adi_tmc2209_data_##n, &adi_tmc2209_config_##n, POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &adi_tmc2209_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADI_TMC2209_INIT)
