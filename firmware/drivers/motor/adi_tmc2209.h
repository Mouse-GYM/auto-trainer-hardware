#pragma once

#include <zephyr/device.h>

#include "adi_tmc2209_types.h"

/*
 * This low-level API allows for bare register reads and writes to the TMC2209.
 * It should probably only be used by the driver itself, but is exposed here for
 * the shell to use.
 */
struct adi_tmc2209_driver_api {
    int (*write)(const struct device *dev, uint8_t reg_address, const adi_tmc2209_reg_t data);
    int (*read)(const struct device *dev, uint8_t reg_address, adi_tmc2209_reg_t *data);
};

/* Faults found by the post-init configuration check (`adi_tmc2209_init_check_t.faults`). */
#define ADI_TMC2209_FAULT_UART_READ BIT(0)   // A verification read got no valid reply.
#define ADI_TMC2209_FAULT_WRITE_LOST BIT(1)  // IFCNT did not advance by the number of writes sent.
#define ADI_TMC2209_FAULT_READBACK BIT(2)    // GCONF or CHOPCONF did not read back as written.

/*
 * Result of checking the configuration written at init. IHOLD_IRUN is write-only on the IC, so
 * `ihold_irun_read` is informational; the IFCNT comparison is what confirms that write.
 */
typedef struct {
    uint32_t faults;       // ADI_TMC2209_FAULT_* bits, 0 if every check passed.
    uint8_t ifcnt_before;  // IFCNT before the first configuration write.
    uint8_t ifcnt_after;   // IFCNT after the last configuration write.
    uint8_t writes_sent;   // Write datagrams sent in between, modulo 256.
    adi_tmc2209_reg_t gconf_written;
    adi_tmc2209_reg_t gconf_read;
    adi_tmc2209_reg_t chopconf_written;
    adi_tmc2209_reg_t chopconf_read;
    adi_tmc2209_reg_t ihold_irun_written;
    adi_tmc2209_reg_t ihold_irun_read;
} adi_tmc2209_init_check_t;

/**
 * Get the result of the configuration check run at the end of init. Safe to call from ISRs.
 *
 * @param dev ADI TMC2209 device; must not be NULL.
 * @return the stored check result.
 */
const adi_tmc2209_init_check_t *adi_tmc2209_get_init_check(const struct device *dev);

/**
 * Set the IHOLD_IRUN register according to register values.
 *
 * @param dev ADI TMC2209 device to be modified.
 * @param hold_current Hold current as numerator of 32.
 * @param run_current Run current as numerator of 32.
 * @param hold_delay Delay after STEP pin goes low before current is reduced, in 2^18 clocks.
 * @return 0 on success, -EINVAL if the current values are out of range, -errno on IO error.
 */
int adi_tmc2209_set_ihold_irun(const struct device *dev, uint8_t hold_current, uint8_t run_current, uint8_t hold_delay);

/**
 *
 * Set the microstep resolution for `dev`.
 *
 * @param dev Device to set microstep.
 * @param steps_per_fullstep The desired number of STEP pin pulses to effect one full step. One of 1, 2, 4, 8, 16, 32,
 * 64, 128, 256.
 * change the
 * @return 0 on success
 * @return -errno on failure
 */
int adi_tmc2209_set_microstep(const struct device *dev, uint32_t steps_per_fullstep);

/**
 * Read DRV_STATUS (otpw, ot, short and open-load flags, standstill). Blocks for the UART exchange (about 10 ms);
 * don't call from an ISR.
 *
 * @param dev ADI TMC2209 device.
 * @param drv_status receives the register; use its `drv_status` fields.
 * @return 0 on success, -errno if the IC didn't give a valid reply.
 */
int adi_tmc2209_read_drv_status(const struct device *dev, adi_tmc2209_reg_t *drv_status);

/**
 * Switch the driver's power stage off by writing CHOPCONF.toff = 0, the datasheet's software driver disable
 * (~EN isn't routed to the MCU). The motor freewheels. Only toff changes; the current settings (vsense, IRUN,
 * IHOLD) and the rest of CHOPCONF are left as the IC reports them. Confirmed by reading CHOPCONF back. Blocks for
 * the UART exchanges; don't call from an ISR.
 *
 * @return 0 once the IC reads back toff = 0, -EIO if it doesn't, -errno if the IC can't be read.
 */
int adi_tmc2209_output_disable(const struct device *dev);

/**
 * Undo `adi_tmc2209_output_disable`: restore the last nonzero toff written (the value set at init), confirmed by
 * reading CHOPCONF back. Blocks for the UART exchanges; don't call from an ISR.
 *
 * @return 0 once the IC reads back the restored toff, -EINVAL if no nonzero toff was ever written, -EIO if the
 *         readback doesn't match, -errno if the IC can't be read.
 */
int adi_tmc2209_output_enable(const struct device *dev);