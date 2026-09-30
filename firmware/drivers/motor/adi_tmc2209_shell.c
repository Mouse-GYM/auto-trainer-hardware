#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>

#include "adi_tmc2209.h"
#include "adi_tmc2209_types.h"

#define DEV_GET_COMMA(id) DEVICE_DT_GET(id),
static const struct device *adi_devs[] = {DT_FOREACH_STATUS_OKAY(adi_tmc2209, DEV_GET_COMMA)};

static adi_tmc2209_reg_t read_device_reg(const struct shell *shell, const int device, const int reg) {
    const struct device *dev = adi_devs[device];
    struct adi_tmc2209_driver_api *api = (struct adi_tmc2209_driver_api *)dev->api;
    adi_tmc2209_reg_t val;
    int ret = api->read(dev, reg, &val);
    if (ret < 0) {
        shell_print(shell, "Failed to read register: %d", ret);
    }

    return val;
}

static int dump_gconf(const struct shell *shell, const int device) {
    // GCONF
    struct GCONF_data_fields gconf = read_device_reg(shell, device, REG_GCONF).gconf;
    shell_print(shell, "GCONF:");
    shell_print(shell, "  i_scale_analog: %d", gconf.i_scale_analog);
    shell_print(shell, "  internal_Rsense: %d", gconf.internal_Rsense);
    shell_print(shell, "  en_spreadcycle: %d", gconf.en_spreadcycle);
    shell_print(shell, "  shaft: %d", gconf.shaft);
    shell_print(shell, "  index_otpw: %d", gconf.index_otpw);
    shell_print(shell, "  index_step: %d", gconf.index_step);
    shell_print(shell, "  pdn_disable: %d", gconf.pdn_disable);
    shell_print(shell, "  mstep_reg_select: %d", gconf.mstep_reg_select);
    shell_print(shell, "  multistep_filt: %d", gconf.multistep_filt);
    shell_print(shell, "  test_mode: %d", gconf.test_mode_DO_NOT_USE);

    return 0;
}

static int dump_gstat(const struct shell *shell, const int device) {
    // GSTAT
    struct GSTAT_data_fields gstat = read_device_reg(shell, device, REG_GSTAT).gstat;
    shell_print(shell, "GSTAT:");
    shell_print(shell, "  reset: %d", gstat.reset);
    shell_print(shell, "  drv_err: %d", gstat.drv_err);
    shell_print(shell, "  uv_cp: %d", gstat.uv_cp);

    return 0;
}

static int dump_ifcnt(const struct shell *shell, const int device) {
    // IFCNT
    shell_print(shell, "IFCNT: %d", read_device_reg(shell, device, REG_IFCNT).ifcnt);

    return 0;
}

static int dump_ihold_irun(const struct shell *shell, const int device) {
    // IHOLD_IRUN
    struct IHOLD_IRUN_data_fields ihold_irun = read_device_reg(shell, device, REG_IHOLD_IRUN).ihold_irun;
    shell_print(shell, "IHOLD_IRUN:");
    shell_print(shell, "  iholddelay: %d", ihold_irun.iholddelay);
    shell_print(shell, "  irun: %d", ihold_irun.irun);
    shell_print(shell, "  ihold: %d", ihold_irun.ihold);

    return 0;
}

static int dump_ioin(const struct shell *shell, const int device) {
    // IOIN
    struct IOIN_data_fields ioin = read_device_reg(shell, device, REG_IOIN).ioin;
    shell_print(shell, "IOIN:");
    shell_print(shell, "  enn: %d", ioin.enn);
    shell_print(shell, "  ms1: %d", ioin.ms1);
    shell_print(shell, "  ms2: %d", ioin.ms2);
    shell_print(shell, "  diag: %d", ioin.diag);
    shell_print(shell, "  pdn_uart: %d", ioin.pdn_uart);
    shell_print(shell, "  step: %d", ioin.step);
    shell_print(shell, "  spread_en: %d", ioin.spread_en);
    shell_print(shell, "  dir: %d", ioin.dir);
    shell_print(shell, "  version: 0x%x", ioin.version);

    return 0;
}

static int dump_tstep(const struct shell *shell, const int device) {
    // TSTEP
    shell_print(shell, "TSTEP: %d", read_device_reg(shell, device, REG_TSTEP).tstep);
    return 0;
}

static int dump_sg_result(const struct shell *shell, const int device) {
    // SG_RESULT
    shell_print(shell, "SG_RESULT: %d", read_device_reg(shell, device, REG_SG_RESULT).sg_result);
    return 0;
}

static int dump_mscnt(const struct shell *shell, const int device) {
    // MSCNT
    shell_print(shell, "MSCNT: %d", read_device_reg(shell, device, REG_MSCNT).mscnt);
    return 0;
}

static int dump_mscuract(const struct shell *shell, const int device) {
    // MSCURACT
    struct MSCURACT_data_fields mscuract = read_device_reg(shell, device, REG_MSCURACT).mscuract;
    shell_print(shell, "MSCURACT:");
    shell_print(shell, "  cur_b: %d", mscuract.cur_b);
    shell_print(shell, "  cur_a: %d", mscuract.cur_a);
    return 0;
}

static int dump_chopconf(const struct shell *shell, const int device) {
    // CHOPCONF
    struct CHOPCONF_data_fields chopconf = read_device_reg(shell, device, REG_CHOPCONF).chopconf;
    shell_print(shell, "CHOPCONF:");
    shell_print(shell, "  toff: %d", chopconf.toff);
    shell_print(shell, "  hstrt: %d", chopconf.hstrt);
    shell_print(shell, "  hend: %d", chopconf.hend);
    shell_print(shell, "  tbl: %d", chopconf.tbl);
    shell_print(shell, "  vsense: %d", chopconf.vsense);
    shell_print(shell, "  mres: %d", chopconf.mres);
    shell_print(shell, "  intpol: %d", chopconf.intpol);
    shell_print(shell, "  dedge: %d", chopconf.dedge);
    shell_print(shell, "  diss2g: %d", chopconf.diss2g);
    shell_print(shell, "  diss2vs: %d", chopconf.diss2vs);

    return 0;
}

static int dump_drv_status(const struct shell *shell, const int device) {
    // DRV_STATUS
    struct DRV_STATUS_data_fields drv_status = read_device_reg(shell, device, REG_DRV_STATUS).drv_status;
    shell_print(shell, "DRV_STATUS:");
    shell_print(shell, "  otpw: %d", drv_status.otpw);
    shell_print(shell, "  ot: %d", drv_status.ot);
    shell_print(shell, "  s2ga: %d", drv_status.s2ga);
    shell_print(shell, "  s2gb: %d", drv_status.s2gb);
    shell_print(shell, "  s2vsa: %d", drv_status.s2vsa);
    shell_print(shell, "  s2vsb: %d", drv_status.s2vsb);
    shell_print(shell, "  ola: %d", drv_status.ola);
    shell_print(shell, "  olb: %d", drv_status.olb);
    shell_print(shell, "  t120: %d", drv_status.t120);
    shell_print(shell, "  t143: %d", drv_status.t143);
    shell_print(shell, "  t150: %d", drv_status.t150);
    shell_print(shell, "  t157: %d", drv_status.t157);
    shell_print(shell, "  cs_actual: %d", drv_status.cs_actual);
    shell_print(shell, "  stealth: %d", drv_status.stealth);
    shell_print(shell, "  stst: %d", drv_status.stst);

    return 0;
}

static int dump_pwmconf(const struct shell *shell, const int device) {
    // DRV_STATUS
    struct PWMCONF_data_fields pwmconf = read_device_reg(shell, device, REG_PWMCONF).pwmconf;
    shell_print(shell, "PWMCONF:");
    shell_print(shell, "  pwm_ofs: %d", pwmconf.pwm_ofs);
    shell_print(shell, "  pwm_grad: %d", pwmconf.pwm_grad);
    shell_print(shell, "  pwm_freq: %d", pwmconf.pwm_freq);
    shell_print(shell, "  pwm_autoscale: %d", pwmconf.pwm_autoscale);
    shell_print(shell, "  pwm_autograd: %d", pwmconf.pwm_autograd);
    shell_print(shell, "  freewheel: %d", pwmconf.freewheel);
    shell_print(shell, "  pwm_reg: %d", pwmconf.pwm_reg);
    shell_print(shell, "  pwm_lim: %d", pwmconf.pwm_lim);

    return 0;
}

static int cmd_dump_registers(const struct shell *shell, const int argc, const char *argv[]) {
    if (argc < 2) {
        shell_print(shell, "Usage: %s <device>", argv[0]);
        return -EINVAL;
    }

    char *endptr;
    const int device = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid device number: %s", argv[1]);
        return -EINVAL;
    }

    if (device < 0 || device >= ARRAY_SIZE(adi_devs)) {
        shell_print(shell, "Invalid device number: %d", device);
        return -EINVAL;
    }

    dump_gconf(shell, device);
    dump_gstat(shell, device);
    dump_ifcnt(shell, device);
    dump_ioin(shell, device);

    dump_ihold_irun(shell, device);
    dump_tstep(shell, device);

    dump_sg_result(shell, device);

    dump_mscnt(shell, device);
    dump_mscuract(shell, device);

    dump_chopconf(shell, device);
    dump_pwmconf(shell, device);
    dump_drv_status(shell, device);

    return 0;
}

static int dump_otp(const struct shell *shell, const int device) {
    const struct device *dev = adi_devs[device];
    struct adi_tmc2209_driver_api *api = (struct adi_tmc2209_driver_api *)dev->api;
    adi_tmc2209_reg_t current_otp = {0};

    const int ret = api->read(dev, REG_OTP_READ, &current_otp);
    if (ret < 0) {
        shell_print(shell, "Failed to read OTP register: %d", ret);
        return ret;
    }
    shell_print(shell, "OTP:");
    shell_print(shell, "  fclktrim: %d", current_otp.otp_read.otp_fclktrim_DO_NOT_USE);
    shell_print(shell, "  ottrim: %d", current_otp.otp_read.otp_ottrim);
    shell_print(shell, "  internalrsense: %d", current_otp.otp_read.otp_internalrsense);
    shell_print(shell, "  tbl: %d", current_otp.otp_read.otp_tbl);
    shell_print(shell, "  pwm_grad: %d", current_otp.otp_read.otp_pwm_grad);
    shell_print(shell, "  pwm_autograd: %d", current_otp.otp_read.otp_pwm_autograd);
    shell_print(shell, "  tpwmthrs: %d", current_otp.otp_read.otp_tpwmthrs);
    shell_print(shell, "  pwm_ofs: %d", current_otp.otp_read.otp_pwm_ofs);
    shell_print(shell, "  pwm_reg: %d", current_otp.otp_read.otp_pwm_reg);
    shell_print(shell, "  pwm_freq: %d", current_otp.otp_read.otp_pwm_freq);
    shell_print(shell, "  iholddelay: %d", current_otp.otp_read.otp_iholddelay);
    shell_print(shell, "  ihold: %d", current_otp.otp_read.otp_ihold);
    shell_print(shell, "  otp_en_spreadcycle: %d", current_otp.otp_read.otp_en_spreadcycle);

    return 0;
}

static int cmd_dump_otp(const struct shell *shell, const int argc, const char *argv[]) {
    if (argc < 2) {
        shell_print(shell, "Usage: %s <device>", argv[0]);
        return -EINVAL;
    }

    char *endptr;
    const int device = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid device number: %s", argv[1]);
        return -EINVAL;
    }

    if (device < 0 || device >= ARRAY_SIZE(adi_devs)) {
        shell_print(shell, "Invalid device number: %d", device);
        return -EINVAL;
    }

    return dump_otp(shell, device);
}

static int cmd_init_check(const struct shell *shell, const int argc, const char *argv[]) {
    char *endptr;
    const int device = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0' || device < 0 || device >= ARRAY_SIZE(adi_devs)) {
        shell_print(shell, "Invalid device number: %s", argv[1]);
        return -EINVAL;
    }

    const adi_tmc2209_init_check_t *check = adi_tmc2209_get_init_check(adi_devs[device]);
    shell_print(shell, "Init check: %s (faults 0x%X)", check->faults == 0 ? "PASS" : "FAIL", check->faults);
    shell_print(shell, "  IFCNT: %u -> %u, %u writes sent", check->ifcnt_before, check->ifcnt_after,
                check->writes_sent);
    shell_print(shell, "  GCONF: wrote 0x%08X, read 0x%08X", check->gconf_written.as_uint32,
                check->gconf_read.as_uint32);
    shell_print(shell, "  CHOPCONF: wrote 0x%08X, read 0x%08X (vsense %d)", check->chopconf_written.as_uint32,
                check->chopconf_read.as_uint32, check->chopconf_read.chopconf.vsense);
    shell_print(shell, "  IHOLD_IRUN: wrote 0x%08X (irun %d, ihold %d), write-only, read 0x%08X",
                check->ihold_irun_written.as_uint32, check->ihold_irun_written.ihold_irun.irun,
                check->ihold_irun_written.ihold_irun.ihold, check->ihold_irun_read.as_uint32);

    return 0;
}

static int cmd_read_register(const struct shell *shell, const int argc, const char *argv[]) {
    if (argc < 3) {
        shell_print(shell, "Usage: %s <device> 0x<register>", argv[0]);
        return -EINVAL;
    }

    char *endptr;
    const int device = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid device number: %s", argv[1]);
        return -EINVAL;
    }

    if (device < 0 || device >= ARRAY_SIZE(adi_devs)) {
        shell_print(shell, "Invalid device number: %d", device);
        return -EINVAL;
    }

    const struct device *dev = adi_devs[device];
    struct adi_tmc2209_driver_api *api = (struct adi_tmc2209_driver_api *)dev->api;

    const int reg = strtol(argv[2], &endptr, 16);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid register: %s", argv[2]);
        return -EINVAL;
    }

    if (reg < 0 || reg > 0xFF) {
        shell_print(shell, "Invalid register: 0x%02X", reg);
        return -EINVAL;
    }

    switch (reg) {
        case REG_GCONF:
            dump_gconf(shell, device);
            break;
        case REG_GSTAT:
            dump_gstat(shell, device);
            break;
        case REG_IFCNT:
            dump_ifcnt(shell, device);
            break;
        case REG_IHOLD_IRUN:
            dump_ihold_irun(shell, device);
            break;
        case REG_IOIN:
            dump_ioin(shell, device);
            break;
        case REG_TSTEP:
            dump_tstep(shell, device);
            break;
        case REG_SG_RESULT:
            dump_sg_result(shell, device);
            break;
        case REG_MSCNT:
            dump_mscnt(shell, device);
            break;
        case REG_MSCURACT:
            dump_mscuract(shell, device);
            break;
        case REG_CHOPCONF:
            dump_chopconf(shell, device);
            break;
        case REG_DRV_STATUS:
            dump_drv_status(shell, device);
            break;
        case REG_PWMCONF:
            dump_pwmconf(shell, device);
            break;
        default: {
            adi_tmc2209_reg_t val;
            const int ret = api->read(dev, reg, &val);

            shell_print(shell, "Data at 0x%02X: 0x%02X 0x%02X 0x%02X 0x%02X, returned %d", reg, val.as_bytes[0],
                        val.as_bytes[1], val.as_bytes[2], val.as_bytes[3], ret);
        } break;
    }

    return 0;
}

static int cmd_write_register(const struct shell *shell, const int argc, const char *argv[]) {
    if (argc < 4) {
        shell_print(shell, "Usage: %s <device> 0x<register> 0x<data>", argv[0]);
        return -EINVAL;
    }

    char *endptr;
    const int device = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid device number: %s", argv[1]);
        return -EINVAL;
    }

    if (device < 0 || device >= ARRAY_SIZE(adi_devs)) {
        shell_print(shell, "Invalid device number: %d", device);
        return -EINVAL;
    }

    const struct device *dev = adi_devs[device];
    struct adi_tmc2209_driver_api *api = (struct adi_tmc2209_driver_api *)dev->api;

    const int reg = strtol(argv[2], &endptr, 16);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid register: %s", argv[2]);
        return -EINVAL;
    }

    if (reg < 0 || reg > 0xFF) {
        shell_print(shell, "Invalid register: 0x%02X", reg);
        return -EINVAL;
    }

    // Use `strtoll` here otherwise we run into sign errors.
    adi_tmc2209_reg_t data;
    data.as_uint32 = (uint32_t)strtoll(argv[3], &endptr, 16);
    if (*endptr != '\0') {
        shell_print(shell, "Invalid data: %s", argv[3]);
        return -EINVAL;
    }

    const int ret = api->write(dev, reg, data);

    shell_print(shell, "Wrote 0x%08X to 0x%02X, returned %d", data.as_uint32, reg, ret);
    return ret < 0 ? ret : 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(subcmds, SHELL_CMD_ARG(read, NULL, "Read a register", cmd_read_register, 3, 0),
                               SHELL_CMD_ARG(write, NULL, "Write a register", cmd_write_register, 4, 0),
                               SHELL_CMD_ARG(dump, NULL, "Dump all registers", cmd_dump_registers, 2, 0),
                               SHELL_CMD_ARG(dump_otp, NULL, "Dump the OTP memory", cmd_dump_otp, 2, 0),
                               SHELL_CMD_ARG(init_check, NULL, "Show the configuration check run at init",
                                             cmd_init_check, 2, 0),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(adi_tmc2209, &subcmds, "TMC2209 register access", NULL);
