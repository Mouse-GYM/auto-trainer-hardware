#include <stdlib.h>
#include <zephyr/shell/shell.h>

#include "load_cell.h"

#define DEV_GET_COMMA(id) DEVICE_DT_GET(id),
static const struct device *load_cell_devs[] = {DT_FOREACH_STATUS_OKAY(ll_load_cell, DEV_GET_COMMA)};

/* Mapping of k_work_submit errors to string  */
static const char *k_work_submit_error_to_str[] = {
    [EBUSY] = "work item is cancelling; or queue is draining; or queue is plugged",
    [EINVAL] = "queue is null and the work item has never been run",
    [ENODEV] = "queue has not been started",
};

static void load_cell_dump_handler(struct k_work *work);

typedef struct {
    bool initialized;
    struct k_work dump_work;
    bool dump_enabled;
    const struct shell *shell;
} dump_context_t;

/* Print the most recent load cell reading from the given load cell to the shell */
static int cmd_load_cell_read(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_print(shell, "Usage: %s <load_cell>", argv[0]);
        return -EINVAL;
    }

    int load_cell = atoi(argv[1]);

    /* Make sure the specified tone generator exists */
    if (load_cell < 0 || load_cell >= ARRAY_SIZE(load_cell_devs)) {
        shell_print(shell, "Invalid load cell instance: %d", load_cell);
        return -EINVAL;
    }

    float load = ll_load_cell_get_load_mv_float(load_cell_devs[load_cell]);
    shell_print(shell, "load_cell%d: %1.3fmV", load_cell,
                (double)load);  // Cast to double to avoid [-Wdouble-promotion]

    return 0;
}

static void load_cell_dump_handler(struct k_work *work) {
    dump_context_t *dump_context = CONTAINER_OF(work, dump_context_t, dump_work);

    for (int i = 0; i < ARRAY_SIZE(load_cell_devs); i++) {
        float load = ll_load_cell_get_load_mv_float(load_cell_devs[i]);
        shell_print(dump_context->shell, "load_cell%d: %1.3fmV", i,
                    (double)load);  // Cast to double to avoid [-Wdouble-promotion]
    }

    if (dump_context->dump_enabled) {
        /* Submit dump work item to the system workqueue */
        int ret = k_work_submit(work);
        switch (ret) {
            case 0: /* fall-through */
                /* work was already submitted to a queue */
            case 1: /* fall-through */
                /* work was not submitted and has been queued to queue */
            case 2:
                /* work was running and has been queued to the queue that was running it */
                break;
            case -EBUSY:  /* fall-through */
            case -EINVAL: /* fall-through */
            case -ENODEV:
                shell_print(dump_context->shell, "Failed to submit dump_work to the system workqueue: %s",
                            k_work_submit_error_to_str[-ret]);
                break;
            default:
                shell_print(dump_context->shell,
                            "Failed to submit dump_work to the system workqueue: Unknown error - %d", ret);
                break;
        }
    }
}

/* Dump all load cell data from all present load cells to the shell */
static int cmd_load_cell_dump(const struct shell *shell, size_t argc, char **argv) {
    static dump_context_t dump_context = {.initialized = false, .dump_enabled = false, .dump_work = {}, .shell = NULL};

    if (argc != 1) {
        shell_print(shell, "Usage: %s", argv[0]);
        return -EINVAL;
    }

    int load_cell = atoi(argv[1]);

    /* Make sure the specified tone generator exists */
    if (load_cell < 0 || load_cell >= ARRAY_SIZE(load_cell_devs)) {
        shell_print(shell, "Invalid load cell instance: %d", load_cell);
        return -EINVAL;
    }

    /* Make sure that the dump context is initialized */
    if (dump_context.initialized == false) {
        k_work_init(&dump_context.dump_work, load_cell_dump_handler);
        dump_context.dump_enabled = false;
        dump_context.shell = shell;
        dump_context.initialized = true;
    }

    if (dump_context.dump_enabled) {
        dump_context.dump_enabled = false;
        shell_print(shell, "Load Cell dumping disabled.");
        return 0;
    }

    dump_context.dump_enabled = true;
    shell_print(shell, "Load Cell dumping enabled.");
    int ret = k_work_submit(&dump_context.dump_work);
    switch (ret) {
        case 0: /* fall-through */
            /* work was already submitted to a queue */
        case 1: /* fall-through */
            /* work was not submitted and has been queued to queue */
        case 2:
            /* work was running and has been queued to the queue that was running it */
            return 0;
            break;
        case -EBUSY:  /* fall-through */
        case -EINVAL: /* fall-through */
        case -ENODEV:
            shell_print(shell, "Failed to submit dump_work to the system workqueue: %s",
                        k_work_submit_error_to_str[-ret]);
            break;
        default:
            shell_print(shell, "Failed to submit dump_work to the system workqueue: Unknown error - %d", ret);
            break;
    }

    return ret;
}

static int cmd_load_cell_tare(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_print(shell, "Usage: %s <load_cell>", argv[0]);
        return -EINVAL;
    }

    int load_cell = atoi(argv[1]);

    /* Make sure the specified tone generator exists */
    if (load_cell < 0 || load_cell >= ARRAY_SIZE(load_cell_devs)) {
        shell_print(shell, "Invalid load cell instance: %d", load_cell);
        return -EINVAL;
    }

    int ret = ll_load_cell_tare(load_cell_devs[load_cell], NULL, NULL);
    if (ret == -EBUSY) {
        shell_print(shell, "Failed to tare load_cell%d: a tare is already in progress", load_cell);
        return ret;
    }
    if (ret < 0) {
        shell_print(shell, "Failed to tare load_cell%d: %s", load_cell, k_work_submit_error_to_str[-ret]);
        return ret;
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_load_cell, SHELL_CMD_ARG(read, NULL, "Read from a load cell", cmd_load_cell_read, 2, 0),
    SHELL_CMD_ARG(dump, NULL, "Dump all readings from all load cells", cmd_load_cell_dump, 1, 0),
    SHELL_CMD_ARG(tare, NULL, "Tare a load cell", cmd_load_cell_tare, 2, 0), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(load_cell, &sub_load_cell, "Load Cell commands", NULL);
