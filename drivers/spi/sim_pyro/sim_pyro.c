#define DT_DRV_COMPAT zephyr_sim_pyro

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(sim_pyro, LOG_LEVEL_INF);

// Pyro command definitions
#define PYRO_CMD_FIRE_DROGUE 0x01
#define PYRO_CMD_FIRE_MAIN 0x02
#define PYRO_CMD_STATUS_REQ 0x55

// Pyro status bit definitions
#define PYRO_STATUS_DROGUE_FIRED (1 << 0)
#define PYRO_STATUS_MAIN_FIRED (1 << 1)
#define PYRO_STATUS_DROGUE_FAIL (1 << 2)
#define PYRO_STATUS_MAIN_FAIL (1 << 3)
#define PYRO_STATUS_DROGUE_CONT_OK (1 << 4)
#define PYRO_STATUS_MAIN_CONT_OK (1 << 5)
#define PYRO_STATUS_DROGUE_FIRE_ACK (1 << 6)
#define PYRO_STATUS_MAIN_FIRE_ACK (1 << 7)

struct sim_pyro_data {
    uint8_t status_byte;
    bool drogue_fired;
    bool main_fired;
    bool drogue_fail;
    bool main_fail;
    bool drogue_cont_ok;
    bool main_cont_ok;
    bool drogue_fire_ack;
    bool main_fire_ack;
};

/**
 * @brief SPI emulator transceive function
 */
static int sim_pyro_spi_transceive(const struct emul *emul, const struct spi_config *config,
                                   const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
    struct sim_pyro_data *data = emul->data;

    if (!tx || tx->count == 0) {
        // LOG_ERR("[SIM_PYRO] No TX buffers");
        return -EINVAL;
    }

    if (!tx->buffers || !tx->buffers[0].buf) {
        // LOG_ERR("[SIM_PYRO] Invalid TX buffer pointer");
        return -EINVAL;
    }

    uint8_t cmd = *(uint8_t *)tx->buffers[0].buf;

    /* Parse command and update state */
    switch (cmd) {
    case PYRO_CMD_FIRE_DROGUE:
        data->drogue_fire_ack = true;
        data->drogue_fired = true;
        break;

    case PYRO_CMD_FIRE_MAIN:
        data->main_fire_ack = true;
        data->main_fired = true;
        break;

    case PYRO_CMD_STATUS_REQ:
        break;

    default:
        LOG_WRN("[SIM_PYRO] Unknown command: 0x%02x", cmd);
    }

    /* Build status byte, updating from previous states */
    data->status_byte = 0;
    if (data->drogue_fired) {
        data->status_byte |= PYRO_STATUS_DROGUE_FIRED;
    }
    if (data->main_fired) {
        data->status_byte |= PYRO_STATUS_MAIN_FIRED;
    }
    if (data->drogue_fire_ack) {
        data->status_byte |= PYRO_STATUS_DROGUE_FIRE_ACK;
    }
    if (data->main_fire_ack) {
        data->status_byte |= PYRO_STATUS_MAIN_FIRE_ACK;
    }
    if (data->drogue_fail) {
        data->status_byte |= PYRO_STATUS_DROGUE_FAIL;
    }
    if (data->main_fail) {
        data->status_byte |= PYRO_STATUS_MAIN_FAIL;
    }

    /* Always indicate continuity is OK for simulation */
    data->status_byte |= PYRO_STATUS_DROGUE_CONT_OK;
    data->status_byte |= PYRO_STATUS_MAIN_CONT_OK;

    /* Return status byte in RX buffer if provided */
    if (rx && rx->count > 0) {
        if (!rx->buffers || !rx->buffers[0].buf) {
            LOG_ERR("[SIM_PYRO] Invalid RX buffer pointer");
            return -EINVAL;
        }
        uint8_t *rx_buf = (uint8_t *)rx->buffers[0].buf;
        rx_buf[0] = data->status_byte;
    }

    return 0;
}

static const struct spi_emul_api sim_pyro_spi_api = {
    .io = sim_pyro_spi_transceive,
};

static int sim_pyro_emul_init(const struct emul *emul, const struct device *parent)
{
    struct sim_pyro_data *data = emul->data;

    data->status_byte = PYRO_STATUS_DROGUE_CONT_OK | PYRO_STATUS_MAIN_CONT_OK;
    data->drogue_fired = false;
    data->main_fired = false;
    data->drogue_fire_ack = false;
    data->main_fire_ack = false;

    LOG_INF("[SIM_PYRO] Emulator initialized (parent=%s)", parent->name);

    return 0;
}

/* Device init function - creates the actual device */
static int sim_pyro_device_init(const struct device *dev)
{
    LOG_INF("[SIM_PYRO] Device initialized");
    return 0;
}

#define SIM_PYRO_INIT(inst)                                                                       \
    static struct sim_pyro_data sim_pyro_data_##inst = {                                          \
        .status_byte = PYRO_STATUS_DROGUE_CONT_OK | PYRO_STATUS_MAIN_CONT_OK,                     \
        .drogue_fired = false,                                                                    \
        .main_fired = false,                                                                      \
        .drogue_fire_ack = false,                                                                 \
        .main_fire_ack = false,                                                                   \
    };                                                                                            \
    /* Create the actual SPI device */                                                            \
    SPI_DEVICE_DT_INST_DEFINE(inst, sim_pyro_device_init, NULL, NULL, NULL, POST_KERNEL,          \
                              CONFIG_SPI_INIT_PRIORITY, NULL);                                    \
    /* Create the emulator */                                                                     \
    EMUL_DT_INST_DEFINE(inst, sim_pyro_emul_init, &sim_pyro_data_##inst, NULL, &sim_pyro_spi_api, \
                        NULL);

DT_INST_FOREACH_STATUS_OKAY(SIM_PYRO_INIT)
