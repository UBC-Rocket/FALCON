#define DT_DRV_COMPAT zephyr_sim_radio

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(sim_radio, LOG_LEVEL_INF);

/* SPI protocol opcodes (ulysses-gnss-radio spec) */
#define SPI_CMD_RADIO_TX 0x04
#define SPI_CMD_GPS_RX 0x05

/**
 * @brief SPI emulator transceive function.
 *
 * Minimal stand-in for the GNSS/radio board: telemetry TX is accepted and
 * counted, and every read (GPS RX, radio command RX) returns an all-zero
 * payload, which the firmware treats as "nothing pending".
 */
static int sim_radio_spi_transceive(const struct emul *emul, const struct spi_config *config,
                                    const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
    uint32_t *tx_count = emul->data;

    if (!tx || tx->count == 0 || !tx->buffers || !tx->buffers[0].buf) {
        return -EINVAL;
    }

    uint8_t cmd = *(uint8_t *)tx->buffers[0].buf;

    if (cmd == SPI_CMD_RADIO_TX) {
        (*tx_count)++;
        LOG_DBG("[SIM_RADIO] Telemetry frame %u accepted", *tx_count);
    }

    if (rx && rx->count > 0) {
        if (!rx->buffers || !rx->buffers[0].buf) {
            LOG_ERR("[SIM_RADIO] Invalid RX buffer pointer");
            return -EINVAL;
        }
        memset(rx->buffers[0].buf, 0, rx->buffers[0].len);
    }

    return 0;
}

static const struct spi_emul_api sim_radio_spi_api = {
    .io = sim_radio_spi_transceive,
};

static int sim_radio_emul_init(const struct emul *emul, const struct device *parent)
{
    LOG_INF("[SIM_RADIO] Emulator initialized (parent=%s)", parent->name);
    return 0;
}

static int sim_radio_device_init(const struct device *dev)
{
    LOG_INF("[SIM_RADIO] Device initialized");
    return 0;
}

#define SIM_RADIO_INIT(inst)                                                              \
    static uint32_t sim_radio_tx_count_##inst;                                            \
    /* Create the actual SPI device */                                                    \
    SPI_DEVICE_DT_INST_DEFINE(inst, sim_radio_device_init, NULL, NULL, NULL, POST_KERNEL, \
                              CONFIG_SPI_INIT_PRIORITY, NULL);                            \
    /* Create the emulator */                                                             \
    EMUL_DT_INST_DEFINE(inst, sim_radio_emul_init, &sim_radio_tx_count_##inst, NULL,      \
                        &sim_radio_spi_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(SIM_RADIO_INIT)
