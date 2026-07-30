#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "rfd900x.h"
#include "rfd900x_at.h"
#include "gnss_spi.h"
#include "radio_thread.h"

LOG_MODULE_REGISTER(rfd900x, LOG_LEVEL_INF);

/*
 * The modem serial belongs to the ulysses-gnss-radio board, not to FALCON --
 * the direct-UART assumption (USART2) was wrong and that port is disabled.
 * So the AT session runs over the SPI link to that board using its AT
 * passthrough opcodes (0x06-0x09), which expose the modem line as a raw byte
 * pipe. rfd900x_at.c drives it unchanged; only these two callbacks differ
 * from a local UART.
 */

/* One AT session at a time; today only the command executor calls in */
K_MUTEX_DEFINE(rfd_mutex);

/* Poll interval while waiting on modem bytes. The AT engine's timeouts are
 * hundreds of milliseconds, so a short sleep costs nothing and keeps the SPI
 * bus free for telemetry-adjacent traffic. */
#define RFD_AT_POLL_INTERVAL_MS 5

/* Bytes fetched from the board but not yet handed to the AT engine. Reads
 * come back in blocks; the engine consumes one byte at a time. */
static uint8_t rx_buf[GNSS_SPI_AT_DATA_MAX];
static size_t rx_len;
static size_t rx_pos;

static int spi_at_write(const uint8_t *data, size_t len)
{
    while (len > 0) {
        size_t chunk = MIN(len, (size_t)GNSS_SPI_AT_DATA_MAX);
        int ret = gnss_spi_at_write(data, chunk);

        if (ret < 0) {
            return ret;
        }

        data += chunk;
        len -= chunk;
    }

    return 0;
}

static int spi_at_read_byte(uint8_t *byte, int32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        if (rx_pos < rx_len) {
            *byte = rx_buf[rx_pos++];
            return 0;
        }

        int ret = gnss_spi_at_read(rx_buf, &rx_len);

        if (ret < 0) {
            return ret;
        }
        rx_pos = 0;

        if (rx_len > 0) {
            continue;
        }

        /* Checked after a fetch so a zero timeout still polls once, which is
         * what the engine's input flush relies on. */
        if (k_uptime_get() >= deadline) {
            return -EAGAIN;
        }
        k_sleep(K_MSEC(RFD_AT_POLL_INTERVAL_MS));
    }
}

static const struct rfd900x_transport spi_transport = {
    .write = spi_at_write,
    .read_byte = spi_at_read_byte,
};

int rfd900x_init(void)
{
    if (!gnss_spi_ready()) {
        LOG_ERR("GNSS/radio SPI not ready; RFD900x reconfig unavailable");
        return -ENODEV;
    }
    LOG_INF("RFD900x AT passthrough ready (via GNSS board SPI)");
    return 0;
}

int rfd900x_apply_config(const RfdConfig *cfg)
{
    if (!gnss_spi_ready()) {
        LOG_ERR("GNSS/radio SPI not ready");
        return -ENODEV;
    }

    k_mutex_lock(&rfd_mutex, K_FOREVER);

    /* Telemetry flowing into the modem serial would break the guard-time
     * silence "+++" needs, so pause downlink for the session. Suspending
     * FALCON's side first means the ENTER below finds little or nothing
     * still queued on the board. */
    radio_tx_suspend(true);

    rx_len = 0;
    rx_pos = 0;

    int ret = gnss_spi_at_enter();

    if (ret < 0) {
        LOG_ERR("Failed to open AT session on the GNSS board: %d", ret);
    } else {
        ret = rfd900x_at_apply(&spi_transport, cfg);
    }

    /* Always close the session: leaving it open would strand the modem
     * serial in raw mode and silently kill the downlink. */
    int exit_ret = gnss_spi_at_exit();

    if (exit_ret < 0) {
        LOG_ERR("Failed to close AT session: %d; radio TX may stay parked", exit_ret);
        if (ret == 0) {
            ret = exit_ret;
        }
    }

    radio_tx_suspend(false);

    k_mutex_unlock(&rfd_mutex);
    return ret;
}
