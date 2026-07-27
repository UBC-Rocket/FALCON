#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "rfd900x.h"
#include "rfd900x_at.h"
#include "radio_thread.h"

LOG_MODULE_REGISTER(rfd900x, LOG_LEVEL_INF);

#if DT_NODE_HAS_STATUS(DT_ALIAS(rfd_uart), okay)

static const struct device *const rfd_uart = DEVICE_DT_GET(DT_ALIAS(rfd_uart));

/* One AT session at a time; today only the command executor calls in */
K_MUTEX_DEFINE(rfd_mutex);

static int rfd_uart_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(rfd_uart, data[i]);
    }
    return 0;
}

static int rfd_uart_read_byte(uint8_t *byte, int32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        unsigned char c;

        if (uart_poll_in(rfd_uart, &c) == 0) {
            *byte = c;
            return 0;
        }
        if (k_uptime_get() >= deadline) {
            return -EAGAIN;
        }
        k_sleep(K_MSEC(1));
    }
}

static const struct rfd900x_transport uart_transport = {
    .write = rfd_uart_write,
    .read_byte = rfd_uart_read_byte,
};

int rfd900x_init(void)
{
    if (!device_is_ready(rfd_uart)) {
        LOG_ERR("RFD900x UART not ready");
        return -ENODEV;
    }
    LOG_INF("RFD900x UART ready");
    return 0;
}

int rfd900x_apply_config(const RfdConfig *cfg)
{
    if (!device_is_ready(rfd_uart)) {
        LOG_ERR("RFD900x UART not ready");
        return -ENODEV;
    }

    k_mutex_lock(&rfd_mutex, K_FOREVER);

    /* Telemetry flowing into the modem serial would break the guard-time
     * silence "+++" needs, so pause downlink for the session. */
    radio_tx_suspend(true);
    int ret = rfd900x_at_apply(&uart_transport, cfg);
    radio_tx_suspend(false);

    k_mutex_unlock(&rfd_mutex);
    return ret;
}

#else /* rfd-uart absent or disabled (native_sim, or hardware team says
       * UART2 must stay off -- the board dts has no usart2 node and PD6 is
       * the vtx_pwr GPIO instead): simulate the modem */

int rfd900x_init(void)
{
    LOG_WRN("No usable rfd-uart devicetree alias; simulating RFD900x config");
    return 0;
}

int rfd900x_apply_config(const RfdConfig *cfg)
{
    LOG_INF("(sim) RFD900x reconfig: air_speed=%s%u net_id=%s%u tx_power=%s%u "
            "min_freq=%s%u max_freq=%s%u num_channels=%s%u",
            cfg->has_air_speed_kbps ? "" : "(unset) ", cfg->air_speed_kbps,
            cfg->has_net_id ? "" : "(unset) ", cfg->net_id, cfg->has_tx_power_dbm ? "" : "(unset) ",
            cfg->tx_power_dbm, cfg->has_min_freq_khz ? "" : "(unset) ", cfg->min_freq_khz,
            cfg->has_max_freq_khz ? "" : "(unset) ", cfg->max_freq_khz,
            cfg->has_num_channels ? "" : "(unset) ", cfg->num_channels);
    return 0;
}

#endif
