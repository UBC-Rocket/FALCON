#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "pyro_thread.h"
#include "data.h"

LOG_MODULE_REGISTER(pyro_thread, LOG_LEVEL_INF);

// Thread configuration
#define PYRO_THREAD_STACK_SIZE 2048
#define PYRO_THREAD_PRIORITY 0
#define PYRO_STATUS_POLL_INTERVAL_MS 100

// Thread stack and data
K_THREAD_STACK_DEFINE(pyro_stack, PYRO_THREAD_STACK_SIZE);
static struct k_thread pyro_thread_data;

// Message queue for pyro commands
#define PYRO_CMD_QUEUE_SIZE 10
K_MSGQ_DEFINE(pyro_cmd_queue, sizeof(uint8_t), PYRO_CMD_QUEUE_SIZE, 4);

// SPI device configuration
static const struct spi_dt_spec pyro_spi =
    SPI_DT_SPEC_GET(DT_ALIAS(pyro0), SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8));

/**
 * @brief Send a command to the pyro board and receive status
 * @param cmd Command byte to send
 * @param status_out Pointer to store received status byte (can be NULL)
 * @return 0 on success, negative errno on failure
 */
static int pyro_spi_transact(uint8_t cmd, uint8_t *status_out)
{
    int ret;
    uint8_t tx_buf = cmd;
    uint8_t rx_buf = 0;

    const struct spi_buf tx_spi_buf = {
        .buf = &tx_buf,
        .len = sizeof(tx_buf),
    };

    const struct spi_buf rx_spi_buf = {
        .buf = &rx_buf,
        .len = sizeof(rx_buf),
    };

    const struct spi_buf_set tx_set = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };

    const struct spi_buf_set rx_set = {
        .buffers = &rx_spi_buf,
        .count = 1,
    };

    // Log what we're about to send
    LOG_DBG("SPI TX: 0x%02x", tx_buf);

    ret = spi_transceive_dt(&pyro_spi, &tx_set, &rx_set);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }

    // Log what we received
    LOG_DBG("SPI RX: 0x%02x", rx_buf);

    if (status_out) {
        *status_out = rx_buf;
    }

    return 0;
}

/**
 * @brief Parse status byte and update status structure
 */
static void parse_status_byte(uint8_t status_byte, struct pyro_data *status)
{
    status->status_byte = status_byte;
    status->timestamp = k_uptime_get();
    status->drogue_fired = (status_byte & PYRO_STATUS_DROGUE_FIRED) != 0;
    status->main_fired = (status_byte & PYRO_STATUS_MAIN_FIRED) != 0;
    status->drogue_fail = (status_byte & PYRO_STATUS_DROGUE_FAIL) != 0;
    status->main_fail = (status_byte & PYRO_STATUS_MAIN_FAIL) != 0;
    status->drogue_cont_ok = (status_byte & PYRO_STATUS_DROGUE_CONT_OK) != 0;
    status->main_cont_ok = (status_byte & PYRO_STATUS_MAIN_CONT_OK) != 0;
    status->drogue_fire_ack = (status_byte & PYRO_STATUS_DROGUE_FIRE_ACK) != 0;
    status->main_fire_ack = (status_byte & PYRO_STATUS_MAIN_FIRE_ACK) != 0;
}

/**
 * @brief Request status from pyro board
 */
static int request_pyro_status(void)
{
    uint8_t status_byte;
    int ret = pyro_spi_transact(PYRO_CMD_STATUS_REQ, &status_byte);

    if (ret == 0) {
        struct pyro_data new_status;
        get_pyro_data(&new_status);
        parse_status_byte(status_byte, &new_status);
        set_pyro_data(&new_status);

        LOG_DBG("Pyro status: 0x%02x [D:%d M:%d DF:%d MF:%d DC:%d MC:%d DA:%d MA:%d]", status_byte,
                new_status.drogue_fired, new_status.main_fired, new_status.drogue_fail,
                new_status.main_fail, new_status.drogue_cont_ok, new_status.main_cont_ok,
                new_status.drogue_fire_ack, new_status.main_fire_ack);
    }

    return ret;
}

/**
 * @brief Check if a fire command has been acknowledged
 * @param cmd Command that was sent
 * @param status Status byte received from pyro board
 * @return true if command was acknowledged
 */
static bool is_fire_command_acked(uint8_t cmd, const struct pyro_data *status)
{
    if (cmd == PYRO_CMD_FIRE_DROGUE) {
        return status->drogue_fire_ack;
    } else if (cmd == PYRO_CMD_FIRE_MAIN) {
        return status->main_fire_ack;
    }
    return true;
}

/**
 * @brief Check if a fire command has completed (success or failure)
 * @param cmd Command that was sent
 * @param status Status received from pyro board
 * @return true if command completed, false if still in progress
 */
static bool is_fire_command_complete(uint8_t cmd, const struct pyro_data *status)
{
    if (cmd == PYRO_CMD_FIRE_DROGUE) {
        return status->drogue_fired || status->drogue_fail;
    } else if (cmd == PYRO_CMD_FIRE_MAIN) {
        return status->main_fired || status->main_fail;
    }
    return true;
}

/**
 * @brief Log the result of a fire command
 * @param cmd Command that was executed
 * @param status Final status received
 * @param retry_count Number of retries that were needed
 */
static void log_fire_result(uint8_t cmd, const struct pyro_data *status, int retry_count)
{
    if (cmd == PYRO_CMD_FIRE_DROGUE) {
        if (status->drogue_fired) {
            LOG_INF("DROGUE FIRED (attempt %d)", retry_count + 1);
        } else if (status->drogue_fail) {
            LOG_ERR("DROGUE FIRE FAILED (attempt %d)", retry_count + 1);
        }
    } else if (cmd == PYRO_CMD_FIRE_MAIN) {
        if (status->main_fired) {
            LOG_INF("MAIN FIRED (attempt %d)", retry_count + 1);
        } else if (status->main_fail) {
            LOG_ERR("MAIN FIRE FAILED (attempt %d)", retry_count + 1);
        }
    }
}

/**
 * @brief Execute a pyro command with retry logic
 * @param cmd Command to execute
 */
static void execute_pyro_command(uint8_t cmd)
{
    LOG_INF("Executing pyro command: 0x%02x", cmd);

    bool acked = false;
    int retry_count = 0;
    const int MAX_RETRIES = 100; // Retry for ~1 second (10ms per attempt)

    // Keep sending command until we get an ACK
    while (!acked && retry_count < MAX_RETRIES) {
        uint8_t status_byte;
        int ret = pyro_spi_transact(cmd, &status_byte);

        if (ret == 0) {
            struct pyro_data new_status;
            get_pyro_data(&new_status);
            parse_status_byte(status_byte, &new_status);
            set_pyro_data(&new_status);

            // Check if command was acknowledged
            if (is_fire_command_acked(cmd, &new_status)) {
                LOG_INF("Pyro command 0x%02x acknowledged (attempt %d)", cmd, retry_count + 1);
                acked = true;

                // Log immediate result if available
                if (is_fire_command_complete(cmd, &new_status)) {
                    log_fire_result(cmd, &new_status, retry_count);
                }
            }
        } else {
            LOG_ERR("SPI error on pyro command 0x%02x: %d", cmd, ret);
        }

        if (!acked) {
            retry_count++;
            k_sleep(K_MSEC(10));
        }
    }

    if (!acked) {
        LOG_ERR("Pyro command 0x%02x not acknowledged after %d attempts", cmd, retry_count);
    }
}

/**
 * @brief Pyro thread main function
 */
static void pyro_thread_fn(void *p1, void *p2, void *p3)
{
    LOG_INF("Pyro thread started");

    // Check if SPI device is ready
    if (!spi_is_ready_dt(&pyro_spi)) {
        LOG_ERR("Pyro SPI device not ready");
        return;
    }
    LOG_INF("Pyro SPI device ready");

    // Initial status request
    request_pyro_status();

    while (1) {
        uint8_t cmd;
        int ret;

        // Wait for commands (cpu is yielding) or timeout after 100ms
        ret = k_msgq_get(&pyro_cmd_queue, &cmd, K_MSEC(PYRO_STATUS_POLL_INTERVAL_MS));

        if (ret == 0) {
            // Command received - execute it with retry logic
            execute_pyro_command(cmd);
        }

        // Poll status
        request_pyro_status();
    }
}

/**
 * @brief Send a command to the pyro thread
 */
static int send_pyro_command(uint8_t cmd)
{
    int ret = k_msgq_put(&pyro_cmd_queue, &cmd, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Failed to queue pyro command 0x%02x: %d", cmd, ret);
        return ret;
    }
    return 0;
}

// Public API functions

void start_pyro_thread(void)
{
    k_thread_create(&pyro_thread_data, pyro_stack, K_THREAD_STACK_SIZEOF(pyro_stack),
                    pyro_thread_fn, NULL, NULL, NULL, PYRO_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&pyro_thread_data, "pyro");
}

int pyro_fire_drogue(void)
{
    LOG_INF("Drogue fire command requested");
    struct pyro_data pd;
    get_pyro_data(&pd);
    pd.drogue_fire_requested = true;
    set_pyro_data(&pd);
    return send_pyro_command(PYRO_CMD_FIRE_DROGUE);
}

int pyro_fire_main(void)
{
    LOG_INF("Main fire command requested");
    struct pyro_data pd;
    get_pyro_data(&pd);
    pd.main_fire_requested = true;
    set_pyro_data(&pd);
    return send_pyro_command(PYRO_CMD_FIRE_MAIN);
}
