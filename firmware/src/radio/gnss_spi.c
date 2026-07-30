#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "gnss_spi.h"
#include "uplink_config.h"

LOG_MODULE_REGISTER(gnss_spi, LOG_LEVEL_INF);

/* SPI protocol opcodes (ulysses-gnss-radio spec) */
#define SPI_CMD_RADIO_TX 0x04
#define SPI_CMD_GPS_RX 0x05
/* Radio RX opcode is a placeholder, see uplink_config.h */

/* AT passthrough family (ulysses-gnss-radio Core/Inc/protocol_config.h) */
#define SPI_CMD_RADIO_AT_ENTER 0x06
#define SPI_CMD_RADIO_AT_TX 0x07
#define SPI_CMD_RADIO_AT_RX 0x08
#define SPI_CMD_RADIO_AT_EXIT 0x09

#define SPI_RADIO_TX_SIZE (GNSS_SPI_HEADER_SIZE + GNSS_SPI_MAX_COBS_SIZE)  /* 261 */
#define SPI_GPS_RX_SIZE (GNSS_SPI_HEADER_SIZE + GNSS_SPI_GPS_PAYLOAD_SIZE) /* 92 */
#define SPI_RADIO_RX_SIZE (GNSS_SPI_HEADER_SIZE + GNSS_SPI_MAX_COBS_SIZE)  /* 261 */

/* AT payload is [LEN:1][DATA:64]; control opcodes carry no payload at all */
#define SPI_AT_PAYLOAD_SIZE (1 + GNSS_SPI_AT_DATA_MAX)                  /* 65 */
#define SPI_AT_SIZE (GNSS_SPI_HEADER_SIZE + SPI_AT_PAYLOAD_SIZE)        /* 70 */
#define SPI_AT_CTRL_SIZE GNSS_SPI_HEADER_SIZE                           /* 5 */

static const struct spi_dt_spec gnss_spi =
    SPI_DT_SPEC_GET(DT_ALIAS(radio0), SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8));

/* Serializes the three consumers of the GNSS/radio board (telemetry TX,
 * GPS RX, radio command RX). */
K_MUTEX_DEFINE(gnss_spi_mutex);

/*
 * Transaction buffers live here rather than on the caller's stack: at 261
 * bytes each they are far too big for it. The AT session in particular
 * reaches this code five frames below rfd900x_at_apply(), and the pair
 * overflowed cmd_exec's stack outright.
 *
 * Every transaction already runs under gnss_spi_mutex, so sharing them is
 * safe -- but only if all reads and writes happen inside the lock, payload
 * copy-out included.
 */
static uint8_t spi_tx_buf[SPI_RADIO_TX_SIZE];
static uint8_t spi_rx_buf[SPI_RADIO_RX_SIZE];

bool gnss_spi_ready(void)
{
    return spi_is_ready_dt(&gnss_spi);
}

int gnss_spi_radio_tx(const uint8_t *cobs_data, size_t cobs_len)
{
    if (cobs_len > GNSS_SPI_MAX_COBS_SIZE) {
        return -EINVAL;
    }

    const struct spi_buf spi_tx = {
        .buf = spi_tx_buf,
        .len = SPI_RADIO_TX_SIZE,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &spi_tx,
        .count = 1,
    };

    k_mutex_lock(&gnss_spi_mutex, K_FOREVER);

    memset(spi_tx_buf, 0, SPI_RADIO_TX_SIZE);
    spi_tx_buf[0] = SPI_CMD_RADIO_TX;
    /* bytes 1-4 are dummy, remaining payload zero-padded by the memset */
    memcpy(&spi_tx_buf[GNSS_SPI_HEADER_SIZE], cobs_data, cobs_len);

    int ret = spi_write_dt(&gnss_spi, &tx_set);

    k_mutex_unlock(&gnss_spi_mutex);

    return ret;
}

/**
 * @brief Full-duplex read transaction: [CMD][dummy][zeros] out, payload in
 */
static int gnss_spi_read(uint8_t cmd, uint8_t *payload_out, size_t payload_size)
{
    size_t transfer_size = GNSS_SPI_HEADER_SIZE + payload_size;

    if (transfer_size > SPI_RADIO_RX_SIZE) {
        return -EINVAL;
    }

    const struct spi_buf spi_tx = {
        .buf = spi_tx_buf,
        .len = transfer_size,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &spi_tx,
        .count = 1,
    };

    const struct spi_buf spi_rx = {
        .buf = spi_rx_buf,
        .len = transfer_size,
    };
    const struct spi_buf_set rx_set = {
        .buffers = &spi_rx,
        .count = 1,
    };

    k_mutex_lock(&gnss_spi_mutex, K_FOREVER);

    memset(spi_tx_buf, 0, transfer_size);
    spi_tx_buf[0] = cmd;
    /* bytes 1-4 are dummy, rest is zero (master clocks out zeros) */

    int ret = spi_transceive_dt(&gnss_spi, &tx_set, &rx_set);

    /* Copy out under the lock -- the buffers are shared */
    if (ret >= 0) {
        memcpy(payload_out, &spi_rx_buf[GNSS_SPI_HEADER_SIZE], payload_size);
    }

    k_mutex_unlock(&gnss_spi_mutex);

    return ret < 0 ? ret : 0;
}

int gnss_spi_gps_read(uint8_t *payload_out)
{
    return gnss_spi_read(SPI_CMD_GPS_RX, payload_out, GNSS_SPI_GPS_PAYLOAD_SIZE);
}

int gnss_spi_radio_rx(uint8_t *payload_out)
{
    return gnss_spi_read(UPLINK_SPI_CMD_RADIO_RX, payload_out, GNSS_SPI_MAX_COBS_SIZE);
}

/**
 * @brief Payload-less write transaction: just the opcode and dummy bytes
 */
static int gnss_spi_command(uint8_t cmd)
{
    const struct spi_buf spi_tx = {
        .buf = spi_tx_buf,
        .len = SPI_AT_CTRL_SIZE,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &spi_tx,
        .count = 1,
    };

    k_mutex_lock(&gnss_spi_mutex, K_FOREVER);

    memset(spi_tx_buf, 0, SPI_AT_CTRL_SIZE);
    spi_tx_buf[0] = cmd;

    int ret = spi_write_dt(&gnss_spi, &tx_set);

    k_mutex_unlock(&gnss_spi_mutex);

    return ret;
}

int gnss_spi_at_enter(void)
{
    return gnss_spi_command(SPI_CMD_RADIO_AT_ENTER);
}

int gnss_spi_at_exit(void)
{
    return gnss_spi_command(SPI_CMD_RADIO_AT_EXIT);
}

int gnss_spi_at_write(const uint8_t *data, size_t len)
{
    if (len == 0 || len > GNSS_SPI_AT_DATA_MAX) {
        return -EINVAL;
    }

    const struct spi_buf spi_tx = {
        .buf = spi_tx_buf,
        .len = SPI_AT_SIZE,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &spi_tx,
        .count = 1,
    };

    k_mutex_lock(&gnss_spi_mutex, K_FOREVER);

    memset(spi_tx_buf, 0, SPI_AT_SIZE);
    spi_tx_buf[0] = SPI_CMD_RADIO_AT_TX;
    /* bytes 1-4 are dummy; payload is [LEN:1][DATA:N] zero-padded to 64 */
    spi_tx_buf[GNSS_SPI_HEADER_SIZE] = (uint8_t)len;
    memcpy(&spi_tx_buf[GNSS_SPI_HEADER_SIZE + 1], data, len);

    int ret = spi_write_dt(&gnss_spi, &tx_set);

    k_mutex_unlock(&gnss_spi_mutex);

    return ret;
}

int gnss_spi_at_read(uint8_t *data_out, size_t *len_out)
{
    uint8_t payload[SPI_AT_PAYLOAD_SIZE] = {0};

    int ret = gnss_spi_read(SPI_CMD_RADIO_AT_RX, payload, sizeof(payload));

    if (ret < 0) {
        return ret;
    }

    uint8_t len = payload[0];

    /* The board never reports more than it can carry, but a bad length here
     * would overrun the caller's buffer -- treat it as a bus fault. */
    if (len > GNSS_SPI_AT_DATA_MAX) {
        LOG_ERR("AT read reported %u bytes, max is %u", len, GNSS_SPI_AT_DATA_MAX);
        return -EIO;
    }

    memcpy(data_out, &payload[1], len);
    *len_out = len;

    return 0;
}
