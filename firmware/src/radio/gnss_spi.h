#ifndef GNSS_SPI_H
#define GNSS_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Shared access to the ulysses-gnss-radio board on SPI1 (DT_ALIAS(radio0)).
 * The bus now has three consumers (telemetry TX, GPS RX, radio command RX),
 * so all transactions go through this module, serialized by a mutex.
 */

/* SPI framing (ulysses-gnss-radio spec): [CMD:1][DUMMY:4][PAYLOAD:N] */
#define GNSS_SPI_DUMMY_SIZE 4
#define GNSS_SPI_HEADER_SIZE (1 + GNSS_SPI_DUMMY_SIZE)

/* Max COBS-encoded payload for radio TX/RX transactions */
#define GNSS_SPI_MAX_COBS_SIZE 256

/* Max NMEA sentence length for GPS reads */
#define GNSS_SPI_GPS_PAYLOAD_SIZE 87

/* Max raw bytes per AT passthrough transaction (payload is [LEN:1][DATA:N]) */
#define GNSS_SPI_AT_DATA_MAX 64

/**
 * @brief Check that the GNSS/radio SPI device is ready
 */
bool gnss_spi_ready(void);

/**
 * @brief Send a COBS-encoded telemetry frame to the radio (opcode 0x04)
 * @param cobs_data COBS-encoded frame (zero-padded to the full payload size)
 * @param cobs_len Length of the encoded frame, at most GNSS_SPI_MAX_COBS_SIZE
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_radio_tx(const uint8_t *cobs_data, size_t cobs_len);

/**
 * @brief Read the latest NMEA sentence from the GNSS (opcode 0x05)
 * @param payload_out Buffer of GNSS_SPI_GPS_PAYLOAD_SIZE bytes, all zeros if
 *                    no sentence is pending
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_gps_read(uint8_t *payload_out);

/**
 * @brief Read received radio bytes (uplink) from the radio board
 * @param payload_out Buffer of GNSS_SPI_MAX_COBS_SIZE bytes, all zeros if
 *                    nothing is pending
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_radio_rx(uint8_t *payload_out);

/*
 * RFD900x AT passthrough (opcodes 0x06-0x09).
 *
 * FALCON has no direct line to the modem serial -- the GNSS board owns it.
 * These four calls turn that line into a raw byte pipe for the duration of
 * an AT session so rfd900x_at.c can drive the modem through it unchanged.
 *
 * Ordinary radio traffic is null-terminated and the board splits it on 0x00,
 * but AT commands and their replies contain no 0x00, so they cannot travel
 * as radio messages. Hence a separate opcode family rather than reusing 0x04.
 */

/**
 * @brief Open an AT session (opcode 0x06)
 *
 * The GNSS board stops writing telemetry to the modem serial and starts
 * capturing raw received bytes. Also drops radio frames already queued on
 * the board, so nothing lands in the guard-time silence that follows.
 *
 * Must be paired with gnss_spi_at_exit() even on failure.
 *
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_at_enter(void);

/**
 * @brief Close an AT session (opcode 0x09), resuming normal radio traffic
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_at_exit(void);

/**
 * @brief Write raw bytes to the modem serial verbatim (opcode 0x07)
 *
 * No terminator, no framing -- "+++" goes out as exactly three bytes.
 *
 * @param data Bytes to send
 * @param len Byte count, at most GNSS_SPI_AT_DATA_MAX
 * @return 0 on success, -EINVAL if len is out of range, negative errno on
 *         SPI failure
 */
int gnss_spi_at_write(const uint8_t *data, size_t len);

/**
 * @brief Read raw bytes received from the modem (opcode 0x08)
 *
 * Non-blocking: returns 0 with *len_out == 0 when nothing has arrived yet.
 *
 * @param data_out Buffer of GNSS_SPI_AT_DATA_MAX bytes
 * @param len_out Set to the number of valid bytes in data_out
 * @return 0 on success, negative errno on failure
 */
int gnss_spi_at_read(uint8_t *data_out, size_t *len_out);

#endif /* GNSS_SPI_H */
