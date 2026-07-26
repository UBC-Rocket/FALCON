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

#endif /* GNSS_SPI_H */
