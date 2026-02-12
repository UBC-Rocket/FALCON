#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <lwgps/lwgps.h>
#include <string.h>
#include "data.h"

LOG_MODULE_REGISTER(gps_thread, LOG_LEVEL_INF);

#define GPS_THREAD_STACK_SIZE 2048
#define GPS_THREAD_PRIORITY 6
#define GPS_THREAD_PERIOD_MS 1000

/* SPI protocol constants (ulysses-gnss-radio spec) */
#define SPI_CMD_GPS_RX    0x05
#define SPI_DUMMY_SIZE    4
#define SPI_HEADER_SIZE   (1 + SPI_DUMMY_SIZE) /* 5: cmd + dummy bytes */
#define GPS_PAYLOAD_SIZE  87   /* Max NMEA sentence length */

/* Total SPI transaction: [CMD:1][DUMMY:4][PAYLOAD:87] = 92 bytes */
#define SPI_GPS_RX_SIZE   (SPI_HEADER_SIZE + GPS_PAYLOAD_SIZE) /* 92 */

K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
static struct k_thread gps_thread;

static const struct spi_dt_spec gps_spi =
	SPI_DT_SPEC_GET(DT_ALIAS(radio0),
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8));

static lwgps_t gps;

static int gps_spi_read(uint8_t *payload_out)
{
	uint8_t tx_buf[SPI_GPS_RX_SIZE] = {0};
	uint8_t rx_buf[SPI_GPS_RX_SIZE] = {0};

	tx_buf[0] = SPI_CMD_GPS_RX;
	/* bytes 1-4 are dummy, rest is zero (master clocks out zeros) */

	const struct spi_buf spi_tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &spi_tx,
		.count = 1,
	};

	const struct spi_buf spi_rx = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	const struct spi_buf_set rx_set = {
		.buffers = &spi_rx,
		.count = 1,
	};

	int ret = spi_transceive_dt(&gps_spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	/* Payload starts after header (cmd + dummy bytes) */
	memcpy(payload_out, &rx_buf[SPI_HEADER_SIZE], GPS_PAYLOAD_SIZE);

	return 0;
}

static void gps_thread_fn(void *p1, void *p2, void *p3)
{
	if (!spi_is_ready_dt(&gps_spi)) {
		LOG_ERR("GPS SPI device not ready");
		return;
	}
	LOG_INF("GPS SPI device ready");

	lwgps_init(&gps);

	while (1) {
		uint8_t payload[GPS_PAYLOAD_SIZE];

		int ret = gps_spi_read(payload);
		if (ret < 0) {
			LOG_ERR("GPS SPI read failed: %d", ret);
			k_sleep(K_MSEC(GPS_THREAD_PERIOD_MS));
			continue;
		}

		/* Check for empty payload (all zero bytes from SPI) */
		bool payload_empty = true;
		for (size_t i = 0; i < GPS_PAYLOAD_SIZE; i++) {
			if (payload[i] != 0) {
				payload_empty = false;
				break;
			}
		}
		if (payload_empty) {
			LOG_WRN("GPS SPI payload is empty (all zeros)");
			k_sleep(K_MSEC(GPS_THREAD_PERIOD_MS));
			continue;
		}

		/* Null-terminate and find actual sentence length */
		char nmea[GPS_PAYLOAD_SIZE + 1];
		memcpy(nmea, payload, GPS_PAYLOAD_SIZE);
		nmea[GPS_PAYLOAD_SIZE] = '\0';
		size_t len = strlen(nmea);

		if (len == 0 || nmea[0] != '$') {
			k_sleep(K_MSEC(GPS_THREAD_PERIOD_MS));
			continue;
		}

		lwgps_process(&gps, nmea, len);

		struct gps_data gps_out = {
			.latitude = gps.latitude,
			.longitude = gps.longitude,
			.altitude = gps.altitude,
			.speed = gps.speed,
			.sats = gps.sats_in_use,
			.fix = gps.fix,
			.timestamp = k_uptime_get(),
		};
		set_gps_data(&gps_out);

		LOG_INF("NMEA: %s", nmea);
		LOG_INF("GPS: lat=%.6f, lon=%.6f, alt=%.1f m, "
			"sats=%u, fix=%u, speed=%.1f kn",
			(double)gps.latitude, (double)gps.longitude,
			(double)gps.altitude, gps.sats_in_use, gps.fix,
			(double)gps.speed);

		k_sleep(K_MSEC(GPS_THREAD_PERIOD_MS));
	}
}

void start_gps_thread(void)
{
	k_thread_create(&gps_thread, gps_stack, K_THREAD_STACK_SIZEOF(gps_stack),
			gps_thread_fn, NULL, NULL, NULL, GPS_THREAD_PRIORITY, 0, K_NO_WAIT);
}
