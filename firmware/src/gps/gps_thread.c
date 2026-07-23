#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <lwgps/lwgps.h>
#include <string.h>
#include "data.h"
#include "../radio/gnss_spi.h"

LOG_MODULE_REGISTER(gps_thread, LOG_LEVEL_INF);

#define GPS_THREAD_STACK_SIZE 2048
#define GPS_THREAD_PRIORITY 6
#define GPS_THREAD_PERIOD_MS 1000

#define GPS_PAYLOAD_SIZE GNSS_SPI_GPS_PAYLOAD_SIZE /* Max NMEA sentence length */

K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
static struct k_thread gps_thread;

static lwgps_t gps;

static void gps_thread_fn(void *p1, void *p2, void *p3)
{
	if (!gnss_spi_ready()) {
		LOG_ERR("GPS SPI device not ready");
		return;
	}
	LOG_INF("GPS SPI device ready");

	lwgps_init(&gps);

	while (1) {
		uint8_t payload[GPS_PAYLOAD_SIZE];

		int ret = gnss_spi_gps_read(payload);
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
