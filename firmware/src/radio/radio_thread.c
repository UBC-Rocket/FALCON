#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/data/cobs.h>
#include <zephyr/net_buf.h>
#include <pb_encode.h>
#include <TelemetryPacket.pb.h>
#include "data.h"

LOG_MODULE_REGISTER(radio_thread, LOG_LEVEL_INF);

#define RADIO_THREAD_STACK_SIZE 2048
#define RADIO_THREAD_PRIORITY 5
#define RADIO_THREAD_PERIOD_MS 1000

/* SPI protocol constants (ulysses-gnss-radio spec) */
#define SPI_CMD_RADIO_TX 0x04
#define SPI_DUMMY_SIZE   4
#define SPI_HEADER_SIZE  (1 + SPI_DUMMY_SIZE) /* 5: cmd + dummy bytes */

/* COBS/framing sizing */
#define MAX_COBS_SIZE    256 /* max size of COBS encoded data, defined by GNSS/RADIO SPI spec */
#define MAX_FRAME_SIZE   ((MAX_COBS_SIZE - 2) * 254 / 255) /* 253 */
#define MAX_PAYLOAD_SIZE (MAX_FRAME_SIZE - sizeof(uint16_t)) /* 251 */

/* Total SPI transaction: [CMD:1][DUMMY:4][PAYLOAD:256] = 261 bytes */
#define SPI_TX_SIZE (SPI_HEADER_SIZE + MAX_COBS_SIZE) /* 261 */

NET_BUF_POOL_DEFINE(cobs_src_pool, 1, MAX_FRAME_SIZE, 0, NULL);
NET_BUF_POOL_DEFINE(cobs_dst_pool, 1, MAX_COBS_SIZE, 0, NULL);

K_THREAD_STACK_DEFINE(radio_stack, RADIO_THREAD_STACK_SIZE);
static struct k_thread radio_thread;

BUILD_ASSERT(MAX_FRAME_SIZE + MAX_FRAME_SIZE / 254 + 2 <= MAX_COBS_SIZE,
	     "MAX_FRAME_SIZE too large for MAX_COBS_SIZE");

static const struct spi_dt_spec radio_spi =
	SPI_DT_SPEC_GET(DT_ALIAS(radio0),
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8));

static int radio_spi_send(const uint8_t *cobs_data, size_t cobs_len)
{
	uint8_t tx_buf[SPI_TX_SIZE] = {0};

	tx_buf[0] = SPI_CMD_RADIO_TX;
	/* bytes 1-4 are dummy (already zeroed) */
	// memcpy(&tx_buf[SPI_HEADER_SIZE], cobs_data, cobs_len);
	memcpy(&tx_buf[SPI_HEADER_SIZE], cobs_data, cobs_len);
	/* remaining payload bytes are zero-padded (already zeroed) */

	const struct spi_buf spi_tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &spi_tx,
		.count = 1,
	};

	return spi_write_dt(&radio_spi, &tx_set);
}

static void radio_thread_fn(void *p1, void *p2, void *p3)
{
	uint32_t counter = 0;

	if (!spi_is_ready_dt(&radio_spi)) {
		LOG_ERR("Radio SPI device not ready");
		return;
	}
	LOG_INF("Radio SPI device ready");

	while (1) {
		TelemetryPacket message = TelemetryPacket_init_zero;
		uint8_t buffer[MAX_PAYLOAD_SIZE];

		struct imu_data imu;
		struct baro_data baro;
		struct state_data state;
		struct gps_data gps;

		get_imu_data(&imu);
		get_baro_data(&baro);
		get_state_data(&state);
		get_gps_data(&gps);

		message.counter = counter;
		message.timestamp_ms = (uint32_t)k_uptime_get();
		message.state = (FlightState)state.state;

		message.accel_x = imu.accel[0];
		message.accel_y = imu.accel[1];
		message.accel_z = imu.accel[2];
		message.gyro_x = imu.gyro[0];
		message.gyro_y = imu.gyro[1];
		message.gyro_z = imu.gyro[2];

		message.kf_altitude = baro.altitude;
		message.kf_velocity = baro.velocity;

		message.baro0_healthy = baro.baro0.healthy;
		message.baro1_healthy = baro.baro1.healthy;

		message.ground_altitude = state.ground_altitude;

		message.gps_latitude = gps.latitude;
		message.gps_longitude = gps.longitude;
		message.gps_altitude = gps.altitude;
		message.gps_speed = gps.speed;
		message.gps_sats = gps.sats;
		message.gps_fix = gps.fix;

		pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
		bool status = pb_encode(&stream, TelemetryPacket_fields, &message);
		size_t message_length = stream.bytes_written;

		if (!status) {
			LOG_ERR("Failed to encode TelemetryPacket: %s",
				PB_GET_ERROR(&stream));
			counter++;
			k_sleep(K_MSEC(RADIO_THREAD_PERIOD_MS));
			continue;
		}

		/* CRC16-CCITT over protobuf payload */
		uint16_t crc = crc16_ccitt(0x0000, buffer, message_length);

		/* Pack protobuf data + CRC into net_buf for COBS encoding */
		struct net_buf *src_buf = net_buf_alloc(&cobs_src_pool, K_NO_WAIT);
		struct net_buf *dst_buf = net_buf_alloc(&cobs_dst_pool, K_NO_WAIT);

		if (!src_buf || !dst_buf) {
			LOG_ERR("Failed to allocate net_buf for COBS encoding");
			if (src_buf) {
				net_buf_unref(src_buf);
			}
			if (dst_buf) {
				net_buf_unref(dst_buf);
			}
			counter++;
			k_sleep(K_MSEC(RADIO_THREAD_PERIOD_MS));
			continue;
		}

		net_buf_add_mem(src_buf, buffer, message_length);
		net_buf_add_le16(src_buf, crc);

		/* COBS encode with trailing 0x00 delimiter */
		int ret = cobs_encode(src_buf, dst_buf, COBS_FLAG_TRAILING_DELIMITER);

		if (ret == 0) {
			/* Send over SPI to radio board */
			ret = radio_spi_send(dst_buf->data, dst_buf->len);
			if (ret < 0) {
				LOG_ERR("SPI write failed: %d", ret);
			} else {
				LOG_INF("Sent telemetry: counter=%u, alt=%.1f, vel=%.1f, "
					"state=%d, pb=%zu, cobs=%u bytes",
					counter, (double)message.kf_altitude,
					(double)message.kf_velocity,
					(int)message.state, message_length,
					dst_buf->len);
			}
		} else {
			LOG_ERR("COBS encoding failed: %d", ret);
		}

		net_buf_unref(src_buf);
		net_buf_unref(dst_buf);

		counter++;
		k_sleep(K_MSEC(RADIO_THREAD_PERIOD_MS));
	}
}

void start_radio_thread(void)
{
	k_thread_create(&radio_thread, radio_stack, K_THREAD_STACK_SIZEOF(radio_stack),
			radio_thread_fn, NULL, NULL, NULL, RADIO_THREAD_PRIORITY, 0, K_NO_WAIT);
}
