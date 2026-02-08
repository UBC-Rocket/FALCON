#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/data/cobs.h>
#include <zephyr/net_buf.h>
#include <pb_encode.h>
#include <HelloWorldPacket.pb.h>

LOG_MODULE_REGISTER(radio_thread, LOG_LEVEL_INF);

#define RADIO_THREAD_STACK_SIZE 2048
#define RADIO_THREAD_PRIORITY 5
#define RADIO_THREAD_PERIOD_MS 1000

#define MAX_COBS_SIZE    256 /* max size of COBS encoded data, defined by GNSS/RADIO SPI spec */
#define MAX_FRAME_SIZE   ((MAX_COBS_SIZE - 2) * 254 / 255) /* 253 */
#define MAX_PAYLOAD_SIZE (MAX_FRAME_SIZE - sizeof(uint16_t)) /* 251 */

NET_BUF_POOL_DEFINE(cobs_src_pool, 1, MAX_FRAME_SIZE, 0, NULL);
NET_BUF_POOL_DEFINE(cobs_dst_pool, 1, MAX_COBS_SIZE, 0, NULL);

K_THREAD_STACK_DEFINE(radio_stack, RADIO_THREAD_STACK_SIZE);
static struct k_thread radio_thread;

BUILD_ASSERT(MAX_FRAME_SIZE + MAX_FRAME_SIZE / 254 + 2 <= MAX_COBS_SIZE,
	     "MAX_FRAME_SIZE too large for MAX_COBS_SIZE");

static void radio_thread_fn(void *p1, void *p2, void *p3)
{
	uint32_t counter = 0;

	while (1) {
		HelloWorldPacket message = HelloWorldPacket_init_zero;
		uint8_t buffer[MAX_PAYLOAD_SIZE];

		message.counter = counter;
		strncpy(message.message, "hello world", sizeof(message.message) - 1);

		pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
		bool status = pb_encode(&stream, HelloWorldPacket_fields, &message);
		size_t message_length = stream.bytes_written;

		if (!status) {
			LOG_ERR("Failed to encode HelloWorldPacket: %s",
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
			LOG_INF("Encoded packet: counter=%u, pb=%zu, cobs=%u bytes",
				counter, message_length, dst_buf->len);
			LOG_HEXDUMP_DBG(dst_buf->data, dst_buf->len,
					"COBS encoded packet");
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
