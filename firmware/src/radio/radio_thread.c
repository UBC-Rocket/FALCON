#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pb_encode.h>
#include <HelloWorldPacket.pb.h>

LOG_MODULE_REGISTER(radio_thread, LOG_LEVEL_INF);

#define RADIO_THREAD_STACK_SIZE 2048
#define RADIO_THREAD_PRIORITY 5
#define RADIO_THREAD_PERIOD_MS 1000

K_THREAD_STACK_DEFINE(radio_stack, RADIO_THREAD_STACK_SIZE);
static struct k_thread radio_thread;

static void radio_thread_fn(void *p1, void *p2, void *p3)
{
    uint32_t counter = 0;

    while (1) {
        HelloWorldPacket message = HelloWorldPacket_init_zero;
        uint8_t buffer[HelloWorldPacket_size];
        size_t message_length;

        message.counter = counter;
        strncpy(message.message, "hello world", sizeof(message.message) - 1);

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        bool status = pb_encode(&stream, HelloWorldPacket_fields, &message);
        message_length = stream.bytes_written;

        if (status) {
            LOG_INF("Encoded HelloWorldPacket: counter=%u, length=%zu bytes",
                    counter, message_length);
            LOG_HEXDUMP_DBG(buffer, message_length, "Encoded packet");
        } else {
            LOG_ERR("Failed to encode HelloWorldPacket: %s", PB_GET_ERROR(&stream));
        }

        counter++;
        k_sleep(K_MSEC(RADIO_THREAD_PERIOD_MS));
    }
}

void start_radio_thread(void)
{
    k_thread_create(&radio_thread, radio_stack, K_THREAD_STACK_SIZEOF(radio_stack),
                    radio_thread_fn, NULL, NULL, NULL, RADIO_THREAD_PRIORITY, 0, K_NO_WAIT);
}
