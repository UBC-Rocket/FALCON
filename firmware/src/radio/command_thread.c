#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/data/cobs.h>
#include <zephyr/net_buf.h>
#include <pb_decode.h>
#include <GroundCommand.pb.h>
#include "command_thread.h"
#include "gnss_spi.h"
#include "camera/vtx_power.h"
#include "camera/runcam.h"

LOG_MODULE_REGISTER(command_thread, LOG_LEVEL_INF);

#define COMMAND_RX_STACK_SIZE 2048
#define COMMAND_RX_PRIORITY 6
#define COMMAND_RX_POLL_MS 200

#define COMMAND_EXEC_STACK_SIZE 2048
#define COMMAND_EXEC_PRIORITY 7

/* COBS frame limits, mirroring the telemetry TX path */
#define MAX_FRAME_SIZE ((GNSS_SPI_MAX_COBS_SIZE - 2) * 254 / 255) /* 253 */

K_THREAD_STACK_DEFINE(command_rx_stack, COMMAND_RX_STACK_SIZE);
static struct k_thread command_rx_thread;

K_THREAD_STACK_DEFINE(command_exec_stack, COMMAND_EXEC_STACK_SIZE);
static struct k_thread command_exec_thread;

/* Decoded commands are dispatched to the executor through this queue so
 * slow command handling (UART transactions) never stalls RX polling. */
#define COMMAND_QUEUE_SIZE 8
K_MSGQ_DEFINE(command_msgq, sizeof(GroundCommand), COMMAND_QUEUE_SIZE, 4);

NET_BUF_POOL_DEFINE(cmd_cobs_src_pool, 1, GNSS_SPI_MAX_COBS_SIZE, 0, NULL);
NET_BUF_POOL_DEFINE(cmd_cobs_dst_pool, 1, MAX_FRAME_SIZE, 0, NULL);

/* Dedup: the ground station may retransmit a command until it sees the
 * effect reflected in telemetry; replayed command_ids are ignored. */
static uint32_t last_command_id;
static bool have_last_command;

/**
 * @brief Decode one received payload: COBS -> CRC16 check -> protobuf.
 * @return true if a command was decoded into *cmd
 */
static bool decode_rx_payload(const uint8_t *payload, GroundCommand *cmd)
{
    bool decoded = false;

    /* Empty payload (all zeros from SPI) starts with the COBS delimiter */
    if (payload[0] == 0x00) {
        return false;
    }

    /* Frame is the bytes up to and including the first 0x00 delimiter
     * (COBS-encoded data contains no zero bytes) */
    size_t frame_len = 0;
    while (frame_len < GNSS_SPI_MAX_COBS_SIZE && payload[frame_len] != 0x00) {
        frame_len++;
    }
    if (frame_len == GNSS_SPI_MAX_COBS_SIZE) {
        LOG_WRN("Unterminated COBS frame");
        return false;
    }
    frame_len++; /* include the delimiter */

    struct net_buf *src_buf = net_buf_alloc(&cmd_cobs_src_pool, K_NO_WAIT);
    struct net_buf *dst_buf = net_buf_alloc(&cmd_cobs_dst_pool, K_NO_WAIT);

    if (!src_buf || !dst_buf) {
        LOG_ERR("Failed to allocate net_buf for COBS decoding");
        goto out;
    }

    net_buf_add_mem(src_buf, payload, frame_len);

    int ret = cobs_decode(src_buf, dst_buf, COBS_FLAG_TRAILING_DELIMITER);
    if (ret < 0) {
        LOG_WRN("COBS decode failed: %d", ret);
        goto out;
    }

    if (dst_buf->len < sizeof(uint16_t)) {
        LOG_WRN("Frame too short: %u bytes", dst_buf->len);
        goto out;
    }

    /* CRC16-CCITT (little-endian) over the protobuf payload */
    uint16_t rx_crc = net_buf_remove_le16(dst_buf);
    uint16_t calc_crc = crc16_ccitt(0x0000, dst_buf->data, dst_buf->len);

    if (rx_crc != calc_crc) {
        LOG_WRN("CRC mismatch: got 0x%04x, expected 0x%04x", rx_crc, calc_crc);
        goto out;
    }

    *cmd = (GroundCommand)GroundCommand_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(dst_buf->data, dst_buf->len);

    if (!pb_decode(&stream, GroundCommand_fields, cmd)) {
        LOG_WRN("Failed to decode GroundCommand: %s", PB_GET_ERROR(&stream));
        goto out;
    }

    decoded = true;

out:
    if (src_buf) {
        net_buf_unref(src_buf);
    }
    if (dst_buf) {
        net_buf_unref(dst_buf);
    }
    return decoded;
}

/**
 * @brief Poll the radio board for uplinked commands and enqueue them
 */
static void command_rx_thread_fn(void *p1, void *p2, void *p3)
{
    if (!gnss_spi_ready()) {
        LOG_ERR("GNSS/radio SPI device not ready");
        return;
    }
    LOG_INF("Command RX polling started");

    while (1) {
        uint8_t payload[GNSS_SPI_MAX_COBS_SIZE];
        GroundCommand cmd;

        int ret = gnss_spi_radio_rx(payload);
        if (ret < 0) {
            LOG_ERR("Radio RX SPI read failed: %d", ret);
            k_sleep(K_MSEC(COMMAND_RX_POLL_MS));
            continue;
        }

        if (decode_rx_payload(payload, &cmd)) {
            if (have_last_command && cmd.command_id == last_command_id) {
                LOG_DBG("Ignoring replayed command %u", cmd.command_id);
            } else if (k_msgq_put(&command_msgq, &cmd, K_NO_WAIT) != 0) {
                LOG_ERR("Command queue full, dropping command %u", cmd.command_id);
            } else {
                last_command_id = cmd.command_id;
                have_last_command = true;
            }
        }

        k_sleep(K_MSEC(COMMAND_RX_POLL_MS));
    }
}

/**
 * @brief Apply a camera control command.
 *
 * Only the power switch is acted on: the RunCam has auto-recording
 * enabled, so recording follows the power rail. camera_recording is
 * ignored until the protobufs are reworked to carry RunCam commands
 * (the runcam module is kept but dormant until then).
 */
static void execute_camera_command(const CameraControl *camera)
{
    if (camera->has_vtx_runcam_power) {
        vtx_power_set(camera->vtx_runcam_power);
    }

    if (camera->has_camera_recording) {
        LOG_WRN("Ignoring camera_recording command: RunCam auto-records while powered");
    }
}

/**
 * @brief Execute decoded ground commands from the queue
 */
static void command_exec_thread_fn(void *p1, void *p2, void *p3)
{
    vtx_power_init();
    runcam_init();

    while (1) {
        GroundCommand cmd;

        k_msgq_get(&command_msgq, &cmd, K_FOREVER);

        LOG_INF("Ground command %u (operator '%s', issued_at=%u ms)", cmd.command_id, cmd.operator,
                cmd.issued_at_ms);

        switch (cmd.which_command) {
        case GroundCommand_camera_tag:
            execute_camera_command(&cmd.command.camera);
            break;
        case GroundCommand_rfd_config_tag:
            /* Per GroundCommand.proto, rfd_config is ground-local (applied
             * to the ground-station modem) and should never reach FALCON.
             * Rocket-side modem reconfiguration (checklist Phase 4) is on
             * hold until the team resolves the discrepancy. */
            LOG_WRN("Ignoring rfd_config command %u: ground-local per proto", cmd.command_id);
            break;
        default:
            LOG_WRN("Command %u carries no payload", cmd.command_id);
            break;
        }
    }
}

void start_command_threads(void)
{
    k_thread_create(&command_rx_thread, command_rx_stack, K_THREAD_STACK_SIZEOF(command_rx_stack),
                    command_rx_thread_fn, NULL, NULL, NULL, COMMAND_RX_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&command_rx_thread, "cmd_rx");

    k_thread_create(&command_exec_thread, command_exec_stack,
                    K_THREAD_STACK_SIZEOF(command_exec_stack), command_exec_thread_fn, NULL, NULL,
                    NULL, COMMAND_EXEC_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&command_exec_thread, "cmd_exec");
}
