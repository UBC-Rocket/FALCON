#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "runcam.h"
#include "uplink_config.h"
#include "data.h"

LOG_MODULE_REGISTER(runcam, LOG_LEVEL_INF);

/**
 * @brief Update the shared camera status with the new recording state
 */
static void update_shared_recording(bool recording)
{
    struct camera_data cam;

    get_camera_data(&cam);
    cam.recording = recording;
    cam.timestamp = k_uptime_get();
    set_camera_data(&cam);
}

#if DT_NODE_EXISTS(DT_ALIAS(runcam_uart))

static const struct device *const runcam_uart = DEVICE_DT_GET(DT_ALIAS(runcam_uart));

/* Serializes frame TX between the command executor and the state machine
 * (landed shutdown). */
K_MUTEX_DEFINE(runcam_mutex);

/**
 * @brief Send a RunCam Device Protocol CAMERA_CONTROL frame.
 *
 * Frame: [header][command id][action][CRC8 over the preceding bytes].
 * The protocol defines no response for CAMERA_CONTROL, so success here
 * only means the frame went out on the wire.
 * TODO(unconfirmed): verify command set/ack semantics with David, see
 * uplink_config.h.
 */
static int runcam_send_action(uint8_t action)
{
    uint8_t frame[4] = {RUNCAM_PROTO_HEADER, RUNCAM_CMD_CAMERA_CONTROL, action, 0};

    frame[3] = crc8(frame, 3, RUNCAM_CRC8_POLY, 0x00, false);

    if (!device_is_ready(runcam_uart)) {
        LOG_ERR("RunCam UART not ready");
        return -ENODEV;
    }

    k_mutex_lock(&runcam_mutex, K_FOREVER);
    for (size_t i = 0; i < sizeof(frame); i++) {
        uart_poll_out(runcam_uart, frame[i]);
    }
    k_mutex_unlock(&runcam_mutex);

    return 0;
}

int runcam_init(void)
{
    if (!device_is_ready(runcam_uart)) {
        LOG_ERR("RunCam UART not ready");
        return -ENODEV;
    }
    LOG_INF("RunCam UART ready");
    return 0;
}

int runcam_start_recording(void)
{
    int ret = runcam_send_action(RUNCAM_ACTION_START_RECORDING);
    if (ret < 0) {
        return ret;
    }

    /* Optimistic: CAMERA_CONTROL has no ack (see runcam_send_action) */
    update_shared_recording(true);
    LOG_INF("RunCam recording start sent");
    return 0;
}

int runcam_stop_recording(void)
{
    int ret = runcam_send_action(RUNCAM_ACTION_STOP_RECORDING);
    if (ret < 0) {
        return ret;
    }

    update_shared_recording(false);
    LOG_INF("RunCam recording stop sent");
    return 0;
}

#else /* no runcam-uart alias (e.g. native_sim): simulate the camera */

int runcam_init(void)
{
    LOG_WRN("No runcam-uart devicetree alias; simulating RunCam");
    return 0;
}

int runcam_start_recording(void)
{
    LOG_INF("(sim) RunCam recording start");
    update_shared_recording(true);
    return 0;
}

int runcam_stop_recording(void)
{
    LOG_INF("(sim) RunCam recording stop");
    update_shared_recording(false);
    return 0;
}

#endif
