#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "vtx_power.h"
#include "data.h"

LOG_MODULE_REGISTER(vtx_power, LOG_LEVEL_INF);

/**
 * @brief Update the shared camera status with the new power state.
 * Cutting power also stops any recording in progress.
 */
static void update_shared_power(bool on)
{
    struct camera_data cam;

    get_camera_data(&cam);
    cam.vtx_power_on = on;
    if (!on) {
        cam.recording = false;
    }
    cam.timestamp = k_uptime_get();
    set_camera_data(&cam);
}

#if DT_NODE_EXISTS(DT_ALIAS(vtx_pwr))

static const struct gpio_dt_spec vtx_pwr_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(vtx_pwr), gpios);
static bool initialized;

int vtx_power_init(void)
{
    if (!gpio_is_ready_dt(&vtx_pwr_gpio)) {
        LOG_ERR("VTX power GPIO not ready");
        return -ENODEV;
    }

    /* Default on: a firmware reboot mid-flight must not leave the VTX and
     * RunCam dark. Ground crew can power them off over the uplink. */
    int ret = gpio_pin_configure_dt(&vtx_pwr_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure VTX power GPIO: %d", ret);
        return ret;
    }

    initialized = true;
    update_shared_power(true);
    LOG_INF("VTX/RunCam power on");
    return 0;
}

int vtx_power_set(bool on)
{
    int ret;

    if (!gpio_is_ready_dt(&vtx_pwr_gpio)) {
        LOG_ERR("VTX power GPIO not ready");
        return -ENODEV;
    }

    /* Landed shutdown may run before the command executor has initialized
     * the pin; configure it directly to the requested state in that case */
    if (!initialized) {
        ret = gpio_pin_configure_dt(&vtx_pwr_gpio, on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
        if (ret == 0) {
            initialized = true;
        }
    } else {
        ret = gpio_pin_set_dt(&vtx_pwr_gpio, on ? 1 : 0);
    }

    if (ret < 0) {
        LOG_ERR("Failed to set VTX power GPIO: %d", ret);
        return ret;
    }

    update_shared_power(on);
    LOG_INF("VTX/RunCam power %s", on ? "on" : "off");
    return 0;
}

#else /* no vtx-pwr alias (e.g. native_sim): simulate the power switch */

int vtx_power_init(void)
{
    LOG_WRN("No vtx-pwr devicetree alias; simulating VTX power switch");
    update_shared_power(true);
    return 0;
}

int vtx_power_set(bool on)
{
    LOG_INF("(sim) VTX/RunCam power %s", on ? "on" : "off");
    update_shared_power(on);
    return 0;
}

#endif
