#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "vtx_power.h"
#include "data.h"

LOG_MODULE_REGISTER(vtx_power, LOG_LEVEL_INF);

#if DT_NODE_EXISTS(DT_ALIAS(led0))

static const struct gpio_dt_spec vtx_status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool led_configured;

/**
 * @brief Mirror the VTX power state on led0 (green LED, PA10) so the state of
 * the vtx-pwr output (PD6) is visible on the board without a scope.
 *
 * Best-effort: a failure here is logged but never propagated, since the LED is
 * only an indicator and must not make vtx_power_set() report failure.
 */
static void vtx_status_led_set(bool on)
{
    int ret;

    if (!gpio_is_ready_dt(&vtx_status_led)) {
        LOG_WRN("VTX status LED not ready");
        return;
    }

    /* Same lazy-configure as the power pin: the landed shutdown can run
     * before vtx_power_init() has touched the LED. */
    if (!led_configured) {
        ret = gpio_pin_configure_dt(&vtx_status_led,
                                    on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
        if (ret == 0) {
            led_configured = true;
        }
    } else {
        ret = gpio_pin_set_dt(&vtx_status_led, on ? 1 : 0);
    }

    if (ret < 0) {
        LOG_WRN("Failed to drive VTX status LED: %d", ret);
    }
}

#else /* no led0 alias (e.g. native_sim) */

static void vtx_status_led_set(bool on)
{
    ARG_UNUSED(on);
}

#endif

/**
 * @brief Update the shared camera status with the new power state and mirror
 * the state on led0.
 * Cutting power also stops any recording in progress.
 *
 * Called only after the power pin has actually been driven, so the LED and the
 * telemetry both track the real state of the output.
 */
static void update_shared_power(bool on)
{
    struct camera_data cam;

    vtx_status_led_set(on);

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

    /* Default off: PD6 comes up low and the VTX/RunCam rail stays unpowered
     * until the ground crew switches it on over the uplink. Note this also
     * means a firmware reboot mid-flight drops the rail until commanded back
     * on. */
    int ret = gpio_pin_configure_dt(&vtx_pwr_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure VTX power GPIO: %d", ret);
        return ret;
    }

    initialized = true;
    update_shared_power(false);
    LOG_INF("VTX/RunCam power off");
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
    update_shared_power(false);
    return 0;
}

int vtx_power_set(bool on)
{
    LOG_INF("(sim) VTX/RunCam power %s", on ? "on" : "off");
    update_shared_power(on);
    return 0;
}

#endif
