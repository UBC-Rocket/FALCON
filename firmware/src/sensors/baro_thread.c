#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "../data.h"

LOG_MODULE_REGISTER(baro_thread, LOG_LEVEL_INF);

#define BARO_THREAD_STACK_SIZE 2048
#define BARO_THREAD_PRIORITY 1

#define BARO_THREAD_PERIOD_MS 30

// Debug logging
#define BARO_LOG_ENABLE 0

/* Altitude conversion */
#define P0_PA 101325.0f
#define GAS_CONSTANT_AIR 287.05f
#define GRAVITY 9.80665f

/* Tuning knobs */
#define KF_SIGMA_A 340.0f  // m/s^2, process noise standard deviation of acceleration
#define BARO0_SIGMA_Z 1.5f // m, measurement noise standard deviation of altitude
#define BARO1_SIGMA_Z 1.5f // m

/* Safety limits for dt */
#define KF_DT_MIN_S 0.001f
#define KF_DT_MAX_S 0.200f

typedef struct {
    float h; // altitude estimate (m)
    float v; // vertical velocity estimate (m/s)

    /* Covariance matrix P:
       [ P00 P01 ]
       [ P10 P11 ] */
    float P00;
    float P01;
    float P10;
    float P11;
} kalman_hv_t;

typedef struct {
    bool healthy;
} baro_health_t;

typedef struct {
    float pressure_pa;
    float altitude;
    float temperature_c;
    float nis;
    bool valid;
    bool accepted;
} baro_measurement_t;

K_THREAD_STACK_DEFINE(baro_stack, BARO_THREAD_STACK_SIZE);
static struct k_thread baro_thread;

static float pressure_temp_to_altitude(float pressure_pa, float temp_c)
{
    float temp_k = temp_c + 273.15f;
    return (GAS_CONSTANT_AIR * temp_k / GRAVITY) * logf(P0_PA / pressure_pa);
}

static void kf_predict(kalman_hv_t *kf, float dt_s, float sigma_a)
{
    /* State prediction:
       h = h + v*dt
       v = v
    */
    kf->h = kf->h + kf->v * dt_s;

    // F = [1 dt; 0 1]
    float F00 = 1.0f, F01 = dt_s;
    float F10 = 0.0f, F11 = 1.0f;

    // Q = sigma_a^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]
    float dt2 = dt_s * dt_s;
    float dt3 = dt2 * dt_s;
    float dt4 = dt2 * dt2;

    float sa2 = sigma_a * sigma_a;
    float Q00 = sa2 * (dt4 * 0.25f);
    float Q01 = sa2 * (dt3 * 0.50f);
    float Q10 = Q01;
    float Q11 = sa2 * (dt2);

    // P = F P F^T + Q

    /* First compute FP = F*P */
    float FP00 = F00 * kf->P00 + F01 * kf->P10;
    float FP01 = F00 * kf->P01 + F01 * kf->P11;
    float FP10 = F10 * kf->P00 + F11 * kf->P10;
    float FP11 = F10 * kf->P01 + F11 * kf->P11;

    // Then Pnew = FP * F^T
    float P00 = FP00 * F00 + FP01 * F01;
    float P01 = FP00 * F10 + FP01 * F11;
    float P10 = FP10 * F00 + FP11 * F01;
    float P11 = FP10 * F10 + FP11 * F11;

    kf->P00 = P00 + Q00;
    kf->P01 = P01 + Q01;
    kf->P10 = P10 + Q10;
    kf->P11 = P11 + Q11;
}

static void kf_update_baro(kalman_hv_t *kf, float z_alt, float R)
{
    // H = [1 0]

    float y = z_alt - kf->h;
    float S = kf->P00 + R;

    if (S < 1e-9f) {
        return;
    }

    // K = P H^T / S = [P00; P10] / S
    float K0 = kf->P00 / S;
    float K1 = kf->P10 / S;

    // State update
    kf->h = kf->h + K0 * y;
    kf->v = kf->v + K1 * y;

    /* Cov update: Joseph form for numeric stability
       P = (I - K H) P (I - K H)^T + K R K^T
       With H = [1 0].
    */
    float a00 = 1.0f - K0;
    float a01 = 0.0f;
    float a10 = -K1;
    float a11 = 1.0f;

    float AP00 = a00 * kf->P00 + a01 * kf->P10;
    float AP01 = a00 * kf->P01 + a01 * kf->P11;
    float AP10 = a10 * kf->P00 + a11 * kf->P10;
    float AP11 = a10 * kf->P01 + a11 * kf->P11;

    float P00 = AP00 * a00 + AP01 * a01 + K0 * K0 * R;
    float P01 = AP00 * a10 + AP01 * a11 + K0 * K1 * R;
    float P10 = AP10 * a00 + AP11 * a01 + K1 * K0 * R;
    float P11 = AP10 * a10 + AP11 * a11 + K1 * K1 * R;

    kf->P00 = P00;
    kf->P01 = P01;
    kf->P10 = P10;
    kf->P11 = P11;
}

// Compute NIS using a provided predicted state (so both sensors are judged fairly)
static float kf_compute_nis(const kalman_hv_t *kf_pred, float z_alt, float R, float *out_y,
                            float *out_S)
{
    float y = z_alt - kf_pred->h;
    float S = kf_pred->P00 + R;

    if (out_y) {
        *out_y = y;
    }
    if (out_S) {
        *out_S = S;
    }

    if (S < 1e-9f) {
        return INFINITY;
    }

    return (y * y) / S;
}

static bool read_baro(const struct device *dev, float *pressure_pa, float *altitude,
                      float *temperature_c)
{
    struct sensor_value pressure;
    struct sensor_value temperature;

    if (sensor_sample_fetch(dev) != 0) {
        return false;
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) != 0 ||
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature) != 0) {
        return false;
    }

    // converting from hPa (sensor output) to Pa
    float p_hpa = (float)sensor_value_to_double(&pressure);
    float p_pa = p_hpa * 100.0f;
    if (pressure_pa) {
        *pressure_pa = p_pa;
    }
    *temperature_c = (float)sensor_value_to_double(&temperature);

    // Guard against nonsense pressure
    if (!(p_pa > 1000.0f && p_pa < 200000.0f)) {
        return false;
    }

    *altitude = pressure_temp_to_altitude(p_pa, *temperature_c);
    return true;
}

static void assess_baro_measurement(const kalman_hv_t *kf_pred, float pressure_pa, float altitude,
                                    float temperature_c, float R,
                                    baro_measurement_t *out)
{
    out->pressure_pa = pressure_pa;
    out->altitude = altitude;
    out->temperature_c = temperature_c;
    out->valid = true;

    float nis = kf_compute_nis(kf_pred, altitude, R, NULL, NULL);
    out->nis = nis;

    /* Never reject a valid reading due to NIS. This is because we have no great way to test rejection thresholds on the ground. Perhaps after the first launch of this system we can analyze NIS values and set a threshold for future flights, but for now we will log NIS but accept all valid readings. Essentially, we are relying on successive measurements and the Kalman filter to smooth out any bad readings for state transitions. */
    out->accepted = true;
}

static void log_baro(const char *name, const baro_measurement_t *m, const baro_health_t *h)
{
#if !BARO_LOG_ENABLE
    return;
#endif
    if (!m->valid) {
        LOG_ERR("%s read failed", name);
        return;
    }

    LOG_INF("%s: p=%.1f Pa | alt=%.2f m | T=%.2f C | nis=%.2f | healthy=%d | %s", name,
            (double)m->pressure_pa, (double)m->altitude, (double)m->temperature_c, (double)m->nis,
        h->healthy ? 1 : 0, m->accepted ? "ACCEPTED" : "REJECTED");
}

static void baro_thread_fn(void *p1, void *p2, void *p3)
{
    const struct device *baro0 = DEVICE_DT_GET(DT_ALIAS(baro0));
    const struct device *baro1 = DEVICE_DT_GET(DT_ALIAS(baro1));
    bool using_baro0 = false;

    bool baro0_ready = device_is_ready(baro0);
    bool baro1_ready = device_is_ready(baro1);

    if (!baro0_ready && !baro1_ready) {
        LOG_ERR("No barometers ready");
        return;
    }

    if (baro0_ready) {
        using_baro0 = true;
        LOG_INF("Using BARO0 as primary barometer");
    } else {
        using_baro0 = false;
        LOG_WRN("BARO0 not ready at startup; falling back to BARO1");
    }

    /* Filter init:
       Start with altitude 0 and velocity 0.
       P00 is how uncertain you are about altitude at boot.
       P11 is how uncertain you are about velocity at boot.
    */
    kalman_hv_t kf = {.h = 0.0f, .v = 0.0f, .P00 = 25.0f, .P01 = 0.0f, .P10 = 0.0f, .P11 = 100.0f};

    baro_health_t health_0 = {.healthy = baro0_ready};
    baro_health_t health_1 = {.healthy = baro1_ready};

    bool kf_initialized = false;

    const float R0 = BARO0_SIGMA_Z * BARO0_SIGMA_Z;
    const float R1 = BARO1_SIGMA_Z * BARO1_SIGMA_Z;

    int64_t last_ts_ms = k_uptime_get();

    while (1) {
        int64_t now_ms = k_uptime_get();
        float dt_s = (float)(now_ms - last_ts_ms) / 1000.0f;
        last_ts_ms = now_ms;

        if (dt_s < KF_DT_MIN_S) {
            dt_s = KF_DT_MIN_S;
        }
        if (dt_s > KF_DT_MAX_S) {
            dt_s = KF_DT_MAX_S;
        }

        // Predict
        kf_predict(&kf, dt_s, KF_SIGMA_A);

        // Snapshot predicted state for fair NIS checks
        kalman_hv_t kf_pred = kf;

        baro_measurement_t measurement_0 = {0};
        baro_measurement_t measurement_1 = {0};

        // Read both sensors for telemetry publication (if they initialized ready)
        float p0 = 0.0f, a0 = 0.0f, t0 = 0.0f;
        float p1 = 0.0f, a1 = 0.0f, t1 = 0.0f;

        if (baro0_ready) {
            bool valid_reading_0 = read_baro(baro0, &p0, &a0, &t0);
            if (valid_reading_0) {
                assess_baro_measurement(&kf_pred, p0, a0, t0, R0, &measurement_0);
            } else {
                measurement_0.valid = false;
            }
        } else {
            measurement_0.valid = false;
        }

        if (baro1_ready) {
            bool valid_reading_1 = read_baro(baro1, &p1, &a1, &t1);
            if (valid_reading_1) {
                assess_baro_measurement(&kf_pred, p1, a1, t1, R1, &measurement_1);
            } else {
                measurement_1.valid = false;
            }
        } else {
            measurement_1.valid = false;
        }

        if (!kf_initialized) {
            if (using_baro0 && measurement_0.valid) {
                kf.h = measurement_0.altitude;
                kf.v = 0.0f;
                kf.P00 = R0;
                kf.P01 = 0.0f;
                kf.P10 = 0.0f;
                kf.P11 = 100.0f;
                kf_initialized = true;
            } else if (!using_baro0 && measurement_1.valid) {
                kf.h = measurement_1.altitude;
                kf.v = 0.0f;
                kf.P00 = R1;
                kf.P01 = 0.0f;
                kf.P10 = 0.0f;
                kf.P11 = 100.0f;
                kf_initialized = true;
            }
        }

        // Apply measurement update from only the selected barometer
        if (using_baro0) {
            if (measurement_0.valid && measurement_0.accepted) {
                kf_update_baro(&kf, measurement_0.altitude, R0);
            }
        } else {
            if (measurement_1.valid && measurement_1.accepted) {
                kf_update_baro(&kf, measurement_1.altitude, R1);
            }
        }

        log_baro("BARO0", &measurement_0, &health_0);
        log_baro("BARO1", &measurement_1, &health_1);

        // Read ground altitude from state machine for AGL computation
        struct state_data st;
        get_state_data(&st);

        struct baro_data data = {.baro0 = {.pressure = measurement_0.pressure_pa,
                                           .altitude = measurement_0.altitude,
                                           .temperature = measurement_0.temperature_c,
                                           .nis = measurement_0.nis,
                                           .faults = 0,
                                           .healthy = health_0.healthy},
                                 .baro1 = {.pressure = measurement_1.pressure_pa,
                                           .altitude = measurement_1.altitude,
                                           .temperature = measurement_1.temperature_c,
                                           .nis = measurement_1.nis,
                                           .faults = 0,
                                           .healthy = health_1.healthy},
                                 .altitude = kf.h,
                                 .altitude_agl = st.ground_calibrated ? kf.h - st.ground_altitude : 0.0f,
                                 .alt_variance = kf.P00,
                                 .velocity = kf.v,
                                 .vel_variance = kf.P11,
                                 .timestamp = now_ms};

        set_baro_data(&data);

#if BARO_LOG_ENABLE
        LOG_INF("KF: h=%.2f m | v=%.2f m/s | P_h=%.3f | P_v=%.3f | dt=%.3f", (double)kf.h,
                (double)kf.v, (double)kf.P00, (double)kf.P11, (double)dt_s);
#endif

        k_sleep(K_MSEC(BARO_THREAD_PERIOD_MS));
    }
}

void start_baro_thread(void)
{
    k_thread_create(&baro_thread, baro_stack, K_THREAD_STACK_SIZEOF(baro_stack), baro_thread_fn,
                    NULL, NULL, NULL, BARO_THREAD_PRIORITY, 0, K_NO_WAIT);
}
