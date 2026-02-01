#include "state_machine_internal.h"
#include "state_machine_states.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(state_standby, LOG_LEVEL_DBG);

/**
 * @brief Evaluate transitions while in standby (includes ground averaging).
 */
static flight_state_id_t update_standby(struct flight_sm *sm, const state_sample_t *sample)
{
    if (!sm->ground_ready) {
        if ((sample->timestamp_ms - sm->ground_warmup_start_ms) < GROUND_WARMUP_MS) {
            return FLIGHT_STATE_STANDBY;
        }
        sm->ground_sum_m += sample->altitude_m;
        sm->ground_samples++;
        LOG_DBG("Ground calibration: %d/%d samples, current_alt=%.2f m, avg=%.2f m", 
                sm->ground_samples, GROUND_AVERAGE_SAMPLES, 
                sample->altitude_m, sm->ground_sum_m / sm->ground_samples);
        
        if (sm->ground_samples >= GROUND_AVERAGE_SAMPLES) {
            sm->ground_altitude_m = sm->ground_sum_m / (float)sm->ground_samples;
            sm->ground_ready = true;
            LOG_INF("Ground calibration complete: %.2f m (%d samples @ 50Hz = %d ms)", 
                    sm->ground_altitude_m, sm->ground_samples, sm->ground_samples * 20);
        }
        return FLIGHT_STATE_STANDBY;
    }

    float rel_altitude = get_relative_altitude(sm, sample->altitude_m);
    bool ascent_condition = (rel_altitude > ASCENT_ALTITUDE_THRESHOLD_M) &&
                            (sample->velocity_mps > ASCENT_VELOCITY_THRESHOLD_MPS);

    if (repeated_check_update(&sm->standby_check, ascent_condition, ASCENT_CHECKS)) {
        return FLIGHT_STATE_ASCENT;
    }

    if (ascent_condition && sm->standby_check.count > 0) {
        LOG_WRN("Ascent condition MET but waiting for checks: %d/%d", 
                sm->standby_check.count, ASCENT_CHECKS);
    }

    return FLIGHT_STATE_STANDBY;
}

/**
 * @brief SMF entry handler for standby state.
 */
void state_standby_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_STANDBY);
    reset_repeated_check(&sm->standby_check);
    reset_ground_average(sm);
}

/**
 * @brief SMF run handler for standby state.
 */
enum smf_state_result state_standby_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_standby(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}
