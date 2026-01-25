#ifndef STATE_MACHINE_INTERNAL_H
#define STATE_MACHINE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/smf.h>

#include "data.h"
#include "state_machine_config.h"

typedef struct
{
    uint8_t count;
} repeated_check_t;

typedef struct
{
    float altitude_m;
    float velocity_mps;
    int64_t timestamp_ms;
} state_sample_t;

struct flight_sm
{
    struct smf_ctx ctx;
    flight_state_id_t current_id;
    int64_t entry_time_ms;
    state_sample_t sample;
    float ground_altitude_m;
    float ground_sum_m;
    uint8_t ground_samples;
    bool ground_ready;
    int64_t ground_warmup_start_ms;
    repeated_check_t standby_check;
    repeated_check_t mach_lock_check;
    repeated_check_t mach_unlock_check;
    repeated_check_t drogue_main_check;
    repeated_check_t landed_check;
    int64_t last_landed_check_ms;
    bool drogue_fired;
};

bool repeated_check_update(repeated_check_t *check, bool condition, uint8_t required);
void reset_repeated_check(repeated_check_t *check);
void reset_ground_average(struct flight_sm *sm);
float get_relative_altitude(const struct flight_sm *sm, float altitude_m);

void state_action_fire_drogue(void);
void state_action_fire_main(void);
void state_action_landed(void);

const char *flight_state_to_string(flight_state_id_t state);
void state_entry_common(struct flight_sm *sm, flight_state_id_t state);
void transition_to(struct flight_sm *sm, flight_state_id_t next_state);

#endif
