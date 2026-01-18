#ifndef STATE_MACHINE_TEST_H
#define STATE_MACHINE_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "data.h"

#ifdef CONFIG_ZTEST
void state_machine_test_reset(int64_t start_ms);
void state_machine_test_step(float altitude_m, float velocity_mps, int64_t timestamp_ms);
void state_machine_test_setup_state(flight_state_id_t state, float ground_altitude_m, int64_t timestamp_ms);
flight_state_id_t state_machine_test_get_state(void);
float state_machine_test_get_ground_altitude(void);
bool state_machine_test_get_drogue_fire_triggered(void);
#endif

#endif
