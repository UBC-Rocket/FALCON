#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "data.h"

void start_state_machine_thread(void);
const char *flight_state_to_string(flight_state_id_t state);

#endif
