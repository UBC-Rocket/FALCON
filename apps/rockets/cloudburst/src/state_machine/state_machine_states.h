#ifndef STATE_MACHINE_STATES_H
#define STATE_MACHINE_STATES_H

#include <zephyr/smf.h>

void state_standby_entry(void *obj);
enum smf_state_result state_standby_run(void *obj);

void state_ascent_entry(void *obj);
enum smf_state_result state_ascent_run(void *obj);

void state_mach_lock_entry(void *obj);
enum smf_state_result state_mach_lock_run(void *obj);

void state_drogue_descent_entry(void *obj);
enum smf_state_result state_drogue_descent_run(void *obj);

void state_main_descent_entry(void *obj);
enum smf_state_result state_main_descent_run(void *obj);

void state_landed_entry(void *obj);
enum smf_state_result state_landed_run(void *obj);

#endif
