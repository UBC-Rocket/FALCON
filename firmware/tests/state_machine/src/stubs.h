#ifndef STATE_MACHINE_TEST_STUBS_H
#define STATE_MACHINE_TEST_STUBS_H

#include <stdbool.h>

/* Call counters/state recorded by the hardware-action stubs (stubs.c) */
extern int stub_pyro_fire_drogue_calls;
extern int stub_pyro_fire_main_calls;
extern int stub_runcam_stop_calls;
extern int stub_vtx_power_set_calls;
extern bool stub_vtx_power_last;

void stubs_reset(void);

#endif /* STATE_MACHINE_TEST_STUBS_H */
