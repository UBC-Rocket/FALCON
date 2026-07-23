#ifndef STATE_MACHINE_TEST_STUBS_H
#define STATE_MACHINE_TEST_STUBS_H

/* Call counters recorded by the hardware-action stubs (stubs.c) */
extern int stub_pyro_fire_drogue_calls;
extern int stub_pyro_fire_main_calls;

void stubs_reset(void);

#endif /* STATE_MACHINE_TEST_STUBS_H */
