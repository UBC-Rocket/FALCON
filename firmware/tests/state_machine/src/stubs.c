/*
 * Test doubles for the hardware-facing actions invoked by the state machine
 * (pyro firing). The real implementations talk to SPI and are not compiled
 * into this suite.
 */
#include "stubs.h"
#include "pyro/pyro_thread.h"

int stub_pyro_fire_drogue_calls;
int stub_pyro_fire_main_calls;

void stubs_reset(void)
{
    stub_pyro_fire_drogue_calls = 0;
    stub_pyro_fire_main_calls = 0;
}

int pyro_fire_drogue(void)
{
    stub_pyro_fire_drogue_calls++;
    return 0;
}

int pyro_fire_main(void)
{
    stub_pyro_fire_main_calls++;
    return 0;
}
