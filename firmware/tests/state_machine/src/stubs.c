/*
 * Test doubles for the hardware-facing actions invoked by the state machine
 * (pyro firing, RunCam recording, VTX power). The real implementations talk
 * to SPI/UART/GPIO and are not compiled into this suite.
 */
#include "stubs.h"
#include "pyro/pyro_thread.h"
#include "camera/runcam.h"
#include "camera/vtx_power.h"

int stub_pyro_fire_drogue_calls;
int stub_pyro_fire_main_calls;
int stub_runcam_stop_calls;
int stub_vtx_power_set_calls;
bool stub_vtx_power_last;

void stubs_reset(void)
{
    stub_pyro_fire_drogue_calls = 0;
    stub_pyro_fire_main_calls = 0;
    stub_runcam_stop_calls = 0;
    stub_vtx_power_set_calls = 0;
    stub_vtx_power_last = false;
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

int runcam_init(void)
{
    return 0;
}

int runcam_start_recording(void)
{
    return 0;
}

int runcam_stop_recording(void)
{
    stub_runcam_stop_calls++;
    return 0;
}

int vtx_power_init(void)
{
    return 0;
}

int vtx_power_set(bool on)
{
    stub_vtx_power_set_calls++;
    stub_vtx_power_last = on;
    return 0;
}
