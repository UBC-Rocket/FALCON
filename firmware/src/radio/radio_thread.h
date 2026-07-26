#ifndef RADIO_THREAD_H
#define RADIO_THREAD_H

#include <stdbool.h>

void start_radio_thread(void);

/**
 * @brief Suspend/resume telemetry downlink TX.
 *
 * Used while an RFD900x AT session is in progress: the modem only enters
 * command mode after guard-time silence on its serial line, so telemetry
 * must stop flowing toward the modem for the duration.
 */
void radio_tx_suspend(bool suspend);

#endif
