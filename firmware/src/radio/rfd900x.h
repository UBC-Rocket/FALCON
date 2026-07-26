#ifndef RFD900X_H
#define RFD900X_H

#include <GroundCommand.pb.h>

/**
 * @brief Check that the RFD900x config serial line is ready
 * @return 0 on success, negative errno on failure
 */
int rfd900x_init(void);

/**
 * @brief Reconfigure the rocket-side RFD900x from an uplinked RfdConfig.
 *
 * Suspends telemetry TX for the duration (the AT guard time needs serial
 * silence) and blocks the calling thread for several seconds. On success
 * the modem reboots into the new config and the RF link drops until the
 * ground modem is reconfigured to match.
 *
 * @return 0 on success, negative errno on validation/session failure
 *         (the modem keeps its previous EEPROM config on any failure)
 */
int rfd900x_apply_config(const RfdConfig *cfg);

#endif /* RFD900X_H */
