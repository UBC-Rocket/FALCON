#ifndef VTX_POWER_H
#define VTX_POWER_H

#include <stdbool.h>

/**
 * @brief Configure the VTX/RunCam power switch GPIO (powered off by default)
 * @return 0 on success, negative errno on failure
 */
int vtx_power_init(void);

/**
 * @brief Drive the VTX/RunCam power switch and update shared camera status
 * @param on true to power on, false to power off
 * @return 0 on success, negative errno on failure
 */
int vtx_power_set(bool on);

#endif /* VTX_POWER_H */
