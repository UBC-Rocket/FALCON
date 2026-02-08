#ifndef PYRO_THREAD_H
#define PYRO_THREAD_H

#include <stdint.h>
#include <stdbool.h>

// Pyro status bit definitions
#define PYRO_STATUS_DROGUE_FIRED (1 << 0)
#define PYRO_STATUS_MAIN_FIRED (1 << 1)
#define PYRO_STATUS_DROGUE_FAIL (1 << 2)
#define PYRO_STATUS_MAIN_FAIL (1 << 3)
#define PYRO_STATUS_DROGUE_CONT_OK (1 << 4)
#define PYRO_STATUS_MAIN_CONT_OK (1 << 5)
#define PYRO_STATUS_DROGUE_FIRE_ACK (1 << 6)
#define PYRO_STATUS_MAIN_FIRE_ACK (1 << 7)

// Pyro command definitions
#define PYRO_CMD_FIRE_DROGUE 0x01
#define PYRO_CMD_FIRE_MAIN 0x02
#define PYRO_CMD_STATUS_REQ 0x55

/**
 * @brief Start the pyro communication thread
 */
void start_pyro_thread(void);

/**
 * @brief Fire the drogue parachute
 * @return 0 on success, negative errno on failure
 */
int pyro_fire_drogue(void);

/**
 * @brief Fire the main parachute
 * @return 0 on success, negative errno on failure
 */
int pyro_fire_main(void);

#endif /* PYRO_THREAD_H */
