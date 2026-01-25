#ifndef PYRO_THREAD_H
#define PYRO_THREAD_H

#include <stdint.h>
#include <stdbool.h>

// Pyro status bit definitions
#define PYRO_STATUS_DROGUE_FIRED    (1 << 0)
#define PYRO_STATUS_MAIN_FIRED      (1 << 1)
#define PYRO_STATUS_DROGUE_FAIL     (1 << 2)
#define PYRO_STATUS_MAIN_FAIL       (1 << 3)
#define PYRO_STATUS_DROGUE_CONT_OK  (1 << 4)
#define PYRO_STATUS_MAIN_CONT_OK    (1 << 5)
#define PYRO_STATUS_DROGUE_FIRE_ACK (1 << 6)
#define PYRO_STATUS_MAIN_FIRE_ACK   (1 << 7)

// Pyro command definitions
#define PYRO_CMD_FIRE_DROGUE    0x01
#define PYRO_CMD_FIRE_MAIN      0x02
#define PYRO_CMD_STATUS_REQ     0x55

// Pyro status structure
typedef struct {
    uint8_t status_byte;
    int64_t timestamp_ms;
    bool drogue_fired;
    bool main_fired;
    bool drogue_fail;
    bool main_fail;
    bool drogue_cont_ok;
    bool main_cont_ok;
    bool drogue_fire_ack;
    bool main_fire_ack;
} pyro_status_t;

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

/**
 * @brief Get the current pyro status
 * @param status Pointer to status structure to fill
 */
void pyro_get_status(pyro_status_t *status);

/**
 * @brief Check if drogue continuity is OK
 * @return true if continuity is good
 */
bool pyro_is_drogue_continuity_ok(void);

/**
 * @brief Check if main continuity is OK
 * @return true if continuity is good
 */
bool pyro_is_main_continuity_ok(void);

#endif /* PYRO_THREAD_H */
