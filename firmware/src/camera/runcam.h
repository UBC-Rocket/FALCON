#ifndef RUNCAM_H
#define RUNCAM_H

/**
 * @brief Check that the RunCam control UART is ready
 * @return 0 on success, negative errno on failure
 */
int runcam_init(void);

/**
 * @brief Start RunCam recording and update shared camera status
 * @return 0 on success, negative errno on failure
 */
int runcam_start_recording(void);

/**
 * @brief Stop RunCam recording and update shared camera status
 * @return 0 on success, negative errno on failure
 */
int runcam_stop_recording(void);

#endif /* RUNCAM_H */
