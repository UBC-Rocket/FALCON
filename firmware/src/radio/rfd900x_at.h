#ifndef RFD900X_AT_H
#define RFD900X_AT_H

#include <stdint.h>
#include <stddef.h>
#include <GroundCommand.pb.h>

/*
 * Transport over which the AT session talks to the RFD900x modem.
 * Kept abstract so the session engine can be unit-tested against a mocked
 * modem, and so the physical path (direct UART vs. AT-passthrough via the
 * GNSS board) can change without touching the session logic.
 */
struct rfd900x_transport {
    /**
     * @brief Blocking write of len bytes to the modem serial line
     * @return 0 on success, negative errno on failure
     */
    int (*write)(const uint8_t *data, size_t len);

    /**
     * @brief Read one byte, blocking up to timeout_ms (0 = only poll once)
     * @return 0 on success, -EAGAIN on timeout, other negative errno on
     *         a fatal transport error
     */
    int (*read_byte)(uint8_t *byte, int32_t timeout_ms);
};

/**
 * @brief Apply an RfdConfig to the modem through one AT command session.
 *
 * Session shape (see TWO_WAY_RADIO_CHECKLIST.md Phase 4):
 *   guard-time silence -> "+++" -> "OK"
 *   per populated field: "ATSn=v" -> "OK", then "ATSn?" readback verify
 *   "AT&W" -> "OK" (write EEPROM), "ATZ" (reboot, no response)
 *
 * Safety: every command must be acknowledged before the next is sent.
 * On any failure before AT&W the session aborts with "ATO" (back to the
 * online state, EEPROM untouched) so a half-applied config can never be
 * committed. ATZ is only ever sent after AT&W was acknowledged.
 *
 * Blocks the calling thread for the guard time plus the AT exchanges
 * (seconds). The caller must keep the modem serial line otherwise silent
 * for the whole session or command-mode entry will fail.
 *
 * @return 0 on success (modem is rebooting into the new config),
 *         -EINVAL if cfg is empty or fails validation (modem untouched),
 *         -ETIMEDOUT / -EIO / transport errno on session failure (aborted
 *         without writing EEPROM)
 */
int rfd900x_at_apply(const struct rfd900x_transport *io, const RfdConfig *cfg);

#endif /* RFD900X_AT_H */
