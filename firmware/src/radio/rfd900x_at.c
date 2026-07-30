#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "rfd900x_at.h"
#include "uplink_config.h"

LOG_MODULE_REGISTER(rfd900x_at, LOG_LEVEL_INF);

/* Longest expected response line ("ATS8=915000" echo, values, OK/ERROR) */
#define AT_LINE_MAX 32

/* Longest command we emit ("ATS10=928000\r\n" + NUL) */
#define AT_CMD_MAX 24

struct sreg_write {
    uint8_t sreg;
    uint32_t value;
    const char *name;
};

/**
 * @brief Validate cfg and flatten it into an ordered S-register write list.
 *
 * Rejecting bad values here is the first line of defense: an out-of-range
 * register write must never reach the modem. Bounds live in uplink_config.h
 * and are placeholders pending team confirmation.
 */
static int build_writes(const RfdConfig *cfg, struct sreg_write *writes, size_t *n_out)
{
    size_t n = 0;

    if (cfg->has_air_speed_kbps) {
        if (cfg->air_speed_kbps < RFD_AIR_SPEED_MIN || cfg->air_speed_kbps > RFD_AIR_SPEED_MAX) {
            LOG_ERR("air_speed %u out of range", cfg->air_speed_kbps);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){2, cfg->air_speed_kbps, "AIR_SPEED"};
    }

    if (cfg->has_net_id) {
        if (cfg->net_id > RFD_NET_ID_MAX) {
            LOG_ERR("net_id %u out of range", cfg->net_id);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){3, cfg->net_id, "NETID"};
    }

    if (cfg->has_tx_power_dbm) {
        if (cfg->tx_power_dbm > RFD_TX_POWER_DBM_MAX) {
            LOG_ERR("tx_power %u out of range", cfg->tx_power_dbm);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){4, cfg->tx_power_dbm, "TXPOWER"};
    }

    if (cfg->has_min_freq_khz) {
        if (cfg->min_freq_khz < RFD_FREQ_KHZ_MIN || cfg->min_freq_khz > RFD_FREQ_KHZ_MAX) {
            LOG_ERR("min_freq %u kHz out of range", cfg->min_freq_khz);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){8, cfg->min_freq_khz, "MIN_FREQ"};
    }

    if (cfg->has_max_freq_khz) {
        if (cfg->max_freq_khz < RFD_FREQ_KHZ_MIN || cfg->max_freq_khz > RFD_FREQ_KHZ_MAX) {
            LOG_ERR("max_freq %u kHz out of range", cfg->max_freq_khz);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){9, cfg->max_freq_khz, "MAX_FREQ"};
    }

    if (cfg->has_min_freq_khz && cfg->has_max_freq_khz && cfg->min_freq_khz > cfg->max_freq_khz) {
        LOG_ERR("min_freq %u > max_freq %u", cfg->min_freq_khz, cfg->max_freq_khz);
        return -EINVAL;
    }

    if (cfg->has_num_channels) {
        if (cfg->num_channels < RFD_NUM_CHANNELS_MIN || cfg->num_channels > RFD_NUM_CHANNELS_MAX) {
            LOG_ERR("num_channels %u out of range", cfg->num_channels);
            return -EINVAL;
        }
        writes[n++] = (struct sreg_write){10, cfg->num_channels, "NUM_CHANNELS"};
    }

    if (n == 0) {
        LOG_ERR("RfdConfig carries no fields; refusing to enter AT mode");
        return -EINVAL;
    }

    *n_out = n;
    return 0;
}

/**
 * @brief Drain any stale bytes (telemetry echo, noise) before the session
 */
static void flush_input(const struct rfd900x_transport *io)
{
    uint8_t byte;

    for (int i = 0; i < 1024; i++) {
        if (io->read_byte(&byte, 0) != 0) {
            return;
        }
    }
}

static int at_send(const struct rfd900x_transport *io, const char *cmd)
{
    return io->write((const uint8_t *)cmd, strlen(cmd));
}

/**
 * @brief Read one CR/LF-terminated line, skipping empty lines
 * @param deadline Absolute k_uptime_get() time budget for the whole line
 */
static int at_read_line(const struct rfd900x_transport *io, char *buf, size_t buf_size,
                        int64_t deadline)
{
    size_t len = 0;

    while (1) {
        int64_t remaining = deadline - k_uptime_get();

        if (remaining <= 0) {
            return -ETIMEDOUT;
        }

        uint8_t byte;
        int ret = io->read_byte(&byte, (int32_t)remaining);

        if (ret == -EAGAIN) {
            return -ETIMEDOUT;
        }
        if (ret < 0) {
            return ret;
        }

        if (byte == '\r' || byte == '\n') {
            if (len == 0) {
                continue; /* leftover terminator from the previous line */
            }
            buf[len] = '\0';
            return 0;
        }

        if (len < buf_size - 1) {
            buf[len++] = (char)byte;
        }
        /* oversized lines are truncated; they will not match OK/values */
    }
}

/**
 * @brief Reduce a response line to its final token for matching.
 *
 * Two things observed on hardware (2026-07-30) put junk in front of the
 * part we care about, and anchoring at the start of the line handles
 * neither:
 *
 *   - Multipoint firmware tags responses with the node that answered:
 *     "[1] OK" rather than "OK", "[1] 905000" rather than "905000".
 *   - The modem echoes "+++" back, and since the escape is sent without
 *     CR/LF the echo has no terminator either -- so it fuses with the
 *     reply that follows and the line arrives as "+++[1] OK".
 *
 * Taking the last space/bracket-delimited token covers both, and leaves
 * command echoes ("ATS8=905000") as a single non-matching token so they
 * are still ignored rather than mistaken for a reply.
 */
static const char *last_token(const char *line)
{
    const char *token = line;

    for (const char *c = line; *c != '\0'; c++) {
        if (*c == ' ' || *c == '\t' || *c == ']') {
            token = c + 1;
        }
    }

    return token;
}

/**
 * @brief Wait for an "OK" line, ignoring command echo and other noise
 */
static int at_wait_ok(const struct rfd900x_transport *io, int32_t timeout_ms)
{
    char line[AT_LINE_MAX];
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        int ret = at_read_line(io, line, sizeof(line), deadline);

        if (ret < 0) {
            return ret;
        }
        if (strcmp(last_token(line), "OK") == 0) {
            return 0;
        }
        if (strstr(line, "ERROR") != NULL) {
            LOG_ERR("Modem replied ERROR");
            return -EIO;
        }
    }
}

/**
 * @brief Wait for a bare decimal line (ATSn? readback), ignoring echo
 */
static int at_read_value(const struct rfd900x_transport *io, int32_t timeout_ms,
                         uint32_t *value_out)
{
    char line[AT_LINE_MAX];
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        int ret = at_read_line(io, line, sizeof(line), deadline);

        if (ret < 0) {
            return ret;
        }
        if (strstr(line, "ERROR") != NULL) {
            LOG_ERR("Modem replied ERROR to readback");
            return -EIO;
        }

        /* Multipoint firmware answers "[1] 905000", not "905000" */
        const char *value = last_token(line);
        bool all_digits = value[0] != '\0';

        for (const char *c = value; *c != '\0'; c++) {
            if (*c < '0' || *c > '9') {
                all_digits = false;
                break;
            }
        }

        if (all_digits) {
            *value_out = (uint32_t)strtoul(value, NULL, 10);
            return 0;
        }
    }
}

int rfd900x_at_apply(const struct rfd900x_transport *io, const RfdConfig *cfg)
{
    struct sreg_write writes[6];
    size_t n_writes = 0;
    char cmd[AT_CMD_MAX];
    int ret;

    ret = build_writes(cfg, writes, &n_writes);
    if (ret < 0) {
        return ret;
    }

    flush_input(io);

    /* The modem only honors "+++" after guard-time serial silence */
    k_sleep(K_MSEC(RFD_AT_GUARD_TIME_MS));

    ret = at_send(io, "+++");
    if (ret < 0) {
        return ret;
    }

    ret = at_wait_ok(io, RFD_AT_ENTER_TIMEOUT_MS);
    if (ret < 0) {
        /* No EEPROM write happened, so there is no config to roll back --
         * but "we never saw OK" does not prove the modem ignored us. It may
         * be sitting in command mode with an answer we failed to match, and
         * leaving it there takes the link down for good. Abort through ATO
         * so the modem is returned to the online state either way. */
        LOG_ERR("Modem did not enter AT command mode: %d", ret);
        goto abort;
    }

    for (size_t i = 0; i < n_writes; i++) {
        snprintf(cmd, sizeof(cmd), "ATS%u=%u\r\n", writes[i].sreg, writes[i].value);

        ret = at_send(io, cmd);
        if (ret < 0) {
            goto abort;
        }

        ret = at_wait_ok(io, RFD_AT_RESPONSE_TIMEOUT_MS);
        if (ret < 0) {
            LOG_ERR("S%u (%s) write not acknowledged: %d", writes[i].sreg, writes[i].name, ret);
            goto abort;
        }

        /* Read the register back: an unacknowledged or silently-clamped
         * value must abort before anything reaches EEPROM */
        snprintf(cmd, sizeof(cmd), "ATS%u?\r\n", writes[i].sreg);

        ret = at_send(io, cmd);
        if (ret < 0) {
            goto abort;
        }

        uint32_t readback;

        ret = at_read_value(io, RFD_AT_RESPONSE_TIMEOUT_MS, &readback);
        if (ret < 0) {
            LOG_ERR("S%u (%s) readback failed: %d", writes[i].sreg, writes[i].name, ret);
            goto abort;
        }
        if (readback != writes[i].value) {
            LOG_ERR("S%u (%s) readback %u != requested %u", writes[i].sreg, writes[i].name,
                    readback, writes[i].value);
            ret = -EIO;
            goto abort;
        }
    }

    ret = at_send(io, "AT&W\r\n");
    if (ret < 0) {
        goto abort;
    }

    ret = at_wait_ok(io, RFD_AT_RESPONSE_TIMEOUT_MS);
    if (ret < 0) {
        LOG_ERR("AT&W not acknowledged, EEPROM not written: %d", ret);
        goto abort;
    }

    /* Reboot into the new config; no response expected. The RF link drops
     * until the modem is back up (and until the ground modem matches). */
    ret = at_send(io, "ATZ\r\n");
    if (ret < 0) {
        LOG_ERR("ATZ write failed after AT&W: %d; modem needs a power cycle", ret);
        return ret;
    }

    LOG_INF("RFD900x reconfigured (%u registers written), modem rebooting", (unsigned)n_writes);
    return 0;

abort:
    /* Back to the online state without AT&W: EEPROM keeps the config the
     * link was established with, so communications survive the failure. */
    (void)at_send(io, "ATO\r\n");
    return ret;
}
