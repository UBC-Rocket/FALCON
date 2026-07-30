/*
 * Unit tests for the RFD900x AT session engine (rfd900x_at.c) against a
 * scripted mock modem. This feature must never brick comms, so every
 * abort path is exercised: the invariants under test are
 *   - nothing is sent to the modem when validation fails
 *   - AT&W is never sent unless every prior command was acknowledged and
 *     read back correctly
 *   - ATZ is never sent unless AT&W was acknowledged
 *   - any in-session failure exits command mode with ATO
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include "rfd900x_at.h"
#include "uplink_config.h"

/* ---- Mock modem ------------------------------------------------------ */

/* The mock matches complete commands (CRLF-terminated lines, or the bare
 * "+++" escape) against a script; a match queues that step's scripted
 * reply as RX bytes. Unmatched commands get silence, like a real modem
 * that dropped out of command mode. */
struct script_step {
    const char *expect; /* command without CRLF, e.g. "ATS2=64" or "+++" */
    const char *reply;  /* raw bytes to queue, e.g. "OK\r\n" */
};

#define TX_LOG_MAX 1024
#define RX_QUEUE_MAX 512
#define CMD_MAX 64

static char tx_log[TX_LOG_MAX];
static size_t tx_len;

static char cur_cmd[CMD_MAX];
static size_t cur_len;

static const struct script_step *script;
static size_t script_len;
static size_t script_pos;

static uint8_t rx_queue[RX_QUEUE_MAX];
static size_t rx_head;
static size_t rx_tail;

static void mock_reset(const struct script_step *steps, size_t n_steps)
{
    memset(tx_log, 0, sizeof(tx_log));
    tx_len = 0;
    cur_len = 0;
    script = steps;
    script_len = n_steps;
    script_pos = 0;
    rx_head = 0;
    rx_tail = 0;
}

static void mock_queue_rx(const char *bytes)
{
    size_t len = strlen(bytes);

    zassert_true(rx_tail + len <= RX_QUEUE_MAX, "mock RX queue overflow");
    memcpy(&rx_queue[rx_tail], bytes, len);
    rx_tail += len;
}

static void mock_command_complete(void)
{
    cur_cmd[cur_len] = '\0';

    if (script_pos < script_len && strcmp(cur_cmd, script[script_pos].expect) == 0) {
        mock_queue_rx(script[script_pos].reply);
        script_pos++;
    }
    cur_len = 0;
}

static int mock_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];

        zassert_true(tx_len < TX_LOG_MAX - 1, "mock TX log overflow");
        tx_log[tx_len++] = c;

        if (c == '\n') {
            /* strip the CRLF terminator before matching */
            if (cur_len > 0 && cur_cmd[cur_len - 1] == '\r') {
                cur_len--;
            }
            mock_command_complete();
        } else if (cur_len < CMD_MAX - 1) {
            cur_cmd[cur_len++] = c;

            /* "+++" is sent with no CRLF */
            if (cur_len == 3 && memcmp(cur_cmd, "+++", 3) == 0) {
                mock_command_complete();
            }
        }
    }
    return 0;
}

static int mock_read_byte(uint8_t *byte, int32_t timeout_ms)
{
    if (rx_head < rx_tail) {
        *byte = rx_queue[rx_head++];
        return 0;
    }
    if (timeout_ms > 0) {
        /* Block like a real transport so the engine's deadline advances
         * (native_sim time only moves across sleeps) */
        k_sleep(K_MSEC(timeout_ms));
    }
    return -EAGAIN;
}

static const struct rfd900x_transport mock_transport = {
    .write = mock_write,
    .read_byte = mock_read_byte,
};

static bool script_done(void)
{
    return script_pos == script_len;
}

static bool tx_contains(const char *needle)
{
    return strstr(tx_log, needle) != NULL;
}

static void before_each(void *fixture)
{
    ARG_UNUSED(fixture);
    mock_reset(NULL, 0);
}

ZTEST_SUITE(rfd900x_at, NULL, NULL, before_each, NULL, NULL);

/* ---- Happy paths ----------------------------------------------------- */

ZTEST(rfd900x_at, test_full_config_happy_path)
{
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},
        /* modem echoes commands in AT mode; engine must skip the echo */
        {"ATS2=64", "ATS2=64\r\nOK\r\n"},
        {"ATS2?", "ATS2?\r\n64\r\n"},
        {"ATS3=25", "ATS3=25\r\nOK\r\n"},
        {"ATS3?", "25\r\n"},
        {"ATS4=27", "OK\r\n"},
        {"ATS4?", "27\r\n"},
        {"ATS8=915000", "OK\r\n"},
        {"ATS8?", "915000\r\n"},
        {"ATS9=920000", "OK\r\n"},
        {"ATS9?", "920000\r\n"},
        {"ATS10=20", "OK\r\n"},
        {"ATS10?", "20\r\n"},
        {"AT&W", "OK\r\n"},
        {"ATZ", ""},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_air_speed_kbps = true;
    cfg.air_speed_kbps = 64;
    cfg.has_net_id = true;
    cfg.net_id = 25;
    cfg.has_tx_power_dbm = true;
    cfg.tx_power_dbm = 27;
    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 915000;
    cfg.has_max_freq_khz = true;
    cfg.max_freq_khz = 920000;
    cfg.has_num_channels = true;
    cfg.num_channels = 20;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), 0, "session should succeed");
    zassert_true(script_done(), "every command should be sent, in order, ending with ATZ");
    zassert_false(tx_contains("ATO"), "no abort on the happy path");
}

ZTEST(rfd900x_at, test_partial_config_only_touches_named_registers)
{
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},  {"ATS4=30", "OK\r\n"}, {"ATS4?", "30\r\n"},
        {"AT&W", "OK\r\n"}, {"ATZ", ""},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_tx_power_dbm = true;
    cfg.tx_power_dbm = 30; /* upper boundary value */

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), 0, "session should succeed");
    zassert_true(script_done(), "sequence should complete");
    zassert_false(tx_contains("ATS2"), "unset fields must not be written");
    zassert_false(tx_contains("ATS8"), "unset fields must not be written");
}

ZTEST(rfd900x_at, test_zero_value_is_applied_when_field_present)
{
    /* NETID 0 is a real config (multipoint master net); presence must be
     * driven by the has_ flag, never by the value */
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},  {"ATS3=0", "OK\r\n"}, {"ATS3?", "0\r\n"},
        {"AT&W", "OK\r\n"}, {"ATZ", ""},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_net_id = true;
    cfg.net_id = 0;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), 0, "session should succeed");
    zassert_true(script_done(), "ATS3=0 should be sent and verified");
}

ZTEST(rfd900x_at, test_noise_lines_before_ok_are_ignored)
{
    static const struct script_step steps[] = {
        {"+++", "\r\nRSSI: 210/207  L/R\r\nOK\r\n"},
        {"ATS3=42", "ATS3=42\r\n\r\nOK\r\n"},
        {"ATS3?", "ATS3?\r\n42\r\n"},
        {"AT&W", "OK\r\n"},
        {"ATZ", ""},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_net_id = true;
    cfg.net_id = 42;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), 0,
                  "echo and noise lines must not break OK detection");
    zassert_true(script_done(), "sequence should complete");
}

ZTEST(rfd900x_at, test_multipoint_node_id_prefix_is_accepted)
{
    /* Regression, observed on hardware 2026-07-30: multipoint firmware tags
     * every response with the node that answered ("[1] OK", "[1] 905000").
     * An exact-match compare discarded a valid reply and the session timed
     * out at command-mode entry. */
    static const struct script_step steps[] = {
        {"+++", "[1] OK\r\n"},
        {"ATS8=905000", "[1] OK\r\n"},
        {"ATS8?", "[1] 905000\r\n"},
        {"AT&W", "[1] OK\r\n"},
        {"ATZ", ""},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 905000;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), 0,
                  "node-id prefixed replies must be accepted");
    zassert_true(script_done(), "sequence should complete");
}

ZTEST(rfd900x_at, test_prefixed_readback_mismatch_still_aborts)
{
    /* The prefix must be stripped for parsing only -- it must not make a
     * wrong readback value look acceptable. */
    static const struct script_step steps[] = {
        {"+++", "[1] OK\r\n"},
        {"ATS8=905000", "[1] OK\r\n"},
        {"ATS8?", "[1] 915000\r\n"}, /* modem clamped the value */
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 905000;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_not_equal(rfd900x_at_apply(&mock_transport, &cfg), 0,
                      "a clamped readback must still fail");
    zassert_false(tx_contains("AT&W"), "EEPROM must not be written");
    zassert_true(tx_contains("ATO"), "session must abort back to online state");
}

/* ---- Validation: the modem must never be touched --------------------- */

ZTEST(rfd900x_at, test_empty_config_rejected_without_touching_modem)
{
    RfdConfig cfg = RfdConfig_init_zero;

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL,
                  "empty config must be rejected");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");
}

ZTEST(rfd900x_at, test_min_freq_above_max_freq_rejected)
{
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 920000;
    cfg.has_max_freq_khz = true;
    cfg.max_freq_khz = 915000;

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL, "min > max must be rejected");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");
}

ZTEST(rfd900x_at, test_out_of_range_values_rejected)
{
    RfdConfig cfg;

    cfg = (RfdConfig)RfdConfig_init_zero;
    cfg.has_tx_power_dbm = true;
    cfg.tx_power_dbm = RFD_TX_POWER_DBM_MAX + 1;
    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL, "power out of range");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");

    cfg = (RfdConfig)RfdConfig_init_zero;
    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = RFD_FREQ_KHZ_MIN - 1;
    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL, "freq below band");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");

    cfg = (RfdConfig)RfdConfig_init_zero;
    cfg.has_num_channels = true;
    cfg.num_channels = RFD_NUM_CHANNELS_MAX + 1;
    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL, "channels out of range");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");

    cfg = (RfdConfig)RfdConfig_init_zero;
    cfg.has_net_id = true;
    cfg.net_id = RFD_NET_ID_MAX + 1;
    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EINVAL, "net_id out of range");
    zassert_equal(tx_len, 0, "nothing may be sent to the modem");
}

/* ---- Session failures: abort without committing ---------------------- */

ZTEST(rfd900x_at, test_no_ok_after_escape_aborts_before_any_command)
{
    /* Modem never answers "+++" (e.g. still in data mode) */
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_net_id = true;
    cfg.net_id = 25;

    mock_reset(NULL, 0);

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -ETIMEDOUT,
                  "entry timeout must fail the session");
    zassert_false(tx_contains("ATS"), "no register writes outside command mode");
    zassert_false(tx_contains("AT&W"), "AT&W must never be sent");

    /* ATO is sent even here. Silence does not prove the modem stayed in
     * data mode -- on hardware it answered in a form we failed to match,
     * and a modem left in command mode is a dead link. ATO in data mode is
     * harmless; skipping it when we are wrong is not. */
    zassert_true(tx_contains("ATO"), "modem must be returned to the online state");
}

ZTEST(rfd900x_at, test_error_reply_aborts_with_ato_and_no_eeprom_write)
{
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},
        {"ATS3=25", "ERROR\r\n"},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_net_id = true;
    cfg.net_id = 25;
    cfg.has_num_channels = true;
    cfg.num_channels = 20;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EIO,
                  "ERROR reply must fail the session");
    zassert_false(tx_contains("ATS10"), "later registers must not be attempted");
    zassert_false(tx_contains("AT&W"), "AT&W must never follow a failed command");
    zassert_false(tx_contains("ATZ"), "ATZ must never follow a failed command");
    zassert_true(tx_contains("ATO\r\n"), "abort must exit command mode with ATO");
}

ZTEST(rfd900x_at, test_timeout_mid_sequence_aborts_without_eeprom_write)
{
    /* First register acknowledged, then the modem goes silent */
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},
        {"ATS8=915000", "OK\r\n"},
        {"ATS8?", "915000\r\n"},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 915000;
    cfg.has_max_freq_khz = true;
    cfg.max_freq_khz = 920000;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -ETIMEDOUT,
                  "mid-sequence silence must fail the session");
    zassert_true(tx_contains("ATS9=920000"), "the failing command was attempted");
    zassert_false(tx_contains("AT&W"), "AT&W must never follow a timeout");
    zassert_false(tx_contains("ATZ"), "ATZ must never follow a timeout");
    zassert_true(tx_contains("ATO\r\n"), "abort must exit command mode with ATO");
}

ZTEST(rfd900x_at, test_readback_mismatch_aborts_without_eeprom_write)
{
    /* Modem acknowledges the write but silently clamped the value */
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},
        {"ATS8=915000", "OK\r\n"},
        {"ATS8?", "911000\r\n"},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_min_freq_khz = true;
    cfg.min_freq_khz = 915000;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -EIO,
                  "readback mismatch must fail the session");
    zassert_false(tx_contains("AT&W"), "AT&W must never follow a bad readback");
    zassert_true(tx_contains("ATO\r\n"), "abort must exit command mode with ATO");
}

ZTEST(rfd900x_at, test_unacknowledged_atw_never_reboots)
{
    /* Everything fine until AT&W gets no reply: EEPROM state unknown, so
     * the engine must not reboot the modem into it */
    static const struct script_step steps[] = {
        {"+++", "OK\r\n"},
        {"ATS3=25", "OK\r\n"},
        {"ATS3?", "25\r\n"},
    };
    RfdConfig cfg = RfdConfig_init_zero;

    cfg.has_net_id = true;
    cfg.net_id = 25;

    mock_reset(steps, ARRAY_SIZE(steps));

    zassert_equal(rfd900x_at_apply(&mock_transport, &cfg), -ETIMEDOUT,
                  "unacknowledged AT&W must fail the session");
    zassert_true(tx_contains("AT&W\r\n"), "AT&W was attempted");
    zassert_false(tx_contains("ATZ"), "ATZ must never follow an unacknowledged AT&W");
    zassert_true(tx_contains("ATO\r\n"), "abort must exit command mode with ATO");
}
