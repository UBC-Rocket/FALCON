#ifndef UPLINK_CONFIG_H
#define UPLINK_CONFIG_H

/*
 * Two-way radio uplink: NOT-YET-CONFIRMED constants, collected in one place.
 *
 * Values below are PLACEHOLDERS pending confirmation from the team unless
 * marked CONFIRMED. When a real value is known, changing it here is the
 * only edit needed.
 *
 * Confirmed by the hardware team (live in the devicetree,
 * boards/ubcrocket/polarity/ubcrocket_polarity.dts):
 *   - vtx-pwr GPIO: PD6 (active level assumed high)
 *   - runcam-uart port: UART4 (pin pair still unconfirmed, PA0/PA1 assumed)
 */

/*
 * CONFIRMED (2026-07-26, ulysses-gnss-radio Core/Inc/protocol_config.h):
 * SPI opcode for reading received radio bytes from the ulysses-gnss-radio
 * board -- CMD_RADIO_RX_FIFO, oldest message first so uplinked commands are
 * processed in the order the ground sent them (0x01 is the LIFO variant).
 * Framing mirrors the GPS read (0x05): [CMD:1][DUMMY:4][PAYLOAD:256].
 * The GNSS board splits the radio UART stream on 0x00, strips the
 * terminator, and zero-pads each message into a 256-byte queue slot, so
 * the payload holds one COBS-encoded frame (protobuf + CRC16) whose zero
 * padding acts as the delimiter; all-zero when nothing is pending.
 */
#define UPLINK_SPI_CMD_RADIO_RX 0x02

/*
 * PLACEHOLDER: RunCam UART command set -- waiting on David to confirm the
 * exact protocol. Values below follow the public RunCam Device Protocol:
 * frame = [0xCC][command id][payload...][CRC8 poly 0xD5 over all prior bytes].
 * Note the protocol defines no response for CAMERA_CONTROL, so recording
 * state is updated optimistically until an ack mechanism is confirmed.
 *
 * DORMANT: the RunCam has auto-recording enabled, so nothing currently
 * sends these frames -- recording follows the VTX power rail. The runcam
 * module is kept for when the protobufs are reworked to carry RunCam
 * commands.
 */
#define RUNCAM_PROTO_HEADER 0xCC
#define RUNCAM_CMD_CAMERA_CONTROL 0x01
#define RUNCAM_ACTION_START_RECORDING 0x03
#define RUNCAM_ACTION_STOP_RECORDING 0x04
#define RUNCAM_CRC8_POLY 0xD5

/*
 * RFD900x remote reconfiguration (checklist Phase 4).
 *
 * PLACEHOLDER: the physical path FALCON -> modem serial is unconfirmed
 * (direct UART assumed, see the rfd-uart alias in the devicetree; if the
 * real path is AT-passthrough via the GNSS board, swap the transport in
 * firmware/src/radio/rfd900x.c -- the AT session engine is
 * transport-agnostic).
 *
 * Timing follows the SiK/RFD900x AT spec: 1 s of serial silence before
 * "+++" (no CR/LF), and roughly 1 s more before the modem answers "OK".
 * The #ifndef guards let the unit tests shrink the timing so the
 * timeout-path tests run fast.
 */
#ifndef RFD_AT_GUARD_TIME_MS
#define RFD_AT_GUARD_TIME_MS 1100
#endif
#ifndef RFD_AT_ENTER_TIMEOUT_MS
#define RFD_AT_ENTER_TIMEOUT_MS 2500
#endif
#ifndef RFD_AT_RESPONSE_TIMEOUT_MS
#define RFD_AT_RESPONSE_TIMEOUT_MS 1500
#endif

/*
 * PLACEHOLDER: RfdConfig validation bounds -- confirm against the modem's
 * actual firmware limits with the team. Out-of-range values are rejected
 * before the AT session starts so they can never reach the modem.
 * Frequency band is the RFD900x 902-928 MHz ISM band; NETID/power/air
 * speed/channel bounds follow the SiK S-register documentation.
 */
#define RFD_FREQ_KHZ_MIN 902000
#define RFD_FREQ_KHZ_MAX 928000
#define RFD_NET_ID_MAX 499
#define RFD_TX_POWER_DBM_MAX 30
#define RFD_AIR_SPEED_MIN 4
#define RFD_AIR_SPEED_MAX 750
#define RFD_NUM_CHANNELS_MIN 1
#define RFD_NUM_CHANNELS_MAX 50

#endif /* UPLINK_CONFIG_H */
