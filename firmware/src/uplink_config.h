#ifndef UPLINK_CONFIG_H
#define UPLINK_CONFIG_H

/*
 * Two-way radio uplink: NOT-YET-CONFIRMED constants, collected in one place.
 *
 * Every value below is a PLACEHOLDER pending confirmation from the team.
 * When the real value is known, changing it here is the only edit needed.
 *
 * Placeholders that live in the devicetree instead of this header
 * (boards/ubcrocket/polarity/ubcrocket_polarity.dts):
 *   - vtx-pwr GPIO (currently PB0, active high) -- confirm with hardware team
 *   - runcam-uart port/pins/baud (currently USART2 on PA2/PA3 @ 115200)
 *     -- confirm with hardware team
 */

/*
 * PLACEHOLDER: SPI opcode for reading received radio bytes from the
 * ulysses-gnss-radio board. Confirm opcode and framing with the
 * GNSS-firmware owner. Framing is assumed to mirror the GPS read (0x05):
 * [CMD:1][DUMMY:4][PAYLOAD:256], payload holding one COBS-encoded frame
 * (protobuf + CRC16, 0x00 terminated), zero-filled when nothing is pending.
 */
#define UPLINK_SPI_CMD_RADIO_RX 0x06

/*
 * PLACEHOLDER: RunCam UART command set -- waiting on David to confirm the
 * exact protocol. Values below follow the public RunCam Device Protocol:
 * frame = [0xCC][command id][payload...][CRC8 poly 0xD5 over all prior bytes].
 * Note the protocol defines no response for CAMERA_CONTROL, so recording
 * state is updated optimistically until an ack mechanism is confirmed.
 */
#define RUNCAM_PROTO_HEADER 0xCC
#define RUNCAM_CMD_CAMERA_CONTROL 0x01
#define RUNCAM_ACTION_START_RECORDING 0x03
#define RUNCAM_ACTION_STOP_RECORDING 0x04
#define RUNCAM_CRC8_POLY 0xD5

#endif /* UPLINK_CONFIG_H */
