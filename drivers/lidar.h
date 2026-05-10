// drivers/lidar.h
/**
 * LightWare SF30/D LiDAR — binary protocol driver.
 *
 * Implements the packet framing, CRC-16-CCITT (LightWare variant), and
 * a byte-fed parser from SF30/D Product Guide rev 3.3 §10.1.
 *
 * The parser is spec-compliant for the resync rule in §10.1.3: on either
 * "invalid packet length" or "checksum is invalid", it rolls the byte
 * stream back to one byte after the start byte and resumes hunting.
 *
 * The driver is transport-agnostic — callers feed it raw bytes from any
 * UART (or test harness) one at a time.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* --- Wire format constants ---------------------------------------------- */

/* Spec §10.1.1: payload length field is 10 bits (flags bits 6–15) and the
 * spec wording is "between 1 and 1023 bytes, inclusive". */
#define SF_PAYLOAD_MAX  1023u

#define SF_START_BYTE   0xAAu

/* Largest legal on-wire packet: start(1) + flags(2) + payload(<=1023) + CRC(2). */
#define SF_PACKET_MAX   (1u + 2u + SF_PAYLOAD_MAX + 2u)

/* --- Command IDs (§10.1.6) ---------------------------------------------- */

#define SF_CMD_PRODUCT_NAME             0
#define SF_CMD_HARDWARE_VERSION         1
#define SF_CMD_FIRMWARE_VERSION         2
#define SF_CMD_SERIAL_NUMBER            3
#define SF_CMD_USER_DATA                9
#define SF_CMD_TOKEN                    10
#define SF_CMD_SAVE_PARAMETERS          12
#define SF_CMD_RESET                    14
#define SF_CMD_SYNC_OUTPUT              28
#define SF_CMD_DISTANCE_OUTPUT          29
#define SF_CMD_STREAM                   30
#define SF_CMD_FULL_SPEED_DISTANCE      40
#define SF_CMD_DISTANCE_DATA            44
#define SF_CMD_LASER_FIRING             50
#define SF_CMD_TEMPERATURE              55
#define SF_CMD_OUTPUT_DATA_TYPE         70
#define SF_CMD_UPDATE_RATE              76
#define SF_CMD_RETURN_MODE              77
#define SF_CMD_USB_OUTPUT_RATE          78
#define SF_CMD_SERIAL_OUTPUT_RATE       79
#define SF_CMD_ANALOG_OUTPUT_RATE       80
#define SF_CMD_NOISE                    85
#define SF_CMD_MEDIAN_FILTER_ENABLE     86
#define SF_CMD_MEDIAN_FILTER_SIZE       87
#define SF_CMD_SMOOTHING_FILTER_ENABLE  88
#define SF_CMD_SMOOTHING_FACTOR         89
#define SF_CMD_BAUD_RATE                91
#define SF_CMD_I2C_ADDRESS              92
#define SF_CMD_SENSITIVITY_OFFSET       98
#define SF_CMD_ALARM_DISTANCE           105
#define SF_CMD_ALARM_HYSTERESIS         106
#define SF_CMD_ALARM_LATCH              107
#define SF_CMD_ALARM_STATE              108
#define SF_CMD_ANALOG_RANGE             110
#define SF_CMD_LED                      111
#define SF_CMD_LOST_SIGNAL_CONFIRMS     115
#define SF_CMD_ZERO_OFFSET              116

/* --- CMD 30 stream modes ------------------------------------------------ */

#define SF_STREAM_DISABLED              0u
#define SF_STREAM_DISTANCE_DATA         5u   /* streams CMD 44 */
#define SF_STREAM_FULL_SPEED_DISTANCE   11u  /* streams CMD 40 */

/* --- CMD 29 distance-output bitmask (controls CMD 44 contents) ---------- */

#define SF_DIST_OUT_FIRST_RAW       (1u << 0)
#define SF_DIST_OUT_FIRST_FILTER    (1u << 1)
#define SF_DIST_OUT_FIRST_STRENGTH  (1u << 2)
#define SF_DIST_OUT_LAST_RAW        (1u << 3)
#define SF_DIST_OUT_LAST_FILTER     (1u << 4)
#define SF_DIST_OUT_LAST_STRENGTH   (1u << 5)
#define SF_DIST_OUT_BACKGROUND      (1u << 6)
#define SF_DIST_OUT_TEMPERATURE     (1u << 7)

/* --- Update-rate code lookup (CMDs 76, 79, 80) -------------------------- */

/* Index = command value (0–9), value = samples per second.
 * Source: §10.1.6 update-rate table for CMD 76 / 79 / 80. */
extern const uint16_t SF_UPDATE_RATE_HZ[10];

/* --- Parsed packet ------------------------------------------------------ */

typedef struct {
    uint8_t  cmd_id;                            /* command ID (first payload byte) */
    bool     write;                             /* flags bit 0: write (true) or read (false) */
    uint16_t payload_len;                       /* data length, i.e. bytes after cmd_id */
    uint8_t  payload[SF_PAYLOAD_MAX - 1u];      /* data bytes (cmd_id stripped) */
} sf_packet_t;

/* --- Parser context ----------------------------------------------------- */

typedef struct {
    /* Rolling byte buffer holding the in-flight packet from the start byte
     * onward. On length or CRC error, the parser slides past the start byte
     * to the next 0xAA in this buffer (spec §10.1.3). */
    uint8_t  buf[SF_PACKET_MAX];
    uint16_t buf_len;

    sf_packet_t last_packet;
} sf_parse_ctx_t;

/* --- Public API --------------------------------------------------------- */

void sf_parser_init(sf_parse_ctx_t* p);

/* Feed one received byte. Returns true exactly once when a full, CRC-valid
 * packet has been assembled; the result is in p->last_packet. */
bool sf_parser_feed(sf_parse_ctx_t* p, uint8_t byte);

/* --- Request builders --------------------------------------------------- */

/* Build a read request (no data). Returns the on-wire length, or 0 on
 * insufficient buffer. */
uint16_t sf_build_read_request(uint8_t cmd_id, uint8_t* buf, uint16_t buf_size);

/* Build a write request. data may be NULL iff data_len == 0. */
uint16_t sf_build_write_request(uint8_t cmd_id,
                                const uint8_t* data,
                                uint16_t data_len,
                                uint8_t* buf,
                                uint16_t buf_size);

/* --- Save-parameters helpers (§10.1.5) ---------------------------------- */

/* Build a "save parameters" (CMD 12) write request carrying the safety
 * token previously read from CMD 10. The token is 2 bytes, little endian. */
uint16_t sf_build_save_parameters_request(uint16_t token,
                                          uint8_t* buf,
                                          uint16_t buf_size);

/* Decode the 2-byte uint16 little-endian token returned by a CMD 10 read.
 * Returns true on success. */
bool sf_decode_token(const uint8_t* data, uint16_t len, uint16_t* token_out);
