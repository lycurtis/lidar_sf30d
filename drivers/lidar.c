// drivers/lidar.c
/**
 * LightWare SF30/D LiDAR — binary protocol driver.
 * Implements the framing, CRC, and parsing described in
 * SF30/D Product Guide rev 3.3 §10.1.
 */

#include "lidar.h"
#include <string.h>

/* --- Update-rate lookup (CMDs 76, 79, 80) ------------------------------- */

const uint16_t SF_UPDATE_RATE_HZ[10] = {
    20010u, 10005u, 5002u, 2501u, 1250u,
    625u,   312u,   156u,  78u,   39u,
};

/* --- CRC-16-CCITT 0x1021, LightWare reference implementation (§10.1.2) -- */

static uint16_t sf_crc16_continue(uint16_t crc, const uint8_t* data, uint16_t size)
{
    for (uint32_t i = 0; i < size; ++i) {
        uint16_t code = crc >> 8;
        code ^= data[i];
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
    }
    return crc;
}

static inline uint16_t sf_crc16(const uint8_t* data, uint16_t size)
{
    return sf_crc16_continue(0u, data, size);
}

/* --- Parser ------------------------------------------------------------- */

void sf_parser_init(sf_parse_ctx_t* p)
{
    memset(p, 0, sizeof(*p));
}

/* Slide p->buf past the leading start byte to the next 0xAA, if any.
 * Implements the §10.1.3 resync rule: after a length/CRC error, restart
 * from one byte after the original start byte. */
static void sf_resync_after_error(sf_parse_ctx_t* p)
{
    /* Skip the bad start byte at index 0; hunt for the next one. */
    uint16_t i = 1u;
    while (i < p->buf_len && p->buf[i] != SF_START_BYTE) {
        ++i;
    }

    if (i >= p->buf_len) {
        p->buf_len = 0u;
        return;
    }

    p->buf_len -= i;
    memmove(p->buf, &p->buf[i], p->buf_len);
}

/* Try to validate one packet at the head of p->buf. Returns:
 *   true  -> packet is valid; data copied into last_packet, consumed bytes
 *            removed from buf.
 *   false -> either need more bytes (buf left intact) or an error caused a
 *            slide (buf advanced past the bad start byte).
 *
 * On the "need more bytes" path the caller distinguishes by checking that
 * buf_len did not change. The wrapper loop in sf_parser_feed re-tries after
 * any slide, so the caller doesn't need to. */
static bool sf_try_parse_one(sf_parse_ctx_t* p, bool* slid)
{
    *slid = false;

    if (p->buf_len < 3u) {
        /* need flags */
        return false;
    }

    uint16_t flags = (uint16_t)p->buf[1] | ((uint16_t)p->buf[2] << 8);
    uint16_t payload_len = flags >> 6;

    /* §10.1.1: payload length is 1..1023 inclusive. */
    if (payload_len < 1u || payload_len > SF_PAYLOAD_MAX) {
        sf_resync_after_error(p);
        *slid = true;
        return false;
    }

    uint16_t total = 3u + payload_len + 2u;
    if (p->buf_len < total) {
        /* need payload + CRC */
        return false;
    }

    uint16_t rx_crc = (uint16_t)p->buf[total - 2u] | ((uint16_t)p->buf[total - 1u] << 8);
    uint16_t calc_crc = sf_crc16(p->buf, total - 2u);

    if (rx_crc != calc_crc) {
        sf_resync_after_error(p);
        *slid = true;
        return false;
    }

    /* Valid packet: payload starts at buf[3], cmd_id == buf[3]. */
    p->last_packet.cmd_id      = p->buf[3];
    p->last_packet.write       = (flags & 0x1u) != 0u;
    p->last_packet.payload_len = payload_len - 1u;
    if (p->last_packet.payload_len > 0u) {
        memcpy(p->last_packet.payload, &p->buf[4], p->last_packet.payload_len);
    }

    /* Drop consumed bytes; keep any trailing bytes for the next packet. */
    uint16_t leftover = (uint16_t)(p->buf_len - total);
    if (leftover > 0u) {
        memmove(p->buf, &p->buf[total], leftover);
    }
    p->buf_len = leftover;

    return true;
}

bool sf_parser_feed(sf_parse_ctx_t* p, uint8_t byte)
{
    /* Empty buffer: only a start byte is interesting. */
    if (p->buf_len == 0u) {
        if (byte == SF_START_BYTE) {
            p->buf[0] = byte;
            p->buf_len = 1u;
        }
        return false;
    }

    /* Defensive: should never trip — sf_try_parse_one consumes whenever it
     * detects a valid packet, and slides past garbage on errors. */
    if (p->buf_len >= SF_PACKET_MAX) {
        p->buf_len = 0u;
        if (byte == SF_START_BYTE) {
            p->buf[0] = byte;
            p->buf_len = 1u;
        }
        return false;
    }

    p->buf[p->buf_len++] = byte;

    /* Loop because each error slides past one start byte and we may then
     * find another candidate packet (or another error) within the same
     * residual buffer. */
    for (;;) {
        bool slid = false;
        if (sf_try_parse_one(p, &slid)) {
            return true;
        }
        if (!slid) {
            return false;
        }
        if (p->buf_len == 0u) {
            return false;
        }
    }
}

/* --- Request building --------------------------------------------------- */

static uint16_t sf_build_packet(uint8_t cmd_id,
                                bool write,
                                const uint8_t* data,
                                uint16_t data_len,
                                uint8_t* buf,
                                uint16_t buf_size)
{
    uint16_t payload_len = 1u + data_len;             /* cmd_id + data */
    uint16_t total = 3u + payload_len + 2u;           /* start + flags(2) + payload + CRC(2) */

    if (total > buf_size || payload_len > SF_PAYLOAD_MAX) {
        return 0u;
    }

    buf[0] = SF_START_BYTE;

    uint16_t flags = (payload_len << 6) | (write ? 1u : 0u);
    buf[1] = (uint8_t)(flags & 0xFFu);
    buf[2] = (uint8_t)((flags >> 8) & 0xFFu);

    buf[3] = cmd_id;
    if (data_len > 0u && data != (void*)0) {
        memcpy(&buf[4], data, data_len);
    }

    uint16_t crc = sf_crc16(buf, 3u + payload_len);
    buf[3u + payload_len]      = (uint8_t)(crc & 0xFFu);
    buf[3u + payload_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    return total;
}

uint16_t sf_build_read_request(uint8_t cmd_id, uint8_t* buf, uint16_t buf_size)
{
    return sf_build_packet(cmd_id, false, (void*)0, 0u, buf, buf_size);
}

uint16_t sf_build_write_request(uint8_t cmd_id,
                                const uint8_t* data,
                                uint16_t data_len,
                                uint8_t* buf,
                                uint16_t buf_size)
{
    return sf_build_packet(cmd_id, true, data, data_len, buf, buf_size);
}

/* --- Save-parameters helpers (§10.1.5) ---------------------------------- */

uint16_t sf_build_save_parameters_request(uint16_t token,
                                          uint8_t* buf,
                                          uint16_t buf_size)
{
    uint8_t data[2];
    data[0] = (uint8_t)(token & 0xFFu);
    data[1] = (uint8_t)((token >> 8) & 0xFFu);
    return sf_build_packet(SF_CMD_SAVE_PARAMETERS, true, data, sizeof(data), buf, buf_size);
}

bool sf_decode_token(const uint8_t* data, uint16_t len, uint16_t* token_out)
{
    if (len < 2u || token_out == (void*)0 || data == (void*)0) {
        return false;
    }
    *token_out = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}
