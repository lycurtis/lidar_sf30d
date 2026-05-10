/**
 * @file main.c
 * @brief SF30/D LiDAR live readout for NUCLEO-G431KB
 *
 *   USART1: PB6 TX / PB7 RX  — 115200 baud  (debug serial via FTDI)
 *   USART2: PA2 TX / PA3 RX  — 921600 baud  (LiDAR)
 *
 * Performs the §10.1.4 handshake (CMD 0 twice — first request after
 * powerup is ignored by the sensor), reads the configured serial output
 * rate (CMD 79) so it can be displayed instead of hard-coded, starts the
 * CMD 40 full-speed distance stream, and once per second prints:
 *
 *   [   3.000s] cfg=20010 Hz  meas=19998 Hz  d=  342 cm  pkts=2502
 *
 * The "meas" field is computed from the actual incoming readings, so you
 * can change the SF30/D's serial output rate via the configurator and
 * watch the measured value follow.
 */

#include "stm32g4xx.h"
#include "init.h"
#include "usart.h"
#include "lidar.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Helpers: timing, USART, and SF30/D request/response                       */
/* ------------------------------------------------------------------------ */

static uint32_t ms_now(void)
{
    return BSP_GetTick();
}

static uint32_t ms_since(uint32_t t0)
{
    return ms_now() - t0;
}

static void ms_sleep(uint32_t ms)
{
    uint32_t t0 = ms_now();
    while (ms_since(t0) < ms) {
        __NOP();
    }
}

/* Send a request and wait up to timeout_ms for a response with the
 * matching command ID. Any in-stream packets unrelated to cmd_id are
 * silently discarded. Returns true on a matching response. */
static bool lidar_request(sf_parse_ctx_t* ctx,
                          uint8_t cmd_id,
                          bool write,
                          const uint8_t* data,
                          uint16_t data_len,
                          sf_packet_t* out,
                          uint32_t timeout_ms)
{
    uint8_t  tx[16];
    uint16_t tx_len;

    if (write) {
        tx_len = sf_build_write_request(cmd_id, data, data_len, tx, sizeof(tx));
    } else {
        tx_len = sf_build_read_request(cmd_id, tx, sizeof(tx));
    }
    if (tx_len == 0u) {
        return false;
    }

    usart_write(BSP_USART_LIDAR, tx, tx_len);

    uint32_t deadline = ms_now() + timeout_ms;
    while (ms_now() < deadline) {
        if (!usart_rx_ready(BSP_USART_LIDAR)) {
            continue;
        }
        uint8_t b = usart_read_byte(BSP_USART_LIDAR);
        if (!sf_parser_feed(ctx, b)) {
            continue;
        }
        if (ctx->last_packet.cmd_id == cmd_id) {
            if (out) *out = ctx->last_packet;
            return true;
        }
        /* otherwise ignore (e.g., stale streamed packet) and keep waiting */
    }
    return false;
}

/* Same as lidar_request but retries up to `attempts` times. */
static bool lidar_request_retry(sf_parse_ctx_t* ctx,
                                uint8_t cmd_id,
                                bool write,
                                const uint8_t* data,
                                uint16_t data_len,
                                sf_packet_t* out,
                                uint32_t timeout_ms,
                                uint8_t attempts)
{
    for (uint8_t i = 0; i < attempts; ++i) {
        if (lidar_request(ctx, cmd_id, write, data, data_len, out, timeout_ms)) {
            return true;
        }
    }
    return false;
}

/* §10 handshake: send Product Name once (no response expected after
 * powerup), then send it again and accept the response. */
static bool lidar_handshake(sf_parse_ctx_t* ctx, char product_out[17])
{
    sf_packet_t resp;

    /* 1st probe: discarded by sensor in the post-powerup state. */
    (void)lidar_request(ctx, SF_CMD_PRODUCT_NAME, false, NULL, 0u, NULL, 200u);

    for (uint8_t i = 0; i < 4; ++i) {
        if (lidar_request(ctx, SF_CMD_PRODUCT_NAME, false, NULL, 0u, &resp, 400u)) {
            uint16_t n = resp.payload_len < 16u ? resp.payload_len : 16u;
            memcpy(product_out, resp.payload, n);
            product_out[n] = '\0';
            for (uint16_t j = 0; j < n; ++j) {
                if (product_out[j] == '\0') {
                    /* terminate at first NUL */
                    break;
                }
            }
            return true;
        }
        ms_sleep(50u);
    }
    return false;
}

/* Read CMD 76 (Update rate, the SF30/D's internal sampling/measurement
 * rate) and translate the uint8 code to samples/sec via SF_UPDATE_RATE_HZ.
 * Returns 0 on failure.
 *
 * Per §10.1.6, CMD 40 ("Full speed distance in cm") streams "at the
 * measurement update rate" — i.e. CMD 76, NOT CMD 79. CMD 79 only
 * throttles the legacy ASCII serial mode (CMD 70 = 2). */
static uint16_t lidar_read_measurement_rate_hz(sf_parse_ctx_t* ctx)
{
    sf_packet_t resp;
    if (!lidar_request_retry(ctx, SF_CMD_UPDATE_RATE, false, NULL, 0u,
                             &resp, 250u, 3)) {
        return 0u;
    }
    if (resp.payload_len < 1u) {
        return 0u;
    }
    uint8_t code = resp.payload[0];
    if (code >= (sizeof(SF_UPDATE_RATE_HZ) / sizeof(SF_UPDATE_RATE_HZ[0]))) {
        return 0u;
    }
    return SF_UPDATE_RATE_HZ[code];
}

/* Pack a uint32 little-endian for write payloads. */
static void pack_u32_le(uint32_t v, uint8_t out[4])
{
    out[0] = (uint8_t)(v & 0xFFu);
    out[1] = (uint8_t)((v >> 8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* ------------------------------------------------------------------------ */
/* main                                                                     */
/* ------------------------------------------------------------------------ */

int main(void)
{
    BSP_Init();

    usart_config_t debug = {
        .instance  = BSP_USART_DEBUG,
        .baud_rate = BSP_USART_DEBUG_BAUD,
        .stop_bits = 1,
    };
    usart_init(&debug);

    usart_config_t lidar_cfg = {
        .instance  = BSP_USART_LIDAR,
        .baud_rate = BSP_USART_LIDAR_BAUD,
        .stop_bits = 1,
    };
    usart_init(&lidar_cfg);

    /* printf is line-buffered by default in newlib-nano; switch to
     * unbuffered so each line appears immediately on USART1. */
    setvbuf(stdout, NULL, _IONBF, 0);

    static sf_parse_ctx_t ctx;
    sf_parser_init(&ctx);

    printf("\r\n=== SF30/D LiDAR live readout ===\r\n");
    printf("Debug : USART1 PB6/PB7 @ %lu baud\r\n", (unsigned long)BSP_USART_DEBUG_BAUD);
    printf("Sensor: USART2 PA2/PA3 @ %lu baud\r\n", (unsigned long)BSP_USART_LIDAR_BAUD);

    /* Let the SF30/D settle and pick its interface from our first byte. */
    ms_sleep(2000u);
    /* Discard any garbage bytes that arrived during settling. */
    while (usart_rx_ready(BSP_USART_LIDAR)) {
        (void)usart_read_byte(BSP_USART_LIDAR);
    }

    char product[17] = {0};
    if (!lidar_handshake(&ctx, product)) {
        printf("ERROR: handshake failed (no Product Name response)\r\n");
        for (;;) {
            GPIOA->ODR ^= GPIO_ODR_OD7;
            ms_sleep(100u);
        }
    }
    printf("Handshake OK: \"%s\"\r\n", product);

    /* Make sure no stream is active so the rate read has a clean reply. */
    {
        uint8_t v[4];
        pack_u32_le(SF_STREAM_DISABLED, v);
        (void)lidar_request_retry(&ctx, SF_CMD_STREAM, true, v, sizeof(v),
                                  NULL, 200u, 2);
    }
    ms_sleep(20u);
    while (usart_rx_ready(BSP_USART_LIDAR)) {
        (void)usart_read_byte(BSP_USART_LIDAR);
    }
    sf_parser_init(&ctx);

    /* Read the configured measurement update rate so we can display it.
     * CMD 76 is the rate that actually governs CMD 40 streaming (per
     * §10.1.6); it's the value persisted in flash by the configurator. */
    uint16_t configured_hz = lidar_read_measurement_rate_hz(&ctx);
    if (configured_hz == 0u) {
        printf("WARN : could not read CMD 76 (update rate)\r\n");
    } else {
        printf("CMD 76 measurement update rate (configured) : %u Hz\r\n",
               (unsigned)configured_hz);
    }

    /* Start the CMD 40 full-speed distance stream. The sensor will now
     * push CMD 40 packets at the configured update rate without further
     * requests from us. (CMD 30 is RAM-only — no save needed.) */
    {
        uint8_t v[4];
        pack_u32_le(SF_STREAM_FULL_SPEED_DISTANCE, v);
        if (!lidar_request_retry(&ctx, SF_CMD_STREAM, true, v, sizeof(v),
                                 NULL, 250u, 3)) {
            printf("ERROR: failed to start CMD 40 stream\r\n");
            for (;;) {
                GPIOA->ODR ^= GPIO_ODR_OD7;
                ms_sleep(100u);
            }
        }
    }
    printf("Streaming CMD 40 (full-speed distance). Reporting every 1 s.\r\n");
    printf("--------------------------------------------------\r\n");

    /* ---------------------- Streaming + reporting ---------------------- */

    sf_parser_init(&ctx);

    uint32_t t_start         = ms_now();
    uint32_t t_next_log      = t_start + 1000u;
    uint32_t t_next_heartbeat = t_start + 250u;

    uint32_t readings_in_window = 0u;
    uint32_t packets_in_window  = 0u;
    int16_t  last_distance_cm   = 0;
    bool     have_distance      = false;

    for (;;) {
        uint32_t now = ms_now();

        if (now >= t_next_heartbeat) {
            t_next_heartbeat += 250u;
            GPIOA->ODR ^= GPIO_ODR_OD7;
        }

        /* Drain everything currently in USART2 RDR. */
        while (usart_rx_ready(BSP_USART_LIDAR)) {
            uint8_t b = usart_read_byte(BSP_USART_LIDAR);
            if (!sf_parser_feed(&ctx, b)) {
                continue;
            }
            if (ctx.last_packet.cmd_id != SF_CMD_FULL_SPEED_DISTANCE) {
                continue;
            }

            /* CMD 40 payload: [n: int8][distance_1: int16] ... [distance_n: int16] */
            const sf_packet_t* pkt = &ctx.last_packet;
            if (pkt->payload_len < 1u) {
                continue;
            }
            uint8_t n = pkt->payload[0];
            if (n == 0u) {
                continue;
            }

            uint16_t off = 1u;
            uint16_t parsed = 0u;
            for (uint8_t i = 0; i < n; ++i) {
                if (off + 2u > pkt->payload_len) {
                    break;
                }
                int16_t d = (int16_t)((uint16_t)pkt->payload[off] |
                                      ((uint16_t)pkt->payload[off + 1u] << 8));
                last_distance_cm = d;
                have_distance = true;
                off += 2u;
                ++parsed;
            }
            readings_in_window += parsed;
            packets_in_window  += 1u;
        }

        if (now >= t_next_log) {
            uint32_t window_ms = now - (t_next_log - 1000u);
            t_next_log += 1000u;

            uint32_t meas_hz = (readings_in_window * 1000u) /
                               (window_ms == 0u ? 1u : window_ms);
            uint32_t t_s_x1000 = now - t_start;

            if (have_distance) {
                printf("[%4lu.%03lus] cfg=%5u Hz  meas=%5lu Hz  d=%5d cm  pkts=%lu\r\n",
                       (unsigned long)(t_s_x1000 / 1000u),
                       (unsigned long)(t_s_x1000 % 1000u),
                       (unsigned)configured_hz,
                       (unsigned long)meas_hz,
                       (int)last_distance_cm,
                       (unsigned long)packets_in_window);
            } else {
                printf("[%4lu.%03lus] cfg=%5u Hz  meas=%5lu Hz  (waiting for data)\r\n",
                       (unsigned long)(t_s_x1000 / 1000u),
                       (unsigned long)(t_s_x1000 % 1000u),
                       (unsigned)configured_hz,
                       (unsigned long)meas_hz);
            }

            readings_in_window = 0u;
            packets_in_window  = 0u;
        }
    }
}
