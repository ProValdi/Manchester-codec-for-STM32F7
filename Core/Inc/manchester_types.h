#ifndef MANCHESTER_TYPES_H
#define MANCHESTER_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "manchester_config.h"

typedef enum {
    MAN_ENCODING_STANDARD = 0,
    MAN_ENCODING_DIFFERENTIAL = 1
} man_encoding_t;

typedef enum {
    MAN_MODE_SINGLE = 0,
    MAN_MODE_STREAM = 1
} man_transfer_mode_t;

typedef enum {
    MAN_RATE_1_MBPS = 1000000u,
    MAN_RATE_2_MBPS = 2000000u,
    MAN_RATE_3_MBPS = 3000000u,
    MAN_RATE_4_MBPS = 4000000u
} man_bitrate_t;

enum {
    MAN_FLAG_STREAM = 1u << 0,
    MAN_FLAG_END = 1u << 1,
    MAN_FLAG_FEC = 1u << 2,
    MAN_FLAG_RESET = 1u << 3
};

typedef struct {
    man_encoding_t encoding;
    man_transfer_mode_t transfer_mode;
    uint32_t bitrate_bps;
    uint16_t max_payload;
    uint16_t max_single_message;
    uint8_t preamble_bytes;
    uint16_t sync_word;
    uint8_t glitch_filter_samples;
    uint32_t uart_idle_flush_ms;
    uint16_t uart_explicit_block_length;
    bool fec_enabled;
    bool tx_invert;
} man_runtime_config_t;

typedef struct {
    uint8_t flags;
    uint8_t seq;
    uint16_t length;
    uint8_t payload[MAN_MAX_PAYLOAD];
} man_packet_t;

typedef struct {
    volatile uint32_t uart_rx_bytes;
    volatile uint32_t manchester_tx_frames;
    volatile uint32_t manchester_rx_good_frames;
    volatile uint32_t crc_errors;
    volatile uint32_t sync_losses;
    volatile uint32_t dma_overruns;
    volatile uint32_t queue_overflows;
    volatile uint32_t uart_overruns;
    volatile uint32_t sequence_gaps;
    volatile uint32_t resynchronizations;
    volatile uint32_t line_code_errors;
    volatile uint32_t phase_errors;
    volatile uint32_t fec_errors;
    volatile uint32_t dropped_blocks;
} man_diagnostics_t;

static inline bool man_bitrate_is_valid(uint32_t value)
{
    return value == MAN_RATE_1_MBPS || value == MAN_RATE_2_MBPS ||
           value == MAN_RATE_3_MBPS || value == MAN_RATE_4_MBPS;
}

#endif
