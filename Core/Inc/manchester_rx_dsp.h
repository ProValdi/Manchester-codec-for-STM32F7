#ifndef MANCHESTER_RX_DSP_H
#define MANCHESTER_RX_DSP_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "manchester_fec.h"
#include "manchester_frame.h"
#include "manchester_types.h"

typedef void (*man_rx_frame_callback_t)(void *user, const man_packet_t *packet);

typedef enum {
    MAN_RX_SEARCH = 0,
    MAN_RX_BODY = 1
} man_rx_parse_state_t;

typedef struct {
    uint8_t pair_phase;
    bool have_first;
    uint8_t first_chip;
    bool have_previous_second;
    uint8_t previous_second;

    man_rx_parse_state_t parse_state;
    uint8_t pattern_bits[(MAN_PREAMBLE_BYTES_MAX + 2u) * 8u];
    uint8_t pattern_fail[(MAN_PREAMBLE_BYTES_MAX + 2u) * 8u];
    uint16_t pattern_length;
    uint16_t pattern_match;

    uint8_t body[MAN_BODY_MAX_BYTES];
    uint16_t body_count;
    uint16_t body_expected;
    uint8_t fec_ctx[MAN_FEC_CONTEXT_BYTES];
} man_rx_path_t;

#define MAN_GLITCH_LUT_STATES (2u * (MAN_GLITCH_FILTER_MAX_SAMPLES + 1u))

typedef struct {
    man_runtime_config_t cfg;
    uint32_t sample_rate_hz;
    uint32_t chip_rate_hz;
    uint32_t chip_period_q16;

    /* Byte LUT turns 8 raw samples into 8 filtered samples plus the next filter state. */
    uint8_t glitch_out[MAN_GLITCH_LUT_STATES][256u];
    uint8_t glitch_next[MAN_GLITCH_LUT_STATES][256u];
    uint8_t glitch_state;
    uint8_t glitch_state_count;
    bool glitch_initialized;

    uint8_t previous_filtered_level;
    bool previous_filtered_valid;
    uint64_t raw_sample_index;
    uint64_t timeline_position;
    uint8_t timeline_level;
    bool timeline_valid;

    bool clock_locked;
    int64_t grid_origin_q16;
    int64_t next_chip_center_q16;
    uint64_t next_chip_index;
    uint8_t consecutive_phase_errors;

    man_rx_path_t paths[2];
    const man_fec_codec_t *fec;
    man_rx_frame_callback_t callback;
    void *callback_user;
    man_diagnostics_t *diag;
    bool loss_pending;
} man_rx_decoder_t;

bool man_rx_decoder_init(man_rx_decoder_t *decoder,
                         const man_runtime_config_t *cfg,
                         uint32_t sample_rate_hz,
                         const man_fec_codec_t *fec,
                         man_diagnostics_t *diag,
                         man_rx_frame_callback_t callback,
                         void *callback_user);
void man_rx_decoder_reset(man_rx_decoder_t *decoder);
void man_rx_decoder_feed_packed(man_rx_decoder_t *decoder, const uint8_t *samples, size_t byte_count);

#endif
