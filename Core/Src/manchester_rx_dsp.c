#include "manchester_rx_dsp.h"
#include "manchester_crc.h"
#include <limits.h>
#include <string.h>

#define Q16_ONE 65536LL
#define MAN_PHASE_CORRECTION_DIV 8LL
#define MAN_PHASE_RELOCK_LIMIT 6u
#define MAN_MAX_NO_EDGE_CHIPS 3LL

#define MAN_DEBUG_EDGE_COUNT 64u
#define MAN_SPI_INTERBYTE_GAP_SAMPLES 2u

volatile uint16_t g_dbg_edge_deltas[MAN_DEBUG_EDGE_COUNT];
volatile uint32_t g_dbg_edge_delta_count;
volatile uint32_t g_dbg_invalid_pairs[2];
volatile uint32_t g_dbg_valid_bits[2];
volatile uint16_t g_dbg_max_pattern_match[2];
volatile uint32_t g_dbg_edge_histogram[32];

static uint64_t g_dbg_previous_edge;
static bool g_dbg_previous_edge_valid;

static uint8_t state_index(uint8_t stable, uint8_t candidate, uint8_t span)
{
    return (uint8_t)(stable * span + candidate);
}

static void build_glitch_lut(man_rx_decoder_t *d)
{
    const uint8_t g = d->cfg.glitch_filter_samples;
    const uint8_t span = (uint8_t)(g + 1u);
    d->glitch_state_count = (uint8_t)(2u * span);

    for (uint8_t stable = 0u; stable < 2u; ++stable) {
        for (uint8_t candidate = 0u; candidate <= g; ++candidate) {
            const uint8_t source_state = state_index(stable, candidate, span);
            for (uint16_t raw = 0u; raw < 256u; ++raw) {
                uint8_t s = stable;
                uint8_t c = candidate;
                uint8_t filtered = 0u;
                for (uint8_t bit_index = 0u; bit_index < 8u; ++bit_index) {
                	const uint8_t bit = (uint8_t)((raw >> (7u - bit_index)) & 1u);
                    if (bit == s) {
                        c = 0u;
                    } else if (c >= g) {
                        s ^= 1u;
                        c = 0u;
                    } else {
                        ++c;
                    }
                    filtered = (uint8_t)((filtered << 1u) | s);
                }
                d->glitch_out[source_state][raw] = filtered;
                d->glitch_next[source_state][raw] = state_index(s, c, span);
            }
        }
    }
}

static void path_build_pattern(man_rx_path_t *p, const man_runtime_config_t *cfg)
{
    /* Leave one preamble byte as clock-acquisition margin. */
    uint8_t search_preamble = cfg->preamble_bytes > 1u ? (uint8_t)(cfg->preamble_bytes - 1u) : 1u;
    if (search_preamble < 3u) {
        search_preamble = 3u;
    }
    if (search_preamble > cfg->preamble_bytes) {
        search_preamble = cfg->preamble_bytes;
    }

    uint16_t n = 0u;
    for (uint8_t b = 0u; b < search_preamble; ++b) {
        for (uint8_t i = 0u; i < 8u; ++i) {
            p->pattern_bits[n++] = (uint8_t)((0x55u >> (7u - i)) & 1u);
        }
    }
    const uint8_t sync[2] = {(uint8_t)(cfg->sync_word >> 8u), (uint8_t)cfg->sync_word};
    for (uint8_t b = 0u; b < 2u; ++b) {
        for (uint8_t i = 0u; i < 8u; ++i) {
            p->pattern_bits[n++] = (uint8_t)((sync[b] >> (7u - i)) & 1u);
        }
    }
    p->pattern_length = n;

    p->pattern_fail[0] = 0u;
    uint16_t k = 0u;
    for (uint16_t i = 1u; i < n; ++i) {
        while (k != 0u && p->pattern_bits[i] != p->pattern_bits[k]) {
            k = p->pattern_fail[k - 1u];
        }
        if (p->pattern_bits[i] == p->pattern_bits[k]) {
            ++k;
        }
        p->pattern_fail[i] = (uint8_t)k;
    }
}

static void path_reset_parser(man_rx_decoder_t *d, man_rx_path_t *p, bool keep_pattern_match)
{
    p->parse_state = MAN_RX_SEARCH;
    p->body_count = 0u;
    p->body_expected = 0u;
    if (!keep_pattern_match) {
        p->pattern_match = 0u;
    }
    d->fec->reset(p->fec_ctx);
}

static void reset_paths(man_rx_decoder_t *d)
{
    for (uint8_t i = 0u; i < 2u; ++i) {
        man_rx_path_t *p = &d->paths[i];
        p->pair_phase = i;
        p->have_first = false;
        p->have_previous_second = false;
        path_reset_parser(d, p, false);
    }
}

static void note_loss(man_rx_decoder_t *d)
{
    if (!d->loss_pending) {
        d->loss_pending = true;
        if (d->diag != NULL) {
            ++d->diag->sync_losses;
        }
    }
}

static void path_search_push(man_rx_decoder_t *d, man_rx_path_t *p, uint8_t bit)
{
    while (p->pattern_match != 0u && bit != p->pattern_bits[p->pattern_match]) {
        p->pattern_match = p->pattern_fail[p->pattern_match - 1u];
    }

    const uint8_t path_index =
        (p == &d->paths[0]) ? 0u : 1u;

    if (p->pattern_match >
        g_dbg_max_pattern_match[path_index]) {

        g_dbg_max_pattern_match[path_index] =
            p->pattern_match;
    }

    if (bit == p->pattern_bits[p->pattern_match]) {
        ++p->pattern_match;
    }
    if (p->pattern_match == p->pattern_length) {
        p->pattern_match = p->pattern_fail[p->pattern_match - 1u];
        p->parse_state = MAN_RX_BODY;
        p->body_count = 0u;
        p->body_expected = 0u;
        d->fec->reset(p->fec_ctx);
        if (d->loss_pending && d->diag != NULL) {
            ++d->diag->resynchronizations;
        }
        d->loss_pending = false;
    }
}

static void path_finish_body(man_rx_decoder_t *d, man_rx_path_t *p)
{
    const uint16_t payload_length = (uint16_t)(((uint16_t)p->body[2] << 8u) | p->body[3]);
    const uint16_t crc_offset = (uint16_t)(MAN_HEADER_BYTES + payload_length);
    const uint16_t received_crc = (uint16_t)(((uint16_t)p->body[crc_offset] << 8u) |
                                              p->body[crc_offset + 1u]);
    const uint16_t expected_crc = man_crc16_ccitt_false(p->body, crc_offset);

    if (received_crc == expected_crc) {
        man_packet_t packet;
        packet.flags = p->body[0];
        packet.seq = p->body[1];
        packet.length = payload_length;
        if (payload_length != 0u) {
            memcpy(packet.payload, &p->body[MAN_HEADER_BYTES], payload_length);
        }
        if (d->diag != NULL) {
            ++d->diag->manchester_rx_good_frames;
        }
        if (d->callback != NULL) {
            d->callback(d->callback_user, &packet);
        }
        /* Each frame has its own preamble, so release the recovered clock here.
         * Without this, an idle line is expanded into an endless stream of equal
         * chips until the next edge, which is both useless and very expensive. */
        reset_paths(d);
        d->clock_locked = false;
    } else {
        if (d->diag != NULL) {
            ++d->diag->crc_errors;
        }
        note_loss(d);
        path_reset_parser(d, p, false);
    }
}

static void path_body_push(man_rx_decoder_t *d, man_rx_path_t *p, uint8_t bit)
{
    bool byte_ready = false;
    uint8_t decoded_byte = 0u;
    if (!d->fec->push_rx_bit(p->fec_ctx, bit, &byte_ready, &decoded_byte)) {
        if (d->diag != NULL) {
            ++d->diag->fec_errors;
        }
        note_loss(d);
        path_reset_parser(d, p, false);
        return;
    }
    if (!byte_ready) {
        return;
    }
    if (p->body_count >= MAN_BODY_MAX_BYTES) {
        note_loss(d);
        path_reset_parser(d, p, false);
        return;
    }
    p->body[p->body_count++] = decoded_byte;

    if (p->body_count == MAN_HEADER_BYTES) {
        const uint16_t payload_length = (uint16_t)(((uint16_t)p->body[2] << 8u) | p->body[3]);
        if (payload_length > d->cfg.max_payload || payload_length > MAN_MAX_PAYLOAD) {
            note_loss(d);
            path_reset_parser(d, p, false);
            return;
        }
        p->body_expected = (uint16_t)(MAN_HEADER_BYTES + payload_length + MAN_CRC_BYTES);
    }
    if (p->body_expected != 0u && p->body_count == p->body_expected) {
        path_finish_body(d, p);
    }
}

static void path_push_bit(man_rx_decoder_t *d, man_rx_path_t *p, uint8_t bit)
{
    if (p->parse_state == MAN_RX_SEARCH) {
        path_search_push(d, p, bit);
    } else {
        path_body_push(d, p, bit);
    }
}

static void path_push_chip(man_rx_decoder_t *d, man_rx_path_t *p, uint64_t chip_index, uint8_t chip)
{
    if (!p->have_first) {
        if ((chip_index & 1u) != p->pair_phase) {
            return;
        }
        p->first_chip = chip;
        p->have_first = true;
        return;
    }

    const uint8_t path_index =
        (p == &d->paths[0]) ? 0u : 1u;

    const uint8_t first = p->first_chip;
    const uint8_t second = chip;
    p->have_first = false;

    if (first == second) {
    	++g_dbg_invalid_pairs[path_index];
        if (p->parse_state == MAN_RX_BODY) {
            if (d->diag != NULL) {
                ++d->diag->line_code_errors;
            }
            note_loss(d);
            path_reset_parser(d, p, false);
        }
        p->have_previous_second = false;
        return;
    }

    uint8_t bit;
    if (d->cfg.encoding == MAN_ENCODING_STANDARD) {
        bit = (first == 0u && second == 1u) ? 1u : 0u;
    } else {
        if (!p->have_previous_second) {
            /* The first differential bit after lock is intentionally discarded. */
            p->previous_second = second;
            p->have_previous_second = true;
            return;
        }
        bit = (first == p->previous_second) ? 1u : 0u;
        p->previous_second = second;
    }
    ++g_dbg_valid_bits[path_index];
    path_push_bit(d, p, bit);
}

static void emit_until(man_rx_decoder_t *d, uint64_t sample_exclusive, uint8_t level)
{
    if (!d->clock_locked) {
        return;
    }

    const int64_t requested_limit_q16 = (int64_t)sample_exclusive * Q16_ONE;
    const int64_t no_edge_limit_q16 =
        (int64_t)d->timeline_position * Q16_ONE +
        MAN_MAX_NO_EDGE_CHIPS * (int64_t)d->chip_period_q16;
    const bool line_timeout = requested_limit_q16 > no_edge_limit_q16;
    const int64_t effective_limit_q16 =
        line_timeout ? no_edge_limit_q16 : requested_limit_q16;

    while (d->clock_locked && d->next_chip_center_q16 < effective_limit_q16) {
        const uint64_t index = d->next_chip_index++;
        d->next_chip_center_q16 += (int64_t)d->chip_period_q16;
        path_push_chip(d, &d->paths[0], index, level);
        if (d->clock_locked) {
            path_push_chip(d, &d->paths[1], index, level);
        }
    }

    if (line_timeout && d->clock_locked) {
        if (d->paths[0].parse_state == MAN_RX_BODY ||
            d->paths[1].parse_state == MAN_RX_BODY) {
            note_loss(d);
        }
        d->clock_locked = false;
        d->consecutive_phase_errors = 0u;
        reset_paths(d);
    }
}

static int64_t abs64(int64_t v)
{
    return v < 0 ? -v : v;
}

static void observe_edge(man_rx_decoder_t *d, uint64_t edge_sample)
{
	if (g_dbg_previous_edge_valid &&
	    g_dbg_edge_delta_count < MAN_DEBUG_EDGE_COUNT) {

	    uint64_t delta = edge_sample - g_dbg_previous_edge;

	    if (delta > UINT16_MAX) {
	        delta = UINT16_MAX;
	    }

	    g_dbg_edge_deltas[g_dbg_edge_delta_count++] =
	        (uint16_t)delta;
	}

	if (g_dbg_previous_edge_valid) {
	    uint64_t delta = edge_sample - g_dbg_previous_edge;

	    if (delta < 32u) {
	        ++g_dbg_edge_histogram[delta];
	    }
	}

	g_dbg_previous_edge = edge_sample;
	g_dbg_previous_edge_valid = true;

    const int64_t edge_q16 = (int64_t)edge_sample * Q16_ONE;
    if (!d->clock_locked) {
        d->clock_locked = true;
        d->grid_origin_q16 = edge_q16;
        d->next_chip_index = 0u;
        d->next_chip_center_q16 = edge_q16 + (int64_t)d->chip_period_q16 / 2LL;
        d->consecutive_phase_errors = 0u;
        reset_paths(d);
        return;
    }

    /* emit_until() has already consumed all centers before this edge. Therefore
     * the start boundary of next_chip_index is the nearest expected boundary.
     * This avoids a costly 64-bit division on every line transition. */
    const int64_t predicted_boundary =
        d->next_chip_center_q16 - (int64_t)d->chip_period_q16 / 2LL;
    const int64_t error = edge_q16 - predicted_boundary;
    const int64_t period = (int64_t)d->chip_period_q16;

    if (abs64(error) > period / 3LL) {
        if (d->diag != NULL) {
            ++d->diag->phase_errors;
        }
        if (++d->consecutive_phase_errors >= MAN_PHASE_RELOCK_LIMIT) {
            note_loss(d);
            d->grid_origin_q16 = edge_q16;
            d->next_chip_index = 0u;
            d->next_chip_center_q16 = edge_q16 + period / 2LL;
            d->consecutive_phase_errors = 0u;
            reset_paths(d);
        }
    } else {
        const int64_t correction = error / MAN_PHASE_CORRECTION_DIV;
        d->consecutive_phase_errors = 0u;
        d->grid_origin_q16 += correction;
        d->next_chip_center_q16 += correction;
    }
}

static uint8_t first_transition_offset(uint8_t mask)
{
#if defined(__GNUC__) || defined(__clang__)
    return (uint8_t)(__builtin_clz((uint32_t)mask) - 24);
#else
    uint8_t offset = 0u;
    while ((mask & 0x80u) == 0u) {
        mask <<= 1u;
        ++offset;
    }
    return offset;
#endif
}

static void process_filtered_byte(man_rx_decoder_t *d, uint8_t filtered)
{
    const uint64_t base = d->raw_sample_index;
    const uint8_t first_level = (uint8_t)(filtered >> 7u);

    if (!d->timeline_valid) {
        d->timeline_valid = true;
        d->timeline_position = base;
        d->timeline_level = first_level;
        d->previous_filtered_level = first_level;
        d->previous_filtered_valid = true;
    }

    const uint8_t previous_samples =
        (uint8_t)((d->previous_filtered_level << 7u) | (filtered >> 1u));
    uint8_t transitions = (uint8_t)(filtered ^ previous_samples);

    /* Fast idle path before the first edge of a frame. */
    if (transitions == 0u && !d->clock_locked) {
        d->previous_filtered_level = (uint8_t)(filtered & 1u);
        d->raw_sample_index += 8u;
        return;
    }

    while (transitions != 0u) {
        const uint8_t offset = first_transition_offset(transitions);
        const uint8_t bit_mask = (uint8_t)(0x80u >> offset);
        uint64_t position = base + offset;

        /*
         * offset == 0 означает, что уровень между последней выборкой
         * предыдущего байта и первой выборкой текущего байта изменился.
         *
         * Точный момент перехода внутри SPI-паузы неизвестен.
         * Помещаем его приблизительно в середину интервала.
         */
        if (offset == 0u &&
            base != 0u &&
            MAN_SPI_INTERBYTE_GAP_SAMPLES != 0u) {

            position -= (MAN_SPI_INTERBYTE_GAP_SAMPLES + 1u) / 2u;
        }


        const uint8_t new_level = (uint8_t)((filtered & bit_mask) != 0u);
        emit_until(d, position, d->timeline_level);
        observe_edge(d, position);
        d->timeline_position = position;
        d->timeline_level = new_level;
        transitions = (uint8_t)(transitions & (uint8_t)~bit_mask);
    }

    emit_until(d, base + 8u, d->timeline_level);
    d->previous_filtered_level = (uint8_t)(filtered & 1u);

    /*
     * SPI выдаёт восемь выборок, затем между байтами имеется пауза,
     * эквивалентная двум периодам SCK.
     *
     * Единицей timeline по-прежнему является один период SCK.
     */
    d->raw_sample_index += 8u + MAN_SPI_INTERBYTE_GAP_SAMPLES;
}

bool man_rx_decoder_init(man_rx_decoder_t *d,
                         const man_runtime_config_t *cfg,
                         uint32_t sample_rate_hz,
                         const man_fec_codec_t *fec,
                         man_diagnostics_t *diag,
                         man_rx_frame_callback_t callback,
                         void *callback_user)
{
    if (d == NULL || cfg == NULL || fec == NULL || !man_bitrate_is_valid(cfg->bitrate_bps) ||
        sample_rate_hz < cfg->bitrate_bps * 4u || cfg->preamble_bytes < MAN_PREAMBLE_BYTES_MIN ||
        cfg->preamble_bytes > MAN_PREAMBLE_BYTES_MAX || cfg->glitch_filter_samples > MAN_GLITCH_FILTER_MAX_SAMPLES ||
        cfg->max_payload == 0u || cfg->max_payload > MAN_MAX_PAYLOAD || fec->context_size > MAN_FEC_CONTEXT_BYTES) {
        return false;
    }

    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    d->sample_rate_hz = sample_rate_hz;
    d->chip_rate_hz = cfg->bitrate_bps * 2u;
    d->chip_period_q16 = (uint32_t)(((uint64_t)sample_rate_hz << 16u) / d->chip_rate_hz);
    d->fec = fec;
    d->diag = diag;
    d->callback = callback;
    d->callback_user = callback_user;
    build_glitch_lut(d);

    for (uint8_t i = 0u; i < 2u; ++i) {
        path_build_pattern(&d->paths[i], cfg);
    }

    if (d->paths[0].pattern_length == 0u ||
        d->paths[1].pattern_length == 0u ||
        d->paths[0].pattern_length != d->paths[1].pattern_length) {
        return false;
    }

    man_rx_decoder_reset(d);

    if (d->paths[0].pattern_length == 0u ||
        d->paths[1].pattern_length == 0u) {
        return false;
    }

    return true;
}

void man_rx_decoder_reset(man_rx_decoder_t *d)
{
    if (d == NULL) {
        return;
    }
    d->glitch_initialized = false;
    d->glitch_state = 0u;
    d->previous_filtered_valid = false;
    d->raw_sample_index = 0u;
    d->timeline_valid = false;
    d->clock_locked = false;
    d->grid_origin_q16 = 0LL;
    d->next_chip_center_q16 = 0LL;
    d->next_chip_index = 0u;
    d->consecutive_phase_errors = 0u;
    d->loss_pending = false;
    reset_paths(d);
}

void man_rx_decoder_feed_packed(man_rx_decoder_t *d, const uint8_t *samples, size_t byte_count)
{
    if (d == NULL || samples == NULL) {
        return;
    }
    const uint8_t span = (uint8_t)(d->cfg.glitch_filter_samples + 1u);
    for (size_t i = 0u; i < byte_count; ++i) {
        const uint8_t raw = samples[i];
        if (!d->glitch_initialized) {
            const uint8_t first = (uint8_t)(raw >> 7u);
            d->glitch_state = state_index(first, 0u, span);
            d->glitch_initialized = true;
        }
        const uint8_t filtered = d->glitch_out[d->glitch_state][raw];
        d->glitch_state = d->glitch_next[d->glitch_state][raw];
        process_filtered_byte(d, filtered);
    }
}
