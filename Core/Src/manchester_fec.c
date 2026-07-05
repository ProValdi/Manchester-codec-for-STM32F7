#include "manchester_fec.h"
#include <string.h>

typedef struct {
    uint8_t byte;
    uint8_t bit_count;
} identity_ctx_t;

static void identity_reset(void *ctx)
{
    identity_ctx_t *state = (identity_ctx_t *)ctx;
    state->byte = 0u;
    state->bit_count = 0u;
}

static bool identity_encode(void *ctx, const uint8_t *input, size_t length, man_bit_writer_t *output)
{
    (void)ctx;
    for (size_t i = 0; i < length; ++i) {
        if (!man_bit_writer_put_byte(output, input[i])) {
            return false;
        }
    }
    return true;
}

static bool identity_push_rx_bit(void *ctx, uint8_t bit, bool *byte_ready, uint8_t *decoded_byte)
{
    identity_ctx_t *state = (identity_ctx_t *)ctx;
    state->byte = (uint8_t)((state->byte << 1u) | (bit & 1u));
    ++state->bit_count;
    *byte_ready = false;
    if (state->bit_count == 8u) {
        *decoded_byte = state->byte;
        *byte_ready = true;
        state->byte = 0u;
        state->bit_count = 0u;
    }
    return true;
}

static size_t identity_encoded_bits(size_t plain_bytes)
{
    return plain_bytes * 8u;
}

static const man_fec_codec_t identity_codec = {
    .name = "identity",
    .context_size = sizeof(identity_ctx_t),
    .reset = identity_reset,
    .encode = identity_encode,
    .push_rx_bit = identity_push_rx_bit,
    .encoded_bits_for_bytes = identity_encoded_bits
};

const man_fec_codec_t *man_fec_identity_codec(void)
{
    return &identity_codec;
}

const man_fec_codec_t *man_fec_hamming74_codec(void)
{
    return NULL;
}
