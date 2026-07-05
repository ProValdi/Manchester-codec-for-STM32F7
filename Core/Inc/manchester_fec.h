#ifndef MANCHESTER_FEC_H
#define MANCHESTER_FEC_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "manchester_bits.h"
#include "manchester_config.h"

typedef struct man_fec_codec {
    const char *name;
    size_t context_size;
    void (*reset)(void *ctx);
    bool (*encode)(void *ctx, const uint8_t *input, size_t length, man_bit_writer_t *output);
    bool (*push_rx_bit)(void *ctx, uint8_t bit, bool *byte_ready, uint8_t *decoded_byte);
    size_t (*encoded_bits_for_bytes)(size_t plain_bytes);
} man_fec_codec_t;

const man_fec_codec_t *man_fec_identity_codec(void);
/* Placeholder entry point for a future Hamming(7,4) codec. Returns NULL for now. */
const man_fec_codec_t *man_fec_hamming74_codec(void);

#endif
