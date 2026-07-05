#ifndef MANCHESTER_FRAME_H
#define MANCHESTER_FRAME_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "manchester_bits.h"
#include "manchester_fec.h"
#include "manchester_types.h"

#define MAN_HEADER_BYTES 4u
#define MAN_CRC_BYTES 2u
#define MAN_BODY_MAX_BYTES (MAN_HEADER_BYTES + MAN_MAX_PAYLOAD + MAN_CRC_BYTES)

bool man_frame_build_body(const man_packet_t *packet, uint8_t *body, size_t capacity, size_t *body_length);
bool man_frame_build_wire_bits(const man_runtime_config_t *cfg,
                               const man_packet_t *packet,
                               const man_fec_codec_t *fec,
                               void *fec_ctx,
                               uint8_t *wire_bits,
                               size_t capacity_bits,
                               size_t *wire_bit_count);
#endif
