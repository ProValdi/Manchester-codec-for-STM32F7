#include "manchester_frame.h"
#include "manchester_crc.h"
#include <string.h>

bool man_frame_build_body(const man_packet_t *packet, uint8_t *body, size_t capacity, size_t *body_length)
{
    if (packet == NULL || body == NULL || body_length == NULL || packet->length > MAN_MAX_PAYLOAD) {
        return false;
    }
    const size_t needed = MAN_HEADER_BYTES + packet->length + MAN_CRC_BYTES;
    if (capacity < needed) {
        return false;
    }

    body[0] = packet->flags;
    body[1] = packet->seq;
    body[2] = (uint8_t)(packet->length >> 8u);
    body[3] = (uint8_t)(packet->length & 0xFFu);
    if (packet->length != 0u) {
        memcpy(&body[MAN_HEADER_BYTES], packet->payload, packet->length);
    }

    const uint16_t crc = man_crc16_ccitt_false(body, MAN_HEADER_BYTES + packet->length);
    body[MAN_HEADER_BYTES + packet->length] = (uint8_t)(crc >> 8u);
    body[MAN_HEADER_BYTES + packet->length + 1u] = (uint8_t)crc;
    *body_length = needed;
    return true;
}

bool man_frame_build_wire_bits(const man_runtime_config_t *cfg,
                               const man_packet_t *packet,
                               const man_fec_codec_t *fec,
                               void *fec_ctx,
                               uint8_t *wire_bits,
                               size_t capacity_bits,
                               size_t *wire_bit_count)
{
    if (cfg == NULL || packet == NULL || fec == NULL || fec_ctx == NULL || wire_bits == NULL || wire_bit_count == NULL) {
        return false;
    }
    uint8_t body[MAN_BODY_MAX_BYTES];
    size_t body_length = 0u;
    if (!man_frame_build_body(packet, body, sizeof(body), &body_length)) {
        return false;
    }

    const size_t required = ((size_t)cfg->preamble_bytes + 2u) * 8u + fec->encoded_bits_for_bytes(body_length);
    if (required > capacity_bits) {
        return false;
    }

    man_bit_writer_t writer;
    man_bit_writer_init(&writer, wire_bits, capacity_bits);
    for (uint8_t i = 0; i < cfg->preamble_bytes; ++i) {
        if (!man_bit_writer_put_byte(&writer, 0x55u)) {
            return false;
        }
    }
    if (!man_bit_writer_put_byte(&writer, (uint8_t)(cfg->sync_word >> 8u)) ||
        !man_bit_writer_put_byte(&writer, (uint8_t)cfg->sync_word)) {
        return false;
    }

    fec->reset(fec_ctx);
    if (!fec->encode(fec_ctx, body, body_length, &writer)) {
        return false;
    }
    *wire_bit_count = writer.bit_count;
    return true;
}
