#include "manchester_line.h"
#include "manchester_bits.h"

static void encode_one(uint8_t bit, man_encoding_t encoding, uint8_t *level,
                       uint8_t *first, uint8_t *second)
{
    if (encoding == MAN_ENCODING_STANDARD) {
        *first = bit ? 0u : 1u;
        *second = bit ? 1u : 0u;
        *level = *second;
    } else {
        if (bit == 0u) {
            *level ^= 1u;
        }
        *first = *level;
        *level ^= 1u;
        *second = *level;
    }
}

bool man_line_encode_levels(const uint8_t *wire_bits, size_t bit_count,
                            man_encoding_t encoding, bool invert,
                            uint8_t initial_level,
                            uint8_t *chips, size_t chip_capacity,
                            size_t *chip_count)
{
    if (wire_bits == NULL || chips == NULL || chip_count == NULL || chip_capacity < bit_count * 2u) {
        return false;
    }
    uint8_t level = initial_level & 1u;
    size_t out = 0u;
    for (size_t i = 0; i < bit_count; ++i) {
        uint8_t first;
        uint8_t second;
        encode_one(man_bit_get(wire_bits, i), encoding, &level, &first, &second);
        chips[out++] = (uint8_t)(first ^ (invert ? 1u : 0u));
        chips[out++] = (uint8_t)(second ^ (invert ? 1u : 0u));
    }
    *chip_count = out;
    return true;
}

static uint32_t level_to_bsrr(uint8_t level, uint16_t pin)
{
    return level ? (uint32_t)pin : ((uint32_t)pin << 16u);
}

bool man_line_encode_bsrr(const uint8_t *wire_bits, size_t bit_count,
                          man_encoding_t encoding, bool invert,
                          uint8_t initial_level, uint16_t gpio_pin,
                          uint32_t *bsrr, size_t bsrr_capacity,
                          size_t *chip_count)
{
    if (wire_bits == NULL || bsrr == NULL || chip_count == NULL || bsrr_capacity < bit_count * 2u) {
        return false;
    }
    uint8_t level = initial_level & 1u;
    size_t out = 0u;
    for (size_t i = 0; i < bit_count; ++i) {
        uint8_t first;
        uint8_t second;
        encode_one(man_bit_get(wire_bits, i), encoding, &level, &first, &second);
        first ^= invert ? 1u : 0u;
        second ^= invert ? 1u : 0u;
        bsrr[out++] = level_to_bsrr(first, gpio_pin);
        bsrr[out++] = level_to_bsrr(second, gpio_pin);
    }
    *chip_count = out;
    return true;
}
