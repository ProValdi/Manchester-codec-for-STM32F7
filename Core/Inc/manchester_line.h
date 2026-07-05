#ifndef MANCHESTER_LINE_H
#define MANCHESTER_LINE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "manchester_types.h"

bool man_line_encode_levels(const uint8_t *wire_bits, size_t bit_count,
                            man_encoding_t encoding, bool invert,
                            uint8_t initial_level,
                            uint8_t *chips, size_t chip_capacity,
                            size_t *chip_count);

bool man_line_encode_bsrr(const uint8_t *wire_bits, size_t bit_count,
                          man_encoding_t encoding, bool invert,
                          uint8_t initial_level, uint16_t gpio_pin,
                          uint32_t *bsrr, size_t bsrr_capacity,
                          size_t *chip_count);
#endif
