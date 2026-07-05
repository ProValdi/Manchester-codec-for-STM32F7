#ifndef MANCHESTER_BITS_H
#define MANCHESTER_BITS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;
    size_t capacity_bits;
    size_t bit_count;
} man_bit_writer_t;

void man_bit_writer_init(man_bit_writer_t *w, uint8_t *data, size_t capacity_bits);
bool man_bit_writer_put(man_bit_writer_t *w, uint8_t bit);
bool man_bit_writer_put_byte(man_bit_writer_t *w, uint8_t byte);
uint8_t man_bit_get(const uint8_t *data, size_t bit_index);
#endif
