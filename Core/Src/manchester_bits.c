#include "manchester_bits.h"
#include <string.h>

void man_bit_writer_init(man_bit_writer_t *w, uint8_t *data, size_t capacity_bits)
{
    w->data = data;
    w->capacity_bits = capacity_bits;
    w->bit_count = 0u;
    memset(data, 0, (capacity_bits + 7u) / 8u);
}

bool man_bit_writer_put(man_bit_writer_t *w, uint8_t bit)
{
    if (w->bit_count >= w->capacity_bits) {
        return false;
    }
    const size_t byte_index = w->bit_count >> 3u;
    const uint8_t shift = (uint8_t)(7u - (w->bit_count & 7u));
    if ((bit & 1u) != 0u) {
        w->data[byte_index] |= (uint8_t)(1u << shift);
    }
    ++w->bit_count;
    return true;
}

bool man_bit_writer_put_byte(man_bit_writer_t *w, uint8_t byte)
{
    for (uint8_t i = 0; i < 8u; ++i) {
        if (!man_bit_writer_put(w, (uint8_t)((byte >> (7u - i)) & 1u))) {
            return false;
        }
    }
    return true;
}

uint8_t man_bit_get(const uint8_t *data, size_t bit_index)
{
    return (uint8_t)((data[bit_index >> 3u] >> (7u - (bit_index & 7u))) & 1u);
}
