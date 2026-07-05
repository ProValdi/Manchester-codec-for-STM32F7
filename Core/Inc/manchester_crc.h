#ifndef MANCHESTER_CRC_H
#define MANCHESTER_CRC_H
#include <stddef.h>
#include <stdint.h>
uint16_t man_crc16_ccitt_false(const uint8_t *data, size_t length);
#endif
