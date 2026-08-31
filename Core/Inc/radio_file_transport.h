#ifndef RADIO_FILE_TRANSPORT_H
#define RADIO_FILE_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "manchester_config.h"
#include "uart_file_protocol.h"


#define RADIO_FILE_VERSION              1u

/*
 * Radio payload:
 *
 * [0]      version
 * [1]      type
 * [2..5]   session_id LE
 * [6..9]   sequence LE
 * [10]     fragment_index
 * [11]     fragment_count
 * [12..]   fragment data
 */
#define RADIO_FILE_HEADER_SIZE          12u


typedef bool (*radio_file_emit_callback_t)(
    void *context,
    const uint8_t *data,
    uint16_t length,
    bool last_fragment
);


typedef void (*radio_file_item_callback_t)(
    void *context,
    const uart_file_frame_t *frame
);


typedef struct {
    bool active;

    uint8_t type;
    uint32_t session_id;
    uint32_t sequence;

    uint8_t fragment_count;
    uint8_t next_fragment;

    uint16_t length;

    uint8_t payload[
        UART_FILE_MAX_PAYLOAD
    ];

    radio_file_item_callback_t callback;
    void *callback_context;
} radio_file_reassembler_t;


bool radio_file_fragment_frame(
    const uart_file_frame_t *frame,
    uint16_t max_radio_payload,
    radio_file_emit_callback_t emit,
    void *context
);


void radio_file_reassembler_init(
    radio_file_reassembler_t *reassembler,
    radio_file_item_callback_t callback,
    void *context
);


void radio_file_reassembler_reset(
    radio_file_reassembler_t *reassembler
);


bool radio_file_reassembler_feed(
    radio_file_reassembler_t *reassembler,
    const uint8_t *data,
    uint16_t length
);


#endif
