#include "radio_file_transport.h"

#include <string.h>


static void write_le32(
    uint8_t *data,
    uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
    data[2] = (uint8_t)(value >> 16u);
    data[3] = (uint8_t)(value >> 24u);
}


static uint32_t read_le32(
    const uint8_t *data)
{
    return
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8u) |
        ((uint32_t)data[2] << 16u) |
        ((uint32_t)data[3] << 24u);
}


bool radio_file_fragment_frame(
    const uart_file_frame_t *frame,
    uint16_t max_radio_payload,
    radio_file_emit_callback_t emit,
    void *context)
{
    if (frame == NULL ||
        emit == NULL ||
        max_radio_payload <=
            RADIO_FILE_HEADER_SIZE ||
        max_radio_payload >
            MAN_MAX_PAYLOAD) {

        return false;
    }


    const uint16_t fragment_capacity =
        max_radio_payload -
        RADIO_FILE_HEADER_SIZE;


    uint32_t fragment_count;

    if (frame->payload_length == 0u) {

        fragment_count = 1u;

    } else {

        fragment_count =
            (
                frame->payload_length +
                fragment_capacity -
                1u
            ) /
            fragment_capacity;
    }


    if (fragment_count > 255u) {
        return false;
    }


    uint8_t radio_payload[
        MAN_MAX_PAYLOAD
    ];


    uint16_t offset = 0u;


    for (uint32_t fragment = 0u;
         fragment < fragment_count;
         ++fragment) {

        const uint16_t remaining =
            frame->payload_length -
            offset;

        const uint16_t take =
            remaining > fragment_capacity
                ? fragment_capacity
                : remaining;


        radio_payload[0] =
            RADIO_FILE_VERSION;

        radio_payload[1] =
            frame->type;


        write_le32(
            &radio_payload[2],
            frame->session_id
        );

        write_le32(
            &radio_payload[6],
            frame->sequence
        );


        radio_payload[10] =
            (uint8_t)fragment;

        radio_payload[11] =
            (uint8_t)fragment_count;


        if (take != 0u) {

            memcpy(
                &radio_payload[
                    RADIO_FILE_HEADER_SIZE
                ],
                &frame->payload[offset],
                take
            );
        }


        const bool last =
            fragment + 1u ==
            fragment_count;


        if (!emit(
                context,
                radio_payload,
                RADIO_FILE_HEADER_SIZE +
                    take,
                last)) {

            return false;
        }


        offset += take;
    }


    return true;
}


void radio_file_reassembler_init(
    radio_file_reassembler_t *r,
    radio_file_item_callback_t callback,
    void *context)
{
    if (r == NULL) {
        return;
    }


    memset(
        r,
        0,
        sizeof(*r)
    );


    r->callback =
        callback;

    r->callback_context =
        context;
}


void radio_file_reassembler_reset(
    radio_file_reassembler_t *r)
{
    if (r == NULL) {
        return;
    }


    r->active = false;
    r->length = 0u;
    r->fragment_count = 0u;
    r->next_fragment = 0u;
}


bool radio_file_reassembler_feed(
    radio_file_reassembler_t *r,
    const uint8_t *data,
    uint16_t length)
{
    if (r == NULL ||
        data == NULL ||
        length <
            RADIO_FILE_HEADER_SIZE) {

        return false;
    }


    if (data[0] !=
        RADIO_FILE_VERSION) {

        radio_file_reassembler_reset(r);
        return false;
    }


    const uint8_t type =
        data[1];

    const uint32_t session_id =
        read_le32(&data[2]);

    const uint32_t sequence =
        read_le32(&data[6]);

    const uint8_t fragment_index =
        data[10];

    const uint8_t fragment_count =
        data[11];


    if (fragment_count == 0u ||
        fragment_index >=
            fragment_count) {

        radio_file_reassembler_reset(r);
        return false;
    }


    const uint16_t fragment_length =
        length -
        RADIO_FILE_HEADER_SIZE;


    /*
     * Fragment 0 начинает новый logical item.
     */
    if (fragment_index == 0u) {

        r->active = true;

        r->type =
            type;

        r->session_id =
            session_id;

        r->sequence =
            sequence;

        r->fragment_count =
            fragment_count;

        r->next_fragment =
            0u;

        r->length =
            0u;
    }


    if (!r->active) {
        return false;
    }


    /*
     * Все fragments должны относиться
     * к одному logical FILE_* frame.
     */
    if (r->type != type ||
        r->session_id !=
            session_id ||
        r->sequence !=
            sequence ||
        r->fragment_count !=
            fragment_count ||
        fragment_index !=
            r->next_fragment) {

        radio_file_reassembler_reset(r);
        return false;
    }


    if ((uint32_t)r->length +
            fragment_length >
        sizeof(r->payload)) {

        radio_file_reassembler_reset(r);
        return false;
    }


    if (fragment_length != 0u) {

        memcpy(
            &r->payload[r->length],
            &data[
                RADIO_FILE_HEADER_SIZE
            ],
            fragment_length
        );

        r->length +=
            fragment_length;
    }


    ++r->next_fragment;


    /*
     * Logical UART FILE_* frame восстановлен.
     */
    if (r->next_fragment ==
        r->fragment_count) {

        uart_file_frame_t frame = {
            .type =
                r->type,

            .session_id =
                r->session_id,

            .sequence =
                r->sequence,

            .payload =
                r->payload,

            .payload_length =
                r->length
        };


        if (r->callback != NULL) {

            r->callback(
                r->callback_context,
                &frame
            );
        }


        radio_file_reassembler_reset(r);
    }


    return true;
}
