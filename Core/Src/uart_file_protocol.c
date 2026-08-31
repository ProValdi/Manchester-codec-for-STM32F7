#include "uart_file_protocol.h"

#include <string.h>


volatile uint32_t g_dbg_uart_binary_frames_ok;
volatile uint32_t g_dbg_uart_binary_crc_errors;
volatile uint32_t g_dbg_uart_binary_format_errors;


static uint16_t read_le16(
    const uint8_t *data)
{
    return
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8u);
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

static void write_le16(
    uint8_t *data,
    uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
}


static void write_le32(
    uint8_t *data,
    uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
    data[2] = (uint8_t)(value >> 16u);
    data[3] = (uint8_t)(value >> 24u);
}

/*
 * Совпадает с Python:
 *
 * zlib.crc32(data)
 *
 * CRC-32/IEEE.
 */
static uint32_t crc32_ieee(
    const uint8_t *data,
    size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0u;
         i < length;
         ++i) {

        crc ^= data[i];

        for (uint8_t bit = 0u;
             bit < 8u;
             ++bit) {

            if ((crc & 1u) != 0u) {
                crc =
                    (crc >> 1u) ^
                    0xEDB88320u;
            } else {
                crc >>= 1u;
            }
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

uint16_t uart_file_build_frame(
    uint8_t type,
    uint32_t session_id,
    uint32_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output,
    uint16_t output_capacity)
{
    if (output == NULL ||
        payload_length > UART_FILE_MAX_PAYLOAD) {

        return 0u;
    }

    if (payload_length != 0u &&
        payload == NULL) {

        return 0u;
    }


    const uint16_t total_length =
        UART_FILE_WIRE_HEADER_SIZE +
        payload_length +
        UART_FILE_CRC_SIZE;


    if (output_capacity < total_length) {
        return 0u;
    }


    output[0] = UART_FILE_MAGIC_0;
    output[1] = UART_FILE_MAGIC_1;

    output[2] = UART_FILE_VERSION;
    output[3] = type;

    write_le32(
        &output[4],
        session_id
    );

    write_le32(
        &output[8],
        sequence
    );

    write_le16(
        &output[12],
        payload_length
    );


    if (payload_length != 0u) {
        memcpy(
            &output[
                UART_FILE_WIRE_HEADER_SIZE
            ],
            payload,
            payload_length
        );
    }


    /*
     * Совпадает с Python:
     *
     * CRC считается начиная с VERSION,
     * magic A5 5A не входит.
     */
    const uint32_t crc =
        crc32_ieee(
            &output[2],
            UART_FILE_HEADER_BODY_SIZE +
                payload_length
        );


    write_le32(
        &output[
            UART_FILE_WIRE_HEADER_SIZE +
            payload_length
        ],
        crc
    );


    return total_length;
}

static void reset_frame_state(
    uart_file_parser_t *parser)
{
    parser->state =
        UART_FILE_PARSER_MAGIC_0;

    parser->body_position = 0u;
    parser->expected_payload_length = 0u;
    parser->crc_position = 0u;
}


void uart_file_parser_init(
    uart_file_parser_t *parser,
    uart_file_frame_callback_t callback,
    void *context)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));

    parser->callback = callback;
    parser->callback_context = context;

    reset_frame_state(parser);
}


void uart_file_parser_reset(
    uart_file_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    reset_frame_state(parser);
}


static void process_complete_frame(
    uart_file_parser_t *parser)
{
    const uint16_t payload_length =
        parser->expected_payload_length;

    const uint32_t received_crc =
        read_le32(parser->crc_bytes);

    /*
     * Python считает CRC от:
     *
     * header[2:] + payload
     *
     * body[] как раз начинается с version,
     * то есть magic A5 5A сюда не входит.
     */
    const uint32_t calculated_crc =
        crc32_ieee(
            parser->body,
            UART_FILE_HEADER_BODY_SIZE +
                payload_length
        );


    if (received_crc != calculated_crc) {
        ++g_dbg_uart_binary_crc_errors;
        return;
    }


    uart_file_frame_t frame = {
        .type = parser->body[1],

        .session_id =
            read_le32(
                &parser->body[2]
            ),

        .sequence =
            read_le32(
                &parser->body[6]
            ),

        .payload =
            &parser->body[
                UART_FILE_HEADER_BODY_SIZE
            ],

        .payload_length =
            payload_length
    };


    ++g_dbg_uart_binary_frames_ok;


    if (parser->callback != NULL) {
        parser->callback(
            parser->callback_context,
            &frame
        );
    }
}


void uart_file_parser_feed(
    uart_file_parser_t *parser,
    const uint8_t *data,
    uint16_t length)
{
    if (parser == NULL ||
        data == NULL ||
        length == 0u) {

        return;
    }


    for (uint16_t i = 0u;
         i < length;
         ++i) {

        const uint8_t byte = data[i];


        switch (parser->state) {

        case UART_FILE_PARSER_MAGIC_0:

            if (byte == UART_FILE_MAGIC_0) {
                parser->state =
                    UART_FILE_PARSER_MAGIC_1;
            }

            break;


        case UART_FILE_PARSER_MAGIC_1:

            if (byte == UART_FILE_MAGIC_1) {

                parser->body_position = 0u;
                parser->crc_position = 0u;

                parser->state =
                    UART_FILE_PARSER_HEADER;

            } else if (
                byte ==
                UART_FILE_MAGIC_0) {

                /*
                 * A5 A5 5A:
                 * второе A5 может быть началом magic.
                 */
                parser->state =
                    UART_FILE_PARSER_MAGIC_1;

            } else {

                parser->state =
                    UART_FILE_PARSER_MAGIC_0;
            }

            break;


        case UART_FILE_PARSER_HEADER:

            parser->body[
                parser->body_position++
            ] = byte;


            if (parser->body_position ==
                UART_FILE_HEADER_BODY_SIZE) {

                const uint8_t version =
                    parser->body[0];

                const uint16_t payload_length =
                    read_le16(
                        &parser->body[10]
                    );


                if (version !=
                        UART_FILE_VERSION ||
                    payload_length >
                        UART_FILE_MAX_PAYLOAD) {

                    ++g_dbg_uart_binary_format_errors;

                    reset_frame_state(parser);
                    break;
                }


                parser->expected_payload_length =
                    payload_length;


                if (payload_length == 0u) {
                    parser->state =
                        UART_FILE_PARSER_CRC;
                } else {
                    parser->state =
                        UART_FILE_PARSER_PAYLOAD;
                }
            }

            break;


        case UART_FILE_PARSER_PAYLOAD:

            parser->body[
                parser->body_position++
            ] = byte;


            if (parser->body_position ==
                UART_FILE_HEADER_BODY_SIZE +
                parser->expected_payload_length) {

                parser->crc_position = 0u;

                parser->state =
                    UART_FILE_PARSER_CRC;
            }

            break;


        case UART_FILE_PARSER_CRC:

            parser->crc_bytes[
                parser->crc_position++
            ] = byte;


            if (parser->crc_position ==
                UART_FILE_CRC_SIZE) {

                process_complete_frame(
                    parser
                );

                reset_frame_state(
                    parser
                );
            }

            break;


        default:

            ++g_dbg_uart_binary_format_errors;

            reset_frame_state(parser);

            break;
        }
    }
}
