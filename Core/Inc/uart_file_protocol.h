#ifndef UART_FILE_PROTOCOL_H
#define UART_FILE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


#define UART_FILE_MAGIC_0              0xA5u
#define UART_FILE_MAGIC_1              0x5Au
#define UART_FILE_VERSION              1u

#define UART_FILE_TYPE_START           0x10u
#define UART_FILE_TYPE_DATA            0x11u
#define UART_FILE_TYPE_END             0x12u
#define UART_FILE_TYPE_ACK             0x80u
#define UART_FILE_TYPE_NACK            0x81u

/*
 * Python сейчас отправляет DATA по 192 байта.
 *
 * START может быть больше из-за имени файла:
 * 48 bytes metadata + filename.
 */
#define UART_FILE_MAX_PAYLOAD          512u

#define UART_FILE_HEADER_BODY_SIZE     12u
#define UART_FILE_CRC_SIZE             4u
#define UART_FILE_WIRE_HEADER_SIZE     14u

#define UART_FILE_MAX_WIRE_SIZE \
    (UART_FILE_WIRE_HEADER_SIZE + \
     UART_FILE_MAX_PAYLOAD + \
     UART_FILE_CRC_SIZE)


typedef struct {
    uint8_t type;
    uint32_t session_id;
    uint32_t sequence;

    const uint8_t *payload;
    uint16_t payload_length;
} uart_file_frame_t;


typedef void (*uart_file_frame_callback_t)(
    void *context,
    const uart_file_frame_t *frame
);


typedef enum {
    UART_FILE_PARSER_MAGIC_0 = 0,
    UART_FILE_PARSER_MAGIC_1,
    UART_FILE_PARSER_HEADER,
    UART_FILE_PARSER_PAYLOAD,
    UART_FILE_PARSER_CRC
} uart_file_parser_state_t;


typedef struct {
    uart_file_parser_state_t state;

    /*
     * Содержит:
     *
     * version
     * type
     * session_id
     * sequence
     * payload_length
     * payload
     *
     * Magic сюда не входит.
     */
    uint8_t body[
        UART_FILE_HEADER_BODY_SIZE +
        UART_FILE_MAX_PAYLOAD
    ];

    uint16_t body_position;
    uint16_t expected_payload_length;

    uint8_t crc_bytes[UART_FILE_CRC_SIZE];
    uint8_t crc_position;

    uart_file_frame_callback_t callback;
    void *callback_context;
} uart_file_parser_t;


extern volatile uint32_t g_dbg_uart_binary_frames_ok;
extern volatile uint32_t g_dbg_uart_binary_crc_errors;
extern volatile uint32_t g_dbg_uart_binary_format_errors;


void uart_file_parser_init(
    uart_file_parser_t *parser,
    uart_file_frame_callback_t callback,
    void *context
);

void uart_file_parser_reset(
    uart_file_parser_t *parser
);

void uart_file_parser_feed(
    uart_file_parser_t *parser,
    const uint8_t *data,
    uint16_t length
);

uint16_t uart_file_build_frame(
    uint8_t type,
    uint32_t session_id,
    uint32_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output,
    uint16_t output_capacity
);

#endif
