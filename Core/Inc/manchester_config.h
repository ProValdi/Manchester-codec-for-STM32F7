#ifndef MANCHESTER_CONFIG_H
#define MANCHESTER_CONFIG_H

#include <stdint.h>

/* Compile-time role. The same sources support all three builds. */
#define MAN_ROLE_TX_ONLY       1u
#define MAN_ROLE_RX_ONLY       2u
#define MAN_ROLE_TRANSCEIVER   3u
#ifndef MAN_COMPILETIME_ROLE
#define MAN_COMPILETIME_ROLE MAN_ROLE_TRANSCEIVER
#endif

#define MAN_MAX_PAYLOAD              256u
#define MAN_MAX_SINGLE_MESSAGE       4096u
#define MAN_PREAMBLE_BYTES_DEFAULT   8u
#define MAN_PREAMBLE_BYTES_MIN       4u
#define MAN_PREAMBLE_BYTES_MAX       256u
#define MAN_SYNC_WORD_DEFAULT        0xD391u

/* Parallel raw edge capture through TIM input capture + DMA. */
#define MAN_TIM_IC_RAW_CAPTURE_ENABLED  1u
#define MAN_TIM_IC_DMA_EVENTS           4096u
#define MAN_TIM_IC_DMA_HALF_EVENTS      (MAN_TIM_IC_DMA_EVENTS / 2u)

#define MAN_UART_RX_DMA_BYTES        2048u // 8192u
#define MAN_UART_TX_STAGE_BYTES      512u
#define MAN_TX_MAX_WIRE_BITS         4096u
#define MAN_TX_MAX_CHIPS             (MAN_TX_MAX_WIRE_BITS * 2u)

#define MAN_PACKET_POOL_COUNT        16u
#define MAN_TX_QUEUE_DEPTH           8u
#define MAN_RX_QUEUE_DEPTH           32u
#define MAN_UART_OUT_QUEUE_DEPTH     8u
#define MAN_UART_ISR_EVENT_COUNT     32u

#define MAN_GLITCH_FILTER_MAX_SAMPLES 4u
#define MAN_FEC_CONTEXT_BYTES         64u

#define MAN_FILE_TX_QUEUE_DEPTH       2u
#define MAN_FILE_ACK_QUEUE_DEPTH      8u
#define MAN_FILE_ACK_TIMEOUT_MS       100u
#define MAN_FILE_MAX_RETRIES          15u
#define MAN_FILE_ACK_TURNAROUND_MS    3u

/* Default DMA placement: dedicated linker section. */
#if defined(__GNUC__)
#define MAN_DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#elif defined(__ICCARM__)
#define MAN_DMA_BUFFER _Pragma("location=\".dma_buffer\"") _Pragma("data_alignment=32")
#else
#define MAN_DMA_BUFFER __attribute__((aligned(32)))
#endif

/* Set to 1 only when .dma_buffer is cacheable SRAM rather than DTCM/non-cacheable MPU SRAM. */
#ifndef MAN_DMA_USE_CACHE_MAINTENANCE
#define MAN_DMA_USE_CACHE_MAINTENANCE 0
#endif

#endif
