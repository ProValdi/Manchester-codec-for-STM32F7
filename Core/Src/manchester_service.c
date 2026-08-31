#include "manchester_service.h"
#include "manchester_config.h"
#include "manchester_fec.h"
#include "manchester_frame.h"
#include "manchester_line.h"
#include "uart_file_protocol.h"
#include "radio_file_transport.h"
#include "manchester_rx_dsp.h"
#include "manchester_tim_ic_capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define RX_FLAG_TIM_IC     (1u << 0)
#define TX_FLAG_DONE       (1u << 0)
#define TX_FLAG_HALF       (1u << 1)
#define TX_FLAG_ERROR      (1u << 2)
#define UART_FLAG_RX       (1u << 0)
#define UART_FLAG_TX_DONE  (1u << 1)
#define UART_FLAG_ERROR    (1u << 2)

#define UART_EVENT_IDLE 1u
#define UART_EVENT_HT   2u
#define UART_EVENT_TC   3u

typedef struct {
    uint16_t position;
    uint8_t type;
} uart_isr_event_t;

typedef struct {
    uint16_t length;
    uint8_t data[MAN_UART_TX_STAGE_BYTES];
} uart_output_t;

static man_platform_t g_hw;
static man_runtime_config_t g_cfg;
static man_diagnostics_t g_diag;
static bool g_initialized;
static uint32_t g_rx_sample_rate_hz;
static uint32_t g_rx_chip_ticks;

// Новый режим пакетной передачи BEGIN
static uart_file_parser_t g_uart_file_parser;
static radio_file_reassembler_t g_radio_file_reassembler;

// debug filetransfer BEGIN
volatile uint32_t g_dbg_radio_file_tx_items;
volatile uint32_t g_dbg_radio_file_tx_fragments;
volatile uint32_t g_dbg_radio_file_tx_failures;

volatile uint32_t g_dbg_radio_file_rx_items;
volatile uint32_t g_dbg_radio_file_rx_fragments;
volatile uint32_t g_dbg_radio_file_rx_fragment_errors;

volatile uint32_t g_dbg_radio_file_rx_start;
volatile uint32_t g_dbg_radio_file_rx_data;
volatile uint32_t g_dbg_radio_file_rx_end;

volatile uint32_t g_dbg_radio_file_rx_chunk_gaps;

volatile uint32_t g_dbg_radio_file_rx_session;
volatile uint32_t g_dbg_radio_file_rx_sequence;

volatile uint32_t g_dbg_radio_file_rx_bytes;
volatile uint32_t g_dbg_radio_file_expected_bytes;

static bool g_radio_file_expected_chunk_valid;
static uint32_t g_radio_file_expected_chunk;
static bool g_radio_file_rx_session_active;
static uint32_t g_radio_file_rx_session_id;

static bool g_radio_file_rx_end_forwarded;
// debug filetransfer END

static bool g_uart_file_mode;
static bool g_uart_file_magic_probe;

typedef struct {
    bool active;

    uint32_t session_id;

    uint64_t file_size;
    uint16_t chunk_size;
    uint32_t total_chunks;

    uint32_t next_sequence;
    uint64_t received_bytes;

    uint8_t sha256[32];
} uart_file_session_t;

static uart_file_session_t g_uart_file_session;

/* Debug */
volatile uint32_t g_dbg_file_start_frames;
volatile uint32_t g_dbg_file_data_frames;
volatile uint32_t g_dbg_file_end_frames;

volatile uint32_t g_dbg_file_session_errors;
volatile uint32_t g_dbg_file_sequence_errors;

volatile uint32_t g_dbg_file_session_id;
volatile uint32_t g_dbg_file_total_chunks;
volatile uint32_t g_dbg_file_last_sequence;
volatile uint32_t g_dbg_file_chunk_size;

volatile uint32_t g_dbg_file_received_bytes_lo;
volatile uint32_t g_dbg_file_expected_size_lo;

char g_dbg_file_name[64];
// Новый режим пакетной передачи END


/*
 * TIM8 — 16 bit, поэтому DMA даёт uint16 timestamps.
 *
 * DSP использует собственный 64-bit virtual timeline.
 */
static bool g_ic_previous_valid;
static uint16_t g_ic_previous_capture;
static uint64_t g_ic_virtual_tick;


/*
 * После короткого пакета HT/TC может не произойти.
 * По RTOS idle обнаруживаем конец потока и
 * принудительно продвигаем DSP после последнего edge.
 */
static bool g_ic_idle_armed;
static uint32_t g_ic_last_activity_tick;


/* DEBUG */
volatile uint32_t g_dbg_ic_dsp_edges;
volatile uint32_t g_dbg_ic_dsp_idle_flushes;
volatile uint32_t g_dbg_ic_dsp_large_gaps;
volatile uint32_t g_dbg_ic_dsp_last_delta;

static const man_fec_codec_t *g_fec;
static man_rx_decoder_t g_decoder;

static MAN_DMA_BUFFER uint8_t g_uart_rx_dma[MAN_UART_RX_DMA_BYTES];
static MAN_DMA_BUFFER uint8_t g_uart_tx_dma[MAN_UART_TX_STAGE_BYTES];
static MAN_DMA_BUFFER uint32_t g_tx_bsrr[MAN_TX_MAX_CHIPS];
static uint8_t g_tx_wire_bits[(MAN_TX_MAX_WIRE_BITS + 7u) / 8u];
static uint8_t g_tx_fec_ctx[MAN_FEC_CONTEXT_BYTES];
static uint8_t g_uart_assembly[MAN_MAX_SINGLE_MESSAGE];
static uint8_t g_uart_temporary[MAN_UART_RX_DMA_BYTES];

static osThreadId_t g_rx_task;
static osThreadId_t g_tx_task;
static osThreadId_t g_uart_task;
static osThreadId_t g_app_task;
static osThreadId_t g_diag_task;
static osMessageQueueId_t g_tx_queue;
static osMessageQueueId_t g_rx_queue;
static osMessageQueueId_t g_uart_out_queue;

// ACK
typedef struct {
    uint8_t type;

    uint32_t session_id;
    uint32_t sequence;

    uint16_t payload_length;

    uint8_t payload[
        UART_FILE_MAX_PAYLOAD
    ];
} file_tx_item_t;


typedef struct {
    uint8_t acked_type;

    uint32_t session_id;
    uint32_t sequence;
} file_ack_event_t;


static osMessageQueueId_t g_file_tx_queue;
static osMessageQueueId_t g_file_ack_queue;
static osThreadId_t g_file_task;

static void FileTransportTask(void *argument);
volatile uint32_t g_dbg_file_arq_ok;
volatile uint32_t g_dbg_file_arq_retries;
volatile uint32_t g_dbg_file_arq_timeouts;
volatile uint32_t g_dbg_file_arq_failed;

volatile uint32_t g_dbg_file_ack_tx;
volatile uint32_t g_dbg_file_ack_rx;
volatile uint32_t g_dbg_file_duplicates;
static uint32_t g_radio_file_total_chunks;
volatile uint32_t g_dbg_uart_event_queue_overflows;
volatile uint32_t g_dbg_uart_hal_errors;
volatile uint32_t g_dbg_uart_rx_start_errors;
volatile uint32_t g_dbg_uart_tx_start_errors;

volatile uint32_t g_dbg_host_file_queue_rejects;
volatile uint32_t g_dbg_host_ack_tx;
volatile uint32_t g_dbg_host_nack_tx;
volatile uint32_t g_dbg_host_ack_build_errors;
volatile uint32_t g_dbg_host_file_rx_forwarded;
volatile uint32_t g_dbg_host_file_rx_forward_errors;

static bool g_rf_tx_active;
// ACK


static volatile uint8_t g_uart_event_head;
static volatile uint8_t g_uart_event_tail;
static uart_isr_event_t g_uart_events[MAN_UART_ISR_EVENT_COUNT];
static volatile bool g_uart_tx_busy;
static uint8_t g_tx_sequence;
static uint8_t g_expected_rx_sequence;
static bool g_expected_rx_sequence_valid;
static bool g_tx_reset_pending;

static void ManchesterRxTask(void *argument);
static void ManchesterTxTask(void *argument);
static void UartTask(void *argument);
static void ApplicationTask(void *argument);
static void DiagnosticsTask(void *argument);

static bool handle_file_data(const uart_file_frame_t *frame);
static bool handle_file_start(const uart_file_frame_t *frame);
static bool handle_file_end(const uart_file_frame_t *frame);
static uint64_t uart_read_le64(const uint8_t *data);
static bool send_uart_file_frame_over_radio(const uart_file_frame_t *frame);
static void queue_file_debug(const char *format, ...);
static uint32_t uart_read_le32(const uint8_t *data);
static bool send_radio_file_ack(const uart_file_frame_t *received);
static void radio_file_item_received(void *context, const uart_file_frame_t *frame);
static bool queue_uart_output_wait(const uint8_t *data, uint16_t length, uint32_t timeout);
static bool queue_received_file_to_host(const uart_file_frame_t *frame);

volatile uint32_t g_rx_cycles_last;
volatile uint32_t g_rx_cycles_max;
volatile uint32_t g_rx_blocks_processed;

volatile uint16_t g_dbg_pattern_length_0;
volatile uint16_t g_dbg_pattern_length_1;
volatile uint32_t g_dbg_rx_block_number;
volatile uint32_t g_dbg_pattern_corrupted_at;

volatile uint32_t g_dbg_pclk2_hz;
volatile uint32_t g_dbg_timer_clock_hz;
volatile uint32_t g_dbg_timer_ticks;

volatile uint32_t g_dbg_app_rx_total;
volatile uint32_t g_dbg_app_rx_normal;
volatile uint32_t g_dbg_app_rx_file;

static volatile bool g_rx_muted = false;
static volatile bool g_rx_reset_requested = false;

static void rf_apply_rx_gpio(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  GPIO_PIN_SET);

    HAL_GPIO_WritePin(g_hw.rf_trans_port, g_hw.rf_trans_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_hw.rf_recv_port, g_hw.rf_recv_pin, GPIO_PIN_SET);
}

static void rf_apply_tx_gpio(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  GPIO_PIN_RESET);

    HAL_GPIO_WritePin(g_hw.rf_trans_port, g_hw.rf_trans_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_hw.rf_recv_port, g_hw.rf_recv_pin, GPIO_PIN_RESET);
}

static void tx_set_idle_level(void)
{
    const GPIO_PinState idle =
        g_cfg.tx_invert ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(g_hw.tx_port, g_hw.tx_pin, idle);
}

static bool is_rf_halfduplex(void)
{
    return g_cfg.phy_mode == MAN_PHY_RF_HALFDUPLEX;
}

static void rf_enter_tx_mode(void)
{
    if (!is_rf_halfduplex() || g_rf_tx_active) {
        return;
    }
    g_rx_muted = true;
    g_rx_reset_requested = true;
    rf_apply_tx_gpio();

    g_rf_tx_active = true;

    if (g_hw.rf_tx_settle_ms != 0u) {
        osDelay(g_hw.rf_tx_settle_ms);
    }
}

static void rf_enter_rx_mode(void)
{
    if (!is_rf_halfduplex() || !g_rf_tx_active) {
        return;
    }

    rf_apply_rx_gpio();

    if (g_hw.rf_rx_settle_ms != 0u) {
        osDelay(g_hw.rf_rx_settle_ms);
    }

    g_rf_tx_active = false;

    g_rx_reset_requested = true;
    g_rx_muted = false;
}

__weak void Manchester_TestHookMutateWireBits(uint8_t *wire_bits, size_t bit_count)
{
    (void)wire_bits;
    (void)bit_count;
}

static void led_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    if (port != NULL && pin != 0u) {
        HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void dbg_toggle(GPIO_TypeDef *port, uint16_t pin)
{
    if (port != NULL && pin != 0u) {
        HAL_GPIO_TogglePin(port, pin);
    }
}

static uint32_t apb2_timer_clock_hz(void)
{
    const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    const uint32_t ppre2 = RCC->CFGR & RCC_CFGR_PPRE2;

    return ppre2 == 0u
        ? pclk2
        : pclk2 * 2u;
}

static uint32_t timer_input_clock_hz(TIM_HandleTypeDef *htim)
{
    (void)htim; /* TIM1 is on APB2 for the required target. */
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2);
    return ppre2 == 0u ? pclk : pclk * 2u;
}

static bool configure_tim_rate(void)
{
    const uint32_t timer_clock = timer_input_clock_hz(g_hw.htim_tx);
    const uint32_t chip_rate = g_cfg.bitrate_bps * 2u;
    if (chip_rate == 0u || timer_clock % chip_rate != 0u) {
        return false;
    }
    const uint32_t ticks = timer_clock / chip_rate;
    if (ticks == 0u || ticks > 65536u) {
        return false;
    }
    __HAL_TIM_DISABLE(g_hw.htim_tx);
    __HAL_TIM_SET_PRESCALER(g_hw.htim_tx, 0u);
    __HAL_TIM_SET_AUTORELOAD(g_hw.htim_tx, ticks - 1u);
    __HAL_TIM_SET_COUNTER(g_hw.htim_tx, 0u);
    __HAL_TIM_CLEAR_FLAG(g_hw.htim_tx, TIM_FLAG_UPDATE);

    g_dbg_timer_clock_hz = timer_clock;
    g_dbg_timer_ticks = ticks;
    return true;
}

static void dma_invalidate(void *address, size_t length)
{
#if MAN_DMA_USE_CACHE_MAINTENANCE
    const uintptr_t start = (uintptr_t)address & ~(uintptr_t)31u;
    const uintptr_t end = ((uintptr_t)address + length + 31u) & ~(uintptr_t)31u;
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)address;
    (void)length;
#endif
}

static void dma_clean(void *address, size_t length)
{
#if MAN_DMA_USE_CACHE_MAINTENANCE
    const uintptr_t start = (uintptr_t)address & ~(uintptr_t)31u;
    const uintptr_t end = ((uintptr_t)address + length + 31u) & ~(uintptr_t)31u;
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)address;
    (void)length;
#endif
}

static bool enqueue_file_tx_item(const uart_file_frame_t *frame)
{
    if (frame == NULL || frame->payload_length > UART_FILE_MAX_PAYLOAD) {
        return false;
    }

    file_tx_item_t item;
    memset(&item, 0, sizeof(item));

    item.type = frame->type;
    item.session_id = frame->session_id;
    item.sequence = frame->sequence;
    item.payload_length = frame->payload_length;

    if (frame->payload_length != 0u) {
        memcpy(item.payload, frame->payload, frame->payload_length);
    }

    if (osMessageQueuePut(g_file_tx_queue, &item, 0u, 0u) != osOK) {
        ++g_dbg_host_file_queue_rejects;
        return false;
    }

    return true;
}

static void radio_file_item_received(void *context, const uart_file_frame_t *frame)
{
    (void)context;


    ++g_dbg_radio_file_rx_items;

    g_dbg_radio_file_rx_session =
        frame->session_id;

    g_dbg_radio_file_rx_sequence =
        frame->sequence;


    /*
     * ACK предназначен нашему sender task.
     * ACK на ACK не отправляем.
     */
    if (frame->type ==
        UART_FILE_TYPE_ACK) {

        if (frame->payload_length != 1u) {

            ++g_dbg_radio_file_rx_fragment_errors;
            return;
        }


        file_ack_event_t ack = {
            .acked_type =
                frame->payload[0],

            .session_id =
                frame->session_id,

            .sequence =
                frame->sequence
        };


        if (osMessageQueuePut(
                g_file_ack_queue,
                &ack,
                0u,
                0u) == osOK) {

            ++g_dbg_file_ack_rx;
        }


        return;
    }

    bool should_ack = false;
    bool forward_to_host = false;

    switch (frame->type) {

    case UART_FILE_TYPE_START:

        /*
         * Новый transfer session.
         */
        if (!g_radio_file_rx_session_active ||
            frame->session_id !=
                g_radio_file_rx_session_id) {

            g_radio_file_rx_session_active =
                true;

            g_radio_file_rx_session_id =
                frame->session_id;

            g_radio_file_expected_chunk_valid =
                true;

            g_radio_file_expected_chunk =
                0u;

            g_radio_file_rx_end_forwarded =
                false;

            g_dbg_radio_file_rx_bytes =
                0u;


            if (frame->payload_length < 14u) {
                ++g_dbg_radio_file_rx_fragment_errors;
                break;
            }


            const uint64_t file_size =
                uart_read_le64(
                    frame->payload
                );

            g_dbg_radio_file_expected_bytes =
                (uint32_t)file_size;


            g_radio_file_total_chunks =
                uart_read_le32(
                    &frame->payload[10]
                );


            ++g_dbg_radio_file_rx_start;

            forward_to_host = true;
            should_ack = true;

        } else {

            /*
             * Скорее всего START был принят,
             * но его ACK потерялся.
             *
             * Python START второй раз не нужен.
             */
            ++g_dbg_file_duplicates;

            should_ack = true;
        }

        break;


    case UART_FILE_TYPE_DATA:

		if (!g_radio_file_rx_session_active ||
			frame->session_id !=
				g_radio_file_rx_session_id ||
			!g_radio_file_expected_chunk_valid) {

			++g_dbg_radio_file_rx_chunk_gaps;
			break;
		}


		/*
		 * Именно следующий ожидаемый chunk.
		 */
		if (frame->sequence ==
			g_radio_file_expected_chunk) {

			++g_dbg_radio_file_rx_data;

			g_dbg_radio_file_rx_bytes +=
				frame->payload_length;

			++g_radio_file_expected_chunk;

			forward_to_host = true;
			should_ack = true;


		/*
		 * DATA уже принимали.
		 * Вероятнее всего потерялся ACK.
		 */
		} else if (
			g_radio_file_expected_chunk != 0u &&
			frame->sequence ==
				g_radio_file_expected_chunk - 1u) {

			++g_dbg_file_duplicates;

			/*
			 * Не forward!
			 *
			 * Иначе Python запишет chunk дважды.
			 */
			should_ack = true;


		} else {

			++g_dbg_radio_file_rx_chunk_gaps;
		}

		break;


    case UART_FILE_TYPE_END:
		if (!g_radio_file_rx_session_active ||
			frame->session_id !=
				g_radio_file_rx_session_id) {

			break;
		}


		if (frame->sequence !=
				g_radio_file_total_chunks ||
			g_radio_file_expected_chunk !=
				g_radio_file_total_chunks) {

			++g_dbg_radio_file_rx_chunk_gaps;
			break;
		}


		if (!g_radio_file_rx_end_forwarded) {

			++g_dbg_radio_file_rx_end;

			forward_to_host = true;

		} else {

			/*
			 * END ACK потерялся.
			 */
			++g_dbg_file_duplicates;
		}


		should_ack = true;

		break;


    default:

        ++g_dbg_radio_file_rx_fragment_errors;

        break;
    }


    /*
     * Сначала гарантируем, что logical item
     * передан в UART queue локального host.
     *
     * Только после этого говорим отправителю,
     * что item принят.
     */
    if (forward_to_host) {

        if (!queue_received_file_to_host(
                frame)) {

            /*
             * ACK НЕ отправляем.
             *
             * Sender повторит logical item,
             * и у нас будет ещё одна попытка
             * передать его локальному Python.
             */
            return;
        }


        if (frame->type ==
            UART_FILE_TYPE_END) {

            g_radio_file_rx_end_forwarded =
                true;
        }
    }


    if (should_ack) {

        (void)send_radio_file_ack(
            frame
        );
    }
}

static bool wait_for_file_ack(
    uint8_t expected_type,
    uint32_t expected_session,
    uint32_t expected_sequence)
{
    const uint32_t timeout =
        pdMS_TO_TICKS(
            MAN_FILE_ACK_TIMEOUT_MS
        );

    const uint32_t start =
        osKernelGetTickCount();


    for (;;) {

        const uint32_t elapsed =
            osKernelGetTickCount() -
            start;


        if (elapsed >= timeout) {
            return false;
        }


        const uint32_t remaining =
            timeout - elapsed;


        file_ack_event_t ack;


        if (osMessageQueueGet(
                g_file_ack_queue,
                &ack,
                NULL,
                remaining) != osOK) {

            return false;
        }


        /*
         * Старый/дублированный ACK просто
         * игнорируем.
         */
        if (ack.session_id !=
                expected_session ||
            ack.sequence !=
                expected_sequence ||
            ack.acked_type !=
                expected_type) {

            continue;
        }


        return true;
    }
}

static void discard_pending_file_items(void)
{
    file_tx_item_t discarded;

    while (osMessageQueueGet(
            g_file_tx_queue,
            &discarded,
            NULL,
            0u) == osOK) {
    }
}

static bool queue_received_file_to_host(const uart_file_frame_t *frame)
{
    if (frame == NULL) {
        return false;
    }

    uint8_t wire[
        UART_FILE_MAX_WIRE_SIZE
    ];

    const uint16_t wire_length =
        uart_file_build_frame(
            frame->type,
            frame->session_id,
            frame->sequence,
            frame->payload,
            frame->payload_length,
            wire,
            sizeof(wire)
        );

    if (wire_length == 0u) {
        ++g_dbg_host_file_rx_forward_errors;
        return false;
    }

    /*
     * ApplicationTask может немного подождать.
     *
     * При 460800 baud один chunk ~200 B
     * уйдёт всего за несколько ms.
     *
     * Важно: если локальный UART не принимает
     * logical item, RF ACK отправлять не будем.
     */
    if (!queue_uart_output_wait(
            wire,
            wire_length,
            osWaitForever)) {

        ++g_dbg_host_file_rx_forward_errors;
        return false;
    }

    ++g_dbg_host_file_rx_forwarded;

    return true;
}

static bool queue_host_file_result(
    uint8_t result_type,
    uint8_t acked_type,
    uint32_t session_id,
    uint32_t sequence
);

static bool queue_host_file_result(
    uint8_t result_type,
    uint8_t acked_type,
    uint32_t session_id,
    uint32_t sequence)
{
    uint8_t payload[1] = {
        acked_type
    };


    uint8_t wire[
        UART_FILE_MAX_WIRE_SIZE
    ];


    const uint16_t wire_length =
        uart_file_build_frame(
            result_type,
            session_id,
            sequence,
            payload,
            sizeof(payload),
            wire,
            sizeof(wire)
        );


    if (wire_length == 0u) {

        ++g_dbg_host_ack_build_errors;

        return false;
    }


    /*
     * Вызывается из FileTransportTask,
     * поэтому здесь МОЖНО ждать UART queue.
     *
     * UartTask при этом независимо продолжает
     * обслуживать RX DMA.
     */
    if (!queue_uart_output_wait(
            wire,
            wire_length,
            osWaitForever)) {

        return false;
    }


    if (result_type ==
        UART_FILE_TYPE_ACK) {

        ++g_dbg_host_ack_tx;

    } else {

        ++g_dbg_host_nack_tx;
    }


    return true;
}

static void FileTransportTask(
    void *argument)
{
    (void)argument;


    file_tx_item_t item;


    for (;;) {

        if (osMessageQueueGet(
                g_file_tx_queue,
                &item,
                NULL,
                osWaitForever) != osOK) {

            continue;
        }


        uart_file_frame_t frame = {
            .type =
                item.type,

            .session_id =
                item.session_id,

            .sequence =
                item.sequence,

            .payload =
                item.payload,

            .payload_length =
                item.payload_length
        };


        bool delivered = false;


        for (uint32_t attempt = 0u;
             attempt <=
                 MAN_FILE_MAX_RETRIES;
             ++attempt) {


            if (attempt != 0u) {
                ++g_dbg_file_arq_retries;
            }


            if (!send_uart_file_frame_over_radio(
                    &frame)) {

                continue;
            }


            /*
             * Последний fragment получает END,
             * поэтому ManchesterTxTask после него
             * автоматически переключает RF обратно
             * в RX.
             */
            if (wait_for_file_ack(
                    frame.type,
                    frame.session_id,
                    frame.sequence)) {

                delivered = true;

                ++g_dbg_file_arq_ok;


                /*
                 * Вот теперь Python имеет право
                 * прислать следующий logical item.
                 *
                 * Это означает:
                 *
                 * Python -> STM32 A
                 * -> RF
                 * -> STM32 B
                 * -> RF ACK
                 * -> STM32 A
                 * -> UART ACK
                 * -> Python
                 */
                (void)queue_host_file_result(
                    UART_FILE_TYPE_ACK,
                    frame.type,
                    frame.session_id,
                    frame.sequence
                );


                break;
            }


            ++g_dbg_file_arq_timeouts;
        }


        if (!delivered) {

            ++g_dbg_file_arq_failed;

            ++g_diag.dropped_blocks;

            led_set(
                g_hw.led_error_port,
                g_hw.led_error_pin,
                true
            );

            /*
             * Python должен узнать, что этот item
             * доставить не удалось.
             */
            (void)queue_host_file_result(
                UART_FILE_TYPE_NACK,
                frame.type,
                frame.session_id,
                frame.sequence
            );
        }
    }
}

static bool send_radio_file_ack(
    const uart_file_frame_t *received)
{
    uint8_t payload[1] = {
        received->type
    };

    uart_file_frame_t ack = {
        .type = UART_FILE_TYPE_ACK,
        .session_id = received->session_id,
        .sequence = received->sequence,
        .payload = payload,
        .payload_length = sizeof(payload)
    };


    /*
     * Даём удалённой стороне время:
     *
     * TX -> RX switch
     * RF settle
     * TIM IC Drain
     * DSP reset
     *
     * После этого начинаем ACK.
     */
    if (is_rf_halfduplex()) {
        osDelay(
            pdMS_TO_TICKS(
                MAN_FILE_ACK_TURNAROUND_MS
            )
        );
    }


    if (!send_uart_file_frame_over_radio(
            &ack)) {

        return false;
    }


    ++g_dbg_file_ack_tx;

    return true;
}

static void uart_file_frame_received(void *context, const uart_file_frame_t *frame)
{
    (void)context;
    bool accepted = false;

    switch (frame->type) {
		case UART_FILE_TYPE_START:
			accepted = handle_file_start(frame);
			if (!accepted) {
				g_uart_file_mode = false;
			}
			break;

		case UART_FILE_TYPE_DATA:
			accepted = handle_file_data(frame);
			break;

		case UART_FILE_TYPE_END:
			accepted = handle_file_end(frame);
			break;

		default:
			++g_dbg_file_session_errors;
			break;
    }
    if (accepted) {
        if (!enqueue_file_tx_item(frame)) {
            ++g_dbg_file_session_errors;
        }
    }
}

bool Manchester_ServiceInit(const man_platform_t *platform, const man_runtime_config_t *config)
{
	if (platform == NULL ||
	    config == NULL ||
	    platform->htim_tx == NULL ||
	    platform->htim_rx_ic == NULL ||
	    platform->huart == NULL ||
	    platform->tx_port == NULL ||
	    platform->tx_pin == 0u ||
	    config->max_payload == 0u ||
	    config->max_payload > MAN_MAX_PAYLOAD ||
	    config->preamble_bytes < MAN_PREAMBLE_BYTES_MIN ||
	    config->preamble_bytes > MAN_PREAMBLE_BYTES_MAX ||
	    config->glitch_filter_samples >
	        MAN_GLITCH_FILTER_MAX_SAMPLES) {

	    return false;
	}
    memset(&g_diag, 0, sizeof(g_diag));

    // Участок протокола передачи BEGIN
    memset(&g_uart_file_session, 0,sizeof(g_uart_file_session));
    g_uart_file_mode = false;
    g_uart_file_magic_probe = false;
    // Участок протокола передачи END

    uart_file_parser_init(&g_uart_file_parser, uart_file_frame_received, NULL);
    radio_file_reassembler_init(&g_radio_file_reassembler, radio_file_item_received, NULL);

    g_hw = *platform;
    g_cfg = *config;
    g_fec = config->fec_enabled ? man_fec_hamming74_codec() : man_fec_identity_codec();

    // Инициализация приемника СШП модема - изначально слушаем
    rf_apply_rx_gpio();
    g_rx_muted = false;
	g_rf_tx_active = false;

    if (g_fec == NULL) {
        return false; /* Hamming is intentionally only an extension point in this revision. */
    }
    led_set(g_hw.led_ok_port, g_hw.led_ok_pin, false);
    led_set(g_hw.led_tx_port, g_hw.led_tx_pin, false);
    led_set(g_hw.led_error_port, g_hw.led_error_pin, false);
    tx_set_idle_level();

    if (!configure_tim_rate()) {
        return false;
    }
    g_rx_sample_rate_hz = apb2_timer_clock_hz();
    const uint32_t rx_chip_rate = g_cfg.bitrate_bps * 2u;

    if (rx_chip_rate == 0u ||
        (g_rx_sample_rate_hz % rx_chip_rate) != 0u) {

        return false;
    }

    g_rx_chip_ticks = g_rx_sample_rate_hz / rx_chip_rate;


    /* Reset TIM-IC → DSP frontend state. */
    g_ic_previous_valid = false;
    g_ic_previous_capture = 0u;
    g_ic_virtual_tick = 0u;

    g_ic_idle_armed = false;
    g_ic_last_activity_tick = 0u;

    g_dbg_ic_dsp_edges = 0u;
    g_dbg_ic_dsp_idle_flushes = 0u;
    g_dbg_ic_dsp_large_gaps = 0u;
    g_dbg_ic_dsp_last_delta = 0u;

    g_radio_file_rx_session_active = false;
    g_radio_file_rx_session_id = 0u;

    g_radio_file_expected_chunk_valid = false;
    g_radio_file_expected_chunk = 0u;

    g_radio_file_rx_end_forwarded = false;

    if (!man_rx_decoder_init(&g_decoder, &g_cfg, g_rx_sample_rate_hz, g_fec, &g_diag, NULL, NULL)) {
        return false;
    }
    g_dbg_pattern_length_0 = g_decoder.paths[0].pattern_length;
    g_dbg_pattern_length_1 = g_decoder.paths[1].pattern_length;
    g_initialized = true;
    return true;
}

bool Manchester_CreateRtosObjects(void)
{
    if (!g_initialized) {
        return false;
    }
    g_tx_queue = osMessageQueueNew(MAN_TX_QUEUE_DEPTH, sizeof(man_packet_t), NULL);
    g_rx_queue = osMessageQueueNew(MAN_RX_QUEUE_DEPTH, sizeof(man_packet_t), NULL);
    g_file_tx_queue = osMessageQueueNew(MAN_FILE_TX_QUEUE_DEPTH, sizeof(file_tx_item_t), NULL);
    g_file_ack_queue = osMessageQueueNew(MAN_FILE_ACK_QUEUE_DEPTH, sizeof(file_ack_event_t), NULL);
    g_uart_out_queue = osMessageQueueNew(MAN_UART_OUT_QUEUE_DEPTH, sizeof(uart_output_t), NULL);
    if (g_tx_queue == NULL || g_rx_queue == NULL || g_uart_out_queue == NULL || g_file_tx_queue == NULL || g_file_ack_queue == NULL) {
        return false;
    }

    const osThreadAttr_t file_attr = {.name = "FileTransport", .priority = osPriorityNormal, .stack_size = 3072u};
    const osThreadAttr_t rx_attr = {.name="ManchesterRx", .priority=osPriorityHigh, .stack_size=4096u};
    const osThreadAttr_t tx_attr = {.name="ManchesterTx", .priority=osPriorityAboveNormal, .stack_size=3072u};
    const osThreadAttr_t uart_attr = {.name="Uart", .priority=osPriorityNormal, .stack_size=3072u};
    const osThreadAttr_t app_attr = {.name="Application", .priority=osPriorityNormal, .stack_size=2048u};
    const osThreadAttr_t diag_attr = {.name="Diagnostics", .priority=osPriorityLow, .stack_size=1024u};



#if MAN_COMPILETIME_ROLE != MAN_ROLE_TX_ONLY
    g_rx_task = osThreadNew(ManchesterRxTask, NULL, &rx_attr);
#endif
#if MAN_COMPILETIME_ROLE != MAN_ROLE_RX_ONLY
    g_tx_task = osThreadNew(ManchesterTxTask, NULL, &tx_attr);
#endif
    g_uart_task = osThreadNew(UartTask, NULL, &uart_attr);
    g_app_task = osThreadNew(ApplicationTask, NULL, &app_attr);
    g_diag_task = osThreadNew(DiagnosticsTask, NULL, &diag_attr);
    g_file_task = osThreadNew(FileTransportTask, NULL, &file_attr);
    bool ok = g_uart_task != NULL && g_app_task != NULL && g_diag_task != NULL;
    ok = ok && g_file_task != NULL;
#if MAN_COMPILETIME_ROLE != MAN_ROLE_TX_ONLY
    ok = ok && g_rx_task != NULL;
#endif
#if MAN_COMPILETIME_ROLE != MAN_ROLE_RX_ONLY
    ok = ok && g_tx_task != NULL;
#endif
    return ok;
}

static void rx_frame_to_queue(void *user, const man_packet_t *packet)
{
    (void)user;
    if (osMessageQueuePut(g_rx_queue, packet, 0u, 0u) != osOK) {
        ++g_diag.queue_overflows;
        ++g_diag.dropped_blocks;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
    }
}

static uint8_t g_dbg_raw_previous_level;
static bool g_dbg_raw_previous_valid;
static uint32_t g_dbg_raw_sample_index;
static uint32_t g_dbg_raw_previous_edge;
volatile uint32_t g_dbg_raw_edge_histogram[32];

void Manchester_TimIcCaptureWakeFromIsr(void)
{
    if (g_rx_task != NULL) {
        (void)osThreadFlagsSet(
            g_rx_task,
            RX_FLAG_TIM_IC
        );
    }
}

static void tim_ic_dsp_consumer(const uint16_t *timestamps, uint32_t count, void *context)
{
    (void)context;
    if (timestamps == NULL || count == 0u) {
        return;
    }

    for (uint32_t i = 0u; i < count; ++i) {
        const uint16_t current = timestamps[i];
        /*
         * Первый capture после старта/idle.
         *
         * Реальный промежуток от предыдущего пакета
         * нам больше не нужен.
         */
        if (!g_ic_previous_valid) {
            g_ic_previous_capture = current;
            g_ic_previous_valid = true;
            man_rx_decoder_feed_edge(&g_decoder, g_ic_virtual_tick);
            ++g_dbg_ic_dsp_edges;
            continue;
        }
        /*
         * Modulo-65536 subtraction.
         *
         * Поэтому нормально работает, например:
         *
         * previous = 65500
         * current  = 72
         *
         * delta = 108
         */
        const uint16_t delta = (uint16_t)(current - g_ic_previous_capture);
        g_ic_previous_capture = current;
        g_dbg_ic_dsp_last_delta = delta;

        if (delta == 0u) {
            continue;
        }
        /*
         * Если фронтов долго не было, существующий
         * emit_until() сам обнаружит >3 chip timeout
         * и отпустит clock lock.
         */
        if (delta > g_rx_chip_ticks * 3u) {
            ++g_dbg_ic_dsp_large_gaps;
        }
        g_ic_virtual_tick += (uint64_t)delta;
        man_rx_decoder_feed_edge(&g_decoder, g_ic_virtual_tick);
        ++g_dbg_ic_dsp_edges;
    }
}

static void reset_rx_frontend(void)
{
    /*
     * Всё, что TIM8 накопил во время переключения
     * RF или нашего собственного TX, выбрасываем.
     */
    (void)Manchester_TimIcCaptureDrain(NULL, NULL);

    /*
     * Полностью сбрасываем Manchester DSP.
     */
    man_rx_decoder_reset(&g_decoder);

    /*
     * Сбрасываем TIM8 -> virtual timeline frontend.
     */
    g_ic_previous_valid = false;
    g_ic_previous_capture = 0u;
    g_ic_virtual_tick = 0u;
    g_ic_idle_armed = false;
    g_ic_last_activity_tick = osKernelGetTickCount();
}

static void ManchesterRxTask(void *argument)
{
    (void)argument;
    g_decoder.callback = rx_frame_to_queue;
    g_decoder.callback_user = NULL;

    if (!Manchester_TimIcCaptureStart(g_hw.htim_rx_ic, g_hw.tim_rx_ic_channel)) {
        ++g_diag.dma_overruns;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);

        for (;;) {
            osDelay(1000u);
        }
    }

    for (;;) {
        (void)osThreadFlagsWait(RX_FLAG_TIM_IC, osFlagsWaitAny, 1u);
        /*
         * Пока наша плата сама передаёт,
         * TIM8 может что-то ловить, но в DSP
         * это категорически не отправляем.
         */
        if (g_rx_muted) {
            (void)Manchester_TimIcCaptureDrain(NULL, NULL);
            continue;
        }

        /*
         * Только что вернулись TX -> RX.
         * Старый timeline больше невалиден.
         */
        if (g_rx_reset_requested) {
            reset_rx_frontend();
            g_rx_reset_requested = false;
            continue;
        }

        const uint32_t drained = Manchester_TimIcCaptureDrain(tim_ic_dsp_consumer, NULL);
        const uint32_t now = osKernelGetTickCount();
        if (drained != 0u) {
            g_ic_last_activity_tick = now;
            g_ic_idle_armed = true;

        } else if (g_ic_idle_armed) {
            if ((uint32_t)(now - g_ic_last_activity_tick) >= 1u) {
                const uint64_t flush_tick = g_ic_virtual_tick + (uint64_t)g_rx_chip_ticks * 4u;
                man_rx_decoder_advance_time(&g_decoder, flush_tick);
                g_ic_virtual_tick = flush_tick;
                g_ic_previous_valid = false;
                g_ic_idle_armed = false;
                ++g_dbg_ic_dsp_idle_flushes;
            }
        }
    }
}

static void tx_dma_half(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    if (g_tx_task != NULL) {
        (void)osThreadFlagsSet(g_tx_task, TX_FLAG_HALF);
    }
}

static void tx_dma_done(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(g_hw.htim_tx, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE(g_hw.htim_tx);
    tx_set_idle_level();
    rf_enter_rx_mode();
    if (g_tx_task != NULL) {
        (void)osThreadFlagsSet(g_tx_task, TX_FLAG_DONE);
    }
}

static void tx_dma_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(g_hw.htim_tx, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE(g_hw.htim_tx);
    tx_set_idle_level();
    if (g_tx_task != NULL) {
        (void)osThreadFlagsSet(g_tx_task, TX_FLAG_ERROR);
    }
}

static bool start_tx_frame(const man_packet_t *packet)
{
    size_t wire_bit_count = 0u;
    size_t chip_count = 0u;
    if (!man_frame_build_wire_bits(&g_cfg, packet, g_fec, g_tx_fec_ctx, g_tx_wire_bits, MAN_TX_MAX_WIRE_BITS, &wire_bit_count)) {
        return false;
    }
    Manchester_TestHookMutateWireBits(g_tx_wire_bits, wire_bit_count);
    if (!man_line_encode_bsrr(g_tx_wire_bits, wire_bit_count, g_cfg.encoding, g_cfg.tx_invert, g_cfg.tx_invert ? 1u : 0u, g_hw.tx_pin, g_tx_bsrr, MAN_TX_MAX_CHIPS, &chip_count)) {
        return false;
    }
    DMA_HandleTypeDef *hdma = g_hw.htim_tx->hdma[TIM_DMA_ID_UPDATE];
    if (hdma == NULL || chip_count == 0u || chip_count > 0xFFFFu) {
        return false;
    }
    hdma->XferHalfCpltCallback = tx_dma_half;
    hdma->XferCpltCallback = tx_dma_done;
    hdma->XferErrorCallback = tx_dma_error;
    dma_clean(g_tx_bsrr, chip_count * sizeof(g_tx_bsrr[0]));
    tx_set_idle_level();
    __HAL_TIM_SET_COUNTER(g_hw.htim_tx, 0u);
    __HAL_TIM_CLEAR_FLAG(g_hw.htim_tx, TIM_FLAG_UPDATE);
    if (HAL_DMA_Start_IT(hdma, (uint32_t)(uintptr_t)g_tx_bsrr, (uint32_t)(uintptr_t)&g_hw.tx_port->BSRR, (uint32_t)chip_count) != HAL_OK) {
        return false;
    }
    __HAL_TIM_ENABLE_DMA(g_hw.htim_tx, TIM_DMA_UPDATE);

    //Теперь максимально поздно включаем RF transmit.
    rf_enter_tx_mode();
    // Сразу после этого стартует Manchester.

    __HAL_TIM_ENABLE(g_hw.htim_tx);
    return true;
}

static void ManchesterTxTask(void *argument)
{
    (void)argument;
    man_packet_t packet;
    for (;;) {
        if (osMessageQueueGet(g_tx_queue, &packet, NULL, osWaitForever) != osOK) {
            continue;
        }
        led_set(g_hw.led_tx_port, g_hw.led_tx_pin, true);
        dbg_toggle(g_hw.dbg_tx_port, g_hw.dbg_tx_pin);

//        // Если хотим transceiver для отладки по одному проводу, то включаем MAN_PHY_WIRED_LOOPBACK
//        if (is_rf_halfduplex()) {
//            rf_enter_tx_mode();
//        }

        /*
         * Защита от старого флага TX_DONE/TX_ERROR.
         */
        osThreadFlagsClear(TX_FLAG_DONE | TX_FLAG_ERROR);

        if (!start_tx_frame(&packet)) {
        	// Если хотим transceiver для отладки по одному проводу, то включаем MAN_PHY_WIRED_LOOPBACK
            if (is_rf_halfduplex()) {
                rf_enter_rx_mode();
            }

            ++g_diag.dma_overruns;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
            led_set(g_hw.led_tx_port, g_hw.led_tx_pin, false);
            continue;
        }
        const uint32_t result = osThreadFlagsWait(TX_FLAG_DONE | TX_FLAG_ERROR, osFlagsWaitAny, osWaitForever);
        /*
         * Возвращаемся в RX только когда это последний кадр логического блока.
         *
         * Для single:
         *   END стоит на последнем фрагменте сообщения.
         *
         * Для stream:
         *   END стоит на последнем блоке потока после idle timeout.
         */
//        if ((packet.flags & MAN_FLAG_END) != 0u) {
//            rf_enter_rx_mode();
//        }

        if ((result & TX_FLAG_DONE) != 0u) {
            ++g_diag.manchester_tx_frames;
        } else {
            ++g_diag.dma_overruns;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        }

        /*
         * В RF half-duplex возвращаемся в RX только после последнего
         * кадра логического сообщения.
         *
         * В wired loopback ничего не переключаем вообще.
         */
        if (is_rf_halfduplex() && (((packet.flags & MAN_FLAG_END) != 0u) || ((result & TX_FLAG_ERROR) != 0u))) {
            rf_enter_rx_mode();
        }

        led_set(g_hw.led_tx_port, g_hw.led_tx_pin, false);
    }
}

static bool uart_event_pop(uart_isr_event_t *event)
{
    bool available = false;
    taskENTER_CRITICAL();
    if (g_uart_event_tail != g_uart_event_head) {
        *event = g_uart_events[g_uart_event_tail];
        g_uart_event_tail = (uint8_t)((g_uart_event_tail + 1u) % MAN_UART_ISR_EVENT_COUNT);
        available = true;
    }
    taskEXIT_CRITICAL();
    return available;
}

static bool is_stats_command(const uint8_t *data, uint16_t length)
{
    static const uint8_t command[] = ":stat\r\n";
    return length == sizeof(command) - 1u && memcmp(data, command, sizeof(command) - 1u) == 0;
}

static bool queue_uart_output_wait(const uint8_t *data, uint16_t length, uint32_t timeout)
{
    while (length != 0u) {

        uart_output_t output;

        output.length =
            length > MAN_UART_TX_STAGE_BYTES
                ? MAN_UART_TX_STAGE_BYTES
                : length;


        memcpy(
            output.data,
            data,
            output.length
        );


        if (osMessageQueuePut(
                g_uart_out_queue,
                &output,
                0u,
                timeout) != osOK) {

            ++g_diag.queue_overflows;
            ++g_diag.dropped_blocks;

            led_set(
                g_hw.led_error_port,
                g_hw.led_error_pin,
                true
            );

            return false;
        }


        data += output.length;

        length =
            (uint16_t)(
                length -
                output.length
            );
    }


    return true;
}

static void queue_uart_output(const uint8_t *data, uint16_t length)
{
    (void)queue_uart_output_wait(data, length, 0u);
}

static void queue_file_debug(
    const char *format,
    ...)
{
    char buffer[128];

    va_list args;
    va_start(args, format);

    int length = vsnprintf(
        buffer,
        sizeof(buffer),
        format,
        args
    );

    va_end(args);

    if (length <= 0) {
        return;
    }

    if (length >
        (int)sizeof(buffer)) {

        length =
            sizeof(buffer);
    }

    queue_uart_output(
        (const uint8_t *)buffer,
        (uint16_t)length
    );
}

static void queue_stats(void)
{
    char text[MAN_UART_TX_STAGE_BYTES];
    const int n = snprintf(text, sizeof(text),
        "UART_RX=%lu TX_FRAMES=%lu RX_OK=%lu CRC=%lu SYNC_LOSS=%lu DMA_OVR=%lu "
        "Q_OVR=%lu UART_OVR=%lu SEQ_GAP=%lu RESYNC=%lu LINE=%lu PHASE=%lu DROP=%lu\r\n",
        (unsigned long)g_diag.uart_rx_bytes,
        (unsigned long)g_diag.manchester_tx_frames,
        (unsigned long)g_diag.manchester_rx_good_frames,
        (unsigned long)g_diag.crc_errors,
        (unsigned long)g_diag.sync_losses,
        (unsigned long)g_diag.dma_overruns,
        (unsigned long)g_diag.queue_overflows,
        (unsigned long)g_diag.uart_overruns,
        (unsigned long)g_diag.sequence_gaps,
        (unsigned long)g_diag.resynchronizations,
        (unsigned long)g_diag.line_code_errors,
        (unsigned long)g_diag.phase_errors,
        (unsigned long)g_diag.dropped_blocks);
    if (n > 0) {
        queue_uart_output((const uint8_t *)text, (uint16_t)((n < (int)sizeof(text)) ? n : (int)sizeof(text) - 1));
    }
}

static bool queue_tx_packet_ex(
    const uint8_t *data,
    uint16_t length,
    bool stream,
    bool end,
    uint8_t extra_flags,
    uint32_t timeout)
{
#if MAN_COMPILETIME_ROLE != MAN_ROLE_RX_ONLY

    if (length > g_cfg.max_payload) {
        return false;
    }

    man_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.flags =
        (uint8_t)(
            (stream ? MAN_FLAG_STREAM : 0u)
            | (end ? MAN_FLAG_END : 0u)
            | (g_cfg.fec_enabled ? MAN_FLAG_FEC : 0u)
            | (g_tx_reset_pending ? MAN_FLAG_RESET : 0u)
            | extra_flags
        );

    g_tx_reset_pending = false;
    packet.seq = g_tx_sequence++;
    packet.length = length;

    if (length != 0u) {
        memcpy(packet.payload, data, length);
    }

    if (osMessageQueuePut(g_tx_queue, &packet, 0u, timeout) != osOK) {
        ++g_diag.queue_overflows;
        ++g_diag.dropped_blocks;
        g_tx_reset_pending = true;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        return false;
    }
    return true;
#else

    (void)data;
    (void)length;
    (void)stream;
    (void)end;
    (void)extra_flags;
    (void)timeout;

    return false;

#endif
}

static void queue_tx_packet(const uint8_t *data, uint16_t length, bool stream, bool end)
{
    (void)queue_tx_packet_ex(data, length, stream, end, 0u, 0u);
}

static void consume_uart_block(const uint8_t *data, uint16_t length, bool end_of_block,
                               uint8_t *assembly, uint16_t *assembly_length, bool *stream_open)
{
    if (length != 0u) {
        g_diag.uart_rx_bytes += length;
    }
    if (g_cfg.transfer_mode == MAN_MODE_STREAM) {
        uint16_t offset = 0u;
        bool sent_end = false;
        if (length != 0u) {
            *stream_open = true;
        }
        while (offset < length) {
            const uint16_t space = (uint16_t)(g_cfg.max_payload - *assembly_length);
            const uint16_t take = (uint16_t)(((length - offset) < space) ? (length - offset) : space);
            memcpy(&assembly[*assembly_length], &data[offset], take);
            *assembly_length = (uint16_t)(*assembly_length + take);
            offset = (uint16_t)(offset + take);
            if (*assembly_length == g_cfg.max_payload) {
                sent_end = end_of_block && offset == length;
                queue_tx_packet(assembly, *assembly_length, true, sent_end);
                *assembly_length = 0u;
                if (sent_end) {
                    *stream_open = false;
                }
            }
        }
        if (end_of_block && *assembly_length != 0u) {
            queue_tx_packet(assembly, *assembly_length, true, true);
            *assembly_length = 0u;
            *stream_open = false;
            sent_end = true;
        }
        if (end_of_block && *stream_open && !sent_end) {
            /* Exact multiple of max_payload: terminate with an empty END marker. */
            queue_tx_packet(NULL, 0u, true, true);
            *stream_open = false;
        }
    } else {
        if ((uint32_t)*assembly_length + length > g_cfg.max_single_message ||
            (uint32_t)*assembly_length + length > MAN_MAX_SINGLE_MESSAGE) {
            ++g_diag.uart_overruns;
            ++g_diag.dropped_blocks;
            *assembly_length = 0u;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
            return;
        }
        memcpy(&assembly[*assembly_length], data, length);
        *assembly_length = (uint16_t)(*assembly_length + length);
        if (end_of_block) {
            uint16_t offset = 0u;
            while (offset < *assembly_length) {
                const uint16_t remaining = (uint16_t)(*assembly_length - offset);
                const uint16_t take = remaining > g_cfg.max_payload ? g_cfg.max_payload : remaining;
                queue_tx_packet(&assembly[offset], take, false, (uint16_t)(offset + take) == *assembly_length);
                offset = (uint16_t)(offset + take);
            }
            *assembly_length = 0u;
        }
    }
}

// Новый режим пакетной передачи BEGIN
static uint16_t uart_read_le16(const uint8_t *data)
{
    return
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8u);
}

static uint32_t uart_read_le32(const uint8_t *data)
{
    return
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8u) |
        ((uint32_t)data[2] << 16u) |
        ((uint32_t)data[3] << 24u);
}

static uint64_t uart_read_le64(const uint8_t *data)
{
    return
        (uint64_t)data[0] |
        ((uint64_t)data[1] << 8u) |
        ((uint64_t)data[2] << 16u) |
        ((uint64_t)data[3] << 24u) |
        ((uint64_t)data[4] << 32u) |
        ((uint64_t)data[5] << 40u) |
        ((uint64_t)data[6] << 48u) |
        ((uint64_t)data[7] << 56u);
}

static bool radio_file_emit_fragment(void *context, const uint8_t *data, uint16_t length, bool last_fragment){
    (void)context;

    const bool queued =
        queue_tx_packet_ex(
            data,
            length,

            /*
             * Это отдельный transport packet,
             * старый STREAM здесь не нужен.
             */
            false,

            /*
             * После последнего fragment logical
             * FILE_* item возвращаем RF в RX.
             *
             * Это потом позволит получить ACK.
             */
            last_fragment,

            MAN_FLAG_FILE,

            /*
             * File sender может подождать,
             * пока TX queue освободится.
             */
            osWaitForever
        );

    if (queued) {
        ++g_dbg_radio_file_tx_fragments;
    }

    return queued;
}

static bool send_uart_file_frame_over_radio(const uart_file_frame_t *frame)
{
    if (!radio_file_fragment_frame(
            frame,
            g_cfg.max_payload,
            radio_file_emit_fragment,
            NULL)) {

        ++g_dbg_radio_file_tx_failures;

        return false;
    }


    ++g_dbg_radio_file_tx_items;

    return true;
}

static bool handle_file_start(const uart_file_frame_t *frame)
{
    if (frame->payload_length < 48u) {
        ++g_dbg_file_session_errors;
        return false;
    }


    const uint8_t *p =
        frame->payload;


    const uint64_t file_size =
        uart_read_le64(&p[0]);

    const uint16_t chunk_size =
        uart_read_le16(&p[8]);

    const uint32_t total_chunks =
        uart_read_le32(&p[10]);

    const uint16_t filename_length =
        uart_read_le16(&p[46]);


    if (chunk_size == 0u ||
        filename_length >
            frame->payload_length - 48u) {

        ++g_dbg_file_session_errors;
        return false;
    }


    memset(
        &g_uart_file_session,
        0,
        sizeof(g_uart_file_session)
    );


    g_uart_file_session.active = true;

    g_uart_file_session.session_id =
        frame->session_id;

    g_uart_file_session.file_size =
        file_size;

    g_uart_file_session.chunk_size =
        chunk_size;

    g_uart_file_session.total_chunks =
        total_chunks;

    g_uart_file_session.next_sequence =
        0u;

    memcpy(
        g_uart_file_session.sha256,
        &p[14],
        32u
    );


    memset(
        g_dbg_file_name,
        0,
        sizeof(g_dbg_file_name)
    );


    uint16_t copy_name =
        filename_length;

    if (copy_name >=
        sizeof(g_dbg_file_name)) {

        copy_name =
            sizeof(g_dbg_file_name) - 1u;
    }


    memcpy(
        g_dbg_file_name,
        &p[48],
        copy_name
    );


    ++g_dbg_file_start_frames;

    g_dbg_file_session_id =
        frame->session_id;

    g_dbg_file_total_chunks =
        total_chunks;

    g_dbg_file_chunk_size =
        chunk_size;

    g_dbg_file_expected_size_lo =
        (uint32_t)file_size;


    return true;
}

static bool handle_file_data(const uart_file_frame_t *frame)
{
    if (!g_uart_file_session.active ||
        frame->session_id !=
            g_uart_file_session.session_id) {

        ++g_dbg_file_session_errors;
        return false;
    }


    if (frame->sequence !=
        g_uart_file_session.next_sequence) {

        ++g_dbg_file_sequence_errors;

        /*
         * Пока только диагностируем.
         *
         * На следующем этапе здесь появится
         * bitmap / ACK / retransmit.
         */
        g_uart_file_session.next_sequence =
            frame->sequence;
    }


    if (frame->payload_length > g_uart_file_session.chunk_size) {
        ++g_dbg_file_session_errors;
        return false;
    }

    g_uart_file_session.received_bytes += frame->payload_length;
    g_uart_file_session.next_sequence = frame->sequence + 1u;
    ++g_dbg_file_data_frames;
    g_dbg_file_last_sequence = frame->sequence;
    g_dbg_file_received_bytes_lo = (uint32_t)g_uart_file_session.received_bytes;

    /*
     * ПОКА payload никуда не отправляем.
     * Следующий этап:
     * frame->payload
     *      ↓
     * разбить по g_cfg.max_payload
     *      ↓
     * radio transport packets
     */
    return true;
}

static bool handle_file_end(const uart_file_frame_t *frame)
{
    if (!g_uart_file_session.active || frame->session_id != g_uart_file_session.session_id) {
        ++g_dbg_file_session_errors;
        return false;
    }

    if (frame->payload_length != 32u) {
        ++g_dbg_file_session_errors;
        return false;
    }

    if (memcmp(frame->payload, g_uart_file_session.sha256, 32u) != 0) {
        /*
         * Пока сравниваем только SHA из START
         * с SHA из END.
         *
         * Сам файл STM32 пока не хранит,
         * поэтому его SHA здесь не считаем.
         */
        ++g_dbg_file_session_errors;
    }

    if (frame->sequence != g_uart_file_session.total_chunks) {
        ++g_dbg_file_sequence_errors;
    }

    ++g_dbg_file_end_frames;
    /*
     * Для первого теста особенно интересно:
     *
     * expected == received?
     */
    if (g_uart_file_session.received_bytes != g_uart_file_session.file_size) {
        ++g_dbg_file_session_errors;
    }
    g_uart_file_session.active = false;

    /*
     * Следующие байты UART снова могут быть
     * обычным текстом / :stat.
     */
    g_uart_file_mode = false;
    return true;
}

static void consume_legacy_uart(
    const uint8_t *data,
    uint16_t length,
    uint8_t *assembly,
    uint16_t *assembly_length,
    bool *stream_open)
{
    if (length == 0u) {
        return;
    }

    consume_uart_block(
        data,
        length,
        false,
        assembly,
        assembly_length,
        stream_open
    );
}

static void route_uart_input(
    const uint8_t *data,
    uint16_t length,
    uint8_t *assembly,
    uint16_t *assembly_length,
    bool *stream_open)
{
    if (data == NULL ||
        length == 0u) {

        return;
    }


    /*
     * Если FILE_START уже обнаружен,
     * весь поток до FILE_END принадлежит
     * binary parser.
     */
    if (g_uart_file_mode) {

        uart_file_parser_feed(
            &g_uart_file_parser,
            data,
            length
        );

        return;
    }


    uint16_t position = 0u;


    /*
     * Предыдущий DMA block закончился одним A5.
     * Проверяем, является ли следующий байт 5A.
     */
    if (g_uart_file_magic_probe) {

        g_uart_file_magic_probe = false;


        if (data[0] ==
            UART_FILE_MAGIC_1) {

            /*
             * Перед файлом закрываем старый
             * обычный UART message, если он был.
             */
            if (*assembly_length != 0u ||
                *stream_open) {

                consume_uart_block(
                    NULL,
                    0u,
                    true,
                    assembly,
                    assembly_length,
                    stream_open
                );
            }


            const uint8_t magic[2] = {
                UART_FILE_MAGIC_0,
                UART_FILE_MAGIC_1
            };


            g_uart_file_mode = true;


            uart_file_parser_feed(
                &g_uart_file_parser,
                magic,
                2u
            );


            if (length > 1u) {

                uart_file_parser_feed(
                    &g_uart_file_parser,
                    &data[1],
                    length - 1u
                );
            }

            return;
        }


        /*
         * Предыдущий A5 оказался обычными
         * пользовательскими данными.
         */
        const uint8_t a5 =
            UART_FILE_MAGIC_0;

        consume_legacy_uart(
            &a5,
            1u,
            assembly,
            assembly_length,
            stream_open
        );
    }


    /*
     * Ищем A5 5A.
     */
    for (uint16_t i = 0u;
         i < length;
         ++i) {

        if (data[i] !=
            UART_FILE_MAGIC_0) {

            continue;
        }


        /*
         * A5 оказался последним байтом
         * текущего DMA fragment.
         */
        if (i + 1u >= length) {

            if (i > position) {

                consume_legacy_uart(
                    &data[position],
                    i - position,
                    assembly,
                    assembly_length,
                    stream_open
                );
            }


            g_uart_file_magic_probe =
                true;

            return;
        }


        if (data[i + 1u] ==
            UART_FILE_MAGIC_1) {

            /*
             * Всё до magic было обычным UART.
             */
            if (i > position) {

                consume_legacy_uart(
                    &data[position],
                    i - position,
                    assembly,
                    assembly_length,
                    stream_open
                );
            }


            /*
             * Если старое сообщение ещё
             * не было закрыто idle timeout,
             * закрываем его сейчас.
             */
            if (*assembly_length != 0u ||
                *stream_open) {

                consume_uart_block(
                    NULL,
                    0u,
                    true,
                    assembly,
                    assembly_length,
                    stream_open
                );
            }


            g_uart_file_mode =
                true;


            uart_file_parser_feed(
                &g_uart_file_parser,
                &data[i],
                length - i
            );


            return;
        }
    }


    /*
     * Ни одного binary magic не обнаружено.
     */
    consume_legacy_uart(
        &data[position],
        length - position,
        assembly,
        assembly_length,
        stream_open
    );
}
// Новый режим пакетной передачи END

static void start_uart_rx(void)
{
    dma_invalidate(g_uart_rx_dma, sizeof(g_uart_rx_dma));
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_hw.huart, g_uart_rx_dma, MAN_UART_RX_DMA_BYTES) != HAL_OK) {
    	++g_dbg_uart_rx_start_errors;
        ++g_diag.uart_overruns;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
    }
}

static void service_uart_tx(void)
{
    if (g_uart_tx_busy) {
        return;
    }
    uart_output_t output;
    if (osMessageQueueGet(g_uart_out_queue, &output, NULL, 0u) != osOK) {
        return;
    }
    memcpy(g_uart_tx_dma, output.data, output.length);
    dma_clean(g_uart_tx_dma, output.length);
    g_uart_tx_busy = true;
    if (HAL_UART_Transmit_DMA(g_hw.huart, g_uart_tx_dma, output.length) != HAL_OK) {
    	++g_dbg_uart_tx_start_errors;
        g_uart_tx_busy = false;
        ++g_diag.uart_overruns;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
    }
}

static void UartTask(void *argument)
{
    (void)argument;
    uint16_t last_position = 0u;
    uint16_t assembly_length = 0u;
    bool stream_open = false;
    uint32_t last_rx_tick = osKernelGetTickCount();
    start_uart_rx();

    for (;;) {
        uint32_t flags = osThreadFlagsWait(UART_FLAG_RX | UART_FLAG_TX_DONE | UART_FLAG_ERROR,
                                                  osFlagsWaitAny, 10u);

        /*
         * Timeout нужен для проверки программного UART idle.
         * Это не UART error и не набор thread flags.
         */
        if (flags == osFlagsErrorTimeout) {
            flags = 0u;
        } else if ((flags & osFlagsError) != 0u) {
            /*
             * Настоящая ошибка CMSIS-RTOS API.
             * Не интерпретируем код ошибки как битовую маску flags.
             */
            ++g_diag.uart_overruns;
            flags = 0u;
        }

        if ((flags & UART_FLAG_ERROR) != 0u) {
            (void)HAL_UART_AbortReceive(g_hw.huart);
            taskENTER_CRITICAL();
            g_uart_event_head = 0u;
            g_uart_event_tail = 0u;
            taskEXIT_CRITICAL();
            last_position = 0u;
            assembly_length = 0u;
            stream_open = false;
            g_tx_reset_pending = true;
            start_uart_rx();

            g_uart_file_mode = false;
            g_uart_file_magic_probe = false;
            memset(&g_uart_file_session, 0, sizeof(g_uart_file_session));
            uart_file_parser_reset( &g_uart_file_parser);
        }
        uart_isr_event_t event;
        while (uart_event_pop(&event)) {
            uint16_t copied = 0u;
            if (event.position >= last_position) {
                copied = (uint16_t)(event.position - last_position);
                if (copied != 0u) {
                    dma_invalidate(&g_uart_rx_dma[last_position], copied);
                    memcpy(g_uart_temporary, &g_uart_rx_dma[last_position], copied);
                }
            } else {
                const uint16_t first = (uint16_t)(MAN_UART_RX_DMA_BYTES - last_position);
                dma_invalidate(&g_uart_rx_dma[last_position], first);
                memcpy(g_uart_temporary, &g_uart_rx_dma[last_position], first);
                dma_invalidate(&g_uart_rx_dma[0], event.position);
                memcpy(&g_uart_temporary[first], &g_uart_rx_dma[0], event.position);
                copied = (uint16_t)(first + event.position);
            }
            last_position = event.position % MAN_UART_RX_DMA_BYTES;
            dbg_toggle(g_hw.dbg_uart_port, g_hw.dbg_uart_pin);

            const bool hardware_idle = event.type == UART_EVENT_IDLE;
            if (!g_uart_file_mode && hardware_idle && assembly_length == 0u && !g_uart_file_magic_probe && is_stats_command(g_uart_temporary, copied)) {
                queue_stats();
            } else {
                /*
                 * Hardware IDLE не завершает логический блок.
                 * Он может возникать между USB-пакетами одного port.write().
                 */
                //consume_uart_block(g_uart_temporary, copied, false, g_uart_assembly, &assembly_length, &stream_open);
                route_uart_input(g_uart_temporary, copied, g_uart_assembly, &assembly_length, &stream_open);
            }
            if (copied != 0u) {
                last_rx_tick = osKernelGetTickCount();
            }
            if (g_cfg.uart_explicit_block_length != 0u &&
                assembly_length >= g_cfg.uart_explicit_block_length) {
                consume_uart_block(NULL, 0u, true, g_uart_assembly, &assembly_length, &stream_open);
            }
        }
        const bool block_pending = !g_uart_file_mode && (assembly_length != 0u || stream_open);
        if (block_pending && g_cfg.uart_idle_flush_ms != 0u && (osKernelGetTickCount() - last_rx_tick) >= g_cfg.uart_idle_flush_ms) {
            consume_uart_block(NULL, 0u, true, g_uart_assembly, &assembly_length, &stream_open);
        }

        service_uart_tx();
    }
}

static void ApplicationTask(void *argument)
{
    (void)argument;
    man_packet_t packet;
    for (;;) {
        if (osMessageQueueGet(g_rx_queue, &packet, NULL, osWaitForever) != osOK) {
            continue;
        }
        ++g_dbg_app_rx_total;
        if ((packet.flags & MAN_FLAG_RESET) != 0u) {
            g_expected_rx_sequence_valid = false;
        }
        if (g_expected_rx_sequence_valid && packet.seq != g_expected_rx_sequence) {
            ++g_diag.sequence_gaps;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        }
        g_expected_rx_sequence = (uint8_t)(packet.seq + 1u);
        g_expected_rx_sequence_valid = true;

        // файловая загрузка через протокол
        if ((packet.flags & MAN_FLAG_FILE) != 0u) {
        	++g_dbg_app_rx_file;
            ++g_dbg_radio_file_rx_fragments;
            if (!radio_file_reassembler_feed(&g_radio_file_reassembler, packet.payload, packet.length)) {
                ++g_dbg_radio_file_rx_fragment_errors;
            }
            /*
             * Binary file transport пока
             * НЕ выводим как сырой мусор в UART.
             */
            continue;
        }
        ++g_dbg_app_rx_normal;
        queue_uart_output(packet.payload, packet.length);
        led_set(g_hw.led_ok_port, g_hw.led_ok_pin, true);
        continue;
    }
}

static void DiagnosticsTask(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(500u);
        HAL_GPIO_TogglePin(g_hw.led_ok_port, g_hw.led_ok_pin);

        if (g_diag.crc_errors == 0u && g_diag.dma_overruns == 0u &&
            g_diag.queue_overflows == 0u && g_diag.uart_overruns == 0u) {
            if (g_hw.led_ok_port != NULL) {
                //HAL_GPIO_TogglePin(g_hw.led_ok_port, g_hw.led_ok_pin);
            }
        } else {
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        }
    }
}

void Manchester_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t position)
{
    if (huart != g_hw.huart || g_uart_task == NULL) {
        return;
    }
    uint8_t type = UART_EVENT_IDLE;
    const HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart);
    if (event_type == HAL_UART_RXEVENT_HT) type = UART_EVENT_HT;
    else if (event_type == HAL_UART_RXEVENT_TC) type = UART_EVENT_TC;
    const uint8_t next = (uint8_t)((g_uart_event_head + 1u) % MAN_UART_ISR_EVENT_COUNT);
    if (next == g_uart_event_tail) {
    	++g_dbg_uart_event_queue_overflows;
        ++g_diag.uart_overruns;
        ++g_diag.dropped_blocks;
        return;
    }
    g_uart_events[g_uart_event_head].position = position % MAN_UART_RX_DMA_BYTES;
    g_uart_events[g_uart_event_head].type = type;
    g_uart_event_head = next;
    (void)osThreadFlagsSet(g_uart_task, UART_FLAG_RX);
}

void Manchester_OnUartTxComplete(UART_HandleTypeDef *huart)
{
    if (huart == g_hw.huart && g_uart_task != NULL) {
        g_uart_tx_busy = false;
        (void)osThreadFlagsSet(g_uart_task, UART_FLAG_TX_DONE);
    }
}

void Manchester_OnUartError(UART_HandleTypeDef *huart)
{
	++g_dbg_uart_hal_errors;
    if (huart == g_hw.huart && g_uart_task != NULL) {
        ++g_diag.uart_overruns;
        (void)osThreadFlagsSet(g_uart_task, UART_FLAG_ERROR);
    }
}

const man_diagnostics_t *Manchester_GetDiagnostics(void)
{
    return &g_diag;
}

uint32_t Manchester_GetRxTimerClockHz(void)
{
    return g_rx_sample_rate_hz;
}
