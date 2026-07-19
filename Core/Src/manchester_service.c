#include "manchester_service.h"
#include "manchester_config.h"
#include "manchester_fec.h"
#include "manchester_frame.h"
#include "manchester_line.h"
#include "manchester_rx_dsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define RX_FLAG_HALF       (1u << 0)
#define RX_FLAG_FULL       (1u << 1)
#define RX_FLAG_ERROR      (1u << 2)
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
static const man_fec_codec_t *g_fec;
static man_rx_decoder_t g_decoder;

static MAN_DMA_BUFFER uint8_t g_spi_rx_dma[MAN_SPI_DMA_BYTES];
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

static volatile uint32_t g_spi_pending;
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

volatile uint32_t g_rx_cycles_last;
volatile uint32_t g_rx_cycles_max;
volatile uint32_t g_rx_blocks_processed;

volatile uint16_t g_dbg_pattern_length_0;
volatile uint16_t g_dbg_pattern_length_1;
volatile uint32_t g_dbg_rx_block_number;
volatile uint32_t g_dbg_pattern_corrupted_at;

volatile uint32_t g_dbg_pclk2_hz;
volatile uint32_t g_dbg_timer_clock_hz;
volatile uint32_t g_dbg_spi_divisor;
volatile uint32_t g_dbg_timer_ticks;

static volatile bool g_rx_muted = false;
static volatile bool g_rx_reset_requested = false;

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
    if (!is_rf_halfduplex()) {
        return;
    }

    g_rx_muted = true;
    g_rx_reset_requested = true;

    HAL_GPIO_WritePin(g_hw.rf_recv_port, g_hw.rf_recv_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_hw.rf_trans_port, g_hw.rf_trans_pin, GPIO_PIN_SET);

    if (g_hw.rf_tx_settle_ms != 0u) {
        osDelay(g_hw.rf_tx_settle_ms);
    }
}

static void rf_enter_rx_mode(void)
{
    if (!is_rf_halfduplex()) {
        return;
    }

    HAL_GPIO_WritePin(g_hw.rf_trans_port, g_hw.rf_trans_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_hw.rf_recv_port, g_hw.rf_recv_pin, GPIO_PIN_SET);

    if (g_hw.rf_rx_settle_ms != 0u) {
        osDelay(g_hw.rf_rx_settle_ms);
    }

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

static uint32_t spi_prescaler_constant(uint32_t divisor)
{
    switch (divisor) {
    case 2u: return SPI_BAUDRATEPRESCALER_2;
    case 4u: return SPI_BAUDRATEPRESCALER_4;
    case 8u: return SPI_BAUDRATEPRESCALER_8;
    case 16u: return SPI_BAUDRATEPRESCALER_16;
    case 32u: return SPI_BAUDRATEPRESCALER_32;
    case 64u: return SPI_BAUDRATEPRESCALER_64;
    case 128u: return SPI_BAUDRATEPRESCALER_128;
    default: return SPI_BAUDRATEPRESCALER_256;
    }
}

static bool configure_spi_slave(void)
{
    SPI_HandleTypeDef *hspi = g_hw.hspi_rx;

    if (hspi == NULL) {
        return false;
    }

    (void)HAL_SPI_DeInit(hspi);

    hspi->Init.Mode = SPI_MODE_SLAVE;
    hspi->Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;

    /* TIM8 SCLK должен иметь idle LOW. */
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase = SPI_PHASE_1EDGE;

    /*
     * NSS должен быть физически притянут к LOW,
     * пока приёмник должен принимать данные.
     */
    hspi->Init.NSS = SPI_NSS_HARD_INPUT;

    /*
     * В slave mode prescaler игнорируется,
     * но HAL требует корректное значение структуры.
     */
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;

    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 7u;

    return HAL_SPI_Init(hspi) == HAL_OK;
}


static bool calculate_rx_clock_ticks(
    uint32_t timer_clock_hz,
    uint32_t bitrate_bps,
    uint32_t *ticks_out,
    uint32_t *sample_rate_out)
{
    if (ticks_out == NULL ||
        sample_rate_out == NULL ||
        bitrate_bps == 0u) {
        return false;
    }

    const uint64_t chip_rate_hz =
        (uint64_t)bitrate_bps * 2u;

    const uint64_t minimum_sample_rate_hz =
        chip_rate_hz * MAN_RX_MIN_SAMPLES_PER_CHIP;

    if (minimum_sample_rate_hz >
        MAN_RX_SAMPLE_CLOCK_MAX_HZ) {
        return false;
    }

    /*
     * Максимальный divider даёт минимальную частоту,
     * которая всё ещё обеспечивает необходимый oversampling.
     */
    uint32_t max_ticks =
        (uint32_t)(timer_clock_hz /
                   minimum_sample_rate_hz);

    uint32_t min_ticks =
        (timer_clock_hz +
         MAN_RX_SAMPLE_CLOCK_MAX_HZ - 1u) /
        MAN_RX_SAMPLE_CLOCK_MAX_HZ;

    if (min_ticks == 0u) {
        min_ticks = 1u;
    }

    /*
     * TIM8 является 16-битным таймером.
     */
    if (max_ticks > 65536u) {
        max_ticks = 65536u;
    }

    if (max_ticks < min_ticks) {
        return false;
    }

    /*
     * Ищем наибольший чётный divider:
     *
     * - минимальная нагрузка DMA/CPU;
     * - точная частота;
     * - точный duty cycle 50%.
     */
//    for (uint32_t ticks = max_ticks;
//         ticks >= min_ticks;
//         --ticks) {
//
//        if ((ticks & 1u) != 0u) {
//            continue;
//        }
//
//        if ((timer_clock_hz % ticks) != 0u) {
//            continue;
//        }
//
//        const uint32_t actual_rate =
//            timer_clock_hz / ticks;
//
//        if (actual_rate <
//                minimum_sample_rate_hz ||
//            actual_rate >
//                MAN_RX_SAMPLE_CLOCK_MAX_HZ) {
//            continue;
//        }
//
//        *ticks_out = ticks;
//        *sample_rate_out = actual_rate;
//        return true;
//    }



    uint32_t ticks = max_ticks;

    for (;;) {
        if ((ticks & 1u) == 0u &&
            (timer_clock_hz % ticks) == 0u) {

            const uint32_t actual_rate =
                timer_clock_hz / ticks;

            if (actual_rate >= minimum_sample_rate_hz &&
                actual_rate <= MAN_RX_SAMPLE_CLOCK_MAX_HZ) {

                *ticks_out = ticks;
                *sample_rate_out = actual_rate;
                return true;
            }
        }

        if (ticks == min_ticks) {
            break;
        }

        --ticks;
    }

    return false;
}


static bool configure_rx_sample_clock(void)
{
    TIM_HandleTypeDef *htim = g_hw.htim_rx_clk;

    if (htim == NULL) {
        return false;
    }

    const uint32_t timer_clock_hz =
        apb2_timer_clock_hz();

    uint32_t ticks = 0u;
    uint32_t actual_sample_rate_hz = 0u;

    if (!calculate_rx_clock_ticks(
            timer_clock_hz,
            g_cfg.bitrate_bps,
            &ticks,
            &actual_sample_rate_hz)) {
        return false;
    }

    /*
     * Таймер пока только настраивается.
     * Запустим его после запуска SPI RX DMA.
     */
    __HAL_TIM_DISABLE(htim);

    __HAL_TIM_SET_PRESCALER(htim, 0u);
    __HAL_TIM_SET_AUTORELOAD(htim, ticks - 1u);

    /*
     * PWM 50%.
     */
    __HAL_TIM_SET_COMPARE(
        htim,
        g_hw.tim_rx_clk_channel,
        ticks / 2u
    );

    __HAL_TIM_SET_COUNTER(htim, 0u);

    /*
     * Немедленно загрузить PSC/ARR/CCR.
     */
    HAL_TIM_GenerateEvent(
        htim,
        TIM_EVENTSOURCE_UPDATE
    );

    __HAL_TIM_CLEAR_FLAG(
        htim,
        TIM_FLAG_UPDATE
    );

    /*
     * Это теперь фактическая частота внешнего sampling clock.
     * Декодер должен использовать именно её.
     */
    g_rx_sample_rate_hz = actual_sample_rate_hz;

    return true;
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

bool Manchester_ServiceInit(const man_platform_t *platform, const man_runtime_config_t *config)
{
    if (platform == NULL || config == NULL || platform->hspi_rx == NULL || platform->htim_tx == NULL ||
        platform->huart == NULL || platform->tx_port == NULL || platform->tx_pin == 0u ||
        !man_bitrate_is_valid(config->bitrate_bps) || config->max_payload == 0u ||
        config->max_payload > MAN_MAX_PAYLOAD || config->preamble_bytes < MAN_PREAMBLE_BYTES_MIN ||
        config->preamble_bytes > MAN_PREAMBLE_BYTES_MAX ||
        config->glitch_filter_samples > MAN_GLITCH_FILTER_MAX_SAMPLES) {
        return false;
    }
    memset(&g_diag, 0, sizeof(g_diag));
    g_hw = *platform;
    g_cfg = *config;
    g_fec = config->fec_enabled ? man_fec_hamming74_codec() : man_fec_identity_codec();

    // Инициализация приемника СШП модема - изначально слушаем
    HAL_GPIO_WritePin(g_hw.rf_trans_port, g_hw.rf_trans_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(g_hw.rf_recv_port, g_hw.rf_recv_pin, GPIO_PIN_SET);
	g_rx_muted = false;

    if (g_fec == NULL) {
        return false; /* Hamming is intentionally only an extension point in this revision. */
    }
    led_set(g_hw.led_ok_port, g_hw.led_ok_pin, false);
    led_set(g_hw.led_tx_port, g_hw.led_tx_pin, false);
    led_set(g_hw.led_error_port, g_hw.led_error_pin, false);
    tx_set_idle_level();

    if (!configure_spi_slave() || !configure_rx_sample_clock() || !configure_tim_rate()) {
        return false;
    }
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
    g_uart_out_queue = osMessageQueueNew(MAN_UART_OUT_QUEUE_DEPTH, sizeof(uart_output_t), NULL);
    if (g_tx_queue == NULL || g_rx_queue == NULL || g_uart_out_queue == NULL) {
        return false;
    }

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
    bool ok = g_uart_task != NULL && g_app_task != NULL && g_diag_task != NULL;
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

static uint32_t take_spi_pending(void)
{
    taskENTER_CRITICAL();
    const uint32_t value = g_spi_pending;
    g_spi_pending = 0u;
    taskEXIT_CRITICAL();
    return value;
}

static void process_rx_half(const uint8_t *data)
{
    const uint32_t started = DWT->CYCCNT;

    if (g_rx_reset_requested) {
        man_rx_decoder_reset(&g_decoder);
        g_rx_reset_requested = false;
    }

    if (!g_rx_muted) {
        man_rx_decoder_feed_packed(
            &g_decoder,
            data,
            MAN_SPI_DMA_HALF_BYTES
        );
    }

    const uint32_t elapsed = DWT->CYCCNT - started;

    g_rx_cycles_last = elapsed;

    if (elapsed > g_rx_cycles_max) {
        g_rx_cycles_max = elapsed;
    }

    ++g_rx_blocks_processed;
}

static void check_decoder_integrity(void)
{
    ++g_dbg_rx_block_number;

    g_dbg_pattern_length_0 =
        g_decoder.paths[0].pattern_length;

    g_dbg_pattern_length_1 =
        g_decoder.paths[1].pattern_length;

    if (g_dbg_pattern_length_0 == 0u ||
        g_dbg_pattern_length_1 == 0u) {

        if (g_dbg_pattern_corrupted_at == 0u) {
            g_dbg_pattern_corrupted_at =
                g_dbg_rx_block_number;
        }

        __BKPT(0);
    }
}

static uint8_t g_dbg_raw_previous_level;
static bool g_dbg_raw_previous_valid;
static uint32_t g_dbg_raw_sample_index;
static uint32_t g_dbg_raw_previous_edge;
volatile uint32_t g_dbg_raw_edge_histogram[32];

static void debug_scan_raw_msb(
    const uint8_t *samples,
    size_t byte_count)
{
    for (size_t i = 0u; i < byte_count; ++i) {
        const uint8_t raw = samples[i];

        for (uint8_t bit_index = 0u;
             bit_index < 8u;
             ++bit_index) {

            const uint8_t level =
                (uint8_t)((raw >> (7u - bit_index)) & 1u);

            if (!g_dbg_raw_previous_valid) {
                g_dbg_raw_previous_valid = true;
                g_dbg_raw_previous_level = level;
            } else if (level != g_dbg_raw_previous_level) {
                const uint32_t delta =
                    g_dbg_raw_sample_index -
                    g_dbg_raw_previous_edge;

                if (g_dbg_raw_previous_edge != 0u &&
                    delta < 32u) {
                    ++g_dbg_raw_edge_histogram[delta];
                }

                g_dbg_raw_previous_edge =
                    g_dbg_raw_sample_index;

                g_dbg_raw_previous_level = level;
            }

            ++g_dbg_raw_sample_index;
        }
    }
}

static bool start_rx_sampling(void)
{
    dma_invalidate(
        g_spi_rx_dma,
        sizeof(g_spi_rx_dma)
    );

    /*
     * Сначала SPI slave и DMA должны быть готовы
     * принимать самый первый внешний SCLK.
     */
    if (HAL_SPI_Receive_DMA(
            g_hw.hspi_rx,
            g_spi_rx_dma,
            MAN_SPI_DMA_BYTES) != HAL_OK) {
        return false;
    }

    /*
     * Только теперь запускаем внешний clock.
     */
    if (HAL_TIM_PWM_Start(
            g_hw.htim_rx_clk,
            g_hw.tim_rx_clk_channel) != HAL_OK) {

        (void)HAL_SPI_Abort(
            g_hw.hspi_rx
        );

        return false;
    }

    return true;
}

static void stop_rx_sampling(void)
{
    /*
     * Сначала прекращаем внешний SCLK.
     */
    (void)HAL_TIM_PWM_Stop(
        g_hw.htim_rx_clk,
        g_hw.tim_rx_clk_channel
    );

    /*
     * Потом останавливаем SPI/DMA.
     */
    (void)HAL_SPI_Abort(
        g_hw.hspi_rx
    );
}

static void ManchesterRxTask(void *argument)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    (void)argument;
    g_decoder.callback = rx_frame_to_queue;
    g_decoder.callback_user = NULL;

    if (!start_rx_sampling()) {
    	++g_diag.dma_overruns;
    	led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
    }

    for (;;) {
        (void)osThreadFlagsWait(RX_FLAG_HALF | RX_FLAG_FULL | RX_FLAG_ERROR, osFlagsWaitAny, osWaitForever);
        const uint32_t pending = take_spi_pending();

        if ((pending & RX_FLAG_ERROR) != 0u) {
            stop_rx_sampling();
            man_rx_decoder_reset(&g_decoder);
            if (!start_rx_sampling()) {
				++g_diag.dma_overruns;
				led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
			}
        }
        if ((pending & RX_FLAG_HALF) != 0u) {
        	check_decoder_integrity();
            dma_invalidate(&g_spi_rx_dma[0], MAN_SPI_DMA_HALF_BYTES);
            dbg_toggle(g_hw.dbg_rx_port, g_hw.dbg_rx_pin);
//            debug_scan_raw_msb(
//                &g_spi_rx_dma[0],
//                MAN_SPI_DMA_HALF_BYTES);

            process_rx_half(&g_spi_rx_dma[0]);
//            man_rx_decoder_feed_packed(&g_decoder, &g_spi_rx_dma[0], MAN_SPI_DMA_HALF_BYTES);
        }
        if ((pending & RX_FLAG_FULL) != 0u) {
        	check_decoder_integrity();
            dma_invalidate(&g_spi_rx_dma[MAN_SPI_DMA_HALF_BYTES], MAN_SPI_DMA_HALF_BYTES);
            dbg_toggle(g_hw.dbg_rx_port, g_hw.dbg_rx_pin);
//            debug_scan_raw_msb(
//                &g_spi_rx_dma[0],
//                MAN_SPI_DMA_HALF_BYTES);
            process_rx_half(&g_spi_rx_dma[MAN_SPI_DMA_HALF_BYTES]);
//            man_rx_decoder_feed_packed(&g_decoder, &g_spi_rx_dma[MAN_SPI_DMA_HALF_BYTES], MAN_SPI_DMA_HALF_BYTES);
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
    if (!man_frame_build_wire_bits(&g_cfg, packet, g_fec, g_tx_fec_ctx,
                                   g_tx_wire_bits, MAN_TX_MAX_WIRE_BITS, &wire_bit_count)) {
        return false;
    }
    Manchester_TestHookMutateWireBits(g_tx_wire_bits, wire_bit_count);
    if (!man_line_encode_bsrr(g_tx_wire_bits, wire_bit_count, g_cfg.encoding, g_cfg.tx_invert,
    		g_cfg.tx_invert ? 1u : 0u, g_hw.tx_pin, g_tx_bsrr, MAN_TX_MAX_CHIPS, &chip_count)) {
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

        // Если хотим transceiver для отладки по одному проводу, то включаем MAN_PHY_WIRED_LOOPBACK
        if (is_rf_halfduplex()) {
            g_rx_reset_requested = true;
            rf_enter_tx_mode();
        }

        /*
         * Защита от старого флага TX_DONE/TX_ERROR.
         */
        osThreadFlagsClear(TX_FLAG_DONE | TX_FLAG_ERROR);

        if (!start_tx_frame(&packet)) {
        	// Если хотим transceiver для отладки по одному проводу, то включаем MAN_PHY_WIRED_LOOPBACK
            if (is_rf_halfduplex()) {
                rf_enter_rx_mode();
                g_rx_reset_requested = true;
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
        if (is_rf_halfduplex() &&
            (((packet.flags & MAN_FLAG_END) != 0u) ||
             ((result & TX_FLAG_ERROR) != 0u))) {

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

static void queue_uart_output(const uint8_t *data, uint16_t length)
{
    while (length != 0u) {
        uart_output_t output;
        output.length = length > MAN_UART_TX_STAGE_BYTES ? MAN_UART_TX_STAGE_BYTES : length;
        memcpy(output.data, data, output.length);
        if (osMessageQueuePut(g_uart_out_queue, &output, 0u, 0u) != osOK) {
            ++g_diag.queue_overflows;
            ++g_diag.dropped_blocks;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
            return;
        }
        data += output.length;
        length = (uint16_t)(length - output.length);
    }
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

static void queue_tx_packet(const uint8_t *data, uint16_t length, bool stream, bool end)
{
#if MAN_COMPILETIME_ROLE != MAN_ROLE_RX_ONLY
    man_packet_t packet;
    packet.flags = (uint8_t)((stream ? MAN_FLAG_STREAM : 0u) | (end ? MAN_FLAG_END : 0u) |
                             (g_cfg.fec_enabled ? MAN_FLAG_FEC : 0u) |
                             (g_tx_reset_pending ? MAN_FLAG_RESET : 0u));
    g_tx_reset_pending = false;
    packet.seq = g_tx_sequence++;
    packet.length = length;
    if (length != 0u) {
        memcpy(packet.payload, data, length);
    }
    if (osMessageQueuePut(g_tx_queue, &packet, 0u, 0u) != osOK) {
        ++g_diag.queue_overflows;
        ++g_diag.dropped_blocks;
        g_tx_reset_pending = true;
        led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
    }
#else
    (void)data; (void)length; (void)stream; (void)end;
#endif
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

static void start_uart_rx(void)
{
    dma_invalidate(g_uart_rx_dma, sizeof(g_uart_rx_dma));
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_hw.huart, g_uart_rx_dma, MAN_UART_RX_DMA_BYTES) != HAL_OK) {
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
            if (hardware_idle && assembly_length == 0u && is_stats_command(g_uart_temporary, copied)) {
                queue_stats();
            } else {
                /*
                 * Hardware IDLE не завершает логический блок.
                 * Он может возникать между USB-пакетами одного port.write().
                 */
                consume_uart_block(g_uart_temporary, copied, false, g_uart_assembly, &assembly_length, &stream_open);
            }
            if (copied != 0u) {
                last_rx_tick = osKernelGetTickCount();
            }
            if (g_cfg.uart_explicit_block_length != 0u &&
                assembly_length >= g_cfg.uart_explicit_block_length) {
                consume_uart_block(NULL, 0u, true, g_uart_assembly, &assembly_length, &stream_open);
            }
        }
        const bool block_pending = assembly_length != 0u || stream_open;
        if (block_pending &&
            g_cfg.uart_idle_flush_ms != 0u &&
            (osKernelGetTickCount() - last_rx_tick) >= g_cfg.uart_idle_flush_ms) {
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
        if ((packet.flags & MAN_FLAG_RESET) != 0u) {
            g_expected_rx_sequence_valid = false;
        }
        if (g_expected_rx_sequence_valid && packet.seq != g_expected_rx_sequence) {
            ++g_diag.sequence_gaps;
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        }
        g_expected_rx_sequence = (uint8_t)(packet.seq + 1u);
        g_expected_rx_sequence_valid = true;
        queue_uart_output(packet.payload, packet.length);
        led_set(g_hw.led_ok_port, g_hw.led_ok_pin, true);
    }
}

static void DiagnosticsTask(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(500u);
        if (g_diag.crc_errors == 0u && g_diag.dma_overruns == 0u &&
            g_diag.queue_overflows == 0u && g_diag.uart_overruns == 0u) {
            if (g_hw.led_ok_port != NULL) {
                HAL_GPIO_TogglePin(g_hw.led_ok_port, g_hw.led_ok_pin);
            }
        } else {
            led_set(g_hw.led_error_port, g_hw.led_error_pin, true);
        }
    }
}

void Manchester_OnSpiTxRxHalfComplete(SPI_HandleTypeDef *hspi)
{
    if (hspi != g_hw.hspi_rx || g_rx_task == NULL) {
        return;
    }
    if ((g_spi_pending & RX_FLAG_HALF) != 0u) {
        ++g_diag.dma_overruns;
    }
    g_spi_pending |= RX_FLAG_HALF;
    (void)osThreadFlagsSet(g_rx_task, RX_FLAG_HALF);
}

void Manchester_OnSpiTxRxComplete(SPI_HandleTypeDef *hspi)
{
    if (hspi != g_hw.hspi_rx || g_rx_task == NULL) {
        return;
    }
    if ((g_spi_pending & RX_FLAG_FULL) != 0u) {
        ++g_diag.dma_overruns;
    }
    g_spi_pending |= RX_FLAG_FULL;
    (void)osThreadFlagsSet(g_rx_task, RX_FLAG_FULL);
}

void Manchester_OnSpiError(SPI_HandleTypeDef *hspi)
{
    if (hspi == g_hw.hspi_rx && g_rx_task != NULL) {
        g_spi_pending |= RX_FLAG_ERROR;
        ++g_diag.dma_overruns;
        (void)osThreadFlagsSet(g_rx_task, RX_FLAG_ERROR);
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
    if (huart == g_hw.huart && g_uart_task != NULL) {
        ++g_diag.uart_overruns;
        (void)osThreadFlagsSet(g_uart_task, UART_FLAG_ERROR);
    }
}

const man_diagnostics_t *Manchester_GetDiagnostics(void)
{
    return &g_diag;
}

uint32_t Manchester_GetSpiSampleRateHz(void)
{
    return g_rx_sample_rate_hz;
}
