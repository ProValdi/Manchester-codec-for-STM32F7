#include "manchester_tim_ic_capture.h"

#include <limits.h>


#if (MAN_TIM_IC_DMA_EVENTS % 2u) != 0u
#error "MAN_TIM_IC_DMA_EVENTS must be even"
#endif


__weak void Manchester_TimIcCaptureWakeFromIsr(void)
{
}

static TIM_HandleTypeDef *g_tim_ic_htim;
static uint32_t g_tim_ic_channel;

/*
 * Сколько DMA half-boundaries прошло:
 *
 * 0 -> DMA находится в первой половине
 * 1 -> во второй
 * 2 -> снова в первой
 * ...
 */
static volatile uint32_t g_tim_ic_completed_halves;

/*
 * Абсолютное modulo-2^32 количество уже обработанных timestamps.
 */
static uint32_t g_tim_ic_consumed_total;

MAN_DMA_BUFFER uint16_t g_tim_ic_dma[MAN_TIM_IC_DMA_EVENTS];

volatile uint32_t g_dbg_ic_half_callbacks;
volatile uint32_t g_dbg_ic_full_callbacks;
volatile uint32_t g_dbg_ic_error_callbacks;

volatile uint32_t g_dbg_ic_dma_completed_halves;

volatile uint32_t g_dbg_ic_events_total;
volatile uint32_t g_dbg_ic_tail_drains;
volatile uint32_t g_dbg_ic_overruns;
volatile uint32_t g_dbg_ic_pending;
volatile uint32_t g_dbg_ic_write_index;

volatile uint32_t g_dbg_ic_recent_half;
volatile uint32_t g_dbg_ic_recent_min_delta;
volatile uint32_t g_dbg_ic_recent_max_delta;
volatile uint32_t g_dbg_ic_last_timestamp;

volatile uint16_t g_dbg_ic_recent_edges[MAN_TIM_IC_DEBUG_SAMPLES];

volatile uint16_t g_dbg_ic_recent_deltas[MAN_TIM_IC_DEBUG_SAMPLES - 1u];

static DMA_HandleTypeDef *capture_dma_handle(void)
{
    if (g_tim_ic_htim == NULL) {
        return NULL;
    }

    switch (g_tim_ic_channel) {

    case TIM_CHANNEL_1:
        return g_tim_ic_htim->hdma[TIM_DMA_ID_CC1];

    case TIM_CHANNEL_2:
        return g_tim_ic_htim->hdma[TIM_DMA_ID_CC2];

    case TIM_CHANNEL_3:
        return g_tim_ic_htim->hdma[TIM_DMA_ID_CC3];

    case TIM_CHANNEL_4:
        return g_tim_ic_htim->hdma[TIM_DMA_ID_CC4];

    default:
        return NULL;
    }
}

static void reset_debug_state(void)
{
    g_dbg_ic_half_callbacks = 0u;
    g_dbg_ic_full_callbacks = 0u;
    g_dbg_ic_error_callbacks = 0u;

    g_dbg_ic_dma_completed_halves = 0u;

    g_dbg_ic_events_total = 0u;
    g_dbg_ic_tail_drains = 0u;
    g_dbg_ic_overruns = 0u;
    g_dbg_ic_pending = 0u;
    g_dbg_ic_write_index = 0u;

    g_dbg_ic_recent_half = 0u;
    g_dbg_ic_recent_min_delta = 0u;
    g_dbg_ic_recent_max_delta = 0u;
    g_dbg_ic_last_timestamp = 0u;

    for (uint32_t i = 0u; i < MAN_TIM_IC_DEBUG_SAMPLES; ++i) {
        g_dbg_ic_recent_edges[i] = 0u;
    }

    for (uint32_t i = 0u; i < MAN_TIM_IC_DEBUG_SAMPLES - 1u; ++i) {
        g_dbg_ic_recent_deltas[i] = 0u;
    }
}

static void snapshot_span(const uint16_t *data, uint32_t count, uint32_t half_index)
{
    if (data == NULL || count == 0u) {
        return;
    }

    uint32_t n = count;

    if (n > MAN_TIM_IC_DEBUG_SAMPLES) {
        n = MAN_TIM_IC_DEBUG_SAMPLES;
    }

    for (uint32_t i = 0u; i < MAN_TIM_IC_DEBUG_SAMPLES; ++i) {
        g_dbg_ic_recent_edges[i] = 0u;
    }

    for (uint32_t i = 0u; i < MAN_TIM_IC_DEBUG_SAMPLES - 1u; ++i) {
        g_dbg_ic_recent_deltas[i] = 0u;
    }

    g_dbg_ic_recent_edges[0] = data[0];
    uint16_t min_delta = UINT16_MAX;
    uint16_t max_delta = 0u;

    for (uint32_t i = 1u; i < n; ++i) {
        const uint16_t previous = data[i - 1u];
        const uint16_t current = data[i];

        /*
         * TIM8 16 bit.
         *
         * uint16_t subtraction автоматически
         * корректно переживает wrap 65535 -> 0.
         */
        const uint16_t delta = (uint16_t)(current - previous);

        g_dbg_ic_recent_edges[i] = current;
        g_dbg_ic_recent_deltas[i - 1u] = delta;

        if (delta < min_delta) {
            min_delta = delta;
        }

        if (delta > max_delta) {
            max_delta = delta;
        }
    }

    g_dbg_ic_recent_half = half_index;

    if (n >= 2u) {
        g_dbg_ic_recent_min_delta = min_delta;
        g_dbg_ic_recent_max_delta = max_delta;
    } else {
        g_dbg_ic_recent_min_delta = 0u;
        g_dbg_ic_recent_max_delta = 0u;
    }

    g_dbg_ic_last_timestamp = data[count - 1u];
}


/*
 * Возвращает абсолютное количество timestamps,
 * которые DMA уже произвёл.
 *
 * Используем:
 *
 * completed halves
 * +
 * текущий NDTR
 *
 * поэтому видим и незаполненный DMA tail.
 */
static uint32_t produced_total(void)
{
    DMA_HandleTypeDef *hdma = capture_dma_handle();

    if (hdma == NULL) {
        return g_tim_ic_consumed_total;
    }

    for (;;) {
        const uint32_t halves_before = g_tim_ic_completed_halves;
        __DMB();
        uint32_t remaining = __HAL_DMA_GET_COUNTER(hdma);
        __DMB();

        const uint32_t halves_after = g_tim_ic_completed_halves;

        /*
         * Пока читали NDTR DMA успел перейти
         * через HT/TC boundary.
         *
         * Просто читаем ещё раз.
         */
        if (halves_before != halves_after) {
            continue;
        }

        if (remaining > MAN_TIM_IC_DMA_EVENTS) {
            remaining = MAN_TIM_IC_DMA_EVENTS;
        }

        const uint32_t write_index = MAN_TIM_IC_DMA_EVENTS - remaining;

        g_dbg_ic_write_index = write_index;

        uint32_t inside_half;

        if ((halves_before & 1u) == 0u) {
            /*
             * DMA сейчас пишет первую половину:
             * [0 .. HALF)
             */
            if (write_index >
                MAN_TIM_IC_DMA_HALF_EVENTS) {

                continue;
            }
            inside_half = write_index;
        } else {
            /*
             * DMA сейчас пишет вторую половину:
             * [HALF .. SIZE)
             */
            if (write_index < MAN_TIM_IC_DMA_HALF_EVENTS) {
                continue;
            }
            inside_half = write_index - MAN_TIM_IC_DMA_HALF_EVENTS;
        }

        return halves_before * MAN_TIM_IC_DMA_HALF_EVENTS + inside_half;
    }
}

uint32_t Manchester_TimIcCapturePending(void)
{
    if (g_tim_ic_htim == NULL) {
        return 0u;
    }
    const uint32_t produced = produced_total();
    uint32_t pending = produced - g_tim_ic_consumed_total;
    if (pending > MAN_TIM_IC_DMA_EVENTS) {
        pending = MAN_TIM_IC_DMA_EVENTS;
    }
    g_dbg_ic_pending = pending;
    return pending;
}

uint32_t Manchester_TimIcCaptureDrain(ManchesterTimIcSpanConsumer consumer, void *context)
{
    if (g_tim_ic_htim == NULL) {
        return 0u;
    }
    const uint32_t produced = produced_total();
    uint32_t pending = produced - g_tim_ic_consumed_total;

    /*
     * DMA успел сделать полный круг и
     * перезаписать необработанные данные.
     */
    if (pending > MAN_TIM_IC_DMA_EVENTS) {
        ++g_dbg_ic_overruns;
        g_tim_ic_consumed_total = produced - MAN_TIM_IC_DMA_EVENTS;
        pending = MAN_TIM_IC_DMA_EVENTS;
    }

    if (pending == 0u) {
        g_dbg_ic_pending = 0u;
        return 0u;
    }
    const uint32_t original_pending = pending;

    /*
     * Это не обязательно packet tail.
     * Это означает: Drain произошёл до
     * заполнения полной DMA half.
     */
    if (pending < MAN_TIM_IC_DMA_HALF_EVENTS) {
        ++g_dbg_ic_tail_drains;
    }
    bool snapshot_done = false;

    while (pending != 0u) {
        const uint32_t read_index = g_tim_ic_consumed_total % MAN_TIM_IC_DMA_EVENTS;
        uint32_t contiguous = MAN_TIM_IC_DMA_EVENTS - read_index;
        if (contiguous > pending) {
            contiguous = pending;
        }

        const uint16_t *data = &g_tim_ic_dma[read_index];

        if (!snapshot_done) {
            snapshot_span(data, contiguous, 2u);
            snapshot_done = true;
        }

        if (consumer != NULL) {
            consumer(data, contiguous, context);
        }
        g_tim_ic_consumed_total += contiguous;
        pending -= contiguous;
    }

    g_dbg_ic_events_total += original_pending;
    g_dbg_ic_pending = 0u;
    return original_pending;
}


bool Manchester_TimIcCaptureStart(
    TIM_HandleTypeDef *htim,
    uint32_t channel)
{
    if (htim == NULL ||
        MAN_TIM_IC_DMA_EVENTS == 0u) {

        return false;
    }

    g_tim_ic_htim = htim;
    g_tim_ic_channel = channel;
    g_tim_ic_completed_halves = 0u;
    g_tim_ic_consumed_total = 0u;
    reset_debug_state();

    /*
     * Очень важная проверка:
     * MSP должен был связать CCx с DMA.
     */
    if (capture_dma_handle() == NULL) {
        ++g_dbg_ic_error_callbacks;
        g_tim_ic_htim = NULL;
        return false;
    }

    __HAL_TIM_SET_COUNTER(
        htim,
        0u
    );

    if (HAL_TIM_IC_Start_DMA(htim, channel, (uint32_t *)g_tim_ic_dma, (uint16_t)MAN_TIM_IC_DMA_EVENTS) != HAL_OK) {
        ++g_dbg_ic_error_callbacks;
        g_tim_ic_htim = NULL;
        return false;
    }

    return true;
}

void Manchester_TimIcCaptureStop(void)
{
    if (g_tim_ic_htim == NULL) {
        return;
    }

    (void)HAL_TIM_IC_Stop_DMA(g_tim_ic_htim, g_tim_ic_channel);
    g_tim_ic_htim = NULL;
}

void Manchester_OnTimIcCaptureHalfComplete(
    TIM_HandleTypeDef *htim)
{
    if (htim != g_tim_ic_htim) {
        return;
    }
    ++g_dbg_ic_half_callbacks;
    ++g_tim_ic_completed_halves;
    g_dbg_ic_dma_completed_halves = g_tim_ic_completed_halves;
    snapshot_span(&g_tim_ic_dma[0], MAN_TIM_IC_DMA_HALF_EVENTS, 0u);
    Manchester_TimIcCaptureWakeFromIsr();
}

void Manchester_OnTimIcCaptureComplete(
    TIM_HandleTypeDef *htim)
{
    if (htim != g_tim_ic_htim) {
        return;
    }

    ++g_dbg_ic_full_callbacks;
    ++g_tim_ic_completed_halves;
    g_dbg_ic_dma_completed_halves = g_tim_ic_completed_halves;

    snapshot_span(&g_tim_ic_dma[MAN_TIM_IC_DMA_HALF_EVENTS], MAN_TIM_IC_DMA_HALF_EVENTS, 1u);
    Manchester_TimIcCaptureWakeFromIsr();
}

void Manchester_OnTimIcError(
    TIM_HandleTypeDef *htim)
{
    if (htim != g_tim_ic_htim) {
        return;
    }
    ++g_dbg_ic_error_callbacks;
    Manchester_TimIcCaptureWakeFromIsr();
}
