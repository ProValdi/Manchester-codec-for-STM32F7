#ifndef MANCHESTER_TIM_IC_CAPTURE_H
#define MANCHESTER_TIM_IC_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "manchester_config.h"


typedef void (*ManchesterTimIcSpanConsumer)(
    const uint16_t *timestamps,
    uint32_t count,
    void *context
);


bool Manchester_TimIcCaptureStart(
    TIM_HandleTypeDef *htim,
    uint32_t channel
);

void Manchester_TimIcCaptureStop(void);


/*
 * Забирает всё, что DMA успел записать с прошлого Drain().
 *
 * consumer == NULL разрешён:
 * данные просто считаются обработанными.
 */
uint32_t Manchester_TimIcCaptureDrain(
    ManchesterTimIcSpanConsumer consumer,
    void *context
);

/* Сколько timestamps сейчас ждут обработки. */
uint32_t Manchester_TimIcCapturePending(void);


/* HAL callbacks */
void Manchester_OnTimIcCaptureHalfComplete(
    TIM_HandleTypeDef *htim
);

void Manchester_OnTimIcCaptureComplete(
    TIM_HandleTypeDef *htim
);

void Manchester_OnTimIcError(
    TIM_HandleTypeDef *htim
);


/*
 * Реализуется в manchester_service.c.
 * HT/TC DMA будит ManchesterRxTask.
 */
void Manchester_TimIcCaptureWakeFromIsr(void);


extern uint16_t g_tim_ic_dma[
    MAN_TIM_IC_DMA_EVENTS
];


#define MAN_TIM_IC_DEBUG_SAMPLES 32u


extern volatile uint32_t g_dbg_ic_half_callbacks;
extern volatile uint32_t g_dbg_ic_full_callbacks;
extern volatile uint32_t g_dbg_ic_error_callbacks;

extern volatile uint32_t g_dbg_ic_dma_completed_halves;

extern volatile uint32_t g_dbg_ic_events_total;
extern volatile uint32_t g_dbg_ic_tail_drains;
extern volatile uint32_t g_dbg_ic_overruns;
extern volatile uint32_t g_dbg_ic_pending;
extern volatile uint32_t g_dbg_ic_write_index;

extern volatile uint32_t g_dbg_ic_recent_half;
extern volatile uint32_t g_dbg_ic_recent_min_delta;
extern volatile uint32_t g_dbg_ic_recent_max_delta;
extern volatile uint32_t g_dbg_ic_last_timestamp;

extern volatile uint16_t
    g_dbg_ic_recent_edges[
        MAN_TIM_IC_DEBUG_SAMPLES
    ];

extern volatile uint16_t
    g_dbg_ic_recent_deltas[
        MAN_TIM_IC_DEBUG_SAMPLES - 1u
    ];


#endif
