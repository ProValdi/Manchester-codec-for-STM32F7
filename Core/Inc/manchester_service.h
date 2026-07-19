#ifndef MANCHESTER_SERVICE_H
#define MANCHESTER_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"
#include "manchester_types.h"

typedef struct {
    SPI_HandleTypeDef *hspi_rx;       /* expected: &hspi4 */
    TIM_HandleTypeDef *htim_tx;       /* expected: &htim1 */
    UART_HandleTypeDef *huart;        /* expected: &huart3 */

    TIM_HandleTypeDef *htim_rx_clk;   /* Continuous SPI SCLK: expected &htim8 */
    uint32_t tim_rx_clk_channel;      /* TIM_CHANNEL_1 ... TIM_CHANNEL_4: expected TIM_CHANNEL_1 */

    GPIO_TypeDef *tx_port;
    uint16_t tx_pin;

    GPIO_TypeDef *rf_recv_port;
    uint16_t rf_recv_pin;

    GPIO_TypeDef *rf_trans_port;
    uint16_t rf_trans_pin;

    uint32_t rf_tx_settle_ms;
    uint32_t rf_rx_settle_ms;

    GPIO_TypeDef *led_ok_port;
    uint16_t led_ok_pin;
    GPIO_TypeDef *led_tx_port;
    uint16_t led_tx_pin;
    GPIO_TypeDef *led_error_port;
    uint16_t led_error_pin;

    GPIO_TypeDef *dbg_rx_port;
    uint16_t dbg_rx_pin;
    GPIO_TypeDef *dbg_tx_port;
    uint16_t dbg_tx_pin;
    GPIO_TypeDef *dbg_uart_port;
    uint16_t dbg_uart_pin;
} man_platform_t;

/* Call after MX_GPIO/SPI1/TIM1/USART3/DMA_Init and before osKernelStart(). */
bool Manchester_ServiceInit(const man_platform_t *platform, const man_runtime_config_t *config);

/* Call inside MX_FREERTOS_Init(), after osKernelInitialize() and before osKernelStart(). */
bool Manchester_CreateRtosObjects(void);

/* HAL callback dispatchers. Call these from the matching USER CODE callback functions. */
void Manchester_OnSpiTxRxHalfComplete(SPI_HandleTypeDef *hspi);
void Manchester_OnSpiTxRxComplete(SPI_HandleTypeDef *hspi);
void Manchester_OnSpiError(SPI_HandleTypeDef *hspi);
void Manchester_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t position);
void Manchester_OnUartTxComplete(UART_HandleTypeDef *huart);
void Manchester_OnUartError(UART_HandleTypeDef *huart);

const man_diagnostics_t *Manchester_GetDiagnostics(void);
uint32_t Manchester_GetSpiSampleRateHz(void);

/* Weak no-op test hook. Override in user code to inject a bit error before line encoding. */
void Manchester_TestHookMutateWireBits(uint8_t *wire_bits, size_t bit_count);

#endif
