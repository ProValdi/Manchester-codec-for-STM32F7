#include "manchester_service.h"

/*
 * Keep this file as-is when the project has no other HAL callbacks.
 * If the application already defines one of these callbacks, omit this file and
 * call the matching Manchester_On... dispatcher from the existing callback.
 */
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    Manchester_OnSpiTxRxHalfComplete(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Manchester_OnSpiTxRxComplete(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Manchester_OnSpiError(hspi);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Manchester_OnUartRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Manchester_OnUartTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Manchester_OnUartError(huart);
}
