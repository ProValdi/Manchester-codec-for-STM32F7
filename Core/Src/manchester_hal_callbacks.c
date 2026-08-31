#include "manchester_service.h"
#include "manchester_tim_ic_capture.h"

/*
 * Keep this file as-is when the project has no other HAL callbacks.
 * If the application already defines one of these callbacks, omit this file and
 * call the matching Manchester_On... dispatcher from the existing callback.
 */
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

void HAL_TIM_IC_CaptureHalfCpltCallback(
    TIM_HandleTypeDef *htim)
{
    Manchester_OnTimIcCaptureHalfComplete(htim);
}

void HAL_TIM_IC_CaptureCallback(
    TIM_HandleTypeDef *htim)
{
    Manchester_OnTimIcCaptureComplete(htim);
}

void HAL_TIM_ErrorCallback(
    TIM_HandleTypeDef *htim)
{
    Manchester_OnTimIcError(htim);
}
