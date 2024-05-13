#include <Arduino.h>
#include "dw3000.h"
#include "rtls.h"

TaskHandle_t Tx_Callback_Handle = NULL, Rx_Callback_Handle = NULL, Rx_Timeout_Handle = NULL, Rx_Error_Handle = NULL, Spi_Error_Handle = NULL, Spi_Ready_Handle = NULL;

void Tx_Callback_ISR(const dwt_cb_data_t *cb_data);

void Rx_Callback_ISR(const dwt_cb_data_t *cb_data);

void Rx_Timeout_ISR(const dwt_cb_data_t *cb_data);

void Rx_Error_ISR(const dwt_cb_data_t *cb_data);

void Spi_Error_ISR(const dwt_cb_data_t *cb_data);

void Spi_Ready_ISR(const dwt_cb_data_t *cb_data);

void setup()
{
    Serial.begin(115200);

    // dwt_setcallbacks(&Tx_Callback_ISR, &Rx_Callback_ISR, &Rx_Timeout_ISR, &Rx_Error_ISR, &Spi_Error_ISR, &Spi_Ready_ISR);
    dwt_setcallbacks(NULL, &Rx_Callback_ISR, NULL, NULL, NULL, NULL);

    // dwt_setinterrupt(
    //               SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
    //               SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
    //               0,
    //               DWT_ENABLE_INT);
    dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);

    /* Create FreeRTOS Tasks Begin */
    xTaskCreatePinnedToCore(RTLS_Task, "RTLS_Task", 4096, NULL, 1, &Rx_Callback_Handle, 1);
}

void loop()
{
    
}

void Tx_Callback_ISR(const dwt_cb_data_t *cb_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Tx_Callback_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Rx_Callback_ISR(const dwt_cb_data_t *cb_data)
{
    // Notify RTLS_Task that Rx_Callback_ISR has been called (Posting for "uint32_t notify_return = ulTaskNotifyTake(pdTRUE, portMAX_DELAY)")
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Rx_Callback_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Rx_Timeout_ISR(const dwt_cb_data_t *cb_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Rx_Timeout_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Rx_Error_ISR(const dwt_cb_data_t *cb_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Rx_Error_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Spi_Error_ISR(const dwt_cb_data_t *cb_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Spi_Error_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Spi_Ready_ISR(const dwt_cb_data_t *cb_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Spi_Ready_Handle, cb_data->status, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}